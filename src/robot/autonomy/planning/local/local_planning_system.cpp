#include "robot/autonomy/planning/local/local_planning_system.hpp"

#include "robot/autonomy/planning/local/ackermann_local.hpp"
#include "robot/autonomy/planning/local/command_window.hpp"
#include "robot/autonomy/planning/local/dynamic_clearance.hpp"
#include "robot/autonomy/planning/local/holonomic_local.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>

namespace sim {
namespace {

double rollout_horizon(const LocalPlanningRequest& request) {
 const double speed = std::max(0.0, request.max_speed_mps);
 const double yaw_rate = std::max(1e-6, request.max_yaw_rate_radps);
 // During an intentional in-place turn, translational speed is zero. Treating
 // that as an epsilon speed stretches the rollout to the full prediction
 // horizon; a constant yaw sample can then make a full turn and receive the
 // same terminal score as the short rotation. Use only the heading/braking
 // horizon for that maneuver.
 const double distance_time = speed > 1e-6
 ? std::hypot(request.target.x - request.position.x,
 request.target.y - request.position.y) / speed
 : 0.0;
 double target_heading = request.target_heading_rad.value_or(std::atan2(
 request.target.y - request.position.y,
 request.target.x - request.position.x));
 if (!request.target_heading_rad && request.route_is_reverse &&
 request.command_space == LocalCommandSpace::DifferentialTwist) {
 target_heading = wrap_angle(target_heading + kPi);
 }
 const double heading_error = std::abs(wrap_angle(target_heading - request.heading_rad));
 double heading_time = heading_error / yaw_rate;
 if (request.command_space == LocalCommandSpace::HolonomicTwist &&
 request.terminal_position && heading_error > request.terminal_heading_tolerance_rad) {
 // Near a pose goal the feedback law asks for yaw ~= 1.5 * error. Roll out
 // long enough to observe that bounded proportional correction; dividing by
 // the chassis maximum yaw rate made a small residual look finished before
 // any candidate could enter the terminal heading tolerance.
 const double regulated_yaw_rate = std::min(yaw_rate, std::max(0.1, 1.5 * heading_error));
 heading_time = heading_error / regulated_yaw_rate;
 }
 const double braking_time = std::abs(request.forward_mps) /
 std::max(1e-6, request.max_decel_mps2 > 0.0
 ? request.max_decel_mps2 : request.max_accel_mps2);
 const double angular_braking_time = std::abs(request.yaw_rate_radps) /
 std::max(1e-6, request.max_yaw_accel_radps2);
 const double stopping_horizon = request.emergency_stop_delay_s +
 std::max(braking_time, angular_braking_time);
 // A closed behavior stop plane is a control-barrier boundary. Evaluate the
 // next receding-horizon command and its complete independently generated
 // braking tail; do not pretend the same nonzero velocity command is held all
 // the way to the stop pose and only then begin braking. That longer rollout
 // makes every positive approach infeasible near the line and deadlocks the
 // vehicle at an arbitrary upstream pose. Heading alignment may still need a
 // longer zero-translation rollout, hence the retained heading horizon.
 if (request.stop_plane_active) {
 return std::min(
 request.prediction_horizon_s,
 std::max({2.0 * request.step_s, heading_time,
 request.emergency_stop_delay_s + request.step_s}));
 }
 // A rollout represents one receding-horizon action toward the current local
 // target. Extending a constant yaw command well past that target makes a
 // valid turn look like a multi-revolution loop and causes stop/turn limit
 // cycles. Prediction may cover farther into the future; executable rollout
 // time is the kinematic time needed for this local target and safe braking.
 return std::min(request.prediction_horizon_s,
 std::max({2.0 * request.step_s, distance_time, heading_time, stopping_horizon}));
}

struct FreeSpaceProjection {
 Vec2 point;
 double distance_m{std::numeric_limits<double>::infinity()};
};

FreeSpaceProjection nearest_free_space(
 const GridMap& map, Vec2 point, Vec2 preferred_target,
 int free_neighborhood_cells = 0) {
 if (map.isFree(point.x, point.y) && free_neighborhood_cells == 0)
 return {point, 0.0};
 FreeSpaceProjection best;
 double best_heading_error_rad = std::numeric_limits<double>::infinity();
 const double preferred_heading_rad = std::atan2(
 preferred_target.y - point.y, preferred_target.x - point.x);
 for (int gy = 0; gy < map.h(); ++gy) {
 for (int gx = 0; gx < map.w(); ++gx) {
 if (!map.isFreeCell(gx, gy))
 continue;
 bool stable_free = true;
 for (int dy = -free_neighborhood_cells;
 stable_free && dy <= free_neighborhood_cells; ++dy) {
 for (int dx = -free_neighborhood_cells;
 dx <= free_neighborhood_cells; ++dx) {
 if (!map.isFreeCell(gx + dx, gy + dy)) {
 stable_free = false;
 break;
 }
 }
 }
 if (!stable_free)
 continue;
 const Vec2 free_point = map.gridToWorld(gx, gy);
 const double distance_m = std::hypot(
 point.x - free_point.x, point.y - free_point.y);
 const double heading_error_rad = std::abs(wrap_angle(
 std::atan2(free_point.y - point.y, free_point.x - point.x) -
 preferred_heading_rad));
 constexpr double kDistanceEquivalenceM = 1e-9;
 if (distance_m + kDistanceEquivalenceM < best.distance_m ||
 (std::abs(distance_m - best.distance_m) <= kDistanceEquivalenceM &&
 heading_error_rad < best_heading_error_rad)) {
 best = {free_point, distance_m};
 best_heading_error_rad = heading_error_rad;
 }
 }
 }
 return best;
}

double distance_to_free_space(const GridMap& map, Vec2 point) {
 return map.distanceToFreeSpace(point.x, point.y);
}

DwaResult generate(const LocalPlanningRequest& request,
 const GridMap& planning_map,
 const std::vector<std::vector<DynamicObjectState>>& predicted_frames,
 double required_clearance_m, double minimum_forward_mps,
 const StaticFeasibility& terrain_feasible,
 double minimum_horizon_s = 0.0,
 bool stop_horizon_creep = false) {
 (void)predicted_frames;
 // Candidate generation owns kinematic reachability, static geometry and
 // terrain feasibility. Dynamic hypotheses stay out of this early filter:
 // the independent trajectory validator compares every complete candidate
 // at matching timestamps and can distinguish an ordinary rejection from a
 // measured minimum-risk escape. Deleting a rollout at its first predicted
 // overlap made a committed crossing return an empty set and brake in the
 // conflict zone before any contingency could be assessed.
 const std::vector<std::vector<DynamicObjectState>> generation_frames;
 const double predict_time_s = std::min(
 request.prediction_horizon_s,
 std::max(minimum_horizon_s, rollout_horizon(request)));
 if (request.command_space == LocalCommandSpace::HolonomicTwist) {
 HolonomicLocalConfig config;
 config.max_speed_mps = request.max_speed_mps;
 config.min_forward_speed_mps = minimum_forward_mps;
 config.max_yaw_radps = request.max_yaw_rate_radps;
 config.max_accel_mps2 = request.max_accel_mps2;
 config.max_yaw_accel_radps2 = request.max_yaw_accel_radps2;
 config.predict_time_s = predict_time_s;
 config.dt_s = request.step_s;
 config.frame_dt_s = request.step_s;
 config.safety_margin_m = required_clearance_m;
 config.robot_radius_m = request.robot_radius_m;
 config.terminal_position = request.terminal_position;
 config.terminal_heading_rad = request.target_heading_rad;
 config.terminal_position_tolerance_m = request.terminal_position_tolerance_m;
 config.terminal_max_speed_mps = request.terminal_max_speed_mps;
 config.terminal_heading_tolerance_rad = request.terminal_heading_tolerance_rad;
 config.terminal_max_yaw_rate_radps = request.terminal_max_yaw_rate_radps;
 return holonomic_local_compute(
 request.position, request.heading_rad, request.forward_mps,
 request.lateral_mps, request.yaw_rate_radps, request.target,
 planning_map, request.weights, generation_frames, config, terrain_feasible,
 request.reference);
 }
 if (request.command_space == LocalCommandSpace::SteeringAngle) {
 AckermannLocalConfig config;
 config.wheel_base_m = request.steering_wheel_base_m;
 config.max_steer_rad = request.max_steer_rad;
 config.max_steer_rate_radps = request.max_steer_rate_radps;
 config.steering_response_s = request.steering_response_s;
 config.max_accel_mps2 = request.max_accel_mps2;
 config.max_speed_mps = request.max_speed_mps;
 config.min_forward_speed_mps = minimum_forward_mps;
 config.preferred_direction = request.preferred_direction;
 config.predict_time_s = predict_time_s;
 config.dt_s = request.step_s;
 config.frame_dt_s = request.step_s;
 config.safety_margin_m = required_clearance_m;
 config.robot_radius_m = request.robot_radius_m;
 return ackermann_local_compute(
 request.position, request.heading_rad, request.forward_mps,
 request.steering_angle_rad, request.steering_rate_radps,
 request.target, planning_map, request.weights,
 generation_frames, config, terrain_feasible);
 }

  // The global route owns travel direction. A local sampler must not switch
  // into reverse merely because a backward rollout scores better while the
  // body is temporarily pointed away from its lookahead target. That creates
  // long forward/reverse limit cycles and violates the RoutePoint direction
  // contract. Deliberate recovery is requested separately below and is still
  // subjected to the same swept trajectory validator.
 double speed_limit_mps = std::max(0.0, request.max_speed_mps);
 if (stop_horizon_creep && request.emergency_stop_required &&
 !request.crossing_committed) {
 const double stop_time_s = std::max(
 0.0, request.prediction_horizon_s - request.emergency_stop_delay_s);
 speed_limit_mps = std::min(
 speed_limit_mps,
 std::max(0.0, request.emergency_decel_mps2) * stop_time_s);
 }
 const double minimum_velocity_mps = request.route_is_reverse ? -speed_limit_mps : 0.0;
 const double maximum_velocity_mps = request.route_is_reverse ? 0.0 : speed_limit_mps;
 auto target_heading_rad = request.target_heading_rad;
 if (!target_heading_rad && request.route_is_reverse) {
 target_heading_rad = wrap_angle(std::atan2(
 request.target.y - request.position.y,
 request.target.x - request.position.x) + kPi);
 }
 return dwa_compute_trajectory(
 request.position, request.heading_rad, request.forward_mps,
 request.yaw_rate_radps, request.target, planning_map, request.weights,
 generation_frames, maximum_velocity_mps, request.max_yaw_rate_radps,
 request.max_accel_mps2, predict_time_s, request.step_s,
 request.step_s, required_clearance_m, request.robot_radius_m,
 request.max_yaw_accel_radps2, terrain_feasible,
 minimum_forward_mps > 0.0 ? minimum_forward_mps : minimum_velocity_mps,
 target_heading_rad, request.reference,
 request.differential_track_m, request.max_wheel_speed_mps,
 request.actuator_response_delay_s, request.max_decel_mps2,
 request.drive_axle_x_m, request.measured_wheel_speeds);
}

ValidatedTrajectorySelection validate(const DwaResult& proposals,
 const LocalPlanningRequest& request, const GridMap& planning_map,
 const std::vector<std::vector<DynamicObjectState>>& predicted_frames,
 double required_clearance_m, const StaticFeasibility& terrain_feasible,
 const TerrainSampleQuery& terrain_sample) {
 TrajectoryConstraints constraints;
 constraints.stop_plane_active = request.stop_plane_active;
 constraints.stop_point = request.stop_point;
 constraints.stop_direction = request.stop_direction;
 constraints.stop_tolerance_m = request.stop_tolerance_m;
 constraints.emergency_stop_required = request.emergency_stop_required;
 constraints.emergency_stop_delay_s = request.emergency_stop_delay_s;
 constraints.emergency_decel_mps2 = request.emergency_decel_mps2;
 constraints.safe_stop_decel = request.safe_stop_decel;
 constraints.differential_track_m = request.differential_track_m;
 constraints.committed_transit =
 request.crossing_committed && request.committed_no_return;
 constraints.committed_progress_floor_mps = request.committed_progress_floor_mps;
 constraints.dynamic_reverse_escape = request.dynamic_reverse_escape;
 constraints.dynamic_reverse_speed_floor_mps =
 request.dynamic_reverse_speed_floor_mps;
 constraints.committed_reference_path = request.committed_reference_path;
 constraints.committed_tracking_corridor_m = request.committed_tracking_corridor_m;
 constraints.committed_reference_trajectory = request.committed_reference_trajectory;
 constraints.terrain_sample = terrain_sample;
 constraints.support_width_m = request.support_width_m;
 constraints.support_polygon = request.support_polygon;
 constraints.center_of_mass_x_m = request.center_of_mass_x_m;
 constraints.center_of_mass_height_m = request.center_of_mass_height_m;
 constraints.max_dynamic_rollover_risk = request.max_dynamic_rollover_risk;
 return select_executable_trajectory(
 proposals, planning_map, predicted_frames, request.step_s,
 request.robot_radius_m, required_clearance_m, terrain_feasible, constraints);
}

ValidatedTrajectorySelection select_static_margin_escape(
 const DwaResult& proposals, const LocalPlanningRequest& request,
 const GridMap& preferred_map, const GridMap& physical_collision_map,
 const std::vector<std::vector<DynamicObjectState>>& predicted_frames,
 double required_clearance_m, const StaticFeasibility& terrain_feasible,
 bool allow_incremental_escape, const TerrainSampleQuery& terrain_sample,
 std::optional<double> escape_heading_override = std::nullopt) {
 ValidatedTrajectorySelection output;
 int best_rank = 0;
 double best_margin_depth_m = std::numeric_limits<double>::infinity();
 double best_heading_error_rad = std::numeric_limits<double>::infinity();
 const double initial_margin_depth_m = allow_incremental_escape
 ? distance_to_free_space(preferred_map, request.position)
 : std::numeric_limits<double>::infinity();
 const auto escape_projection = escape_heading_override
 ? FreeSpaceProjection{}
 : nearest_free_space(preferred_map, request.position, request.target);
 TrajectoryConstraints constraints;
 constraints.stop_plane_active = request.stop_plane_active;
 constraints.stop_point = request.stop_point;
 constraints.stop_direction = request.stop_direction;
 constraints.stop_tolerance_m = request.stop_tolerance_m;
 constraints.emergency_stop_required = request.emergency_stop_required;
 constraints.emergency_stop_delay_s = request.emergency_stop_delay_s;
 constraints.emergency_decel_mps2 = request.emergency_decel_mps2;
 constraints.safe_stop_decel = request.safe_stop_decel;
 constraints.differential_track_m = request.differential_track_m;
 constraints.committed_transit =
 request.crossing_committed && request.committed_no_return;
 constraints.committed_progress_floor_mps = request.committed_progress_floor_mps;
 constraints.dynamic_reverse_escape = request.dynamic_reverse_escape;
 constraints.dynamic_reverse_speed_floor_mps =
 request.dynamic_reverse_speed_floor_mps;
 constraints.committed_reference_path = request.committed_reference_path;
 constraints.committed_tracking_corridor_m = request.committed_tracking_corridor_m;
 constraints.committed_reference_trajectory = request.committed_reference_trajectory;
 constraints.terrain_sample = terrain_sample;
 constraints.support_width_m = request.support_width_m;
 constraints.support_polygon = request.support_polygon;
 constraints.center_of_mass_x_m = request.center_of_mass_x_m;
 constraints.center_of_mass_height_m = request.center_of_mass_height_m;
 constraints.max_dynamic_rollover_risk = request.max_dynamic_rollover_risk;
 for (const auto& rollout : proposals.rollouts) {
 if (rollout.collision || !rollout.trajectory.command.valid ||
 rollout.trajectory.samples.empty()) {
 continue;
 }
 // A margin escape may cross the soft inflation already occupied at t=0,
 // but it must remain outside physical collision occupancy for the complete
 // swept trajectory. A differential chassis may need a physically safe
 // in-place alignment stage before either forward or reverse motion can leave
 // a long inflated region; requiring one constant-command rollout to perform
 // both stages deadlocks that valid recovery.
 const auto& terminal = rollout.trajectory.samples.back().position;
 const auto& start = rollout.trajectory.samples.front();
 const bool starts_inside_preferred_margin =
 !preferred_map.isFree(start.position.x, start.position.y);
 const bool exits_preferred_margin = preferred_map.isFree(terminal.x, terminal.y);
 const auto& finish = rollout.trajectory.samples.back();
 const double displacement_m = std::hypot(
 finish.position.x - start.position.x,
 finish.position.y - start.position.y);
 const auto differential_control_point = [&](const TrajectorySample& sample) {
 return Vec2{
 sample.position.x + request.drive_axle_x_m * std::cos(sample.heading_rad),
 sample.position.y + request.drive_axle_x_m * std::sin(sample.heading_rad),
 };
 };
 const Vec2 start_control = differential_control_point(start);
 const Vec2 finish_control = differential_control_point(finish);
 const double control_displacement_m = std::hypot(
 finish_control.x - start_control.x,
 finish_control.y - start_control.y);
 const double escape_travel_heading_rad = escape_heading_override.value_or(
 std::isfinite(escape_projection.distance_m)
 ? std::atan2(
 escape_projection.point.y - request.position.y,
 escape_projection.point.x - request.position.x)
 : std::atan2(
 request.target.y - request.position.y,
 request.target.x - request.position.x));
 const double escape_body_heading_rad = wrap_angle(
 escape_travel_heading_rad + (request.route_is_reverse ? kPi : 0.0));
 const double initial_heading_error = std::abs(
 std::remainder(escape_body_heading_rad - start.heading_rad, 2.0 * kPi));
 const double terminal_heading_error = std::abs(
 std::remainder(escape_body_heading_rad - finish.heading_rad, 2.0 * kPi));
 // Differential translation is defined at the driven axle. The base_link of
 // a caster-supported robot follows a small arc during an in-place turn when
 // that axle is offset from the body origin; treating that rigid transform as
 // commanded translation rejects the only alignment that points out of the
 // margin. The complete base-link sweep is still checked against the physical
 // map by the independent validator below.
 const double measured_translation_mps =
 request.command_space == LocalCommandSpace::DifferentialTwist
 ? std::abs(request.forward_mps)
 : std::hypot(request.forward_mps, request.lateral_mps);
 const double predicted_stop_translation_m =
 measured_translation_mps * std::max(0.0, request.emergency_stop_delay_s) +
 (request.emergency_decel_mps2 > 0.0
 ? measured_translation_mps * measured_translation_mps /
 (2.0 * request.emergency_decel_mps2)
 : (measured_translation_mps > 1e-6
 ? std::numeric_limits<double>::infinity() : 0.0));
 const bool improves_soft_margin = starts_inside_preferred_margin &&
 distance_to_free_space(preferred_map, finish.position) + 0.01 <
 distance_to_free_space(preferred_map, start.position);
 const bool stages_alignment =
 request.command_space == LocalCommandSpace::DifferentialTwist &&
 std::abs(rollout.trajectory.command.v) <= 1e-9 &&
 std::abs(rollout.trajectory.command.w) > 1e-6 &&
 // A zero target is not an in-place maneuver while the measured chassis is
 // still translating. Require its complete braking envelope to fit inside
 // the same displacement tolerance used to classify the rollout; otherwise
 // control must brake first and replan from the measured stopped state.
 predicted_stop_translation_m <= 0.02 &&
 // Rotating around an offset driven axle moves base_link along a real,
 // bounded arc. Accept that geometric sweep as alignment progress; the
 // swept physical validator below still rejects an arc that contacts an
 // obstacle or violates the execution reserve.
 control_displacement_m <=
 0.02 + std::abs(request.drive_axle_x_m) * std::abs(wrap_angle(
 finish.heading_rad - start.heading_rad)) &&
 (terminal_heading_error + 0.05 < initial_heading_error ||
 improves_soft_margin);
 const bool differential_rotation_at_rest =
 request.command_space == LocalCommandSpace::DifferentialTwist &&
 std::abs(request.forward_mps) <= 1e-6 &&
 std::abs(rollout.trajectory.command.v) <= 1e-9;
 // Crossing a raster cell boundary because base_link arcs around an offset
 // drive axle is alignment progress, not proof that translation has escaped
 // the inflation layer. It is eligible only when it also improves the escape
 // heading. Otherwise normal planning immediately turns back into the same
 // cell and produces a recovery limit cycle.
 const double escape_progress_m =
 (finish.position.x - start.position.x) * std::cos(escape_travel_heading_rad) +
 (finish.position.y - start.position.y) * std::sin(escape_travel_heading_rad);
 const bool requires_outward_progress =
 escape_heading_override.has_value() &&
 preferred_map.isFree(request.position.x, request.position.y);
 const bool exits_with_translation = starts_inside_preferred_margin &&
 exits_preferred_margin &&
 !differential_rotation_at_rest &&
 (!requires_outward_progress || escape_progress_m >= 0.02);
 const bool commanded_translation =
 std::hypot(rollout.trajectory.command.v,
 rollout.trajectory.command.lateral_v) > 1e-9;
 const double terminal_margin_depth_m =
 distance_to_free_space(preferred_map, terminal);
 const bool reduces_margin_incursion = starts_inside_preferred_margin &&
 allow_incremental_escape &&
 !exits_preferred_margin && commanded_translation &&
 displacement_m >= 0.05 && terminal_margin_depth_m + 0.05 <
 initial_margin_depth_m;
 const bool restores_preferred_clearance = allow_incremental_escape &&
 preferred_map.clearance(terminal.x, terminal.y) >
 preferred_map.clearance(start.position.x, start.position.y) + 0.005 &&
 (commanded_translation || std::abs(rollout.trajectory.command.w) > 1e-6);
 const bool stages_existing_incursion_alignment =
 starts_inside_preferred_margin && stages_alignment;
 // If the measured chassis is still drifting toward a soft boundary, a
 // validated zero command is a real actuator braking step, not an idle
 // fallback. Accept it so the next cycle can rebuild a reverse window from
 // the stopped measured wheel speeds; otherwise every reverse candidate can
 // be unreachable while the body continues to creep.
 const bool stages_braking_hold = starts_inside_preferred_margin &&
 std::abs(rollout.trajectory.command.v) <= 1e-9 &&
 std::abs(rollout.trajectory.command.w) <= 1e-9 &&
 predicted_stop_translation_m <= 0.02;
 const bool continues_committed_transit = request.crossing_committed &&
 rollout.trajectory.command.v + 1e-9 >= request.committed_progress_floor_mps &&
 // A restart from rest may move less than the ordinary 5 cm escape
 // evidence threshold during its first rate-limited rollout. It is still
 // real forward progress when the command is positive and the simulated
 // body displacement is non-zero; later replans raise the reachable floor.
 rollout.trajectory.command.v > 1e-9 && displacement_m > 1e-6;
 if (!exits_with_translation && !continues_committed_transit &&
 !reduces_margin_incursion && !restores_preferred_clearance &&
 !stages_existing_incursion_alignment &&
 !stages_braking_hold) {
 continue;
 }
 auto validation = validate_executable_trajectory(
 rollout.trajectory, physical_collision_map, predicted_frames,
 request.step_s, request.robot_radius_m, required_clearance_m,
 terrain_feasible, constraints);
if (!validation.accepted()) {
 continue;
 }
 ++output.accepted_candidates;
 const int rank = exits_with_translation
 ? 4
 : (continues_committed_transit ? 3
 : ((reduces_margin_incursion || restores_preferred_clearance ||
 (stages_existing_incursion_alignment &&
 std::abs(request.drive_axle_x_m) > 1e-9)) ? 2 : 1));
 constexpr double kDepthEquivalenceM = 1e-6;
 constexpr double kHeadingEquivalenceRad = 1e-6;
 const bool shallower_margin =
 terminal_margin_depth_m + kDepthEquivalenceM < best_margin_depth_m;
 const bool equivalent_margin =
 std::abs(terminal_margin_depth_m - best_margin_depth_m) <=
 kDepthEquivalenceM;
 const bool better_heading =
 terminal_heading_error + kHeadingEquivalenceRad < best_heading_error_rad;
 const bool equivalent_heading =
 std::abs(terminal_heading_error - best_heading_error_rad) <=
 kHeadingEquivalenceRad;
const bool preferred_within_rank = shallower_margin ||
 (equivalent_margin && (better_heading ||
 (equivalent_heading && dwa_command_preferred(
rollout.trajectory.command, output.trajectory.command))));
 if (rank < best_rank || (rank == best_rank && !preferred_within_rank))
 continue;
 validation.verdict = continues_committed_transit && !exits_preferred_margin
 ? TrajectoryVerdict::CommittedTransit
 : TrajectoryVerdict::MinimumRiskEscape;
 validation.diagnostic = exits_with_translation
 ? "trajectory exits an existing preferred static-margin incursion"
 : (continues_committed_transit
 ? "physically validated transit continues through a preferred static margin"
 : (reduces_margin_incursion
 ? "trajectory reduces an existing preferred static-margin incursion"
: "in-place alignment stages a physically validated static-margin escape"));
 best_rank = rank;
 best_margin_depth_m = terminal_margin_depth_m;
 best_heading_error_rad = terminal_heading_error;
 output.trajectory = rollout.trajectory;
 output.validation = std::move(validation);
 }
 return output;
}

ValidatedTrajectorySelection select_near_boundary_clearance_escape(
 const DwaResult& proposals, const LocalPlanningRequest& request,
 const GridMap& preferred_map, const GridMap& physical_collision_map,
 const std::vector<std::vector<DynamicObjectState>>& predicted_frames,
 double required_clearance_m, const StaticFeasibility& terrain_feasible,
 const TerrainSampleQuery& terrain_sample,
 double minimum_clearance_gain_m) {
 ValidatedTrajectorySelection output;
 const double initial_clearance_m = preferred_map.clearance(
 request.position.x, request.position.y);
 double best_clearance_m = initial_clearance_m + minimum_clearance_gain_m;
 double best_target_distance_m = std::numeric_limits<double>::infinity();
 for (const auto& rollout : proposals.rollouts) {
 if (rollout.collision || !rollout.trajectory.command.valid ||
 rollout.trajectory.samples.size() < 2) {
  continue;
 }
 // A body which still starts in preferred free space may not use the
 // physical-map contingency to cut through soft inflation. Once the measured
 // pose is already inside that soft band, however, including the initial
 // sample in this test makes every escape impossible: the first sample is
 // necessarily occupied even when the rollout immediately restores
 // clearance. In that case the terminal clearance gain below is the control
 // barrier; the physical swept validator remains authoritative.
 const bool starts_inside_preferred = !preferred_map.isFree(
 request.position.x, request.position.y);
 const bool all_preferred_free = starts_inside_preferred ||
 std::none_of(rollout.trajectory.samples.begin(),
 rollout.trajectory.samples.end(), [&](const auto& sample) {
 return !preferred_map.isFree(sample.position.x, sample.position.y);
 });
 if (!all_preferred_free) {
  continue;
 }
 const auto& terminal = rollout.trajectory.samples.back();
 const double terminal_clearance_m = preferred_map.clearance(
 terminal.position.x, terminal.position.y);
 // The hard continuous constraint is the physical execution reserve in the
 // swept validator. Preferred inflation is a soft planning envelope; an
 // escaping body may briefly consume it while reversing an offset-axle turn,
 // provided the terminal state restores clearance. Requiring monotonic soft
 // clearance here rejected the only braking/translating rollouts and handed
 // control back to the route tracker, which repeated the same inward turn.
 if (terminal_clearance_m <=
 initial_clearance_m + minimum_clearance_gain_m - 1e-9) {
  continue;
 }
 const auto candidate = validate(
 DwaResult{rollout.trajectory, {rollout}}, request,
 physical_collision_map, predicted_frames, required_clearance_m,
 terrain_feasible, terrain_sample);
 if (!candidate.validation.accepted()) {
  continue;
 }
 const double target_distance_m = std::hypot(
 terminal.position.x - request.target.x,
 terminal.position.y - request.target.y);
 const bool more_clearance =
 terminal_clearance_m > best_clearance_m + 1e-9;
 const bool equivalent_clearance_better_target =
 std::abs(terminal_clearance_m - best_clearance_m) <= 1e-9 &&
 (target_distance_m < best_target_distance_m - 1e-9 ||
 (std::abs(target_distance_m - best_target_distance_m) <= 1e-9 &&
 dwa_command_preferred(
 rollout.trajectory.command, output.trajectory.command)));
 if (!more_clearance && !equivalent_clearance_better_target)
 continue;
 best_clearance_m = terminal_clearance_m;
 best_target_distance_m = target_distance_m;
 output = candidate;
 output.validation.verdict = TrajectoryVerdict::MinimumRiskEscape;
 output.validation.diagnostic =
 "trajectory restores clearance from a near-boundary local minimum";
 }
 return output;
}

} // namespace

LocalPlanningOutput LocalPlanningSystem::plan(
 const LocalPlanningRequest& requested, const GridMap& planning_map,
 const std::vector<std::vector<DynamicObjectState>>& predicted_frames,
 StaticFeasibility terrain_feasible,
 const GridMap* physical_collision_map,
 StaticFeasibility physical_terrain_feasible,
 TerrainSampleQuery terrain_sample) {
 // A behavior progress floor is an intent for this update, not permission to
 // skip actuator dynamics.  After an independently validated stop inside a
 // committed crossing, the next reachable command can be below the nominal
 // floor.  Use that reachable positive command as this cycle's contingency
 // threshold, then raise it on later replans as the dynamic window expands.
 // Keeping the unreachable nominal floor in the validator rejects every
 // generated rollout and permanently strands the body in the conflict zone.
 auto request = requested;
 const double localization_confidence = std::isfinite(request.localization_confidence)
 ? std::clamp(request.localization_confidence, 0.0, 1.0) : 0.0;
 const double localization_age_s = std::isfinite(request.localization_age_s)
 ? std::max(0.0, request.localization_age_s)
 : std::numeric_limits<double>::infinity();
 // Localization may constrain an ordinary motion plan, but it never supplies
 // a command. Severely stale or badly degraded health closes the drive window;
 // the actuator window below still produces a physically reachable braking
 // command from the measured speed. Reduced evidence retains a usable but
 // deliberately slower operating envelope.
 // Invalid is a quality signal, not an automatic command to freeze: a finite
 // but degraded wheel/IMU estimate can still support a bounded slow motion.
 // Reserve the hold state for confidence below the minimum usable envelope or
 // an actually stale stream whose age has exceeded the recovery grace period.
	const auto& authority = motion_policy_.localization_motion;
	// `localization_valid` is the estimator's hard covariance/finite-state
	// contract. A valid but low soft score is therefore limited to the cautious
	// envelope, not the most severe reduced envelope; otherwise sparse-map
	// scenes turn a bounded odometric estimate into an unnecessarily slow
	// mission timeout. Invalid estimates still use the full reduced scale.
	const double effective_localization_confidence = request.localization_valid
		? std::max(localization_confidence, authority.reduced_confidence)
		: localization_confidence;
	// Age is already reflected in localization_confidence through the stale
	// map-evidence term. Do not turn an old map fix into an unconditional stop
	// when the independent covariance/IMU/encoder estimate remains valid; stop
	// only when health is invalid, confidence is below the hold floor, or stale
	// evidence has also fallen below the reduced-motion floor.
	const bool localization_hold = !std::isfinite(localization_confidence) ||
		localization_confidence < authority.hold_confidence ||
		(localization_age_s > authority.hold_max_age_s &&
		 effective_localization_confidence < authority.reduced_confidence);
	const double localization_speed_scale = localization_hold ? 0.0
		: !request.localization_valid || effective_localization_confidence < authority.reduced_confidence
		? authority.reduced_speed_scale
		: effective_localization_confidence < authority.cautious_confidence
		? authority.cautious_speed_scale : 1.0;
 request.max_speed_mps =
 std::max(0.0, request.max_speed_mps) * localization_speed_scale;
 if (request.command_space == LocalCommandSpace::DifferentialTwist &&
 request.crossing_committed && request.committed_no_return &&
 request.committed_progress_floor_mps > 0.0 && request.step_s > 0.0) {
 const auto reachable = next_velocity_command_window(
 request.forward_mps, request.committed_progress_floor_mps,
 request.max_speed_mps,
 request.max_accel_mps2 * request.step_s,
 request.max_decel_mps2 * request.step_s);
 request.committed_progress_floor_mps = std::max(
 0.0, std::min(request.committed_progress_floor_mps, reachable.maximum));
 }
 LocalPlanningOutput output;
 const auto with_recovery_telemetry = [&](LocalPlanningOutput value) {
 value.static_margin_recovery_active = static_margin_recovery_gear_ != 0;
 value.static_margin_recovery_gear = static_margin_recovery_gear_;
 value.static_margin_escape_origin = static_margin_escape_origin_;
 value.static_margin_escape_heading_rad = static_margin_escape_heading_rad_;
 value.static_margin_escape_required_progress_m =
 static_margin_escape_required_progress_m_;
 value.static_margin_escape_progress_m =
 (request.position.x - static_margin_escape_origin_.x) *
 std::cos(static_margin_escape_heading_rad_) +
 (request.position.y - static_margin_escape_origin_.y) *
 std::sin(static_margin_escape_heading_rad_);
 value.near_boundary_clearance_recovery_active =
 near_boundary_clearance_recovery_active_;
 value.status.decision_time_s = request.time_s;
 value.status.speed_cap_mps = request.max_speed_mps;
 if (!value.selection.validation.accepted()) {
  value.status.mode = LocalMotionMode::NoSafeAction;
  value.status.motion_authorized = false;
  value.status.reason = value.selection.validation.diagnostic.empty()
  ? "no candidate passed the complete safety validation"
  : value.selection.validation.diagnostic;
  value.status.recovery_condition =
  "a collision-free, actuator-feasible and stable action must be proved";
 } else if (value.selection.validation.minimum_risk_escape()) {
  value.status.mode = LocalMotionMode::MinimumRiskEscape;
  value.status.motion_authorized = true;
  value.status.reason = value.selection.validation.diagnostic;
  value.status.recovery_condition =
  "ordinary collision-free planning must become available";
 } else if (localization_hold) {
  value.status.mode = LocalMotionMode::LocalizationHold;
  value.status.motion_authorized = true;
  value.status.speed_cap_mps = 0.0;
  value.status.reason = "localization is invalid, stale or insufficient for ordinary motion";
  value.status.recovery_condition =
  "a fresh accepted localization correction must restore confidence and coverage";
 } else if (localization_speed_scale < 1.0) {
  value.status.mode = LocalMotionMode::LocalizationReduced;
  value.status.motion_authorized = true;
  value.status.reason = "localization confidence limits the ordinary speed envelope";
  value.status.recovery_condition =
  "sustained accepted localization evidence must restore the nominal envelope";
 } else {
  value.status.mode = LocalMotionMode::Nominal;
  value.status.motion_authorized = true;
  value.status.reason = value.selection.validation.diagnostic;
 }
 return value;
 };
 const bool dynamic_forecast_present = std::any_of(
 predicted_frames.begin(), predicted_frames.end(),
 [](const auto& frame) { return !frame.empty(); });
 const bool inside_preferred_margin =
 !planning_map.isFree(request.position.x, request.position.y);
 const bool physical_escape_available = physical_collision_map &&
 physical_collision_map->isFree(request.position.x, request.position.y);
 if (static_margin_recovery_gear_ != 0 && !inside_preferred_margin) {
 const double measured_escape_progress_m =
 (request.position.x - static_margin_escape_origin_.x) *
 std::cos(static_margin_escape_heading_rad_) +
 (request.position.y - static_margin_escape_origin_.y) *
 std::sin(static_margin_escape_heading_rad_);
 // A base_link can cross a raster boundary while rotating around an offset
 // driven axle, so a bare isFree transition is insufficient evidence of an
 // escape. Continuous clearance provides the missing distinction: once the
 // measured pose is at least a quarter cell inside free space, retaining the
 // old escape heading can turn a completed recovery back toward the obstacle.
 const bool robustly_outside_margin =
 planning_map.clearance(request.position.x, request.position.y) + 1e-9 >=
 0.25 * planning_map.res();
 if (robustly_outside_margin || measured_escape_progress_m + 1e-9 >=
 static_margin_escape_required_progress_m_) {
 static_margin_recovery_gear_ = 0;
 static_margin_escape_required_progress_m_ = 0.0;
 }
 }
 const bool static_margin_recovery_required = !localization_hold &&
 (inside_preferred_margin || static_margin_recovery_gear_ != 0);
 if (static_margin_recovery_required && physical_escape_available) {
 const double measured_margin_depth_m = distance_to_free_space(planning_map, request.position);
 // Recovery state follows measured motion, not the endpoint of a predicted
 // rollout. A valid command can fail to make progress because of tracking
 // error, contact or a short receding horizon. Treating that prediction as
 // success restarted the fallback timer every cycle and left the robot
 // oscillating inside the same inflation band.
 constexpr double kMeasuredProgressM = 0.02;
 if (!std::isfinite(static_margin_best_depth_m_) ||
 measured_margin_depth_m + kMeasuredProgressM < static_margin_best_depth_m_) {
 static_margin_best_depth_m_ = measured_margin_depth_m;
 static_margin_blocked_s_ = 0.0;
 } else {
 static_margin_blocked_s_ += std::max(0.0, request.step_s);
 }
 if (static_margin_recovery_gear_ == 0) {
const auto escape_projection = nearest_free_space(
planning_map, request.position, request.target);
 static_margin_escape_origin_ = request.position;
 static_margin_escape_heading_rad_ = std::atan2(
 escape_projection.point.y - request.position.y,
 escape_projection.point.x - request.position.x);
 static_margin_escape_required_progress_m_ =
std::max(0.0, escape_projection.distance_m - 0.5 * planning_map.res());
 static_margin_recovery_gear_ = request.route_is_reverse ? -1 : 1;
 const bool reverse_allowed =
 request.command_space == LocalCommandSpace::DifferentialTwist &&
 !request.route_is_reverse && !request.crossing_committed &&
 !request.committed_no_return;
 if (reverse_allowed) {
 const double forward_error_rad = std::abs(wrap_angle(
 static_margin_escape_heading_rad_ - request.heading_rad));
 const double reverse_error_rad = std::abs(wrap_angle(
 static_margin_escape_heading_rad_ + kPi - request.heading_rad));
 // A small hysteresis keeps an almost perpendicular body in forward gear;
 // reverse is selected only when it saves a meaningful alignment turn.
 constexpr double kReverseAlignmentAdvantageRad = 0.10;
 if (reverse_error_rad + kReverseAlignmentAdvantageRad < forward_error_rad)
 static_margin_recovery_gear_ = -1;
 }
 }
 auto escape_request = request;
 // A committed conflict-zone transit already owns a proven minimum-progress
 // contract. Static soft-margin recovery may slow that command, but it may
 // not make the dynamic window empty by capping vmax below the committed
 // floor. Physical collision, terrain, dynamic clearance and stability are
 // still validated below against the recovery rollout.
 const double recovery_speed_floor_mps = request.crossing_committed
 ? request.committed_progress_floor_mps
 : 0.0;
 escape_request.max_speed_mps = std::min(
 request.max_speed_mps,
 std::max(request.recovery_max_speed_mps, recovery_speed_floor_mps));
 escape_request.max_yaw_rate_radps = std::min(
 escape_request.max_yaw_rate_radps, escape_request.recovery_max_yaw_rate_radps);
 if (request.command_space == LocalCommandSpace::DifferentialTwist) {
 escape_request.route_is_reverse = static_margin_recovery_gear_ < 0;
 escape_request.target_heading_rad = wrap_angle(
 static_margin_escape_heading_rad_ +
 (escape_request.route_is_reverse ? kPi : 0.0));
 }
 const auto attempt_escape = [&](const StaticFeasibility& feasibility,
 bool allow_incremental_escape) {
 output.proposals = generate(
 escape_request, *physical_collision_map, predicted_frames,
 escape_request.required_clearance_m, escape_request.committed_progress_floor_mps,
 // Candidate generation is only a bounded geometric sampler here. The
 // complete terrain and execution feasibility contract is applied by the
 // independent validator below; filtering the same recovery twice made a
 // non-worsening escape disappear before it could be evaluated.
 {}, 1.0);
 output.selection = select_static_margin_escape(
 output.proposals, escape_request, planning_map, *physical_collision_map,
 predicted_frames, escape_request.required_clearance_m, feasibility,
 allow_incremental_escape, terrain_sample,
 static_margin_escape_heading_rad_);
 };
 // Preserve the normal static-clearance contract first. If that envelope
 // already contains the body and rejects every recovery, retry against the
 // physical footprint map with terrain feasibility only. This fallback does
 // not weaken collision validation; it removes duplicate soft inflation.
 attempt_escape(terrain_feasible, false);
 const bool nominal_escape_accepted = output.selection.validation.accepted();
 LocalPlanningOutput nominal_output = output;
 // Static-inflation recovery and dynamic clearance are independent
 // contracts. A nearby tracked actor must not permanently disable recovery
 // from a static soft margin: every candidate below is still checked against
 // the complete time-indexed forecast and can only be accepted as a named
 // minimum-risk escape. Using the actor's current distance as a gate created
 // a deadlock when the body was simultaneously inside both soft margins.
 // Once the body already violates soft inflation, waiting cannot restore
 // clearance. Compare the ordinary result with an immediate physical-map
 // escape that must make measured-direction progress and still pass the
 // complete collision, terrain, prediction and stability validators.
 if (physical_terrain_feasible && !request.stop_plane_active) {
 // Route tracking exposes max_speed=0 while it waits for an in-place
 // alignment. Inside a preferred-margin incursion that zero is the local
 // minimum being escaped, not a behavior stop plane (which is excluded
 // above), and a zero-speed window leaves only alignment rollouts. Those
 // rollouts can walk an offset axle deeper into the margin without ever
 // translating out. Keep any positive behavior cap, but replace only the
 // zero cap with the independently bounded recovery capability.
 if (escape_request.max_speed_mps <= 1e-9) {
  escape_request.max_speed_mps = std::min(
   request.recovery_max_speed_mps,
   request.collision_escape_max_speed_mps);
 }
 attempt_escape(physical_terrain_feasible, true);
 if (output.selection.validation.accepted()) {
 physical_margin_recovery_active_ = true;
 return with_recovery_telemetry(std::move(output));
 }
 }
 if (nominal_escape_accepted)
 return with_recovery_telemetry(std::move(nominal_output));
 // A committed maneuver beyond its no-return line must still reach the
 // dynamic collision-escape comparison below. Returning the empty soft-
 // margin result here strands the body even when the physical map contains
 // a lower-risk motion. Ordinary margin recovery keeps the bounded early
 // return used by non-committed navigation.
 if (!(request.crossing_committed && request.committed_no_return))
 return with_recovery_telemetry(std::move(output));
 }
 static_margin_blocked_s_ = 0.0;
 static_margin_best_depth_m_ = std::numeric_limits<double>::infinity();
 physical_margin_recovery_active_ = false;
 static_margin_recovery_gear_ = 0;
 static_margin_escape_origin_ = {};
 static_margin_escape_heading_rad_ = 0.0;
 static_margin_escape_required_progress_m_ = 0.0;
 output.proposals = generate(
 request, planning_map, predicted_frames, request.required_clearance_m,
 request.committed_progress_floor_mps, terrain_feasible);
 output.selection = validate(output.proposals, request, planning_map,
 predicted_frames, request.required_clearance_m, terrain_feasible,
 terrain_sample);

 // Keep ordinary motion governed by the actuator dynamic window and the
 // complete validated braking tail. If its coarse velocity grid contains no
 // safe candidate, add a second deterministic creep sample whose complete
 // stop fits in the forecast. Applying this cap before the first pass turns a
 // local fallback into a permanent route-wide speed limit.
 if (!output.selection.validation.accepted() &&
 request.emergency_stop_required && !request.crossing_committed) {
 auto creep_request = request;
 output.proposals = generate(
 creep_request, planning_map, predicted_frames,
 request.required_clearance_m, request.committed_progress_floor_mps,
 terrain_feasible, 0.0, true);
 output.selection = validate(
 output.proposals, creep_request, planning_map, predicted_frames,
 request.required_clearance_m, terrain_feasible, terrain_sample);
 }

 // The body can still be outside an inflated obstacle cell while every
 // reachable committed rollout enters that soft band. A crossing commitment
 // must then use the same physical-map contingency as an already-incurred
 // margin: the route, dynamic forecast, terrain and stability validators stay
 // authoritative, while soft inflation may not strand the body in traffic.
 if (!output.selection.validation.accepted() && request.crossing_committed &&
 physical_collision_map) {
 const auto& feasibility = physical_terrain_feasible
 ? physical_terrain_feasible
 : terrain_feasible;
 auto physical_proposals = generate(
 request, *physical_collision_map, predicted_frames,
 request.required_clearance_m, request.committed_progress_floor_mps,
 feasibility);
 auto physical_selection = select_static_margin_escape(
 physical_proposals, request, planning_map, *physical_collision_map,
 predicted_frames, request.required_clearance_m, feasibility, true,
 terrain_sample);
 if (physical_selection.validation.accepted()) {
 output.proposals = std::move(physical_proposals);
 output.selection = std::move(physical_selection);
 }
 }

 // A constant-command local planner can reach a genuine local minimum next
 // to a boundary. With an offset differential axle, the shortest turn toward
 // the route may sweep base_link farther into inflation before it moves away;
 // every translating rollout is then rejected while a zero command remains
 // valid forever. Detect that lack of both position and heading progress and
 // run one physically validated clearance-restoring step. This is independent
 // of mission semantics and is deliberately disabled around dynamic traffic,
 // behavior stop planes, and committed maneuvers.
 const bool near_boundary_context_allowed = physical_collision_map &&
 physical_terrain_feasible && !dynamic_forecast_present &&
 !request.stop_plane_active && !request.crossing_committed &&
 !request.committed_no_return &&
 !localization_hold &&
 request.command_space == LocalCommandSpace::DifferentialTwist &&
 planning_map.isFree(request.position.x, request.position.y);
 // A differential base with an offset drive axle moves base_link along an
 // arc during a nominal in-place turn. Leaving recovery with only a raster
 // epsilon can therefore make the next heading correction immediately sweep
 // back into inflation. Reserve the axle offset plus half a cell on entry and
 // another quarter-cell of hysteresis before returning authority to tracking.
 const double near_boundary_entry_clearance_m =
 std::abs(request.drive_axle_x_m) + 0.50 * planning_map.res();
 // A release at only one axle offset puts the next in-place alignment
 // back into the band it just escaped. Keep recovery latched until a
 // half-turn sweep around the offset axle still leaves the entry reserve.
 const double near_boundary_release_clearance_m =
 near_boundary_entry_clearance_m +
 2.0 * std::abs(request.drive_axle_x_m) + 0.25 * planning_map.res();
 if (!near_boundary_context_allowed ||
 planning_map.clearance(request.position.x, request.position.y) + 1e-9 >=
 near_boundary_release_clearance_m) {
 near_boundary_clearance_recovery_active_ = false;
 }
 if (near_boundary_context_allowed &&
 std::hypot(request.target.x - request.position.x,
 request.target.y - request.position.y) > planning_map.res()) {
 const auto& selected = output.selection.trajectory;
 const auto& terminal = selected.samples.empty()
 ? TrajectorySample{0.0, request.position, request.heading_rad}
 : selected.samples.back();
 const double initial_target_distance_m = std::hypot(
 request.target.x - request.position.x,
 request.target.y - request.position.y);
 const double terminal_target_distance_m = std::hypot(
 request.target.x - terminal.position.x,
 request.target.y - terminal.position.y);
 const double target_heading_rad = request.target_heading_rad.value_or(
 std::atan2(request.target.y - request.position.y,
 request.target.x - request.position.x));
 const double initial_heading_error_rad = std::abs(wrap_angle(
 target_heading_rad - request.heading_rad));
 const double terminal_heading_error_rad = std::abs(wrap_angle(
 target_heading_rad - terminal.heading_rad));
 const bool ordinary_stalled =
 !output.selection.validation.accepted() ||
 (initial_target_distance_m - terminal_target_distance_m < 0.01 &&
 initial_heading_error_rad - terminal_heading_error_rad < 0.02);
 const bool no_collision_free_translation = std::none_of(
 output.proposals.rollouts.begin(), output.proposals.rollouts.end(),
 [](const auto& rollout) {
 return !rollout.collision && rollout.trajectory.command.valid &&
 std::hypot(rollout.trajectory.command.v,
 rollout.trajectory.command.lateral_v) > 1e-6;
 });
 if (!near_boundary_clearance_recovery_active_ &&
 planning_map.clearance(request.position.x, request.position.y) + 1e-9 <
 near_boundary_entry_clearance_m &&
 (ordinary_stalled || no_collision_free_translation)) {
 near_boundary_clearance_recovery_active_ = true;
 }
 if (near_boundary_clearance_recovery_active_) {
 auto clearance_request = request;
 // Ordinary route tracking may intentionally expose max_speed=0 while it
 // waits for an in-place alignment. That zero is the local minimum we are
 // escaping, not a behavior safety wall (those contexts were excluded
 // above). Use the independently bounded recovery capability so a reverse
 // translation can move an offset-axle body away from the boundary.
 clearance_request.max_speed_mps = std::min(
 clearance_request.recovery_max_speed_mps,
 clearance_request.collision_escape_max_speed_mps);
 clearance_request.max_yaw_rate_radps = std::min(
 clearance_request.max_yaw_rate_radps,
 clearance_request.recovery_max_yaw_rate_radps);
 const auto refuge = nearest_free_space(
 planning_map, request.position, request.target, 1);
 if (std::isfinite(refuge.distance_m) && refuge.distance_m > 1e-6) {
 clearance_request.target = refuge.point;
 clearance_request.target_heading_rad.reset();
 }
 auto recovery_proposals = generate(
 clearance_request, *physical_collision_map, predicted_frames,
 clearance_request.required_clearance_m, 0.0,
 physical_terrain_feasible);
 // A local clearance gradient may lie behind the chassis. Recovery owns an
 // explicit, bounded gear choice rather than asking the forward route tracker
 // to invent a reverse command. Both sets pass the same swept validators.
 auto reverse_request = clearance_request;
 reverse_request.route_is_reverse = !clearance_request.route_is_reverse;
 auto reverse_proposals = generate(
 reverse_request, *physical_collision_map, predicted_frames,
 reverse_request.required_clearance_m, 0.0,
 physical_terrain_feasible);
 recovery_proposals.rollouts.insert(
 recovery_proposals.rollouts.end(),
 reverse_proposals.rollouts.begin(), reverse_proposals.rollouts.end());
 auto recovery = select_near_boundary_clearance_escape(
 recovery_proposals, clearance_request, planning_map, *physical_collision_map,
 predicted_frames, clearance_request.required_clearance_m,
 // Once latched, actuator response may make one rollout merely preserve the
 // newly gained clearance while braking or reversing yaw. Keep that verified
 // non-worsening step inside the recovery mode instead of alternating back to
 // an ordinary command that immediately consumes the margin again.
 physical_terrain_feasible, terrain_sample, -0.001);
 if (recovery.validation.accepted()) {
 output.proposals = recovery_proposals;
 output.selection = std::move(recovery);
 }
 }
 }

 const auto stationary_clearance = minimum_stationary_clearance(
 request.position, predicted_frames, request.step_s,
 request.robot_radius_m);
 // Waiting is a trajectory. A behavior stop cap cannot remain authoritative
 // when its own time-indexed occupancy is unsafe. Search a bounded escape
 // only after ordinary planning failed, and keep every downstream validator
 // active. This applies with or without mapped crossing semantics.
 const bool dynamic_collision_escape_required = dynamic_forecast_present &&
 stationary_clearance.conservative_m + 1e-9 <
 request.required_clearance_m;
 if (!localization_hold && !output.selection.validation.accepted() &&
 request.command_space == LocalCommandSpace::DifferentialTwist &&
 !request.route_is_reverse &&
 ((!request.crossing_committed && !request.committed_no_return) ||
 dynamic_collision_escape_required)) {
 auto recovery_request = request;
 recovery_request.route_is_reverse = true;
 if (dynamic_collision_escape_required) {
 // The behavior commitment remains active in its owner. For this one-cycle
 // contingency, remove the forward-only corridor/progress constraint so an
 // independently validated reverse motion can escape an actor that would
 // otherwise strike the stopped body. Static-only margin failures cannot
 // enter this branch because a current dynamic forecast is mandatory. An
 // uncommitted behavior stop plane remains active: risk reduction cannot
 // ratchet the body forward into the conflict it was waiting to enter.
 recovery_request.crossing_committed = false;
 recovery_request.committed_no_return = false;
 recovery_request.committed_progress_floor_mps = 0.0;
 recovery_request.dynamic_reverse_escape = true;
 recovery_request.dynamic_reverse_speed_floor_mps = std::min(
 0.05, std::max(0.0, recovery_request.max_speed_mps));
 recovery_request.committed_reference_path.clear();
 recovery_request.committed_tracking_corridor_m = 0.0;
 recovery_request.max_speed_mps = std::max(
 recovery_request.max_speed_mps,
 recovery_request.collision_escape_max_speed_mps);
 }
 const auto& recovery_map = dynamic_collision_escape_required && physical_collision_map
 ? *physical_collision_map
 : planning_map;
 const auto& recovery_feasibility =
 dynamic_collision_escape_required && physical_terrain_feasible
 ? physical_terrain_feasible
 : terrain_feasible;
 auto recovery = generate(
 recovery_request, recovery_map, predicted_frames,
 request.required_clearance_m, 0.0, recovery_feasibility);
 auto selection = validate(
 recovery, recovery_request, recovery_map, predicted_frames,
 request.required_clearance_m, recovery_feasibility, terrain_sample);
 if (selection.validation.minimum_risk_escape()) {
 output.proposals = std::move(recovery);
 output.selection = std::move(selection);
 }
 }
 // If every ordinary and forward-committed candidate fails while an actor is
 // predicted to strike the stopped body, compare physically feasible forward
 // and reverse motions directly against brake-and-hold. This is a last-resort
 // collision escape: it may enter the soft clearance margin, but never a
 // physical overlap, static obstacle, infeasible terrain, or stability limit.
 if (!localization_hold && !output.selection.validation.accepted() &&
 dynamic_collision_escape_required &&
 physical_collision_map) {
 auto escape_request = request;
 escape_request.crossing_committed = false;
 escape_request.committed_no_return = false;
 escape_request.committed_progress_floor_mps = 0.0;
 escape_request.committed_reference_path.clear();
 escape_request.committed_tracking_corridor_m = 0.0;
 escape_request.max_speed_mps = std::max(
 escape_request.max_speed_mps,
 escape_request.collision_escape_max_speed_mps);
 const auto& feasibility = physical_terrain_feasible
 ? physical_terrain_feasible
 : terrain_feasible;
 auto escape_proposals = generate(
 escape_request, *physical_collision_map, predicted_frames,
 request.required_clearance_m, 0.0, feasibility);
 auto reverse_request = escape_request;
 reverse_request.route_is_reverse = true;
 auto reverse_proposals = generate(
 reverse_request, *physical_collision_map, predicted_frames,
 request.required_clearance_m, 0.0, feasibility);
 escape_proposals.rollouts.insert(
 escape_proposals.rollouts.end(),
 reverse_proposals.rollouts.begin(), reverse_proposals.rollouts.end());

 ValidatedTrajectorySelection escape_selection;
 double best_gain_m = 0.02;
 double best_target_distance_m = std::numeric_limits<double>::infinity();
 const double clearance_equivalence_m = 0.5 * planning_map.res();
 const bool started_inside_preferred_margin =
 !planning_map.isFree(request.position.x, request.position.y);
 for (const auto& rollout : escape_proposals.rollouts) {
 if (rollout.collision || !rollout.trajectory.command.valid ||
 rollout.trajectory.samples.size() < 2)
 continue;
 const auto validation = validate(
 DwaResult{rollout.trajectory, {rollout}}, escape_request,
 *physical_collision_map, predicted_frames,
 request.required_clearance_m, feasibility, terrain_sample).validation;
 if (validation.verdict == TrajectoryVerdict::StaticCollision ||
 validation.verdict == TrajectoryVerdict::DynamicStabilityViolation ||
 validation.verdict == TrajectoryVerdict::BehaviorConstraint ||
 validation.verdict == TrajectoryVerdict::InvalidStructure)
 continue;
 // The physical-footprint map is a recovery tool for a body that is already
 // trapped inside conservative inflation. It is not permission for a dynamic
 // escape to abandon known static clearance or drive into a map boundary.
 // Starting from ordinary free space, retain that stronger invariant for the
 // complete contingency rollout. If the body already violates the preferred
 // margin, the dedicated recovery path above remains responsible for reducing
 // and eventually exiting that incursion.
 if (!started_inside_preferred_margin &&
 std::any_of(rollout.trajectory.samples.begin(),
 rollout.trajectory.samples.end(), [&](const auto& sample) {
 return !planning_map.isFree(sample.position.x, sample.position.y);
 })) {
 continue;
 }
 double motion_conservative_m = std::numeric_limits<double>::infinity();
 double motion_physical_m = std::numeric_limits<double>::infinity();
 double hold_conservative_m = std::numeric_limits<double>::infinity();
 for (const auto& sample : rollout.trajectory.samples) {
 const auto motion = dynamic_clearance_at(
 sample.position, sample.t_s, predicted_frames, request.step_s,
 request.robot_radius_m);
 const auto hold = dynamic_clearance_at(
 request.position, sample.t_s, predicted_frames, request.step_s,
 request.robot_radius_m);
 motion_conservative_m = std::min(
 motion_conservative_m, motion.conservative_m);
 motion_physical_m = std::min(motion_physical_m, motion.physical_m);
 hold_conservative_m = std::min(
 hold_conservative_m, hold.conservative_m);
 }
 const double displacement_m = std::hypot(
 rollout.trajectory.samples.back().position.x -
 rollout.trajectory.samples.front().position.x,
 rollout.trajectory.samples.back().position.y -
 rollout.trajectory.samples.front().position.y);
 const double gain_m = motion_conservative_m - hold_conservative_m;
 if (motion_physical_m < -1e-9 || displacement_m < 0.05 ||
 gain_m <= 0.02 + 1e-9)
 continue;
 const auto& terminal = rollout.trajectory.samples.back().position;
 const double target_distance_m = std::hypot(
 terminal.x - request.target.x, terminal.y - request.target.y);
 const bool meaningfully_safer =
 gain_m > best_gain_m + clearance_equivalence_m;
 const bool equivalent_risk_better_recovery =
 std::abs(gain_m - best_gain_m) <= clearance_equivalence_m + 1e-9 &&
 (target_distance_m < best_target_distance_m - 1e-9 ||
 (std::abs(target_distance_m - best_target_distance_m) <= 1e-9 &&
 dwa_command_preferred(
 rollout.trajectory.command,
 escape_selection.trajectory.command)));
 // Predicted clearance is discretized by the same map/control resolution.
 // Differences below half a planning cell are not a sound reason to abandon
 // the admitted refuge direction; use target recovery and then the stable DWA
 // order inside that numerical/representation equivalence band.
 if (!meaningfully_safer && !equivalent_risk_better_recovery)
 continue;
 best_gain_m = gain_m;
 best_target_distance_m = target_distance_m;
 escape_selection.trajectory = rollout.trajectory;
 escape_selection.validation = validation;
 escape_selection.validation.verdict = TrajectoryVerdict::MinimumRiskEscape;
 escape_selection.validation.min_dynamic_clearance_m = motion_conservative_m;
 escape_selection.validation.escape_clearance_gain_m = gain_m;
 escape_selection.validation.diagnostic =
 "motion reduces predicted collision risk relative to brake-and-hold";
 ++escape_selection.accepted_candidates;
 }
 if (escape_selection.validation.accepted()) {
 output.proposals = std::move(escape_proposals);
 output.selection = std::move(escape_selection);
 }
 }
 return with_recovery_telemetry(std::move(output));
}

} // namespace sim
