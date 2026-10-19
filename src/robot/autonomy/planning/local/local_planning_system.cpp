#include "robot/autonomy/planning/local/local_planning_system.hpp"

#include "robot/autonomy/planning/local/ackermann_local.hpp"
#include "robot/autonomy/planning/local/holonomic_local.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
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
 const double target_heading = request.target_heading_rad.value_or(std::atan2(
 request.target.y - request.position.y,
 request.target.x - request.position.x));
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
 std::max(1e-6, request.max_accel_mps2);
 const double angular_braking_time = std::abs(request.yaw_rate_radps) /
 std::max(1e-6, request.max_yaw_accel_radps2);
 const double stopping_horizon = request.emergency_stop_delay_s +
 std::max(braking_time, angular_braking_time);
 // A rollout represents one receding-horizon action toward the current local
 // target. Extending a constant yaw command well past that target makes a
 // valid turn look like a multi-revolution loop and causes stop/turn limit
 // cycles. Prediction may cover farther into the future; executable rollout
 // time is the kinematic time needed for this local target and safe braking.
 return std::min(request.prediction_horizon_s,
 std::max({2.0 * request.step_s, distance_time, heading_time, stopping_horizon}));
}

double distance_to_free_space(const GridMap& map, Vec2 point) {
 if (map.isFree(point.x, point.y))
 return 0.0;
 double best_m = std::numeric_limits<double>::infinity();
 for (int gy = 0; gy < map.h(); ++gy) {
 for (int gx = 0; gx < map.w(); ++gx) {
 if (!map.isFreeCell(gx, gy))
 continue;
 const Vec2 free_point = map.gridToWorld(gx, gy);
 best_m = std::min(best_m, std::hypot(
 point.x - free_point.x, point.y - free_point.y));
 }
 }
 return best_m;
}

DwaResult generate(const LocalPlanningRequest& request,
 const GridMap& planning_map,
 const std::vector<std::vector<DynamicObjectState>>& predicted_frames,
 double required_clearance_m, double minimum_forward_mps,
 const StaticFeasibility& terrain_feasible,
 double minimum_horizon_s = 0.0) {
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
 planning_map, request.weights, predicted_frames, config, terrain_feasible,
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
 predicted_frames, config, terrain_feasible);
 }

  // The global route owns travel direction. A local sampler must not switch
  // into reverse merely because a backward rollout scores better while the
  // body is temporarily pointed away from its lookahead target. That creates
  // long forward/reverse limit cycles and violates the RoutePoint direction
  // contract. Deliberate recovery is requested separately below and is still
  // subjected to the same swept trajectory validator.
 const double speed_limit_mps = std::max(0.0, request.max_speed_mps);
 const double minimum_velocity_mps = request.route_is_reverse ? -speed_limit_mps : 0.0;
 const double maximum_velocity_mps = request.route_is_reverse ? 0.0 : speed_limit_mps;
 return dwa_compute_trajectory(
 request.position, request.heading_rad, request.forward_mps,
 request.yaw_rate_radps, request.target, planning_map, request.weights,
 predicted_frames, maximum_velocity_mps, request.max_yaw_rate_radps,
 request.max_accel_mps2, predict_time_s, request.step_s,
 request.step_s, required_clearance_m, request.robot_radius_m,
 request.max_yaw_accel_radps2, terrain_feasible,
 minimum_forward_mps > 0.0 ? minimum_forward_mps : minimum_velocity_mps,
 request.target_heading_rad, request.reference,
 request.differential_track_m, request.max_wheel_speed_mps,
 request.actuator_response_delay_s);
}

ValidatedTrajectorySelection validate(const DwaResult& proposals,
 const LocalPlanningRequest& request, const GridMap& planning_map,
 const std::vector<std::vector<DynamicObjectState>>& predicted_frames,
 double required_clearance_m, const StaticFeasibility& terrain_feasible) {
 TrajectoryConstraints constraints;
 constraints.stop_plane_active = request.stop_plane_active;
 constraints.stop_point = request.stop_point;
 constraints.stop_direction = request.stop_direction;
 constraints.stop_tolerance_m = request.stop_tolerance_m;
 constraints.emergency_stop_delay_s = request.emergency_stop_delay_s;
 constraints.emergency_decel_mps2 = request.emergency_decel_mps2;
 constraints.differential_track_m = request.differential_track_m;
 constraints.committed_transit = request.crossing_committed;
 constraints.committed_progress_floor_mps = request.committed_progress_floor_mps;
 return select_executable_trajectory(
 proposals, planning_map, predicted_frames, request.step_s,
 request.robot_radius_m, required_clearance_m, terrain_feasible, constraints);
}

ValidatedTrajectorySelection select_static_margin_escape(
 const DwaResult& proposals, const LocalPlanningRequest& request,
 const GridMap& preferred_map, const GridMap& physical_collision_map,
 const std::vector<std::vector<DynamicObjectState>>& predicted_frames,
 double required_clearance_m, const StaticFeasibility& terrain_feasible,
 bool allow_incremental_escape) {
 ValidatedTrajectorySelection output;
 double best_score = -std::numeric_limits<double>::infinity();
 int best_rank = 0;
 const double initial_margin_depth_m = allow_incremental_escape
 ? distance_to_free_space(preferred_map, request.position)
 : std::numeric_limits<double>::infinity();
 TrajectoryConstraints constraints;
 constraints.stop_plane_active = request.stop_plane_active;
 constraints.stop_point = request.stop_point;
 constraints.stop_direction = request.stop_direction;
 constraints.stop_tolerance_m = request.stop_tolerance_m;
 constraints.emergency_stop_delay_s = request.emergency_stop_delay_s;
 constraints.emergency_decel_mps2 = request.emergency_decel_mps2;
 constraints.differential_track_m = request.differential_track_m;
 constraints.committed_transit = request.crossing_committed;
 constraints.committed_progress_floor_mps = request.committed_progress_floor_mps;
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
 const bool exits_preferred_margin = preferred_map.isFree(terminal.x, terminal.y);
 const auto& start = rollout.trajectory.samples.front();
 const auto& finish = rollout.trajectory.samples.back();
 const double displacement_m = std::hypot(
 finish.position.x - start.position.x,
 finish.position.y - start.position.y);
 const double escape_heading_rad = std::atan2(
 request.target.y - request.position.y,
 request.target.x - request.position.x);
 const double initial_heading_error = std::abs(
 std::remainder(escape_heading_rad - start.heading_rad, 2.0 * kPi));
 const double terminal_heading_error = std::abs(
 std::remainder(escape_heading_rad - finish.heading_rad, 2.0 * kPi));
 const double measured_translation_mps = std::hypot(
 request.forward_mps, request.lateral_mps);
 const double predicted_stop_translation_m =
 measured_translation_mps * std::max(0.0, request.emergency_stop_delay_s) +
 (request.emergency_decel_mps2 > 0.0
 ? measured_translation_mps * measured_translation_mps /
 (2.0 * request.emergency_decel_mps2)
 : (measured_translation_mps > 1e-6
 ? std::numeric_limits<double>::infinity() : 0.0));
 const bool stages_alignment =
 request.command_space == LocalCommandSpace::DifferentialTwist &&
 preferred_map.isFree(request.target.x, request.target.y) &&
 std::abs(rollout.trajectory.command.v) <= 1e-9 &&
 std::abs(rollout.trajectory.command.w) > 1e-6 &&
 // A zero target is not an in-place maneuver while the measured chassis is
 // still translating. Require its complete braking envelope to fit inside
 // the same displacement tolerance used to classify the rollout; otherwise
 // control must brake first and replan from the measured stopped state.
 predicted_stop_translation_m <= 0.02 &&
 displacement_m <= 0.02 &&
 terminal_heading_error + 0.05 < initial_heading_error;
 const bool reduces_margin_incursion = allow_incremental_escape &&
 !exits_preferred_margin && displacement_m >= 0.05 &&
 distance_to_free_space(preferred_map, terminal) + 0.05 <
 initial_margin_depth_m;
 if (!exits_preferred_margin && !reduces_margin_incursion && !stages_alignment)
 continue;
 auto validation = validate_executable_trajectory(
 rollout.trajectory, physical_collision_map, predicted_frames,
 request.step_s, request.robot_radius_m, required_clearance_m,
 terrain_feasible, constraints);
 if (!validation.accepted())
 continue;
 ++output.accepted_candidates;
 const int rank = exits_preferred_margin ? 3 : (reduces_margin_incursion ? 2 : 1);
 if (rank < best_rank ||
 (rank == best_rank && rollout.trajectory.command.score <= best_score))
 continue;
 validation.verdict = TrajectoryVerdict::MinimumRiskEscape;
 validation.diagnostic = exits_preferred_margin
 ? "trajectory exits an existing preferred static-margin incursion"
 : (reduces_margin_incursion
 ? "trajectory reduces an existing preferred static-margin incursion"
 : "in-place alignment stages a physically validated static-margin escape");
 best_rank = rank;
 best_score = rollout.trajectory.command.score;
 output.trajectory = rollout.trajectory;
 output.validation = std::move(validation);
 }
 return output;
}

} // namespace

LocalPlanningOutput LocalPlanningSystem::plan(
 const LocalPlanningRequest& request, const GridMap& planning_map,
 const std::vector<std::vector<DynamicObjectState>>& predicted_frames,
 StaticFeasibility terrain_feasible,
 const GridMap* physical_collision_map,
 StaticFeasibility physical_terrain_feasible) {
 LocalPlanningOutput output;
 const bool inside_preferred_margin =
 !planning_map.isFree(request.position.x, request.position.y);
 const bool physical_escape_available = physical_collision_map &&
 physical_collision_map->isFree(request.position.x, request.position.y);
 if (inside_preferred_margin && physical_escape_available) {
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
 auto escape_request = request;
 escape_request.max_speed_mps = std::min(
 escape_request.max_speed_mps, escape_request.recovery_max_speed_mps);
 escape_request.max_yaw_rate_radps = std::min(
 escape_request.max_yaw_rate_radps, escape_request.recovery_max_yaw_rate_radps);
 const auto attempt_escape = [&](const StaticFeasibility& feasibility,
 bool allow_incremental_escape) {
 output.proposals = generate(
 escape_request, *physical_collision_map, predicted_frames,
 escape_request.required_clearance_m, escape_request.committed_progress_floor_mps,
 feasibility, 1.0);
 output.selection = select_static_margin_escape(
 output.proposals, escape_request, planning_map, *physical_collision_map,
 predicted_frames, escape_request.required_clearance_m, feasibility,
 allow_incremental_escape);
 // A skid-steer or differential chassis can enter the soft planning margin
 // while its nose points toward the obstacle. If no forward rollout exits
 // that margin, make one deliberate reverse search through the same physical
 // map and continuous validator.
 if (!output.selection.validation.accepted() &&
 request.command_space == LocalCommandSpace::DifferentialTwist &&
 !request.route_is_reverse && !request.crossing_committed && !request.body_in_conflict) {
 auto recovery_request = escape_request;
 recovery_request.route_is_reverse = true;
 auto recovery = generate(
 recovery_request, *physical_collision_map, predicted_frames,
 request.required_clearance_m, 0.0, feasibility, 1.0);
 auto selection = select_static_margin_escape(
 recovery, recovery_request, planning_map, *physical_collision_map,
 predicted_frames, request.required_clearance_m, feasibility,
 allow_incremental_escape);
 if (selection.validation.accepted()) {
 output.proposals = std::move(recovery);
 output.selection = std::move(selection);
 }
 }
 };
 // Preserve the normal static-clearance contract first. If that envelope
 // already contains the body and rejects every recovery, retry against the
 // physical footprint map with terrain feasibility only. This fallback does
 // not weaken collision validation; it removes duplicate soft inflation.
 attempt_escape(terrain_feasible, false);
 const bool nominal_escape_accepted = output.selection.validation.accepted();
 LocalPlanningOutput nominal_output = output;
 constexpr double kPhysicalMarginFallbackDelayS = 10.0;
 if (nominal_escape_accepted && !physical_margin_recovery_active_ &&
 static_margin_blocked_s_ + 1e-9 < kPhysicalMarginFallbackDelayS) {
 return output;
 }
 double nearest_dynamic_clearance_m = std::numeric_limits<double>::infinity();
 for (const auto& frame : predicted_frames) {
 for (const auto& actor : frame) {
 nearest_dynamic_clearance_m = std::min(
 nearest_dynamic_clearance_m,
 std::hypot(request.position.x - actor.p.x, request.position.y - actor.p.y) -
 request.robot_radius_m - actor.occupied_radius_m());
 }
 }
 const bool dynamic_scene_is_remote =
 nearest_dynamic_clearance_m > request.required_clearance_m +
 escape_request.max_speed_mps * request.prediction_horizon_s;
 if (physical_terrain_feasible && dynamic_scene_is_remote &&
 !request.stop_plane_active &&
 (physical_margin_recovery_active_ ||
 static_margin_blocked_s_ + 1e-9 >= kPhysicalMarginFallbackDelayS)) {
 attempt_escape(physical_terrain_feasible, true);
 if (output.selection.validation.accepted()) {
 physical_margin_recovery_active_ = true;
 return output;
 }
 }
 if (nominal_escape_accepted)
 return nominal_output;
 return output;
 }
 static_margin_blocked_s_ = 0.0;
 static_margin_best_depth_m_ = std::numeric_limits<double>::infinity();
 physical_margin_recovery_active_ = false;
 output.proposals = generate(
 request, planning_map, predicted_frames, request.required_clearance_m,
 request.committed_progress_floor_mps, terrain_feasible);
 output.selection = validate(output.proposals, request, planning_map,
 predicted_frames, request.required_clearance_m, terrain_feasible);

 if (!output.selection.validation.accepted() &&
 request.command_space == LocalCommandSpace::DifferentialTwist &&
 !request.route_is_reverse && !request.crossing_committed && !request.body_in_conflict) {
 auto recovery_request = request;
 recovery_request.route_is_reverse = true;
 auto recovery = generate(
 recovery_request, planning_map, predicted_frames,
 request.required_clearance_m, 0.0, terrain_feasible);
 auto selection = validate(recovery, request, planning_map, predicted_frames,
 request.required_clearance_m, terrain_feasible);
 if (selection.validation.minimum_risk_escape()) {
 output.proposals = std::move(recovery);
 output.selection = std::move(selection);
 }
 }
 return output;
}

} // namespace sim
