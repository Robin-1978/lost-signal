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
 const double heading_time = std::abs(wrap_angle(target_heading - request.heading_rad)) / yaw_rate;
 const double braking_time = std::abs(request.forward_mps) /
 std::max(1e-6, request.max_accel_mps2);
 const double stopping_horizon = request.emergency_stop_delay_s + braking_time;
 // A rollout represents one receding-horizon action toward the current local
 // target. Extending a constant yaw command well past that target makes a
 // valid turn look like a multi-revolution loop and causes stop/turn limit
 // cycles. Prediction may cover farther into the future; executable rollout
 // time is the kinematic time needed for this local target and safe braking.
 return std::min(request.prediction_horizon_s,
 std::max({2.0 * request.step_s, distance_time, heading_time, stopping_horizon}));
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
  const double minimum_velocity_mps = request.route_is_reverse ? -0.5 : 0.0;
 return dwa_compute_trajectory(
 request.position, request.heading_rad, request.forward_mps,
 request.yaw_rate_radps, request.target, planning_map, request.weights,
 predicted_frames, request.max_speed_mps, request.max_yaw_rate_radps,
 request.max_accel_mps2, predict_time_s, request.step_s,
 request.step_s, required_clearance_m, request.robot_radius_m,
 request.max_yaw_accel_radps2, terrain_feasible,
 minimum_forward_mps > 0.0 ? minimum_forward_mps : minimum_velocity_mps,
 request.target_heading_rad, request.reference,
 request.differential_track_m, request.max_wheel_speed_mps);
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
 double required_clearance_m, const StaticFeasibility& terrain_feasible) {
 ValidatedTrajectorySelection output;
 double best_score = -std::numeric_limits<double>::infinity();
 TrajectoryConstraints constraints;
 constraints.stop_plane_active = request.stop_plane_active;
 constraints.stop_point = request.stop_point;
 constraints.stop_direction = request.stop_direction;
 constraints.stop_tolerance_m = request.stop_tolerance_m;
 constraints.emergency_stop_delay_s = request.emergency_stop_delay_s;
 constraints.emergency_decel_mps2 = request.emergency_decel_mps2;
 constraints.committed_transit = request.crossing_committed;
 constraints.committed_progress_floor_mps = request.committed_progress_floor_mps;
 for (const auto& rollout : proposals.rollouts) {
 if (rollout.collision || !rollout.trajectory.command.valid ||
 rollout.trajectory.samples.empty()) {
 continue;
 }
 // A margin escape exists only to return the robot centre to the preferred
 // static envelope. It may cross the soft inflation already occupied at t=0,
 // but it must remain outside physical collision occupancy for the complete
 // swept trajectory and end in preferred free space.
 const auto& terminal = rollout.trajectory.samples.back().position;
 if (!preferred_map.isFree(terminal.x, terminal.y))
 continue;
 auto validation = validate_executable_trajectory(
 rollout.trajectory, physical_collision_map, predicted_frames,
 request.step_s, request.robot_radius_m, required_clearance_m,
 terrain_feasible, constraints);
 if (!validation.accepted())
 continue;
 ++output.accepted_candidates;
 if (rollout.trajectory.command.score <= best_score)
 continue;
 validation.verdict = TrajectoryVerdict::MinimumRiskEscape;
 validation.diagnostic =
 "trajectory exits an existing preferred static-margin incursion";
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
 const GridMap* physical_collision_map) const {
 LocalPlanningOutput output;
 const bool inside_preferred_margin =
 !planning_map.isFree(request.position.x, request.position.y);
 const bool physical_escape_available = physical_collision_map &&
 physical_collision_map->isFree(request.position.x, request.position.y);
 if (inside_preferred_margin && physical_escape_available) {
 output.proposals = generate(
 request, *physical_collision_map, predicted_frames,
 request.required_clearance_m, request.committed_progress_floor_mps,
 terrain_feasible, 1.0);
 output.selection = select_static_margin_escape(
 output.proposals, request, planning_map, *physical_collision_map,
 predicted_frames, request.required_clearance_m, terrain_feasible);
 // A skid-steer or differential chassis can enter the soft planning margin
 // while its nose points toward the obstacle. If no forward rollout exits
 // that margin, make one deliberate reverse search through the same physical
 // map and continuous validator. Waiting cannot improve a static geometry
 // trap, and the reverse command is still bounded by wheel speed and swept
 // collision checks.
 if (!output.selection.validation.accepted() &&
 request.command_space == LocalCommandSpace::DifferentialTwist &&
 !request.route_is_reverse && !request.crossing_relevant) {
 auto recovery_request = request;
 recovery_request.route_is_reverse = true;
 auto recovery = generate(
 recovery_request, *physical_collision_map, predicted_frames,
 request.required_clearance_m, 0.0, terrain_feasible, 1.0);
 auto selection = select_static_margin_escape(
 recovery, recovery_request, planning_map, *physical_collision_map,
 predicted_frames, request.required_clearance_m, terrain_feasible);
 if (selection.validation.accepted()) {
 output.proposals = std::move(recovery);
 output.selection = std::move(selection);
 }
 }
 return output;
 }
 output.proposals = generate(
 request, planning_map, predicted_frames, request.required_clearance_m,
 request.committed_progress_floor_mps, terrain_feasible);
 output.selection = validate(output.proposals, request, planning_map,
 predicted_frames, request.required_clearance_m, terrain_feasible);

 if (!output.selection.validation.accepted() &&
 request.command_space == LocalCommandSpace::DifferentialTwist &&
 !request.route_is_reverse && !request.crossing_relevant) {
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
