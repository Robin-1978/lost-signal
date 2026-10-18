#pragma once

#include "robot/autonomy/planning/global/global_planner.hpp"
#include "robot/autonomy/planning/local/dwa.hpp"
#include "robot/autonomy/planning/validation/trajectory_validator.hpp"

#include <optional>

namespace sim {

struct LocalPlanningRequest {
 LocalCommandSpace command_space{LocalCommandSpace::DifferentialTwist};
 Vec2 position;
 double heading_rad{0.0};
 double forward_mps{0.0};
 double lateral_mps{0.0};
 double yaw_rate_radps{0.0};
 double steering_angle_rad{0.0};
 double steering_rate_radps{0.0};
 Vec2 target;
 // Optional terminal body heading for pose-constrained maneuvers such as a
 // crossing stop line. A point alone is singular at arrival and can make a
 // differential robot align with localization noise instead of the route.
 std::optional<double> target_heading_rad;
 // Optional final pose contract. `target` remains the receding-horizon
 // lookahead point; a final destination is separate so a holonomic planner
 // can decelerate and make signed, fine translations after passing the last
 // lookahead without treating every route waypoint as a docking pose.
 std::optional<Vec2> terminal_position;
 double terminal_position_tolerance_m{0.0};
 double terminal_max_speed_mps{0.0};
 double terminal_heading_tolerance_rad{0.12};
 double terminal_max_yaw_rate_radps{0.05};
 DwaWeights weights;
 double max_speed_mps{0.0};
 double max_yaw_rate_radps{0.0};
 double max_accel_mps2{0.0};
 double max_yaw_accel_radps2{0.0};
 double prediction_horizon_s{0.0};
 double step_s{0.1};
 double required_clearance_m{0.0};
 double robot_radius_m{0.0};
 // Differential-drive actuator feasibility. A body (v,w) candidate is valid
 // only when both v +/- w*track/2 remain within the selected motor speed.
 double differential_track_m{0.0};
 double max_wheel_speed_mps{0.0};
 double steering_wheel_base_m{0.0};
 double max_steer_rad{0.0};
 double max_steer_rate_radps{0.0};
 double steering_response_s{0.0};
 TravelDirection preferred_direction{TravelDirection::Unspecified};
 bool route_is_reverse{false};
 bool crossing_relevant{false};
 bool crossing_committed{false};
 bool body_in_conflict{false};
 bool stop_plane_active{false};
 Vec2 stop_point{0, 0};
 Vec2 stop_direction{1, 0};
 double stop_tolerance_m{0.0};
 double committed_progress_floor_mps{0.0};
 double emergency_stop_delay_s{0.0};
 double emergency_decel_mps2{0.0};
 // Closed-loop delay used inside nominal differential rollout simulation.
 // This is actuator response only; emergency_stop_delay_s additionally
 // includes the next decision interval.
 double actuator_response_delay_s{0.0};
 // Preferred-margin recovery is deliberately a low-energy maneuver. These
 // caps bound contact risk while the body is closer to static geometry than
 // the nominal planning envelope permits.
 double recovery_max_speed_mps{0.15};
 double recovery_max_yaw_rate_radps{0.8};
 DwaReference reference;
};

struct LocalPlanningOutput {
 DwaResult proposals;
 ValidatedTrajectorySelection selection;
};

 // Generates and validates one executable local trajectory. Behavior supplies
 // constraints (speed cap, direction and crossing commitment); this system
 // alone dispatches chassis-feasible rollout algorithms and applies the shared
 // continuous static/dynamic validator before publishing a command. Commitment
 // may require forward progress, but it never weakens static or dynamic safety.
class LocalPlanningSystem {
 public:
 void reset() {
 static_margin_blocked_s_ = 0.0;
 physical_margin_recovery_active_ = false;
 }
 LocalPlanningOutput plan(
 const LocalPlanningRequest& request,
 const GridMap& planning_map,
 const std::vector<std::vector<DynamicObjectState>>& predicted_frames,
 StaticFeasibility terrain_feasible = {},
 const GridMap* physical_collision_map = nullptr,
 StaticFeasibility physical_terrain_feasible = {});

 private:
 double static_margin_blocked_s_{0.0};
 bool physical_margin_recovery_active_{false};
};

} // namespace sim
