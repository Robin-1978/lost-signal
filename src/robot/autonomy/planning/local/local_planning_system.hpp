#pragma once

#include "robot/autonomy/planning/global/global_planner.hpp"
#include "robot/autonomy/planning/local/dwa.hpp"
#include "robot/autonomy/planning/validation/trajectory_validator.hpp"
#include "robot/config/motion_policy.hpp"

#include <limits>
#include <optional>

namespace sim {

struct LocalPlanningRequest {
 double time_s{0.0};
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
 // Nominal actuator reachability. These limits shape ordinary candidate
 // commands; they do not carry the stability/slope safety policy used by the
 // independently validated emergency-stop tail below.
 double max_accel_mps2{0.0};
 double max_decel_mps2{0.0};
 double max_yaw_accel_radps2{0.0};
 double prediction_horizon_s{0.0};
 double step_s{0.1};
 double required_clearance_m{0.0};
 double robot_radius_m{0.0};
 // Raw LiDAR returns expressed in the current base frame. These are a
 // localization-independent guard for candidate trajectories; they remain
 // separate from the map raster because a drifting pose may project the same
 // return into the wrong world cell.
 std::vector<Vec2> live_obstacle_points_in_base;
 double live_obstacle_margin_m{0.05};
 // Differential-drive actuator feasibility. A body (v,w) candidate is valid
 // only when both v +/- w*track/2 remain within the selected motor speed.
 double differential_track_m{0.0};
 // Longitudinal offset from base_link to the differential axle. A yawing
 // base_link has lateral velocity -offset*yaw_rate.
 double drive_axle_x_m{0.0};
 double max_wheel_speed_mps{0.0};
 // Physical encoder feedback for wheel-space reachability. This remains
 // separate from body twist because skid-steer contact permits real slip.
 std::optional<DifferentialWheelSpeeds> measured_wheel_speeds;
 double steering_wheel_base_m{0.0};
 double max_steer_rad{0.0};
 double max_steer_rate_radps{0.0};
 double steering_response_s{0.0};
 TravelDirection preferred_direction{TravelDirection::Unspecified};
 bool route_is_reverse{false};
 bool crossing_relevant{false};
 bool crossing_committed{false};
 // Latches once a committed maneuver crosses its no-return line. It remains
 // true through the proved exit, so a late rejection cannot turn into a stop
 // or reverse inside the traffic corridor.
 bool committed_no_return{false};
 bool dynamic_reverse_escape{false};
 double dynamic_reverse_speed_floor_mps{0.0};
 bool stop_plane_active{false};
 Vec2 stop_point{0, 0};
 Vec2 stop_direction{1, 0};
 double stop_tolerance_m{0.0};
 double committed_progress_floor_mps{0.0};
 // Behavior may lock a previously validated spatial execution tube when it
 // commits to a conflict-zone transit. This is separate from the soft DWA
 // path critic: leaving the tube would invalidate the admission proof.
 std::vector<Vec2> committed_reference_path;
 double committed_tracking_corridor_m{0.0};
 std::vector<TimedPose> committed_reference_trajectory;
 // A required brake contract remains active even when current conditions
 // reduce safe braking authority to zero.
 bool emergency_stop_required{false};
 double emergency_stop_delay_s{0.0};
 double emergency_decel_mps2{0.0};
 SafeStopDecelQuery safe_stop_decel;
 // Closed-loop delay used inside nominal differential rollout simulation.
 // This is actuator response only; emergency_stop_delay_s additionally
 // includes the next decision interval.
 double actuator_response_delay_s{0.0};
 // Preferred-margin recovery is deliberately a low-energy maneuver. These
 // caps bound contact risk while the body is closer to static geometry than
 // the nominal planning envelope permits.
 double recovery_max_speed_mps{0.15};
 double recovery_max_yaw_rate_radps{0.8};
 // A behavior hold may reduce the ordinary speed cap to zero. If prediction
 // shows that remaining stationary is itself unsafe, the independent safety
 // planner may use this bounded platform speed to search for an escape. The
 // resulting motion still has to pass static, terrain, stability, actuator
 // and same-time dynamic validation.
 double collision_escape_max_speed_mps{0.15};
 // Geometry and margin for the shared candidate-level rollover validator.
 // The terrain observation itself is supplied to plan() by the map provider.
 double support_width_m{0.0};
 std::vector<Vec2> support_polygon;
 double center_of_mass_x_m{0.0};
 double center_of_mass_height_m{0.0};
 double max_dynamic_rollover_risk{0.0};
 DwaReference reference;
 // Localization quality is a constraint on this planner. The executive does
 // not provide an absolute speed or brake torque: this chassis-aware planner
 // consumes the localization health it already receives, converts it into its
 // ordinary drive and braking windows, and reports the resulting motion mode.
 // Safety validation remains independent for every resulting trajectory.
 double localization_confidence{1.0};
 bool localization_valid{true};
 double localization_age_s{0.0};
};

enum class LocalMotionMode {
 Nominal,
 LocalizationReduced,
 LocalizationHold,
 MinimumRiskEscape,
 NoSafeAction,
};

inline const char* local_motion_mode_name(LocalMotionMode mode) {
 switch (mode) {
 case LocalMotionMode::Nominal: return "nominal";
 case LocalMotionMode::LocalizationReduced: return "localization_reduced";
 case LocalMotionMode::LocalizationHold: return "localization_hold";
 case LocalMotionMode::MinimumRiskEscape: return "minimum_risk_escape";
 case LocalMotionMode::NoSafeAction: return "no_safe_action";
 }
 return "unknown";
}

// Deployable motion-capability report for the executive and operator UI. It
// describes what the planner proved for one decision cycle; it is not a wheel
// command and does not transfer speed authority out of planning/control.
struct LocalPlanningStatus {
 LocalMotionMode mode{LocalMotionMode::Nominal};
 bool motion_authorized{false};
 double decision_time_s{0.0};
 double speed_cap_mps{0.0};
 std::string reason;
 std::string recovery_condition;
};

struct LocalPlanningOutput {
 DwaResult proposals;
 ValidatedTrajectorySelection selection;
 LocalPlanningStatus status;
 bool static_margin_recovery_active{false};
 int static_margin_recovery_gear{0};
 Vec2 static_margin_escape_origin;
 double static_margin_escape_heading_rad{0.0};
 double static_margin_escape_required_progress_m{0.0};
 double static_margin_escape_progress_m{0.0};
 bool near_boundary_clearance_recovery_active{false};
};

 // Generates and validates one executable local trajectory. Behavior supplies
 // constraints (speed cap, direction and crossing commitment); this system
 // alone dispatches chassis-feasible rollout algorithms and applies the shared
 // continuous static/dynamic validator before publishing a command. Commitment
 // may require forward progress, but it never weakens static or dynamic safety.
class LocalPlanningSystem {
 public:
	// The motion policy carries the deployed localization-to-speed envelope.
	// Composition roots load it from robot_data; tests keep the baseline.
	explicit LocalPlanningSystem(MotionPolicy motion_policy = {}) : motion_policy_(motion_policy) {}

	void set_motion_policy(const MotionPolicy& policy) { motion_policy_ = policy; }
	const MotionPolicy& motion_policy() const { return motion_policy_; }

 void reset() {
 static_margin_blocked_s_ = 0.0;
 static_margin_best_depth_m_ = std::numeric_limits<double>::infinity();
 physical_margin_recovery_active_ = false;
 near_boundary_clearance_recovery_active_ = false;
 static_margin_recovery_gear_ = 0;
 static_margin_escape_origin_ = {};
 static_margin_escape_heading_rad_ = 0.0;
 static_margin_escape_required_progress_m_ = 0.0;
 }
 LocalPlanningOutput plan(
 const LocalPlanningRequest& request,
 const GridMap& planning_map,
 const std::vector<std::vector<DynamicObjectState>>& predicted_frames,
 StaticFeasibility terrain_feasible = {},
 const GridMap* physical_collision_map = nullptr,
 StaticFeasibility physical_terrain_feasible = {},
 TerrainSampleQuery terrain_sample = {});

 private:
	MotionPolicy motion_policy_;

 double static_margin_blocked_s_{0.0};
 double static_margin_best_depth_m_{std::numeric_limits<double>::infinity()};
 bool physical_margin_recovery_active_{false};
 bool near_boundary_clearance_recovery_active_{false};
 // Stable gear selected on entry to a preferred static margin: +1 forward,
 // -1 reverse, 0 outside recovery. It is held until measured exit so
 // receding-horizon alignment cannot alternate forward and reverse modes.
 int static_margin_recovery_gear_{0};
 Vec2 static_margin_escape_origin_;
 double static_margin_escape_heading_rad_{0.0};
 double static_margin_escape_required_progress_m_{0.0};
};

} // namespace sim
