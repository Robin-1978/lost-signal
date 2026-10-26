#include "robot/autonomy/decision/goal_arrival.hpp"

#include <algorithm>
#include <cmath>

#include "robot/autonomy/decision/arrival_confidence.hpp"
#include "robot/core/geometry.hpp"

namespace lost_signal::robot::autonomy {

GoalArrivalPolicy::GoalArrivalPolicy(GoalArrivalConfig config)
 : config_(config) {}

void GoalArrivalPolicy::reset() { settled_cycles_ = 0; }

GoalArrivalDecision GoalArrivalPolicy::evaluate(
 const EstimatedState& estimate, const RoutePoint& goal,
 bool route_active, bool observation_motion_requested) {
 GoalArrivalDecision out;
 if (!route_active || !estimate.valid ||
 !std::isfinite(estimate.x_m) || !std::isfinite(estimate.y_m) ||
 !std::isfinite(estimate.yaw_rad) ||
 !std::isfinite(estimate.forward_mps) ||
 !std::isfinite(estimate.yaw_rate_radps) ||
 !std::isfinite(goal.x_m) || !std::isfinite(goal.y_m) ||
 !std::isfinite(goal.yaw_rad)) {
 reset();
 return out;
 }

 out.position_error_m =
 std::hypot(estimate.x_m - goal.x_m, estimate.y_m - goal.y_m);
 out.heading_error_rad =
 std::abs(core::wrap_angle(estimate.yaw_rad - goal.yaw_rad));
 out.position_confidence_radius_m = position_confidence_radius(
 estimate.position_sigma_m, config_.confidence_probability);
 out.heading_confidence_radius_rad = heading_confidence_radius(
 estimate.heading_sigma_rad, config_.confidence_probability);
 const double position_tolerance =
 std::max(0.0, config_.position_tolerance_m);
 const double heading_tolerance =
 std::max(0.0, config_.heading_tolerance_rad);
 const bool confidence_fits =
 out.position_confidence_radius_m <= position_tolerance &&
 out.heading_confidence_radius_rad <= heading_tolerance;
 if (!confidence_fits) {
 reset();
 // Reaching the raw geometric envelope with a confidence region wider than
 // that envelope is a request for more evidence, not permission to report
 // arrival or to freeze forever. The decision pipeline bounds and validates
 // the resulting observation motion through the normal planner, Validator,
 // and Safety Monitor. Outside the raw envelope, ordinary goal approach is
 // already the required motion.
 out.observation_motion_required =
 out.position_error_m <= position_tolerance &&
 out.heading_error_rad <= heading_tolerance;
 out.hold = false;
 out.reason = "localization-uncertain";
 return out;
 }
 const bool pose_reached =
 out.position_error_m + out.position_confidence_radius_m <=
 position_tolerance &&
 out.heading_error_rad + out.heading_confidence_radius_rad <=
 heading_tolerance;
 if (!pose_reached) {
 reset();
 out.reason = "approaching";
 return out;
 }

 const bool localization_resolved =
 std::isfinite(estimate.position_sigma_m) &&
 std::isfinite(estimate.heading_sigma_rad) &&
 estimate.position_sigma_m <=
 std::max(0.0, config_.maximum_position_sigma_m) &&
 estimate.heading_sigma_rad <=
 std::max(0.0, config_.maximum_heading_sigma_rad);
 if (!localization_resolved) {
 reset();
 // A covariance threshold is not motion authority. Hold the geometrically
 // reached pose unless localization has explicitly requested a supervised
 // observation manoeuvre; this prevents a local planner from wandering in
 // a featureless scene merely because confidence is low.
 out.hold = !observation_motion_requested;
 out.reason = "localization-uncertain";
 return out;
 }

 // Crossing the goal region at speed is not an arrival. Command a hold and
 // require consecutive settled measurements before reporting completion.
 out.hold = true;
 const bool stopped =
 std::abs(estimate.forward_mps) <=
 std::max(0.0, config_.maximum_forward_speed_mps) &&
 std::abs(estimate.yaw_rate_radps) <=
 std::max(0.0, config_.maximum_yaw_rate_radps);
 if (!stopped) {
 reset();
 out.reason = "settling";
 return out;
 }
 const int required = std::max(1, config_.settle_cycles);
 settled_cycles_ = std::min(required, settled_cycles_ + 1);
 out.arrived = settled_cycles_ >= required;
 out.reason = out.arrived ? "arrived" : "settling";
 return out;
}

} // namespace lost_signal::robot::autonomy
