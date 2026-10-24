#include "robot/autonomy/decision/goal_arrival.hpp"

#include <algorithm>
#include <cmath>

#include "robot/core/geometry.hpp"

namespace lost_signal::robot::autonomy {

GoalArrivalPolicy::GoalArrivalPolicy(GoalArrivalConfig config)
 : config_(config) {}

void GoalArrivalPolicy::reset() { settled_cycles_ = 0; }

GoalArrivalDecision GoalArrivalPolicy::evaluate(
 const EstimatedState& estimate, const RoutePoint& goal,
 bool route_active) {
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
 const bool pose_reached =
 out.position_error_m <= std::max(0.0, config_.position_tolerance_m) &&
 out.heading_error_rad <= std::max(0.0, config_.heading_tolerance_rad);
 if (!pose_reached) {
 reset();
 out.reason = "approaching";
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
