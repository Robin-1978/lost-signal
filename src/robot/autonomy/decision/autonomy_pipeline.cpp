#include "robot/autonomy/decision/autonomy_pipeline.hpp"

#include <algorithm>

#include "robot/autonomy/decision/motion_pipeline.hpp"
#include "robot/autonomy/planning/route_pipeline.hpp"

namespace lost_signal::robot::autonomy {

MotionDecisionResult DecisionCyclePipeline::run(
 IGlobalPlanner& global_planner, ILocalPlanner& local_planner,
 IBehaviorPolicy& behavior, IValidator& validator, IController& controller,
 const EstimatedState& estimate, const PerceptionResult& perception,
 const LocalMap& local_map, Route& active_route, const RoutePoint& goal,
 bool has_goal, const iface::MapSnapshot& map, bool force_route_refresh,
 bool reset_local_plan,
 const platform::ChassisLimits& limits,
 const platform::ChassisLimits& behavior_limits,
 const iface::ObservationFrame& observation, const SurveyContext& survey,
 core::MonotonicNs now_ns, bool forward_blocked,
 bool observation_motion_requested) {
 const bool replan = force_route_refresh || local_replan_requested_;
 local_replan_requested_ = false;
 if (reset_local_plan) local_planner.reset_tracking();
 // Route departure is measured from the route centerline. The admissible
 // cross-track radius is the declared chassis envelope; using its diameter
 // lets a car enter a wall-side corner before the route is refreshed.
 const bool route_state_changed = route_.advance(
 global_planner, estimate, goal, has_goal,
 map, replan, active_route,
  std::max(0.0, limits.footprint_radius_m));
 if (route_state_changed) local_planner.reset_tracking();
 // A failed global route is a ROBOT-side planning observation: the current
 // estimate may have landed inside an inflated/occupied raster cell after
 // drift, while the raw local scan still contains an executable escape. Feed
 // that fact into the same validated observation-recovery path instead of
 // converting it into a permanent stop. The fallback remains bounded by the
 // chassis capabilities and the independent Validator/Safety gate.
 const bool route_recovery_requested = route_.failed();
 MotionDecisionResult result = motion_.run(
 estimate, perception, local_map, active_route, limits, behavior_limits,
 observation, survey, now_ns, behavior, local_planner, validator,
 controller, forward_blocked,
 observation_motion_requested || route_recovery_requested);
 local_replan_requested_ = result.route_replan_requested;
 if (!active_route.valid && route_.pending()) {
 result.constraint = "route_hold";
 result.reason = "global-plan-pending";
 } else if (!active_route.valid && route_.failed() &&
            result.command.is_safe_stop()) {
 result.constraint = "route_hold";
 result.reason = "global-plan-failed";
 result.route_diagnostic = route_.diagnostic();
 } else if (!active_route.valid && route_.failed()) {
  // Preserve the validated recovery command and carry the planner failure as
  // structured diagnostics. A map-frame route failure must not suppress a
  // safe raw-scan manoeuvre that can expose new geometry.
  result.route_diagnostic = route_.diagnostic();
 }
 return result;
}

} // namespace lost_signal::robot::autonomy
