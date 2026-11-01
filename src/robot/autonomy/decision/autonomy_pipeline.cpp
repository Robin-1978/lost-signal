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
 // Route departure is measured from the route centerline. The generic
 // pipeline contract remains the declared chassis radius; Ackermann gets a
 // separate raster-tracking allowance below, without changing collision
 // inflation.
 // The global route is a centerline reference, not the lethal footprint
 // boundary.  Ackermann tracking can legitimately sit a raster cell or two
 // off that centerline while a bounded steering arc settles; using only the
 // footprint radius here starts a replacement plan during a valid cusp and
 // resets the gear cursor.  Keep the generic route-pipeline contract at the
 // chassis radius, but give car-like tracking the same map-quantization
 // allowance used by its local route tube.  Collision clearance remains
 // owned by the costmap, Validator, and Safety Monitor.
 const double route_deviation_tolerance_m =
     platform::chassis_is_ackermann(limits)
         ? std::max(0.0, limits.footprint_radius_m) +
               2.0 * std::max(0.0, local_map.resolution_m)
         : std::max(0.0, limits.footprint_radius_m);
 const bool route_state_changed = route_.advance(
 global_planner, estimate, goal, has_goal,
 map, replan, active_route,
 route_deviation_tolerance_m);
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
