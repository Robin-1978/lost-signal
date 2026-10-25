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
 if (reset_local_plan) local_planner.reset();
 const bool route_state_changed = route_.advance(
 global_planner, estimate, goal, has_goal,
 map, replan, active_route,
 2.0 * std::max(0.0, limits.footprint_radius_m));
 if (route_state_changed) local_planner.reset();
 MotionDecisionResult result = motion_.run(
 estimate, perception, local_map, active_route, limits, behavior_limits,
 observation, survey, now_ns, behavior, local_planner, validator,
 controller, forward_blocked, observation_motion_requested);
 local_replan_requested_ = result.route_replan_requested;
 if (!active_route.valid && route_.pending()) {
 result.constraint = "route_hold";
 result.reason = "global-plan-pending";
 } else if (!active_route.valid && route_.failed()) {
 result.constraint = "route_hold";
 result.reason = "global-plan-failed";
 }
 return result;
}

} // namespace lost_signal::robot::autonomy
