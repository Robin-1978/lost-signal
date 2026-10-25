#include "robot/autonomy/decision/autonomy_pipeline.hpp"

#include "robot/autonomy/decision/motion_pipeline.hpp"
#include "robot/autonomy/planning/route_pipeline.hpp"

namespace lost_signal::robot::autonomy {

MotionDecisionResult DecisionCyclePipeline::run(
 IGlobalPlanner& global_planner, ILocalPlanner& local_planner,
 IBehaviorPolicy& behavior, IValidator& validator, IController& controller,
 const EstimatedState& estimate, const PerceptionResult& perception,
 const LocalMap& local_map, Route& active_route, const RoutePoint& goal,
 bool has_goal, const iface::MapSnapshot& map, bool force_route_refresh,
 const platform::ChassisLimits& limits,
 const platform::ChassisLimits& behavior_limits,
 const iface::ObservationFrame& observation, const SurveyContext& survey,
 core::MonotonicNs now_ns, bool forward_blocked) {
 RoutePlanningPipeline::refresh(global_planner, estimate, goal, has_goal,
 map, force_route_refresh, active_route);
 return motion_.run(
 estimate, perception, local_map, active_route, limits, behavior_limits,
 observation, survey, now_ns, behavior, local_planner, validator,
 controller, forward_blocked);
}

} // namespace lost_signal::robot::autonomy
