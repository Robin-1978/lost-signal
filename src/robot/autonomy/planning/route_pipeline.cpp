#include "robot/autonomy/planning/route_pipeline.hpp"

namespace lost_signal::robot::autonomy {

bool RoutePlanningPipeline::refresh(IGlobalPlanner& planner,
                                    const EstimatedState& start,
                                    const RoutePoint& goal, bool has_goal,
                                    const iface::MapSnapshot& map,
                                    bool force_replan,
                                    Route& active_route) {
  if (!has_goal) return false;
  if (!force_replan && active_route.valid) return false;

  // A planner may provide a fail-closed invalid route while localization is
  // still starting. Do not retain a previous route across that refresh.
  active_route = planner.plan(start, goal, map);
  return true;
}

}  // namespace lost_signal::robot::autonomy
