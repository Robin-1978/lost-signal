#pragma once

#include "robot/autonomy/estimation/estimator.hpp"
#include "robot/autonomy/planning/global_planner.hpp"
#include "robot/iface/map.hpp"

namespace lost_signal::robot::autonomy {

// Event-driven route refresh policy. The coordinator owns lifecycle and
// transport ordering; this module owns the decision of when a global planner
// is called and replaces the active route atomically.
class RoutePlanningPipeline {
 public:
  static bool refresh(IGlobalPlanner& planner, const EstimatedState& start,
                      const RoutePoint& goal, bool has_goal,
                      const iface::MapSnapshot& map, bool force_replan,
                      Route& active_route);
};

}  // namespace lost_signal::robot::autonomy
