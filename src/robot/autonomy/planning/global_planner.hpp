#pragma once

#include <vector>

#include "robot/autonomy/estimation/estimator.hpp"
#include "robot/autonomy/mapping/traversal.hpp"
#include "robot/iface/map.hpp"

namespace lost_signal::robot::autonomy {

// Route point: position + nominal body heading, one value end to end.
struct RoutePoint {
  double x_m = 0.0;
  double y_m = 0.0;
  double yaw_rad = 0.0;
};

struct Route {
  bool valid = false;
  std::vector<RoutePoint> points;
};

// IGlobalPlanner: estimate + semantic goal + map snapshot -> route.
// Event-driven (goal/map change), not per-tick.
class IGlobalPlanner {
 public:
  virtual ~IGlobalPlanner() = default;
  virtual Route plan(const EstimatedState& start, const RoutePoint& goal,
                     const iface::MapSnapshot& map) = 0;
  // Traversal-belief hook (optional): the coordinator hands its live
  // TraversalModel here; planners that price roughness read it. Default
  // ignores (no belief, today's behavior). Never owned, never stored
  // beyond the pointer (the coordinator owns the model).
  virtual void set_traversal(const TraversalModel* model) {
    (void)model;
  }
};

// v1: straight line sampled every 0.5 m. No obstacle awareness;
// the local planner + validator own feasibility and safety.
class StraightLinePlanner : public IGlobalPlanner {
 public:
  Route plan(const EstimatedState& start, const RoutePoint& goal,
             const iface::MapSnapshot& /*map*/) override;
};

}  // namespace lost_signal::robot::autonomy
