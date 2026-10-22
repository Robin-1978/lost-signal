#pragma once

#include <vector>

#include "robot/autonomy/estimation/estimator.hpp"
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
};

// v1: straight line sampled every 0.5 m. No obstacle awareness;
// the local planner + validator own feasibility and safety.
class StraightLinePlanner : public IGlobalPlanner {
 public:
  Route plan(const EstimatedState& start, const RoutePoint& goal,
             const iface::MapSnapshot& /*map*/) override;
};

}  // namespace lost_signal::robot::autonomy
