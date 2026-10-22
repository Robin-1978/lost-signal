#pragma once

#include "robot/autonomy/estimation/estimator.hpp"
#include "robot/autonomy/planning/global_planner.hpp"
#include "robot/iface/map.hpp"

namespace lost_signal::robot::autonomy {

// A* over the static map raster, native chain types. Unknown cells are
// traversable at a penalty (assume free, replan on new revisions); observed
// occupied cells are impassable, with corner cutting forbidden. No raster
// (NullMapProvider, vector-only maps) degrades to a straight-line route;
// a raster with no path yields an invalid route (fail closed upstream).
class AStarPlanner : public IGlobalPlanner {
 public:
  // Extra traversal cost per meter through unknown cells.
  explicit AStarPlanner(double unknown_cost_per_m = 0.5);

  Route plan(const EstimatedState& start, const RoutePoint& goal,
             const iface::MapSnapshot& map) override;

 private:
  double unknown_cost_per_m_;
  StraightLinePlanner fallback_;
};

}  // namespace lost_signal::robot::autonomy
