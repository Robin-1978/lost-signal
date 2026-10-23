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
  struct Config {
    // Extra traversal cost per meter through unknown cells.
    double unknown_cost_per_m = 0.5;
    // Extra cost per meter of absolute elevation change (ramps cost more
    // than flat ground; cliffs are rejected outright, see below).
    // Cells whose elevation is unknown pay no grade cost.
    double grade_cost_per_m = 2.0;
    // Steps steeper than this rise/run ratio are impassable.
    double max_grade = 0.5;
    // Clearance preference: extra cost per meter scaled by nearby occupancy
    // (0 in the open, up to 1x beside a wall). Keeps routes centered in
    // corridors so the follower keeps clearance; the validator still owns
    // the hard veto.
    double clearance_cost_per_m = 1.0;
    // Believed-roughness price per meter of roughness amplitude (the
    // ROBOT's own IMU observer via the traversal model, never the author's
    // amplitude). Zero model (unbound) pays nothing.
    double roughness_cost_per_m = 8.0;
  };

  explicit AStarPlanner(Config config);
  AStarPlanner();
  // Back-compat single-knob ctor.
  explicit AStarPlanner(double unknown_cost_per_m);

  Route plan(const EstimatedState& start, const RoutePoint& goal,
             const iface::MapSnapshot& map) override;
  void set_traversal(const TraversalModel* model) override {
    traversal_ = model;
  }

 private:
  Config config_;
  StraightLinePlanner fallback_;
  const TraversalModel* traversal_ = nullptr;
};

}  // namespace lost_signal::robot::autonomy
