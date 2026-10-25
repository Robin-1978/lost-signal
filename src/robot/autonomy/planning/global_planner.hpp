#pragma once

#include <cstddef>
#include <memory>
#include <vector>

#include "robot/autonomy/estimation/estimator.hpp"
#include "robot/autonomy/mapping/traversal.hpp"
#include "robot/iface/map.hpp"
#include "robot/platform/description.hpp"

namespace lost_signal::robot::autonomy {

// Route point: position + nominal body heading, one value end to end.
struct RoutePoint {
  double x_m = 0.0;
  double y_m = 0.0;
  double yaw_rad = 0.0;
 // Signed travel direction for car-like routes: +1 forward, -1 reverse.
 // Zero means unspecified (legacy grid/straight planners); local planning
 // keeps an explicit Hybrid-A* gear decision intact through the pipeline.
 int gear = 0;
};

struct Route {
  bool valid = false;
  std::vector<RoutePoint> points;
};

enum class GlobalPlanStatus {
 kPending,
 kSucceeded,
 kFailed,
};

struct GlobalPlanProgress {
 GlobalPlanStatus status = GlobalPlanStatus::kFailed;
 Route route;
};

// IGlobalPlanner: estimate + semantic goal + map snapshot -> route.
// Event-driven (goal/map change), not per-tick.
class IGlobalPlanner {
 public:
  virtual ~IGlobalPlanner() = default;
  virtual Route plan(const EstimatedState& start, const RoutePoint& goal,
                     const iface::MapSnapshot& map) = 0;
 // Large planners may cooperatively expand a deterministic number of states
 // per decision cycle. Synchronous planners keep the default seam.
 virtual bool supports_incremental_plan() const { return false; }
 virtual std::size_t incremental_plan_budget() const { return 0; }
 virtual void begin_plan(const EstimatedState&, const RoutePoint&,
 const iface::MapSnapshot&) {}
 virtual GlobalPlanProgress continue_plan(std::size_t) {
 return {GlobalPlanStatus::kFailed, {}};
 }
 virtual void cancel_plan() {}
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

std::unique_ptr<IGlobalPlanner> make_global_planner(
 const platform::ChassisLimits& limits, double wheel_base_m,
 const platform::NavigationSafetyPolicy& navigation = {});

}  // namespace lost_signal::robot::autonomy
