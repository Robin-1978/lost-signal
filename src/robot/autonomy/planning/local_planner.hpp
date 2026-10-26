#pragma once

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "robot/autonomy/estimation/estimator.hpp"
#include "robot/autonomy/mapping/mapper.hpp"
#include "robot/autonomy/perception/perception.hpp"
#include "robot/autonomy/planning/global_planner.hpp"
#include "robot/autonomy/planning/prediction.hpp"
#include "robot/platform/description.hpp"

namespace lost_signal::robot::autonomy {

// Terrain gate for rollout sampling (master's StaticFeasibility doctrine):
// one segment of a rollout is admissible only when the terrain belief
// agrees. The planner owns sampling, the belief owns the grade math; the
// coordinator wires them. Empty gate = pure obstacle sampling (legacy).
using TerrainGate =
    std::function<bool(double x0_m, double y0_m, double x1_m, double y1_m,
                       double yaw_rad)>;

// One chassis-feasible candidate with explicit cost. Position, heading,
// velocity, and yaw rate travel together; never a geometry-only polyline.
// lateral_mps is body-left velocity, nonzero only for holonomic bases
// (sampled only when limits.max_lateral_mps > 0); steer intent travels as
// yaw rate and the chassis resolves the bicycle angle.
struct LocalCandidate {
  double forward_mps = 0.0;
  double yaw_rate_radps = 0.0;
  double lateral_mps = 0.0;
  double cost = 0.0;
 // Planner evidence for route-aware ranking. Progress is measured forward
 // from the current route cursor and stops at the next gear change; tracking
 // error is the rollout endpoint's distance to that same route interval.
 double route_progress_m = 0.0;
 double route_tracking_error_m = 0.0;
 double route_heading_error_rad = 0.0;
  bool feasible = false;
};

struct LocalPlan {
  bool has_plan = false;
 // The tracked route can remain collision-free yet become kinematically
 // exhausted, for example when a car reaches its final position with too
 // much heading error to rotate in place. This requests a new global route
 // from the current ROBOT estimate; it never authorizes motion by itself.
 bool route_replan_requested = false;
 // Generation-side evidence for diagnostics. The independent validator may
 // still reject these candidates before control.
 std::size_t moving_candidate_count = 0;
 int requested_gear = 0;
 double route_error_m = 0.0;
 std::size_t route_segment = 0;
 bool gear_change_pending = false;
 bool gear_change_braking_armed = false;
 double gear_change_endpoint_distance_m = 0.0;
  LocalCandidate best;
 // Planner-ranked proposals. Safety selection happens after generation:
 // one high-scoring proposal failing the independent validator must not
 // turn a non-empty dynamic window into a one-cycle "no plan". `best`
 // remains the first proposal for compatibility with existing callers.
 std::vector<LocalCandidate> candidates;
};

// ILocalPlanner: estimate + perception + map + route + limits -> one
// candidate. Decision-rate (e.g. 10 Hz), independent of the global planner.
class ILocalPlanner {
 public:
  virtual ~ILocalPlanner() = default;
 // Full reset for a new task/lifecycle. Clears every stateful decision,
 // including terminal capture tied to the previous goal.
  virtual void reset() = 0;
 // Route geometry and rollout history become stale after a map correction
 // or replan, while the task-level terminal goal may remain identical.
 // Stateful planners can preserve goal-scoped state across this narrower
 // reset; stateless planners retain the safe full-reset default.
 virtual void reset_tracking() { reset(); }
  virtual LocalPlan plan(const EstimatedState& est,
                         const PerceptionResult& perception,
 const PredictionBatch& prediction,
                         const LocalMap& map, const Route& route,
                         const platform::ChassisLimits& limits) = 0;
  // Terrain gate for rollout sampling (default ignores: pure obstacle
  // sampling, legacy behavior exactly).
  virtual void set_terrain_gate(TerrainGate gate) { (void)gate; }
};

// Planner construction belongs to the planning module. Unknown names fail
// closed with nullptr; the coordinator only owns and sequences the result.
std::unique_ptr<ILocalPlanner> make_local_planner(
 const std::string& planner_kind,
 const platform::NavigationSafetyPolicy& navigation = {},
 const platform::MotionDynamicsPolicy& motion = {},
 double terminal_position_tolerance_m = 0.25,
 double terminal_heading_tolerance_rad = 0.20,
 double terminal_confidence_probability = 0.95,
 double terminal_maximum_position_sigma_m = 0.25,
 double terminal_maximum_heading_sigma_rad = 0.15,
 int terminal_replan_settle_cycles = 3);

}  // namespace lost_signal::robot::autonomy
