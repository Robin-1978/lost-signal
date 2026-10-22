#pragma once

#include "robot/autonomy/estimation/estimator.hpp"
#include "robot/autonomy/mapping/mapper.hpp"
#include "robot/autonomy/perception/perception.hpp"
#include "robot/autonomy/planning/global_planner.hpp"
#include "robot/platform/description.hpp"

namespace lost_signal::robot::autonomy {

// One chassis-feasible candidate with explicit cost. Position, heading,
// velocity, and yaw rate travel together; never a geometry-only polyline.
struct LocalCandidate {
  double forward_mps = 0.0;
  double yaw_rate_radps = 0.0;
  double cost = 0.0;
  bool feasible = false;
};

struct LocalPlan {
  bool has_plan = false;
  LocalCandidate best;
};

// ILocalPlanner: estimate + perception + map + route + limits -> one
// candidate. Decision-rate (e.g. 10 Hz), independent of the global planner.
class ILocalPlanner {
 public:
  virtual ~ILocalPlanner() = default;
  virtual void reset() = 0;
  virtual LocalPlan plan(const EstimatedState& est,
                         const PerceptionResult& perception,
                         const LocalMap& map, const Route& route,
                         const platform::ChassisLimits& limits) = 0;
};

}  // namespace lost_signal::robot::autonomy
