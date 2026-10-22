#pragma once

#include "robot/autonomy/estimation/estimator.hpp"
#include "robot/autonomy/mapping/mapper.hpp"
#include "robot/autonomy/perception/perception.hpp"
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

// ILocalPlanner: estimate + perception + local map + limits -> one candidate.
// Decision-rate (e.g. 10 Hz), independent of the global planner.
class ILocalPlanner {
 public:
  virtual ~ILocalPlanner() = default;
  virtual void reset() = 0;
  virtual LocalPlan plan(const EstimatedState& est,
                         const PerceptionResult& perception,
                         const LocalMap& map,
                         const platform::ChassisLimits& limits) = 0;
};

// v1: always hold position. A sampling/DWA planner replaces this body;
// the validator below stays the independent safety authority.
class HoldPlanner : public ILocalPlanner {
 public:
  void reset() override {}
  LocalPlan plan(const EstimatedState& est, const PerceptionResult& perception,
                 const LocalMap& map,
                 const platform::ChassisLimits& /*limits*/) override {
    LocalPlan p;
    if (!est.valid || !perception.scan_ok || !map.valid) return p;
    p.has_plan = true;
    p.best = LocalCandidate{};  // zero motion, feasible hold
    p.best.feasible = true;
    return p;
  }
};

}  // namespace lost_signal::robot::autonomy
