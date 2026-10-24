#pragma once

#include <string>

#include "robot/autonomy/estimation/estimator.hpp"
#include "robot/autonomy/perception/perception.hpp"
#include "robot/autonomy/planning/global_planner.hpp"
#include "robot/autonomy/planning/prediction.hpp"
#include "robot/platform/description.hpp"

namespace lost_signal::robot::autonomy {

// Dynamic-decision layer (motion timescale, between task policy and trajectory
// sampling). Consumes tracks + estimate + limits, produces ONE constraint for
// the local planner. It never selects a trajectory and never emits a command;
// the validator stays the independent safety authority.
struct BehaviorDecision {
  bool must_hold = false;           // wait in place (intrusion, no clearance)
  double max_forward_mps = 0.0;     // 0 = no motion allowed by behavior
  std::string reason;               // stable code for logs/tests
};

class IBehaviorPolicy {
 public:
  virtual ~IBehaviorPolicy() = default;
  virtual void reset() = 0;
  virtual BehaviorDecision decide(const EstimatedState& est,
                                  const PerceptionResult& perception,
 const PredictionBatch& prediction,
 const Route& route,
                                  const platform::ChassisLimits& limits) = 0;
};

struct RouteConflictBehaviorConfig {
 double near_route_hold_m = 2.0;
 double prediction_horizon_s = 2.5;
 double brake_deceleration_mps2 = 2.0;
 double extra_clearance_m = 0.10;
};

// Hold or cap only for an anonymous track whose measured/predicted footprint
// conflicts with the active route. All distances and dynamics are explicit
// configuration; the policy uses ROBOT estimate + track state only and has no
// simulator or actor semantics.
class WaitOnIntrusionBehavior : public IBehaviorPolicy {
 public:
 explicit WaitOnIntrusionBehavior(RouteConflictBehaviorConfig config = {});

  void reset() override {}
  BehaviorDecision decide(const EstimatedState& est,
                          const PerceptionResult& perception,
 const PredictionBatch& prediction,
 const Route& route,
                          const platform::ChassisLimits& limits) override;
 // Direct-use convenience for unit tests and small integrations. Production
 // constructs one shared batch in MotionDecisionPipeline.
 BehaviorDecision decide(const EstimatedState& est,
 const PerceptionResult& perception,
 const platform::ChassisLimits& limits) {
 const double now_s = est.stamp_ns != core::kInvalidTime
 ? est.stamp_ns * 1e-9
 : perception.stamp_ns * 1e-9;
 return decide(est, perception,
 predict_perception(perception, now_s,
 {.horizon_s = config_.prediction_horizon_s}), Route{}, limits);
 }

 private:
 RouteConflictBehaviorConfig config_;
};

}  // namespace lost_signal::robot::autonomy
