#pragma once

#include <string>

#include "robot/autonomy/estimation/estimator.hpp"
#include "robot/autonomy/perception/perception.hpp"
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
                                  const platform::ChassisLimits& limits) = 0;
};

// v1: hold while any anonymous track is inside the hold radius around the
// estimate; otherwise allow full chassis speed. Deterministic, no prediction
// beyond current track positions (a predictor enriches this input later).
class WaitOnIntrusionBehavior : public IBehaviorPolicy {
 public:
  explicit WaitOnIntrusionBehavior(double hold_radius_m = 2.0);

  void reset() override {}
  BehaviorDecision decide(const EstimatedState& est,
                          const PerceptionResult& perception,
                          const platform::ChassisLimits& limits) override;

 private:
  double hold_radius_m_;
};

}  // namespace lost_signal::robot::autonomy
