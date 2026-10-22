#pragma once

#include "robot/autonomy/estimation/estimator.hpp"
#include "robot/autonomy/planning/local_planner.hpp"
#include "robot/iface/command.hpp"

namespace lost_signal::robot::autonomy {

// IController: validated candidate -> timestamped, expiring body command.
// Tracks the envelope; never invents motion the validator rejected.
class IController {
 public:
  virtual ~IController() = default;
  virtual void reset() = 0;
  virtual iface::MotionCommand track(const LocalCandidate& candidate,
                                     const EstimatedState& est,
                                     core::MonotonicNs now_ns) = 0;
};

// v1: emits the validated candidate verbatim with a 200 ms watchdog.
// Chassis-specific clamping stays in the IChassis wrapper (DifferentialRobot).
class DirectController : public IController {
 public:
  void reset() override {}
  iface::MotionCommand track(const LocalCandidate& candidate,
                             const EstimatedState& /*est*/,
                             core::MonotonicNs now_ns) override {
    // One canonical representation: zero motion is always kStop, never a
    // zero-valued kYawRate. Anything infeasible is also a safe stop.
    if (!candidate.feasible || (candidate.forward_mps == 0.0 &&
                                candidate.yaw_rate_radps == 0.0)) {
      return iface::MotionCommand::SafeStop(now_ns);
    }
    iface::MotionCommand cmd;
    cmd.space = iface::CommandSpace::kYawRate;
    cmd.stamp_ns = now_ns;
    cmd.expiry_ns = now_ns + 200'000'000;
    cmd.forward_mps = candidate.forward_mps;
    cmd.yaw_rate_radps = candidate.yaw_rate_radps;
    return cmd;
  }
};

}  // namespace lost_signal::robot::autonomy
