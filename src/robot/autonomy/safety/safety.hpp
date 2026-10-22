#pragma once

#include <string>

#include "robot/autonomy/estimation/estimator.hpp"
#include "robot/autonomy/perception/perception.hpp"
#include "robot/core/time.hpp"
#include "robot/iface/channel.hpp"
#include "robot/iface/command.hpp"
#include "robot/iface/observation.hpp"

namespace lost_signal::robot::autonomy {

// Independent safety authority, last gate before publication.
// It re-checks RAW evidence (scan + freshness + channel health), never
// planner state, and can only ever downgrade a command to safe stop.
// Critical faults latch into STOPPED and need deliberate recovery
// (clear_safety, or full shutdown -> configure -> activate).
enum class SafetyState {
  kNominal = 0,
  kDegraded = 1,  // transient: motion blocked, auto-recovers on fresh evidence
  kStopped = 2,   // latched: stays until deliberate recovery
};

struct SafetyVerdict {
  bool allow_motion = false;
  SafetyState state = SafetyState::kStopped;
  std::string reason;  // stable code for logs/tests
};

class ISafetyMonitor {
 public:
  virtual ~ISafetyMonitor() = default;
  virtual void reset() = 0;
  virtual SafetyVerdict evaluate(const EstimatedState& est,
                                 const PerceptionResult& perception,
                                 const iface::ObservationFrame& obs,
                                 const iface::ChannelHealth& channel,
                                 core::MonotonicNs now_ns) = 0;
  virtual iface::MotionCommand gate(const iface::MotionCommand& cmd,
                                    const SafetyVerdict& verdict,
                                    core::MonotonicNs now_ns) = 0;
  virtual void clear_safety() = 0;  // deliberate recovery from a latch
  virtual SafetyVerdict last_verdict() const = 0;
};

// v1 rules (all independent of planner/validator state):
//   - stale observation (> timeout)            -> STOPPED, latched
//   - raw min-range inside near-field envelope  -> STOPPED, latched
//   - channel drops observed since last check   -> DEGRADED, transient
//   - missing scan evidence                     -> DEGRADED, transient
//   - otherwise                                 -> NOMINAL
class SafetyMonitor : public ISafetyMonitor {
 public:
  explicit SafetyMonitor(core::MonotonicNs stale_timeout_ns = 500'000'000,
                         float near_field_m = 0.3f);

  void reset() override;
  SafetyVerdict evaluate(const EstimatedState& est,
                         const PerceptionResult& perception,
                         const iface::ObservationFrame& obs,
                         const iface::ChannelHealth& channel,
                         core::MonotonicNs now_ns) override;
  iface::MotionCommand gate(const iface::MotionCommand& cmd,
                            const SafetyVerdict& verdict,
                            core::MonotonicNs now_ns) override;
  void clear_safety() override;
  SafetyVerdict last_verdict() const override { return last_; }

 private:
  core::MonotonicNs stale_timeout_ns_;
  float near_field_m_;
  bool latched_ = false;
  std::string latch_reason_;
  std::uint64_t last_drops_ = 0;
  SafetyVerdict last_{false, SafetyState::kStopped, "init"};
};

}  // namespace lost_signal::robot::autonomy
