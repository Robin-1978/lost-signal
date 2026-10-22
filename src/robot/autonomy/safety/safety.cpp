#include "robot/autonomy/safety/safety.hpp"

namespace lost_signal::robot::autonomy {

SafetyMonitor::SafetyMonitor(core::MonotonicNs stale_timeout_ns,
                             float near_field_m)
    : stale_timeout_ns_(stale_timeout_ns), near_field_m_(near_field_m) {}

void SafetyMonitor::reset() {
  latched_ = false;
  latch_reason_.clear();
  last_drops_ = 0;
  last_ = {false, SafetyState::kStopped, "init"};
}

SafetyVerdict SafetyMonitor::evaluate(
    const EstimatedState& /*est*/, const PerceptionResult& /*perception*/,
    const iface::ObservationFrame& obs, const iface::ChannelHealth& channel,
    core::MonotonicNs now_ns) {
  if (latched_) {
    last_ = {false, SafetyState::kStopped, latch_reason_};
    return last_;
  }
  if (now_ns - obs.stamp_ns > stale_timeout_ns_) {
    latched_ = true;
    latch_reason_ = "obs-stale";
    last_ = {false, SafetyState::kStopped, latch_reason_};
    return last_;
  }
  if (obs.scan_valid && !obs.ranges_m.empty()) {
    float min_range = obs.ranges_m[0];
    for (float r : obs.ranges_m) min_range = r < min_range ? r : min_range;
    if (min_range < near_field_m_) {
      latched_ = true;
      latch_reason_ = "near-field";
      last_ = {false, SafetyState::kStopped, latch_reason_};
      return last_;
    }
  } else {
    last_ = {false, SafetyState::kDegraded, "no-scan-evidence"};
    return last_;
  }
  if (channel.obs_dropped != last_drops_) {
    last_drops_ = channel.obs_dropped;
    last_ = {false, SafetyState::kDegraded, "channel-drops"};
    return last_;
  }
  last_ = {true, SafetyState::kNominal, "nominal"};
  return last_;
}

iface::MotionCommand SafetyMonitor::gate(const iface::MotionCommand& cmd,
                                         const SafetyVerdict& verdict,
                                         core::MonotonicNs now_ns) {
  if (verdict.allow_motion) return cmd;
  return iface::MotionCommand::SafeStop(now_ns);
}

void SafetyMonitor::clear_safety() {
  latched_ = false;
  latch_reason_.clear();
}

}  // namespace lost_signal::robot::autonomy
