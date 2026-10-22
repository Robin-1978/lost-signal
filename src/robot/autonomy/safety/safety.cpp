#include "robot/autonomy/safety/safety.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "robot/core/geometry.hpp"

namespace lost_signal::robot::autonomy {

SafetyMonitor::SafetyMonitor(SafetyConfig config) : config_(config) {}

void SafetyMonitor::reset() {
  latched_ = false;
  latch_reason_.clear();
  last_drops_ = 0;
  last_ = {false, SafetyState::kStopped, "init"};
}

SafetyVerdict SafetyMonitor::evaluate(
    const EstimatedState& est, const PerceptionResult& /*perception*/,
    const iface::ObservationFrame& obs, const iface::ChannelHealth& channel,
    core::MonotonicNs now_ns) {
  if (latched_) {
    last_ = {false, SafetyState::kStopped, latch_reason_};
    return last_;
  }
  if (now_ns - obs.stamp_ns > config_.stale_timeout_ns) {
    latched_ = true;
    latch_reason_ = "obs-stale";
    last_ = {false, SafetyState::kStopped, latch_reason_};
    return last_;
  }
  if (obs.scan_valid && !obs.ranges_m.empty()) {
    const std::size_t n = obs.ranges_m.size();
    float omni_min = std::numeric_limits<float>::infinity();
    float fwd_min = std::numeric_limits<float>::infinity();
    for (std::size_t i = 0; i < n; ++i) {
      const float r = obs.ranges_m[i];
      if (!(r > 0.0f) || !std::isfinite(r)) continue;
      omni_min = std::min(omni_min, r);
      const double angle =
          -core::kPi + 2.0 * core::kPi * static_cast<double>(i) / n;
      if (std::abs(core::wrap_angle(angle - config_.sensor_yaw_rad)) <=
          config_.forward_half_angle_rad) {
        fwd_min = std::min(fwd_min, r);
      }
    }
    // Contact latch: omnidirectional, last resort only.
    if (omni_min < config_.contact_radius_m) {
      latched_ = true;
      latch_reason_ = "contact";
      last_ = {false, SafetyState::kStopped, latch_reason_};
      return last_;
    }
    // Braking veto in the drive direction: margin + v^2/2a on the
    // measured forward speed. Transient: clears with fresh evidence.
    if (std::isfinite(fwd_min)) {
      const double speed = std::abs(est.forward_mps);
      const double need_m =
          config_.near_field_m +
          (speed * speed) / (2.0 * config_.brake_decel_mps2);
      if (fwd_min < need_m) {
        last_ = {false, SafetyState::kDegraded, "forward-proximity"};
        return last_;
      }
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
