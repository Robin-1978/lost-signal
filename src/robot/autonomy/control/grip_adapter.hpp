#pragma once

#include <algorithm>
#include <cmath>

#include "robot/autonomy/estimation/estimator.hpp"
#include "robot/core/time.hpp"
#include "robot/iface/observation.hpp"
#include "robot/platform/description.hpp"

namespace lost_signal::robot::autonomy {

// Online grip adapter config (all typical values, ROBOT-side beliefs).
// The adapter tightens the speed envelope on measured slip evidence and
// relaxes it on clean rolling evidence — never from truth, which the
// ROBOT never sees. Bounds are the leash: grip never leaves [min_grip,
// 1.0], freezes without evidence, and g=1 reproduces today's behavior
// exactly (adapter off == grip pinned at 1).
struct GripAdapterConfig {
  double min_grip = 0.4;  // floor: tighter would livelock the mission
  double tighten_rate_per_s = 0.4;  // fast: ~1 s from 1.0 to the floor
  double relax_rate_per_s = 0.08;  // slow: ~7 s back to full speed
  // Hysteresis band against dithering: above slip -> tighten, below
  // release -> relax, inside -> freeze. Mirrors the estimator slip gate
  // (0.5) without sharing its code path.
  double slip_residual_threshold_radps = 0.5;
  double release_residual_radps = 0.2;
  double min_evidence_speed_mps = 0.2;  // need motion for either direction
};

// Control-layer speed-envelope governor. Owns the grip factor in
// [min_grip, 1.0]: 1.0 is nominal (typical grip assumed), lower is
// slip-tightened. Measured-only evidence (same OR as the estimator's slip
// gate): sensor-level odom/gyro disagreement gated on body motion, plus
// the observer residual when the twist is observable (the observer
// carries its own 0.5 m/s speed floor). Hysteresis freezes the middle;
// no evidence freezes the factor. Runs every tick, not only on
// decisions: the envelope must tighten within a second of slip evidence,
// not at the next 10 Hz boundary.
class GripAdapter {
 public:
  explicit GripAdapter(GripAdapterConfig config = {}) : config_(config) {}

  void reset() {
    factor_ = 1.0;
    last_ns_ = core::kInvalidTime;
  }

  void update(core::MonotonicNs now_ns,
              const iface::ObservationFrame& obs, const EstimatedState& est,
              const SlipObservation& slip) {
    if (last_ns_ == core::kInvalidTime) {
      last_ns_ = now_ns;
      return;
    }
    const double dt_s = static_cast<double>(now_ns - last_ns_) * 1e-9;
    last_ns_ = now_ns;
    if (!(dt_s > 0.0) || dt_s > 1.0) return;  // freeze
    if (!est.valid) return;                   // freeze: no belief to adapt
    double residual = 0.0;
    bool observable = false;
    const bool has_motion =
        std::abs(est.forward_mps) > config_.min_evidence_speed_mps;
    // Raw sensor pair is only evidence while the body is moving, otherwise
    // gyro noise at standstill would tighten spuriously.
    if (obs.odometry_valid && obs.imu_valid && has_motion) {
      residual =
          std::abs(obs.yaw_rate_radps - obs.imu_gyro_z_radps);
      observable = true;
    }
    if (slip.quality == SlipQuality::kObservable) {
      residual = std::max(residual, std::abs(slip.yaw_residual_rads));
      observable = true;
    }
    if (!observable) return;  // freeze: no evidence either way
    if (residual > config_.slip_residual_threshold_radps) {
      // Tighten even at low estimated speed when the observer fired: it
      // already proved motion via its own floor. The sensor pair above is
      // motion-gated, so reaching here at standstill implies observer
      // evidence only.
      factor_ = std::max(config_.min_grip,
                         factor_ - config_.tighten_rate_per_s * dt_s);
    } else if (has_motion &&
               residual < config_.release_residual_radps) {
      factor_ = std::min(1.0, factor_ + config_.relax_rate_per_s * dt_s);
    }
  }

  double factor() const { return factor_; }

  // Tightened envelope: scale the chassis speed caps (forward, yaw, and
  // lateral when present) by the adapted factor, then apply the behavior
  // cap on top. Behavior still decides hold/cap on full limits; grip can
  // only tighten further, never override a hold or widen anything.
  platform::ChassisLimits effective_limits(
      const platform::ChassisLimits& base, double behavior_max_forward_mps) const {
    platform::ChassisLimits out = base;
    out.max_forward_mps = std::min(base.max_forward_mps,
                                   base.max_forward_mps * factor_);
    out.max_yaw_rate_radps = std::min(base.max_yaw_rate_radps,
                                      base.max_yaw_rate_radps * factor_);
    out.max_lateral_mps = std::min(base.max_lateral_mps,
                                   base.max_lateral_mps * factor_);
    if (behavior_max_forward_mps < out.max_forward_mps) {
      out.max_forward_mps = behavior_max_forward_mps;
    }
    return out;
  }

 private:
  GripAdapterConfig config_;
  double factor_ = 1.0;
  core::MonotonicNs last_ns_ = core::kInvalidTime;
};

}  // namespace lost_signal::robot::autonomy
