#pragma once

#include <algorithm>
#include <cmath>

#include "robot/autonomy/estimation/estimator.hpp"
#include "robot/core/time.hpp"
#include "robot/iface/observation.hpp"
#include "robot/platform/description.hpp"

namespace lost_signal::robot::autonomy {

namespace {

// Standard gravity: physics, not a tuned constant.
constexpr double kGravityMps2 = 9.81;

}  // namespace

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
  // Longitudinal channel (straight-line wheelspin, yaw-blind): wheel accel
  // vs IMU body accel residual. Encoder quantization rides ~0.2 m/s^2
  // after the one-pole low-pass; sustained spin-up reads 10+.
  double long_tighten_accel_mps2 = 2.0;
  double long_release_accel_mps2 = 0.8;
  // Grade transitions and launch inertia ring the residual briefly even
  // gripped; genuine traction loss persists for seconds. The longitudinal
  // channel therefore needs this many CONSECUTIVE over-threshold ticks
  // before it may tighten (yaw stays instant: a 2 rad/s spin is
  // unambiguous). 15 ticks = 0.3 s at the 50 Hz tick.
  int long_confirm_ticks = 15;
};

// Control-layer speed-envelope governor. Owns the grip factor in
// [min_grip, 1.0]: 1.0 is nominal (typical grip assumed), lower is
// slip-tightened. Measured-only evidence on two channels (same OR as the
// estimator's slip gate, plus the longitudinal channel the yaw pair cannot
// see): sensor-level odom/gyro disagreement gated on body motion, the
// observer yaw residual when the twist is observable (its own 0.5 m/s
// floor), and the low-passed wheel-vs-IMU accel residual. Gravity
// compensation for the longitudinal channel uses the SURVEYED grade along
// the track (navigation prior, never the accel-derived attitude: sustained
// body accel masquerades as tilt to a gravity filter, which would eat its
// own evidence). NaN survey = unsurveyed = flat assumption, erring toward
// caution on unknown grades. Hysteresis freezes the middle on every
// channel; no evidence freezes the factor. Runs every tick.
class GripAdapter {
 public:
  explicit GripAdapter(GripAdapterConfig config = {}) : config_(config) {}

  void reset() {
    factor_ = 1.0;
    last_ns_ = core::kInvalidTime;
    prev_wheel_v_mps_ = 0.0;
    has_prev_wheel_v_ = false;
    long_residual_mps2_ = 0.0;
    long_fire_count_ = 0;
    slip_firing_ = false;
  }

  void update(core::MonotonicNs now_ns,
              const iface::ObservationFrame& obs, const EstimatedState& est,
              const SlipObservation& slip, double survey_pitch_rad) {
    if (last_ns_ == core::kInvalidTime) {
      last_ns_ = now_ns;
      if (obs.odometry_valid && std::isfinite(obs.forward_mps)) {
        prev_wheel_v_mps_ = obs.forward_mps;
        has_prev_wheel_v_ = true;
      }
      slip_firing_ = false;
      return;
    }
    const double dt_s = static_cast<double>(now_ns - last_ns_) * 1e-9;
    last_ns_ = now_ns;
    if (!(dt_s > 0.0) || dt_s > 1.0) {
      slip_firing_ = false;
      return;  // freeze
    }
    if (!est.valid) {
      slip_firing_ = false;
      return;  // freeze: no belief to adapt
    }
    double yaw_residual = 0.0;
    bool yaw_observable = false;
    const bool has_motion =
        std::abs(est.forward_mps) > config_.min_evidence_speed_mps;
    // Raw sensor pair is only evidence while the body is moving, otherwise
    // gyro noise at standstill would tighten spuriously.
    if (obs.odometry_valid && obs.imu_valid && has_motion) {
      yaw_residual =
          std::abs(obs.yaw_rate_radps - obs.imu_gyro_z_radps);
      yaw_observable = true;
    }
    if (slip.quality == SlipQuality::kObservable) {
      yaw_residual = std::max(yaw_residual, std::abs(slip.yaw_residual_rads));
      yaw_observable = true;
    }
    // Longitudinal channel: wheel-implied acceleration vs IMU body accel.
    // One-pole low-pass rides over encoder-quantization chatter (~0.2
    // m/s^2 after filtering; sustained spin-up reads 10+). Missing IMU or
    // odometry freezes the channel without clearing it.
    bool long_observable = has_prev_wheel_v_;
    if (obs.odometry_valid && obs.imu_valid && has_prev_wheel_v_ &&
        std::isfinite(obs.forward_mps) &&
        std::isfinite(obs.imu_ax_mps2)) {
 // The IMU gravity estimate is the primary compensation. The surveyed
 // grade is only a prior for startup/attitude dropout; using it as the
 // dominant term turns a legitimate ramp transition into wheel-slip
 // evidence when localization is imperfect.
 const double grade = est.attitude_valid && std::isfinite(est.pitch_rad)
 ? est.pitch_rad
 : (std::isfinite(survey_pitch_rad) ? survey_pitch_rad : 0.0);
      const double wheel_accel =
          (obs.forward_mps - prev_wheel_v_mps_) / dt_s;
      const double body_accel =
          obs.imu_ax_mps2 - kGravityMps2 * std::sin(grade);
      if (std::isfinite(wheel_accel) && std::isfinite(body_accel)) {
        constexpr double kLongSlipTimeConstantS = 0.15;
        const double alpha = dt_s / (kLongSlipTimeConstantS + dt_s);
        long_residual_mps2_ +=
            alpha * ((wheel_accel - body_accel) - long_residual_mps2_);
      } else {
        long_observable = false;
      }
    } else {
      long_observable = false;
    }
    if (obs.odometry_valid && std::isfinite(obs.forward_mps)) {
      prev_wheel_v_mps_ = obs.forward_mps;
      has_prev_wheel_v_ = true;
    }
    const double long_residual = std::abs(long_residual_mps2_);
    if (!yaw_observable && !long_observable) {
      long_fire_count_ = 0;  // no evidence: confirmation lapses too
      slip_firing_ = false;
      return;
    }
    if (long_observable && long_residual > config_.long_tighten_accel_mps2) {
      long_fire_count_ = std::min(long_fire_count_ + 1, 1 << 30);
    } else {
      long_fire_count_ = 0;
    }
    const bool slip_fire =
        (yaw_observable &&
         yaw_residual > config_.slip_residual_threshold_radps) ||
        long_fire_count_ >= config_.long_confirm_ticks;
    slip_firing_ = slip_fire;  // owning module's verdict, exposed for memory
    // Relax needs EVERY observed channel clean: tightening on one channel
    // while relaxing on the other would dither the envelope.
    const bool clean =
        (!yaw_observable || yaw_residual < config_.release_residual_radps) &&
        (!long_observable || long_residual < config_.long_release_accel_mps2);
    if (slip_fire) {
      // Tighten even at low estimated speed when the observer fired: it
      // already proved motion via its own floor. The sensor pair above is
      // motion-gated, so reaching here at standstill implies observer
      // evidence only.
      factor_ = std::max(config_.min_grip,
                         factor_ - config_.tighten_rate_per_s * dt_s);
    } else if (has_motion && clean) {
      factor_ = std::min(1.0, factor_ + config_.relax_rate_per_s * dt_s);
    }
  }

  double factor() const { return factor_; }
  // Whether the last update fired on either channel (the module's own
  // confirmation logic). Traction memory records the cell while this is
  // true; a tightened-but-quiet envelope is not slip evidence.
  bool slip_firing() const { return slip_firing_; }
  // Fire severity in [0, 1] for traction memory (0 when quiet: a
  // structured no-op the belief clamps, never a threshold here). Owned
  // here so the scale stays with the verdict — callers wire, never judge.
  double slip_severity() const {
    if (!slip_firing_) return 0.0;
    return std::clamp(1.0 - factor_, 0.0, 1.0);
  }

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
  bool slip_firing_ = false;
  core::MonotonicNs last_ns_ = core::kInvalidTime;
  double prev_wheel_v_mps_ = 0.0;
  bool has_prev_wheel_v_ = false;
  double long_residual_mps2_ = 0.0;
  int long_fire_count_ = 0;
};

}  // namespace lost_signal::robot::autonomy
