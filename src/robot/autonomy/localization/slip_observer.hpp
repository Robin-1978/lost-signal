#pragma once

#include "robot/autonomy/localization/state_estimator.hpp"

namespace sim {

// Slip residual observer (design slice, no producers/consumers wired).
// The retired SlipDetector asked "did acceleration mismatch transiently?"
// (trigger count) and was blind to steady slide while chattering at creep.
// This module asks the right question instead: "does wheel motion represent
// body motion right now?" — wheel-derived twist MINUS estimator body twist.
//
// Inputs: wheel twist (encoders + IK, the necessary-but-not-sufficient
// observation source) and the estimator view (body twist + covariance +
// status). Output: instantaneous residual + confidence. No latch, no count,
// no threshold, no behavior: consumers decide.
//
// Honesty boundaries (refusal, never silent substitution):
// - estimator Invalid -> kInvalid, confidence 0, residuals 0 (read quality).
// - speed below floor -> kUnobservableLowSpeed: encoder quantization
//   (0.015 m / 0.02 s ~= 0.77 m/s jumps, see plan §5) dominates down here,
//   and the chassis already declares twist below 0.5 m/s unreliable for
//   turning — same floor, stated once.
// - stale estimator -> residual still computed but kStaleEstimator with halved
//   confidence (degraded, not hidden).

enum class SlipQuality {
  kObservable,
  kUnobservableLowSpeed,
  kStaleEstimator,
  kInvalid,
};

struct WheelTwist {
  double v{0.0}; // forward body speed implied by wheels (encoders + IK).
  double w{0.0}; // yaw rate implied by wheels.
};

struct SlipObservation {
  // (v_wheel - v_body) / max(|v_wheel|, floor). Positive = wheels outrun the
  // body (drive slip); negative = body outruns the wheels (lock/skid). May
  // exceed [-1, 1] when the denominator is floored — stated, not clamped.
  double longitudinal_ratio{0.0};
  double yaw_residual_rads{0.0}; // w_wheel - w_body.
  // Reporting carrier shared with the shadow episode: 1/(1+trace) when
  // Fresh, halved when Stale, 0 otherwise. No gate acts on it.
  double confidence{0.0};
  SlipQuality quality{SlipQuality::kInvalid};
};

// Longitudinal speed floor below which slip is unobservable (m/s).
constexpr double kSlipSpeedFloorMps = 0.5;

SlipObservation observe_slip(WheelTwist wheel, const PropagatedEstimate& body);

const char* slip_quality_name(SlipQuality quality);

} // namespace sim
