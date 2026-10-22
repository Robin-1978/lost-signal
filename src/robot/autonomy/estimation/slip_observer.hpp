#pragma once

namespace lost_signal::robot::autonomy {

// Slip residual observer. Asks "does wheel motion represent body motion
// right now?" — wheel-derived twist MINUS estimator body twist.
//
// Inputs: wheel twist (encoders + IK, the necessary-but-not-sufficient
// observation source; in this chain the driver-reduced body twist) and the
// estimator view (body twist + uncertainty + validity). Output:
// instantaneous residual + confidence. No latch, no count, no threshold,
// no behavior: consumers decide (the estimator's gyro-trust gate is one).
//
// Honesty boundaries (refusal, never silent substitution):
// - estimator invalid -> kInvalid, confidence 0, residuals 0.
// - speed below floor -> kUnobservableLowSpeed: encoder quantization
//   dominates down here; twist below the floor is unreliable for turning.
// - stale estimator -> residual still computed but kStaleEstimator with
//   halved confidence (degraded, not hidden).

enum class SlipQuality {
  kObservable,
  kUnobservableLowSpeed,
  kStaleEstimator,
  kInvalid,
};

struct WheelTwist {
  double v{0.0};  // forward body speed implied by wheels.
  double w{0.0};  // yaw rate implied by wheels.
};

// Estimator view the observer is allowed to read: twist, validity,
// staleness by age, and a scalar uncertainty load. Carries no covariance
// matrix and no policy — status, not numbers, carries the warning.
struct SlipBodyView {
  double v{0.0};
  double w{0.0};
  bool valid{false};
  bool stale{false};
  double trace{0.0};  // uncertainty load, e.g. sigma sums; >= 0
};

struct SlipObservation {
  // (v_wheel - v_body) / max(|v_wheel|, floor). Positive = wheels outrun the
  // body (drive slip); negative = body outruns the wheels (lock/skid). May
  // exceed [-1, 1] when the denominator is floored — stated, not clamped.
  double longitudinal_ratio{0.0};
  double yaw_residual_rads{0.0};  // w_wheel - w_body.
  // Reporting carrier: 1/(1+trace) when observable, halved when stale,
  // 0 otherwise. No gate acts on it.
  double confidence{0.0};
  SlipQuality quality{SlipQuality::kInvalid};
};

// Longitudinal speed floor below which slip is unobservable (m/s).
constexpr double kSlipSpeedFloorMps = 0.5;

SlipObservation observe_slip(WheelTwist wheel, const SlipBodyView& body);

const char* slip_quality_name(SlipQuality quality);

}  // namespace lost_signal::robot::autonomy
