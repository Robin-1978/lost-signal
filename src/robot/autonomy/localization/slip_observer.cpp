#include "robot/autonomy/localization/slip_observer.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace sim {

SlipObservation observe_slip(WheelTwist wheel, const PropagatedEstimate& body) {
  SlipObservation out;
  if (body.status == EstimateStatus::kInvalid) {
    return out;
  }
  const double vx = body.estimate.state[3];
  const double vy = body.estimate.state[4];
  const double body_speed = std::copysign(std::hypot(vx, vy), vx);
  const double body_yaw = body.estimate.state[5];
  if (std::max(std::abs(wheel.v), std::abs(body_speed)) < kSlipSpeedFloorMps) {
    out.quality = SlipQuality::kUnobservableLowSpeed;
    return out;
  }
  const double denom = std::max(std::abs(wheel.v), kSlipSpeedFloorMps);
  out.longitudinal_ratio = (wheel.v - body_speed) / denom;
  out.yaw_residual_rads = wheel.w - body_yaw;
  double trace = 0.0;
  for (std::size_t i = 0; i < StateEstimate::kDim; ++i) {
    trace += body.estimate.covariance[i * StateEstimate::kDim + i];
  }
  const double base = 1.0 / (1.0 + trace);
  if (body.status == EstimateStatus::kFresh) {
    out.quality = SlipQuality::kObservable;
    out.confidence = base;
  } else {
    out.quality = SlipQuality::kStaleEstimator;
    out.confidence = 0.5 * base;
  }
  return out;
}

const char* slip_quality_name(SlipQuality quality) {
  switch (quality) {
    case SlipQuality::kObservable:
      return "Observable";
    case SlipQuality::kUnobservableLowSpeed:
      return "UnobservableLowSpeed";
    case SlipQuality::kStaleEstimator:
      return "StaleEstimator";
    case SlipQuality::kInvalid:
      return "Invalid";
  }
  return "Invalid";
}

} // namespace sim
