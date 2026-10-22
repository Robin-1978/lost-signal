#include "robot/autonomy/estimation/slip_observer.hpp"

#include <algorithm>
#include <cmath>

namespace lost_signal::robot::autonomy {

SlipObservation observe_slip(WheelTwist wheel, const SlipBodyView& body) {
  SlipObservation out;
  if (!body.valid) {
    return out;
  }
  if (std::max(std::abs(wheel.v), std::abs(body.v)) < kSlipSpeedFloorMps) {
    out.quality = SlipQuality::kUnobservableLowSpeed;
    return out;
  }
  const double denom = std::max(std::abs(wheel.v), kSlipSpeedFloorMps);
  out.longitudinal_ratio = (wheel.v - body.v) / denom;
  out.yaw_residual_rads = wheel.w - body.w;
  const double base = 1.0 / (1.0 + std::max(0.0, body.trace));
  if (!body.stale) {
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

}  // namespace lost_signal::robot::autonomy
