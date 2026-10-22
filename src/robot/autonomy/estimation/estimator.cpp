#include "robot/autonomy/estimation/estimator.hpp"

#include <cmath>

namespace lost_signal::robot::autonomy {

void DeadReckoningEstimator::reset() {
  estimate_ = EstimatedState{};
  has_prior_ = false;
  prior_ = iface::ObservationFrame{};
}

void DeadReckoningEstimator::predict(const iface::ObservationFrame& obs) {
  if (!has_prior_) {
    estimate_.valid = true;
    estimate_.stamp_ns = obs.stamp_ns;
    has_prior_ = true;
    prior_ = obs;
    if (obs.odometry_valid) {
      estimate_.forward_mps = obs.forward_mps;
      estimate_.yaw_rate_radps = obs.yaw_rate_radps;
    }
    return;
  }
  if (obs.odometry_valid && prior_.odometry_valid) {
    double dt_s =
        static_cast<double>(obs.stamp_ns - prior_.stamp_ns) * 1e-9;
    if (dt_s > 0.0 && dt_s < 1.0) {
      double yaw_mid =
          estimate_.yaw_rad + 0.5 * prior_.yaw_rate_radps * dt_s;
      estimate_.x_m += prior_.forward_mps * std::cos(yaw_mid) * dt_s;
      estimate_.y_m += prior_.forward_mps * std::sin(yaw_mid) * dt_s;
      estimate_.yaw_rad += prior_.yaw_rate_radps * dt_s;
      estimate_.stamp_ns = obs.stamp_ns;
    }
  } else {
    estimate_.stamp_ns = obs.stamp_ns;
  }
  if (obs.odometry_valid) {
    estimate_.forward_mps = obs.forward_mps;
    estimate_.yaw_rate_radps = obs.yaw_rate_radps;
  }
  prior_ = obs;
}

void DeadReckoningEstimator::correct(const iface::ObservationFrame& obs) {
  // v1: no map correction. A map-correction module updates estimate_ here
  // with bounded innovation + degeneracy rejection; never simulator truth.
  (void)obs;
}

}  // namespace lost_signal::robot::autonomy
