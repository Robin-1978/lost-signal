#include "robot/autonomy/estimation/estimator.hpp"

#include <algorithm>
#include <cmath>

#include "robot/core/geometry.hpp"

namespace lost_signal::robot::autonomy {

FusedOdometryEstimator::FusedOdometryEstimator(Config config)
    : config_(config) {}

FusedOdometryEstimator::FusedOdometryEstimator() : config_() {}

void FusedOdometryEstimator::reset() {
  estimate_ = EstimatedState{};
  estimate_.position_sigma_m = 0.05;
  estimate_.heading_sigma_rad = 0.01;
  estimate_.confidence = 1.0;
  has_prior_ = false;
  prior_ = iface::ObservationFrame{};
}

void FusedOdometryEstimator::predict(const iface::ObservationFrame& obs) {
  if (!obs.odometry_valid && !obs.imu_valid) return;
  if (!has_prior_) {
    estimate_.valid = true;
    estimate_.stamp_ns = obs.stamp_ns;
    has_prior_ = true;
    prior_ = obs;
    if (obs.odometry_valid) {
      estimate_.forward_mps = obs.forward_mps;
      estimate_.yaw_rate_radps = obs.yaw_rate_radps;
    }
    if (obs.imu_valid) {
      estimate_.yaw_rate_radps = obs.imu_yaw_rate_radps;
    }
    return;
  }
  const double dt_s =
      static_cast<double>(obs.stamp_ns - prior_.stamp_ns) * 1e-9;
  if (!(dt_s > 0.0) || dt_s >= 1.0) {
    prior_ = obs;
    return;
  }
  // Yaw fusion: gyro-led complementary blend; slip gate trusts the gyro.
  double yaw_rate = estimate_.yaw_rate_radps;
  bool slipped = false;
  if (obs.odometry_valid && obs.imu_valid) {
    const double residual =
        std::abs(obs.yaw_rate_radps - obs.imu_yaw_rate_radps);
    slipped = residual > config_.slip_yaw_residual_radps;
    yaw_rate = slipped ? obs.imu_yaw_rate_radps
                       : (1.0 - config_.gyro_weight) * obs.yaw_rate_radps +
                             config_.gyro_weight * obs.imu_yaw_rate_radps;
  } else if (obs.imu_valid) {
    yaw_rate = obs.imu_yaw_rate_radps;
  } else if (obs.odometry_valid) {
    yaw_rate = obs.yaw_rate_radps;
  }
  const double forward =
      obs.odometry_valid ? obs.forward_mps : estimate_.forward_mps;
  const double yaw_mid = estimate_.yaw_rad + 0.5 * yaw_rate * dt_s;
  const double step_m = forward * dt_s;
  estimate_.x_m += step_m * std::cos(yaw_mid);
  estimate_.y_m += step_m * std::sin(yaw_mid);
  estimate_.yaw_rad = core::wrap_angle(estimate_.yaw_rad + yaw_rate * dt_s);
  estimate_.forward_mps = forward;
  estimate_.yaw_rate_radps = yaw_rate;
  estimate_.stamp_ns = obs.stamp_ns;

  // Covariance grows with motion and time; slip inflates heading now.
  estimate_.position_sigma_m = std::clamp(
      estimate_.position_sigma_m + config_.pos_growth_per_m * std::abs(step_m) +
          config_.pos_growth_per_s * dt_s,
      config_.pos_sigma_min_m, config_.pos_sigma_max_m);
  double yaw_growth =
      config_.yaw_growth_per_s * dt_s +
      (slipped ? config_.slip_heading_inflation_rad : 0.0);
  estimate_.heading_sigma_rad = std::clamp(
      estimate_.heading_sigma_rad + yaw_growth, config_.yaw_sigma_min_rad,
      config_.yaw_sigma_max_rad);
  estimate_.confidence =
      1.0 / (1.0 + estimate_.position_sigma_m + estimate_.heading_sigma_rad);
  prior_ = obs;
}

void FusedOdometryEstimator::correct(const iface::ObservationFrame& obs) {
  // v1: no map correction; a scan/map localizer writes the correction here
  // and shrinks the sigmas. Never simulator truth.
  (void)obs;
}

}  // namespace lost_signal::robot::autonomy
