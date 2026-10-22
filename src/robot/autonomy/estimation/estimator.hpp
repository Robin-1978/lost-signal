#pragma once

#include "robot/core/time.hpp"
#include "robot/iface/observation.hpp"

namespace lost_signal::robot::autonomy {

// ROBOT-owned state estimate. Produced only from LSRI observations.
// Never seeded or corrected by simulator truth.
struct EstimatedState {
  bool valid = false;
  core::MonotonicNs stamp_ns = core::kInvalidTime;
  double x_m = 0.0;
  double y_m = 0.0;
  double yaw_rad = 0.0;
  double forward_mps = 0.0;
  double yaw_rate_radps = 0.0;
  // Uncertainty grows with motion and time since correction, shrinks on
  // map/scan correction. Consumers gate behavior on these, never on hope.
  double position_sigma_m = 0.05;
  double heading_sigma_rad = 0.01;
  double confidence = 1.0;  // 1/(1 + sigma load); separate from validity
};

// IEstimator: odometry/IMU prediction + scan/map correction.
class IEstimator {
 public:
  virtual ~IEstimator() = default;
  virtual void reset() = 0;
  virtual void predict(const iface::ObservationFrame& obs) = 0;
  virtual void correct(const iface::ObservationFrame& obs) = 0;
  virtual const EstimatedState& estimate() const = 0;
};

// Fused odometry: wheel twist for translation, complementary yaw from gyro
// and odometry, covariance growing with motion/time, slip gate that trusts
// the IMU yaw and inflates uncertainty when wheels and gyro disagree.
// Map/scan correction lands in correct(); until then covariance only grows.
class FusedOdometryEstimator : public IEstimator {
 public:
  struct Config {
    // Yaw fusion weight toward the gyro (0 = odom only, 1 = gyro only).
    double gyro_weight = 0.7;
    // Slip gate: |odom yaw rate - gyro yaw rate| above this trusts the gyro
    // and inflates heading covariance.
    double slip_yaw_residual_radps = 0.5;
    double slip_heading_inflation_rad = 0.2;
    // Covariance growth per meter traveled / per second elapsed.
    double pos_growth_per_m = 0.01;
    double pos_growth_per_s = 0.005;
    double yaw_growth_per_s = 0.002;
    double pos_sigma_min_m = 0.005;
    double pos_sigma_max_m = 100.0;
    double yaw_sigma_min_rad = 0.002;
    double yaw_sigma_max_rad = 3.141592653589793;
  };

  explicit FusedOdometryEstimator(Config config);
  FusedOdometryEstimator();

  void reset() override;
  void predict(const iface::ObservationFrame& obs) override;
  void correct(const iface::ObservationFrame& obs) override;
  const EstimatedState& estimate() const override { return estimate_; }

 private:
  Config config_;
  EstimatedState estimate_{};
  bool has_prior_ = false;
  iface::ObservationFrame prior_{};
};

}  // namespace lost_signal::robot::autonomy
