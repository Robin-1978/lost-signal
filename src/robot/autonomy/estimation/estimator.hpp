#pragma once

#include "robot/core/time.hpp"
#include "robot/iface/observation.hpp"

namespace lost_signal::robot::autonomy {

// ROBOT-owned state estimate: planar position + full 3D attitude (2.5D).
// Produced only from LSRI observations. Never seeded or corrected by
// simulator truth.
struct EstimatedState {
  bool valid = false;
  core::MonotonicNs stamp_ns = core::kInvalidTime;
  double x_m = 0.0;
  double y_m = 0.0;
  double yaw_rad = 0.0;
  double forward_mps = 0.0;
  double yaw_rate_radps = 0.0;
  // Gravity-referenced attitude from the gyro/accel complementary filter.
  double roll_rad = 0.0;
  double pitch_rad = 0.0;
  bool attitude_valid = false;
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
  // Map/scan correction with explicit pose + uncertainty. Default ignores:
  // only estimators with a correction path override this.
  virtual void correct_pose(double /*x_m*/, double /*y_m*/,
                            double /*yaw_rad*/, double /*pos_sigma_m*/,
                            double /*yaw_sigma_rad*/,
                            core::MonotonicNs /*stamp_ns*/) {}
  // Deliberate deployment seeding (known dock, operator initialpose).
  // Default ignores; only estimators with a seed path override it.
  virtual void seed_pose(double /*x_m*/, double /*y_m*/, double /*yaw_rad*/,
                         double /*pos_sigma_m*/, double /*yaw_sigma_rad*/,
                         core::MonotonicNs /*stamp_ns*/) {}
  virtual const EstimatedState& estimate() const = 0;
};

// Fused odometry + complementary attitude: wheel twist for translation,
// gyro-led yaw with a slip gate, quaternion attitude from gyro integration
// corrected toward the gravity tilt, covariance growing with motion/time.
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
    // Attitude complementary time constant: gyro integration trusted below
    // this horizon, gravity tilt above it.
    double attitude_time_constant_s = 0.5;
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
  // Deliberate deployment seeding (e.g. known dock pose, operator
  // initialpose): snaps the estimate and its uncertainty. The supervisor
  // still gates every runtime correction; seeding never replaces matching.
  void seed_pose(double x_m, double y_m, double yaw_rad,
                 double pos_sigma_m, double yaw_sigma_rad,
                 core::MonotonicNs stamp_ns) override;
  // Map/scan correction with explicit pose + uncertainty: Kalman-lite blend
  // of estimate and correction weighted by their variances. A rejected match
  // never reaches here; the matcher owns acceptance.
  void correct_pose(double x_m, double y_m, double yaw_rad,
                    double pos_sigma_m, double yaw_sigma_rad,
                    core::MonotonicNs stamp_ns) override;
  const EstimatedState& estimate() const override { return estimate_; }

 private:
  struct Quaternion {
    double w = 1.0, x = 0.0, y = 0.0, z = 0.0;
  };
  static Quaternion quaternion_from_euler(double roll, double pitch,
                                          double yaw);
  static void euler_from_quaternion(const Quaternion& q, double& roll,
                                    double& pitch, double& yaw);

  Config config_;
  EstimatedState estimate_{};
  Quaternion attitude_{};
  bool has_prior_ = false;
  iface::ObservationFrame prior_{};
};

}  // namespace lost_signal::robot::autonomy
