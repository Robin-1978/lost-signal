#pragma once

#include <array>

#include "robot/autonomy/estimation/slip_observer.hpp"
#include "robot/core/time.hpp"
#include "robot/iface/observation.hpp"

namespace lost_signal::robot::autonomy {

// ROBOT-owned state estimate: planar position + full 3D attitude (2.5D).
// Produced only from LSRI observations. Never seeded or corrected by
// simulator truth. z is dead-reckoned from pitch (no map correction
// observes it — the raster is 2D); slopes need it for the sensor height.
struct EstimatedState {
  bool valid = false;
  core::MonotonicNs stamp_ns = core::kInvalidTime;
  double x_m = 0.0;
  double y_m = 0.0;
  double z_m = 0.0;
  double yaw_rad = 0.0;
  double forward_mps = 0.0;
 double lateral_mps = 0.0;
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
 virtual void correct_pose_information(
 double x_m, double y_m, double yaw_rad,
 const std::array<double, 9>& /*pose_information*/,
 double pos_sigma_m, double yaw_sigma_rad,
 core::MonotonicNs stamp_ns) {
 correct_pose(x_m, y_m, yaw_rad, pos_sigma_m, yaw_sigma_rad, stamp_ns);
 }
 // Relative scan/submap tracking can correct incremental pose error, but it
 // does not observe the absolute map frame. Implementations must therefore
 // preserve map-frame covariance while applying the bounded pose update.
 virtual void correct_relative_pose(double /*x_m*/, double /*y_m*/,
 double /*yaw_rad*/, double /*pos_sigma_m*/,
 double /*yaw_sigma_rad*/,
 core::MonotonicNs /*stamp_ns*/) {}
 // A rejected absolute scan match is still information about the current
 // belief: the proposed innovation was outside its declared covariance.
 // Estimators may fold that innovation into their uncertainty without
 // moving the pose. This keeps REP-105 odom continuous while allowing a
 // later recovery search to widen from evidence instead of a fixed retry
 // radius. The default is a no-op for lightweight test estimators.
 virtual void note_absolute_rejection(double /*dx_m*/, double /*dy_m*/,
 double /*dyaw_rad*/, core::MonotonicNs /*stamp_ns*/) {}
  // Deliberate deployment seeding (known dock, operator initialpose).
  // Default ignores; only estimators with a seed path override it.
  virtual void seed_pose(double /*x_m*/, double /*y_m*/, double /*yaw_rad*/,
                         double /*pos_sigma_m*/, double /*yaw_sigma_rad*/,
                         core::MonotonicNs /*stamp_ns*/) {}
  // Latest slip observation (wheel twist vs estimator body twist).
  // Default reports Invalid; estimators with a slip observer override it.
  // Consumers read quality/confidence, never thresholds.
  virtual SlipObservation last_slip() const { return SlipObservation{}; }
 // Optional traction-aware scale for dead-reckoned translation.
 virtual void set_motion_scale(double /*scale*/) {}
 // Map-frame navigation estimate. Absolute localization and loop closure
 // may correct this pose discontinuously.
 virtual const EstimatedState& estimate() const = 0;
 // REP-105 odom-frame estimate. Prediction may drift, but localization must
 // never move it discontinuously. Estimators without a split state retain
 // the map estimate as a conservative compatibility fallback.
 virtual const EstimatedState& odometry_estimate() const {
 return estimate();
 }
};

// Fused odometry + complementary attitude with REP-105 frame separation.
// Wheel/IMU prediction advances both the continuous odom pose and the
// map-frame navigation pose. Map/scan corrections change only the map pose;
// map_T_odom therefore absorbs relocalization and loop-closure jumps.
class FusedOdometryEstimator : public IEstimator {
 public:
  struct Config {
    // Yaw fusion weight toward the gyro (0 = odom only, 1 = gyro only).
    double gyro_weight = 0.7;
 // Slip gate: |odom yaw rate - gyro yaw rate| above this trusts the gyro
 // and publishes wheel-slip evidence to the control layer. With a valid
 // gyro, wheel disagreement is not absolute-heading uncertainty.
    double slip_yaw_residual_radps = 0.5;
    // Attitude complementary time constant: gyro integration trusted below
    // this horizon, gravity tilt above it.
    double attitude_time_constant_s = 0.5;
    // Accelerometer correction is downweighted as |a|-g departs from the
    // gravity reference, then rate-limited. These are physical sensor-model
    // parameters rather than map/scenario thresholds: translation and
    // impacts must not be integrated as attitude in one sample.
    double gravity_mps2 = 9.80665;
    double accel_trust_band_mps2 = 1.96133; // 0.2 g
    double max_accel_tilt_correction_radps = 0.7;
 // Planar process-noise densities used by the 3x3 [x,y,yaw] covariance.
 // Variances, rather than standard deviations, add over independent motion
 // increments. Position is distance-dominated; the tiny time walk avoids a
 // stationary robot manufacturing metres of uncertainty. Heading includes
 // gyro/steering bias random walk and couples into cross-track position via
 // the kinematic Jacobian.
 double position_process_variance_per_m = 0.0025;
 double position_process_variance_per_s = 1.0e-6;
 double heading_process_variance_per_s = 1.0e-4;
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
 void correct_pose_information(
 double x_m, double y_m, double yaw_rad,
 const std::array<double, 9>& pose_information,
 double pos_sigma_m, double yaw_sigma_rad,
 core::MonotonicNs stamp_ns) override;
 void correct_relative_pose(double x_m, double y_m, double yaw_rad,
 double pos_sigma_m, double yaw_sigma_rad,
 core::MonotonicNs stamp_ns) override;
 void note_absolute_rejection(double dx_m, double dy_m, double dyaw_rad,
 core::MonotonicNs stamp_ns) override;
  const EstimatedState& estimate() const override { return estimate_; }
 const EstimatedState& odometry_estimate() const override {
 return odom_estimate_;
 }
  // Latest slip observation (wheel twist vs estimator body twist). Updated
  // on every predict; consumers read quality/confidence, never thresholds.
  SlipObservation last_slip() const override { return last_slip_; }
 void set_motion_scale(double scale) override;

 private:
  struct Quaternion {
    double w = 1.0, x = 0.0, y = 0.0, z = 0.0;
  };
  static Quaternion quaternion_from_euler(double roll, double pitch,
                                          double yaw);
  static void euler_from_quaternion(const Quaternion& q, double& roll,
                                    double& pitch, double& yaw);
 void ensure_odom_origin(core::MonotonicNs stamp_ns);

  Config config_;
  EstimatedState estimate_{};
 EstimatedState odom_estimate_{};
  Quaternion attitude_{};
  bool has_prior_ = false;
  SlipObservation last_slip_{};
  iface::ObservationFrame prior_{};
 double motion_scale_ = 1.0;
 // Row-major covariance of [x, y, yaw] in each REP-105 frame. Public API
 // remains compact; EstimatedState derives radial position sigma from the
 // largest XY eigenvalue and heading sigma from P(yaw,yaw).
 std::array<double, 9> estimate_covariance_{};
 std::array<double, 9> odom_covariance_{};
};

}  // namespace lost_signal::robot::autonomy
