#pragma once

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
 // Relative scan/submap tracking can correct incremental pose error, but it
 // does not observe the absolute map frame. Implementations must therefore
 // preserve map-frame covariance while applying the bounded pose update.
 virtual void correct_relative_pose(double /*x_m*/, double /*y_m*/,
 double /*yaw_rad*/, double /*pos_sigma_m*/,
 double /*yaw_sigma_rad*/,
 core::MonotonicNs /*stamp_ns*/) {}
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
    // and inflates heading covariance.
    double slip_yaw_residual_radps = 0.5;
    double slip_heading_inflation_rad = 0.2;
    // Slip inflation is bounded and forgets: sustained slip holds the extra
    // uncertainty (the heading really is untrusted), but clean rolling
    // relaxes it toward the motion-grown baseline instead of latching at pi
    // forever — a latched heading sigma wedges the supervisor in
    // global-only retries with no local correction ever scheduled.
    double slip_heading_inflation_cap_rad = 1.0;
    double slip_heading_relax_s = 2.0;
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
    // Covariance growth per meter traveled / per second elapsed.
    double pos_growth_per_m = 0.01;
 // Wheel-odometry position uncertainty is distance-dominated. Keep a small
 // time random walk for unmodelled bias without making a stationary robot
 // lose metres of confidence during a long safety hold.
 double pos_growth_per_s = 0.001;
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
 void correct_relative_pose(double x_m, double y_m, double yaw_rad,
 double pos_sigma_m, double yaw_sigma_rad,
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
  // Slip-inflation contribution currently baked into heading_sigma_rad.
  // Tracked separately so clean rolling can relax exactly what slip added
  // (a validated correction or reset clears it outright).
  double slip_heading_extra_rad_ = 0.0;
 double odom_slip_heading_extra_rad_ = 0.0;
 double motion_scale_ = 1.0;
};

}  // namespace lost_signal::robot::autonomy
