#include "robot/autonomy/estimation/estimator.hpp"

#include <algorithm>
#include <cmath>

#include "robot/core/geometry.hpp"

namespace lost_signal::robot::autonomy {

FusedOdometryEstimator::FusedOdometryEstimator(Config config)
    : config_(config) {}

FusedOdometryEstimator::FusedOdometryEstimator() : config_() {}

FusedOdometryEstimator::Quaternion
FusedOdometryEstimator::quaternion_from_euler(double roll, double pitch,
                                              double yaw) {
  const double cr = std::cos(roll * 0.5), sr = std::sin(roll * 0.5);
  const double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
  const double cy = std::cos(yaw * 0.5), sy = std::sin(yaw * 0.5);
  return {cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
          cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy};
}

void FusedOdometryEstimator::euler_from_quaternion(const Quaternion& q,
                                                  double& roll, double& pitch,
                                                  double& yaw) {
  const double sinr = 2.0 * (q.w * q.x + q.y * q.z);
  const double cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
  roll = std::atan2(sinr, cosr);
  const double sinp = std::clamp(2.0 * (q.w * q.y - q.z * q.x), -1.0, 1.0);
  pitch = std::asin(sinp);
  const double siny = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  yaw = std::atan2(siny, cosy);
}

void FusedOdometryEstimator::reset() {
  estimate_ = EstimatedState{};
  estimate_.position_sigma_m = 0.05;
  estimate_.heading_sigma_rad = 0.01;
  estimate_.confidence = 1.0;
  attitude_ = Quaternion{};
  has_prior_ = false;
  prior_ = iface::ObservationFrame{};
  last_slip_ = SlipObservation{};
  slip_heading_extra_rad_ = 0.0;
 motion_scale_ = 1.0;
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
      estimate_.yaw_rate_radps = obs.imu_gyro_z_radps;
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
  // The observer compares wheel twist against the estimator body twist;
  // when it cannot observe (low speed, no prior), the inline sensor-level
  // residual (odom vs gyro in this frame) is the fallback. Same behavior
  // as the old inline gate, now with a quality-carrying observation.
  double yaw_rate = estimate_.yaw_rate_radps;
  bool slipped = false;
  WheelTwist wheel{obs.odometry_valid ? obs.forward_mps : estimate_.forward_mps,
                   obs.odometry_valid ? obs.yaw_rate_radps
                                      : estimate_.yaw_rate_radps};
  SlipBodyView body{estimate_.forward_mps,
                    estimate_.yaw_rate_radps,
                    has_prior_ && estimate_.valid,
                    false,  // prediction rate == sensor rate: never stale here
                    estimate_.position_sigma_m + estimate_.heading_sigma_rad};
  last_slip_ = observe_slip(wheel, body);
  if (obs.odometry_valid && obs.imu_valid) {
    // Fast path: sensor-level disagreement inside this frame (odom vs gyro)
    // reacts within one cycle to a step change. Estimate-level path: the
    // observer residual catches drift the sensors agree on but the filter
    // has not absorbed. Either trusts the gyro and inflates heading.
    const double inline_residual =
        std::abs(obs.yaw_rate_radps - obs.imu_gyro_z_radps);
    const bool observer_slip =
        last_slip_.quality == SlipQuality::kObservable &&
        std::abs(last_slip_.yaw_residual_rads) > config_.slip_yaw_residual_radps;
    slipped = inline_residual > config_.slip_yaw_residual_radps ||
              observer_slip;
    yaw_rate = slipped ? obs.imu_gyro_z_radps
                       : (1.0 - config_.gyro_weight) * obs.yaw_rate_radps +
                             config_.gyro_weight * obs.imu_gyro_z_radps;
  } else if (obs.imu_valid) {
    yaw_rate = obs.imu_gyro_z_radps;
  } else if (obs.odometry_valid) {
    yaw_rate = obs.yaw_rate_radps;
  }
 // Wheel odometry is a commanded/encoder twist. When the control governor
 // has measured longitudinal slip it supplies a bounded scale so the pose
 // estimate does not integrate wheel spin as travelled distance. The slip
 // observer above still consumes the raw twist; only translation is scaled.
 const double forward =
 obs.odometry_valid ? obs.forward_mps * motion_scale_ : estimate_.forward_mps;
  const double yaw_mid = estimate_.yaw_rad + 0.5 * yaw_rate * dt_s;
  const double step_m = forward * dt_s;
  estimate_.x_m += step_m * std::cos(yaw_mid);
  estimate_.y_m += step_m * std::sin(yaw_mid);
  // Slope z: dead-reckoned climb from the current pitch (gravity-filtered
  // attitude below). No map correction observes z — the raster is 2D.
  estimate_.z_m += step_m * std::sin(estimate_.pitch_rad);
  estimate_.yaw_rad = core::wrap_angle(estimate_.yaw_rad + yaw_rate * dt_s);
  estimate_.forward_mps = forward;
  estimate_.yaw_rate_radps = yaw_rate;
  estimate_.stamp_ns = obs.stamp_ns;

  // Attitude: integrate gyro rotation, correct roll/pitch toward the
  // gravity tilt implied by specific force. Mount assumed aligned in v1;
  // calibrated extrinsics rotate the measurement first (see description).
  if (obs.imu_valid) {
    const double wx = obs.imu_gyro_x_radps, wy = obs.imu_gyro_y_radps,
                 wz = obs.imu_gyro_z_radps;
    const double omega = std::hypot(wx, wy, wz);
    Quaternion dq{1.0, 0.0, 0.0, 0.0};
    if (omega > 1e-12) {
      const double ha = omega * dt_s * 0.5;
      const double s = std::sin(ha) / omega;
      dq = {std::cos(ha), wx * s, wy * s, wz * s};
    }
    const Quaternion q = attitude_;
    attitude_ = {q.w * dq.w - q.x * dq.x - q.y * dq.y - q.z * dq.z,
                 q.w * dq.x + q.x * dq.w + q.y * dq.z - q.z * dq.y,
                 q.w * dq.y - q.x * dq.z + q.y * dq.w + q.z * dq.x,
                 q.w * dq.z + q.x * dq.y - q.y * dq.x + q.z * dq.w};
    const double norm = std::hypot(attitude_.w,
                                    std::hypot(attitude_.x, attitude_.y,
                                               attitude_.z));
    if (norm > 1e-12) {
      attitude_.w /= norm;
      attitude_.x /= norm;
      attitude_.y /= norm;
      attitude_.z /= norm;
    }
    // Gravity tilt from specific force (hover assumption: body accel << g
    // in v1; a motion-compensated variant uses the fused velocity).
    const double ax = obs.imu_ax_mps2, ay = obs.imu_ay_mps2,
                 az = obs.imu_az_mps2;
    const double acc_norm = std::hypot(ax, ay, az);
 double roll, pitch, yaw_ignored;
 euler_from_quaternion(attitude_, roll, pitch, yaw_ignored);
    if (acc_norm > 1e-6 && config_.gravity_mps2 > 0.0 &&
        config_.accel_trust_band_mps2 > 0.0 &&
        config_.max_accel_tilt_correction_radps > 0.0) {
 // Specific force points opposite the world gravity vector.  With the
 // REP-103 body frame and R = Rz(yaw) Ry(pitch) Rx(roll), a positive body
 // roll produces a positive Y specific-force component and a positive body
 // pitch produces a negative X component.  Keeping these signs aligned with
 // the transform tree is essential: otherwise a level robot on a MuJoCo
 // ramp appears upside down to the ROBOT and every 3-D scan deprojection is
 // mirrored.
 const double tilt_roll = std::atan2(ay, az);
 const double tilt_pitch = std::atan2(-ax, std::hypot(ay, az));
      const double magnitude_error =
          std::abs(acc_norm - config_.gravity_mps2);
      const double accel_weight = std::clamp(
          1.0 - magnitude_error / config_.accel_trust_band_mps2, 0.0, 1.0);
      const double alpha = accel_weight * dt_s /
                           (config_.attitude_time_constant_s + dt_s);
      const double max_correction =
          config_.max_accel_tilt_correction_radps * dt_s;
      roll += std::clamp(core::wrap_angle(tilt_roll - roll) * alpha,
                         -max_correction, max_correction);
      pitch += std::clamp(core::wrap_angle(tilt_pitch - pitch) * alpha,
                          -max_correction, max_correction);
    }
 const double yaw_keep = estimate_.yaw_rad;
 attitude_ = quaternion_from_euler(roll, pitch, yaw_keep);
 estimate_.roll_rad = roll;
 estimate_.pitch_rad = pitch;
 estimate_.attitude_valid = true;
  }

  // Covariance grows with motion and time; slip inflates heading now.
  estimate_.position_sigma_m = std::clamp(
      estimate_.position_sigma_m + config_.pos_growth_per_m * std::abs(step_m) +
          config_.pos_growth_per_s * dt_s,
      config_.pos_sigma_min_m, config_.pos_sigma_max_m);
  // Recover the motion-grown baseline, grow it, then re-apply the live
  // slip extra (bounded; relaxed on clean ticks). A validated correction
  // or seed clears the extra outright (see correct_pose/seed_pose/reset),
  // so uncertainty relaxes instead of latching once slip stops.
  double heading_base =
      estimate_.heading_sigma_rad - slip_heading_extra_rad_;
  heading_base += config_.yaw_growth_per_s * dt_s;
  if (slipped) {
    slip_heading_extra_rad_ =
        std::min(slip_heading_extra_rad_ + config_.slip_heading_inflation_rad,
                 std::max(0.0, config_.slip_heading_inflation_cap_rad));
  } else if (config_.slip_heading_relax_s > 0.0) {
    slip_heading_extra_rad_ *=
        std::exp(-dt_s / config_.slip_heading_relax_s);
    if (!(slip_heading_extra_rad_ > 1e-12)) slip_heading_extra_rad_ = 0.0;
  }
  estimate_.heading_sigma_rad = std::clamp(
      heading_base + slip_heading_extra_rad_, config_.yaw_sigma_min_rad,
      config_.yaw_sigma_max_rad);
  estimate_.confidence =
      1.0 / (1.0 + estimate_.position_sigma_m + estimate_.heading_sigma_rad);
  prior_ = obs;
}

void FusedOdometryEstimator::set_motion_scale(double scale) {
 if (!std::isfinite(scale)) return;
 motion_scale_ = std::clamp(scale, 0.1, 1.0);
}

void FusedOdometryEstimator::correct(const iface::ObservationFrame& obs) {
  // v1: no map correction; a scan/map localizer calls correct_pose()
  // directly with its validated result. Never simulator truth.
  (void)obs;
}

void FusedOdometryEstimator::correct_pose(double x_m, double y_m,
                                          double yaw_rad, double pos_sigma_m,
                                          double yaw_sigma_rad,
                                          core::MonotonicNs stamp_ns) {
  if (!std::isfinite(x_m) || !std::isfinite(y_m) ||
      !std::isfinite(yaw_rad) || !(pos_sigma_m > 0.0) ||
      !(yaw_sigma_rad > 0.0)) {
    return;  // fail closed on malformed corrections
  }
  const double ev = std::max(1e-9, estimate_.position_sigma_m);
  const double cv = std::max(1e-9, pos_sigma_m);
  const double k = ev / (ev + cv);
  // A validated map correction establishes the estimate: kidnapped or cold
  // starts have no trustworthy prior, so the blend target (not the prior)
  // is what makes this valid. Rejected matches never reach here.
  estimate_.valid = true;
  estimate_.x_m += k * (x_m - estimate_.x_m);
  estimate_.y_m += k * (y_m - estimate_.y_m);
  estimate_.yaw_rad = core::wrap_angle(
      estimate_.yaw_rad + k * core::wrap_angle(yaw_rad - estimate_.yaw_rad));
  estimate_.position_sigma_m =
      std::clamp(1.0 / (1.0 / ev + 1.0 / cv), config_.pos_sigma_min_m,
                 config_.pos_sigma_max_m);
  const double ey = std::max(1e-9, estimate_.heading_sigma_rad);
  const double cy = std::max(1e-9, yaw_sigma_rad);
  estimate_.heading_sigma_rad = std::clamp(
      1.0 / (1.0 / ey + 1.0 / cy), config_.yaw_sigma_min_rad,
      config_.yaw_sigma_max_rad);
  estimate_.stamp_ns = stamp_ns;
  estimate_.confidence =
      1.0 / (1.0 + estimate_.position_sigma_m + estimate_.heading_sigma_rad);
  // A validated correction resolves slip doubt: clear the extra so the
  // next ticks grow from the motion baseline instead of re-adding doubt.
  slip_heading_extra_rad_ = 0.0;
}

void FusedOdometryEstimator::seed_pose(double x_m, double y_m, double yaw_rad,
                                       double pos_sigma_m, double yaw_sigma_rad,
                                       core::MonotonicNs stamp_ns) {
  if (!std::isfinite(x_m) || !std::isfinite(y_m) ||
      !std::isfinite(yaw_rad) || !(pos_sigma_m > 0.0) ||
      !(yaw_sigma_rad > 0.0)) {
    return;  // fail closed on malformed seeds
  }
  estimate_.valid = true;
  estimate_.x_m = x_m;
  estimate_.y_m = y_m;
  estimate_.z_m = 0.0;  // dock/operator seed is ground truth in plane only
  estimate_.yaw_rad = core::wrap_angle(yaw_rad);
  estimate_.forward_mps = 0.0;
  estimate_.yaw_rate_radps = 0.0;
  estimate_.roll_rad = 0.0;
  estimate_.pitch_rad = 0.0;
  estimate_.attitude_valid = false;
  estimate_.position_sigma_m =
      std::clamp(pos_sigma_m, config_.pos_sigma_min_m,
                 config_.pos_sigma_max_m);
  estimate_.heading_sigma_rad = std::clamp(
      yaw_sigma_rad, config_.yaw_sigma_min_rad, config_.yaw_sigma_max_rad);
  estimate_.stamp_ns = stamp_ns;
  estimate_.confidence =
      1.0 / (1.0 + estimate_.position_sigma_m + estimate_.heading_sigma_rad);
  has_prior_ = true;
  prior_ = iface::ObservationFrame{};
 prior_.stamp_ns = stamp_ns;
  attitude_ = quaternion_from_euler(0.0, 0.0, estimate_.yaw_rad);
  slip_heading_extra_rad_ = 0.0;
}

}  // namespace lost_signal::robot::autonomy
