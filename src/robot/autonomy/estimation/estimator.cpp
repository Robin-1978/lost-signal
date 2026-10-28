#include "robot/autonomy/estimation/estimator.hpp"

#include <algorithm>
#include <cmath>

#include "robot/core/geometry.hpp"

namespace lost_signal::robot::autonomy {
namespace {

using PlanarCovariance = std::array<double, 9>;

PlanarCovariance diagonal_covariance(
 double position_sigma_m, double heading_sigma_rad) {
 const double position_variance = position_sigma_m * position_sigma_m;
 return {position_variance, 0.0, 0.0,
 0.0, position_variance, 0.0,
 0.0, 0.0, heading_sigma_rad * heading_sigma_rad};
}

double position_sigma(const PlanarCovariance& covariance) {
 const double trace = covariance[0] + covariance[4];
 const double discriminant = std::sqrt(std::max(
 0.0, (covariance[0] - covariance[4]) *
 (covariance[0] - covariance[4]) +
 4.0 * covariance[1] * covariance[1]));
 return std::sqrt(std::max(0.0, 0.5 * (trace + discriminant)));
}

void propagate_planar_covariance(PlanarCovariance& covariance,
 double delta_x_m, double delta_y_m,
 double position_process_variance) {
 // F for a body-frame SE(2) displacement expressed in the world frame.
 // The yaw Jacobian is the displacement rotated by +90 degrees, including
 // lateral odometry from holonomic chassis.
 const double fx_yaw = -delta_y_m;
 const double fy_yaw = delta_x_m;
 const PlanarCovariance f{
 1.0, 0.0, fx_yaw,
 0.0, 1.0, fy_yaw,
 0.0, 0.0, 1.0};
 PlanarCovariance fp{};
 PlanarCovariance propagated{};
 for (int row = 0; row < 3; ++row) {
 for (int column = 0; column < 3; ++column) {
 for (int inner = 0; inner < 3; ++inner) {
 fp[static_cast<std::size_t>(row * 3 + column)] +=
 f[static_cast<std::size_t>(row * 3 + inner)] *
 covariance[static_cast<std::size_t>(inner * 3 + column)];
 }
 }
 }
 for (int row = 0; row < 3; ++row) {
 for (int column = 0; column < 3; ++column) {
 for (int inner = 0; inner < 3; ++inner) {
 propagated[static_cast<std::size_t>(row * 3 + column)] +=
 fp[static_cast<std::size_t>(row * 3 + inner)] *
 f[static_cast<std::size_t>(column * 3 + inner)];
 }
 }
 }
 propagated[0] += position_process_variance;
 propagated[4] += position_process_variance;
 covariance = propagated;
}

void set_heading_sigma(PlanarCovariance& covariance, double sigma_rad) {
 const double old_sigma = std::sqrt(std::max(0.0, covariance[8]));
 const double ratio = old_sigma > 1e-12 ? sigma_rad / old_sigma : 0.0;
 covariance[2] *= ratio;
 covariance[6] = covariance[2];
 covariance[5] *= ratio;
 covariance[7] = covariance[5];
 covariance[8] = sigma_rad * sigma_rad;
}

void publish_covariance(const PlanarCovariance& covariance,
 const FusedOdometryEstimator::Config& config, EstimatedState& state) {
 state.position_sigma_m = std::clamp(
 position_sigma(covariance), config.pos_sigma_min_m,
 config.pos_sigma_max_m);
 state.heading_sigma_rad = std::clamp(
 std::sqrt(std::max(0.0, covariance[8])),
 config.yaw_sigma_min_rad, config.yaw_sigma_max_rad);
 state.confidence =
 1.0 / (1.0 + state.position_sigma_m + state.heading_sigma_rad);
}

bool invert_3x3(const PlanarCovariance& matrix, PlanarCovariance& inverse) {
 const double a = matrix[0], b = matrix[1], c = matrix[2];
 const double d = matrix[3], e = matrix[4], f = matrix[5];
 const double g = matrix[6], h = matrix[7], i = matrix[8];
 const double determinant =
 a * (e * i - f * h) - b * (d * i - f * g) +
 c * (d * h - e * g);
 if (!std::isfinite(determinant) || std::abs(determinant) < 1e-24) {
 return false;
 }
 const double scale = 1.0 / determinant;
 inverse = {
 (e * i - f * h) * scale,
 (c * h - b * i) * scale,
 (b * f - c * e) * scale,
 (f * g - d * i) * scale,
 (a * i - c * g) * scale,
 (c * d - a * f) * scale,
 (d * h - e * g) * scale,
 (b * g - a * h) * scale,
 (a * e - b * d) * scale};
 return std::all_of(inverse.begin(), inverse.end(), [](double value) {
 return std::isfinite(value);
 });
}

PlanarCovariance multiply_3x3(const PlanarCovariance& left,
 const PlanarCovariance& right) {
 PlanarCovariance product{};
 for (int row = 0; row < 3; ++row) {
 for (int column = 0; column < 3; ++column) {
 for (int inner = 0; inner < 3; ++inner) {
 product[static_cast<std::size_t>(row * 3 + column)] +=
 left[static_cast<std::size_t>(row * 3 + inner)] *
 right[static_cast<std::size_t>(inner * 3 + column)];
 }
 }
 }
 return product;
}

} // namespace

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
 odom_estimate_ = estimate_;
 estimate_covariance_ = diagonal_covariance(
 estimate_.position_sigma_m, estimate_.heading_sigma_rad);
 odom_covariance_ = estimate_covariance_;
  attitude_ = Quaternion{};
  has_prior_ = false;
  prior_ = iface::ObservationFrame{};
  last_slip_ = SlipObservation{};
 motion_scale_ = 1.0;
}

void FusedOdometryEstimator::ensure_odom_origin(
 core::MonotonicNs stamp_ns) {
 if (odom_estimate_.valid) return;
 odom_estimate_ = EstimatedState{};
 odom_estimate_.valid = true;
 odom_estimate_.stamp_ns = stamp_ns;
 odom_estimate_.position_sigma_m = 0.05;
 odom_estimate_.heading_sigma_rad = 0.01;
 odom_estimate_.confidence = 1.0;
 odom_covariance_ = diagonal_covariance(
 odom_estimate_.position_sigma_m, odom_estimate_.heading_sigma_rad);
}

void FusedOdometryEstimator::predict(const iface::ObservationFrame& obs) {
  if (!obs.odometry_valid && !obs.imu_valid) return;
  if (!has_prior_) {
    estimate_.valid = true;
    estimate_.stamp_ns = obs.stamp_ns;
 odom_estimate_.valid = true;
 odom_estimate_.stamp_ns = obs.stamp_ns;
    has_prior_ = true;
    prior_ = obs;
    if (obs.odometry_valid) {
      estimate_.forward_mps = obs.forward_mps;
 estimate_.lateral_mps = obs.lateral_mps;
      estimate_.yaw_rate_radps = obs.yaw_rate_radps;
 odom_estimate_.forward_mps = obs.forward_mps;
 odom_estimate_.lateral_mps = obs.lateral_mps;
 odom_estimate_.yaw_rate_radps = obs.yaw_rate_radps;
    }
    if (obs.imu_valid) {
      estimate_.yaw_rate_radps = obs.imu_gyro_z_radps;
 odom_estimate_.yaw_rate_radps = obs.imu_gyro_z_radps;
    }
    return;
  }
  const double dt_s =
      static_cast<double>(obs.stamp_ns - prior_.stamp_ns) * 1e-9;
 if (!(dt_s > 0.0)) {
 return; // duplicate/late measurement: never rewind the integration clock
 }
 if (dt_s >= 1.0) {
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
 // has not absorbed. Either path trusts the gyro. The disagreement remains
 // available as slip evidence for GripAdapter; it must not be accumulated
 // as absolute-heading error while a valid gyro directly observes yaw rate.
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
 const double lateral = obs.odometry_valid
 ? obs.lateral_mps * motion_scale_ : estimate_.lateral_mps;
 const double half_yaw_step = 0.5 * yaw_rate * dt_s;
 const double sinc = std::abs(half_yaw_step) > 1e-9
 ? std::sin(half_yaw_step) / half_yaw_step : 1.0;
 const double forward_step_m = forward * dt_s * sinc;
 const double lateral_step_m = lateral * dt_s * sinc;
 struct IntegratedStep {
 double delta_x_m = 0.0;
 double delta_y_m = 0.0;
 };
 auto integrate = [&](EstimatedState& state) {
 const double yaw_mid = state.yaw_rad + 0.5 * yaw_rate * dt_s;
 const double c = std::cos(yaw_mid);
 const double s = std::sin(yaw_mid);
 const double delta_x = forward_step_m * c - lateral_step_m * s;
 const double delta_y = forward_step_m * s + lateral_step_m * c;
 state.x_m += delta_x;
 state.y_m += delta_y;
 // Slope z is continuous odometry in both frames. A 2D map correction
 // changes map_T_odom in SE(2) and never resets height or tilt.
    // The estimator's REP-103 pitch convention follows the IMU gravity
    // projection used by SimImu: negative pitch is an uphill nose.  Project
    // forward travel onto world Z with that same convention so a surveyed
    // ramp raises the sensor estimate instead of driving it below ground.
    state.z_m -= forward_step_m * std::sin(state.pitch_rad);
 state.yaw_rad = core::wrap_angle(state.yaw_rad + yaw_rate * dt_s);
 state.forward_mps = forward;
 state.lateral_mps = lateral;
 state.yaw_rate_radps = yaw_rate;
 state.stamp_ns = obs.stamp_ns;
 return IntegratedStep{delta_x, delta_y};
 };
 const IntegratedStep estimate_step = integrate(estimate_);
 const IntegratedStep odom_step = integrate(odom_estimate_);

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
 odom_estimate_.roll_rad = roll;
 odom_estimate_.pitch_rad = pitch;
 odom_estimate_.attitude_valid = true;
  }

 // Covariance grows independently in each REP-105 frame. The full planar
 // Jacobian carries yaw uncertainty into cross-track position and preserves
 // x/y/yaw correlations; process-noise variances add once per increment.
 const double position_process_variance =
 config_.position_process_variance_per_m *
 std::hypot(forward, lateral) * dt_s +
 config_.position_process_variance_per_s * dt_s;
 const double heading_process_variance =
 config_.heading_process_variance_per_s * dt_s;
auto grow_uncertainty = [&](EstimatedState& state,
 PlanarCovariance& covariance, const IntegratedStep& step) {
 propagate_planar_covariance(
 covariance, step.delta_x_m, step.delta_y_m,
 std::max(0.0, position_process_variance));
 covariance[8] += std::max(0.0, heading_process_variance);
 set_heading_sigma(covariance, std::clamp(
 std::sqrt(std::max(0.0, covariance[8])), config_.yaw_sigma_min_rad,
 config_.yaw_sigma_max_rad));
 publish_covariance(covariance, config_, state);
 };
grow_uncertainty(estimate_, estimate_covariance_, estimate_step);
grow_uncertainty(odom_estimate_, odom_covariance_, odom_step);
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
 ensure_odom_origin(stamp_ns);
 const double correction_position_sigma = std::max(1e-9, pos_sigma_m);
 const double correction_position_variance =
 correction_position_sigma * correction_position_sigma;
 const double prior_x_variance = std::max(1e-18, estimate_covariance_[0]);
 const double prior_y_variance = std::max(1e-18, estimate_covariance_[4]);
 const double x_gain = prior_x_variance /
 (prior_x_variance + correction_position_variance);
 const double y_gain = prior_y_variance /
 (prior_y_variance + correction_position_variance);
  // A validated map correction establishes the estimate: kidnapped or cold
  // starts have no trustworthy prior, so the blend target (not the prior)
  // is what makes this valid. Rejected matches never reach here.
  estimate_.valid = true;
 estimate_.x_m += x_gain * (x_m - estimate_.x_m);
 estimate_.y_m += y_gain * (y_m - estimate_.y_m);
 const double correction_heading_sigma = std::max(1e-9, yaw_sigma_rad);
 const double prior_heading_variance =
 std::max(1e-18, estimate_covariance_[8]);
 const double correction_heading_variance =
 correction_heading_sigma * correction_heading_sigma;
 const double heading_gain = prior_heading_variance /
 (prior_heading_variance + correction_heading_variance);
  estimate_.yaw_rad = core::wrap_angle(
 estimate_.yaw_rad + heading_gain *
 core::wrap_angle(yaw_rad - estimate_.yaw_rad));
 const double minimum_position_variance =
 config_.pos_sigma_min_m * config_.pos_sigma_min_m;
 const double minimum_heading_variance =
 config_.yaw_sigma_min_rad * config_.yaw_sigma_min_rad;
 estimate_covariance_ = {
 std::max(minimum_position_variance,
 1.0 / (1.0 / prior_x_variance +
 1.0 / correction_position_variance)), 0.0, 0.0,
 0.0, std::max(minimum_position_variance,
 1.0 / (1.0 / prior_y_variance +
 1.0 / correction_position_variance)), 0.0,
 0.0, 0.0, std::max(minimum_heading_variance,
 1.0 / (1.0 / prior_heading_variance +
 1.0 / correction_heading_variance))};
  estimate_.stamp_ns = stamp_ns;
 publish_covariance(estimate_covariance_, config_, estimate_);
}

void FusedOdometryEstimator::correct_pose_information(
 double x_m, double y_m, double yaw_rad,
 const std::array<double, 9>& pose_information,
 double pos_sigma_m, double yaw_sigma_rad,
 core::MonotonicNs stamp_ns) {
 if (!std::isfinite(x_m) || !std::isfinite(y_m) ||
 !std::isfinite(yaw_rad)) {
 return;
 }
 PlanarCovariance information = pose_information;
 for (double value : information) {
 if (!std::isfinite(value)) return;
 }
 // Measurement information must be symmetric positive semidefinite. The
 // matcher constructs it that way; symmetrize defensively at the interface
 // and fail closed if a diagonal is negative.
 information[1] = information[3] = 0.5 *
 (information[1] + information[3]);
 information[2] = information[6] = 0.5 *
 (information[2] + information[6]);
 information[5] = information[7] = 0.5 *
 (information[5] + information[7]);
 if (information[0] < 0.0 || information[4] < 0.0 ||
 information[8] < 0.0) {
 return;
 }
 const double information_load = information[0] + information[4] +
 information[8];
 if (!(information_load > 1e-12)) return;
 ensure_odom_origin(stamp_ns);
 PlanarCovariance prior_information{};
 if (!invert_3x3(estimate_covariance_, prior_information)) {
 correct_pose(x_m, y_m, yaw_rad, pos_sigma_m, yaw_sigma_rad, stamp_ns);
 return;
 }
 PlanarCovariance posterior_information = prior_information;
 for (std::size_t index = 0; index < posterior_information.size(); ++index) {
 posterior_information[index] += information[index];
 }
 PlanarCovariance posterior_covariance{};
 if (!invert_3x3(posterior_information, posterior_covariance)) return;
 // K = (P^-1 + I)^-1 I. A zero information eigenvalue produces exactly
 // zero gain in that direction, so corridor-axis uncertainty is preserved.
 const PlanarCovariance gain = multiply_3x3(
 posterior_covariance, information);
 const std::array<double, 3> innovation{
 x_m - estimate_.x_m,
 y_m - estimate_.y_m,
 core::wrap_angle(yaw_rad - estimate_.yaw_rad)};
 std::array<double, 3> correction{};
 for (int row = 0; row < 3; ++row) {
 for (int column = 0; column < 3; ++column) {
 correction[static_cast<std::size_t>(row)] +=
 gain[static_cast<std::size_t>(row * 3 + column)] *
 innovation[static_cast<std::size_t>(column)];
 }
 }
 estimate_.valid = true;
 estimate_.x_m += correction[0];
 estimate_.y_m += correction[1];
 estimate_.yaw_rad = core::wrap_angle(estimate_.yaw_rad + correction[2]);
 // Numerical inversion can leave tiny asymmetry. Restore symmetry before
 // publishing and retain the estimator's declared variance floors.
 for (int row = 0; row < 3; ++row) {
 for (int column = row + 1; column < 3; ++column) {
 const double value = 0.5 * (
 posterior_covariance[static_cast<std::size_t>(row * 3 + column)] +
 posterior_covariance[static_cast<std::size_t>(column * 3 + row)]);
 posterior_covariance[static_cast<std::size_t>(row * 3 + column)] = value;
 posterior_covariance[static_cast<std::size_t>(column * 3 + row)] = value;
 }
 }
 posterior_covariance[0] = std::max(
 posterior_covariance[0],
 config_.pos_sigma_min_m * config_.pos_sigma_min_m);
 posterior_covariance[4] = std::max(
 posterior_covariance[4],
 config_.pos_sigma_min_m * config_.pos_sigma_min_m);
 posterior_covariance[8] = std::max(
 posterior_covariance[8],
 config_.yaw_sigma_min_rad * config_.yaw_sigma_min_rad);
 estimate_covariance_ = posterior_covariance;
 estimate_.stamp_ns = stamp_ns;
 publish_covariance(estimate_covariance_, config_, estimate_);
}

void FusedOdometryEstimator::correct_relative_pose(
 double x_m, double y_m, double yaw_rad, double pos_sigma_m,
 double yaw_sigma_rad, core::MonotonicNs stamp_ns) {
 if (!std::isfinite(x_m) || !std::isfinite(y_m) ||
 !std::isfinite(yaw_rad) || !(pos_sigma_m > 0.0) ||
 !(yaw_sigma_rad > 0.0)) {
 return;
 }
 const double position_sigma = estimate_.position_sigma_m;
 ensure_odom_origin(stamp_ns);
 const double heading_sigma = estimate_.heading_sigma_rad;
 const double correction_position_sigma = std::max(1e-9, pos_sigma_m);
 const double correction_heading_sigma = std::max(1e-9, yaw_sigma_rad);
 const double position_gain = position_sigma * position_sigma /
 (position_sigma * position_sigma +
 correction_position_sigma * correction_position_sigma);
 const double heading_gain = heading_sigma * heading_sigma /
 (heading_sigma * heading_sigma +
 correction_heading_sigma * correction_heading_sigma);
 estimate_.valid = true;
 estimate_.x_m += position_gain * (x_m - estimate_.x_m);
 estimate_.y_m += position_gain * (y_m - estimate_.y_m);
 estimate_.yaw_rad = core::wrap_angle(
 estimate_.yaw_rad +
 heading_gain * core::wrap_angle(yaw_rad - estimate_.yaw_rad));
 // The measurement shares the drifted SLAM/keyframe frame with the prior.
 // It improves local consistency but supplies no independent map-frame
 // information, so absolute covariance and slip doubt remain unchanged.
 estimate_.position_sigma_m = position_sigma;
 estimate_.heading_sigma_rad = heading_sigma;
 estimate_.stamp_ns = stamp_ns;
 estimate_.confidence =
 1.0 / (1.0 + estimate_.position_sigma_m + estimate_.heading_sigma_rad);
}

void FusedOdometryEstimator::note_absolute_rejection(
 double dx_m, double dy_m, double dyaw_rad, core::MonotonicNs stamp_ns) {
 // A rejected map correction must never move the estimate. Its innovation is
 // nevertheless a conservative lower bound on the uncertainty that made the
 // innovation gate fail. Add the outer product of that observed residual to
 // the map-frame covariance; this is an evidence-derived update, rather than
 // a retry counter or a scenario-specific multiplier. Keep the continuous
 // odom frame untouched so controllers and external consumers never see a
 // discontinuity from recovery bookkeeping.
 if (!std::isfinite(dx_m) || !std::isfinite(dy_m) ||
 !std::isfinite(dyaw_rad)) {
  return;
 }
 ensure_odom_origin(stamp_ns);
 const double residual[3] = {dx_m, dy_m, dyaw_rad};
 for (int row = 0; row < 3; ++row) {
  for (int column = 0; column < 3; ++column) {
   estimate_covariance_[static_cast<std::size_t>(row * 3 + column)] +=
    residual[row] * residual[column];
  }
 }
 // Preserve symmetry against accumulated floating-point roundoff and the
 // estimator's declared physical floors/caps.
 for (int row = 0; row < 3; ++row) {
  for (int column = row + 1; column < 3; ++column) {
   const double value = 0.5 * (
    estimate_covariance_[static_cast<std::size_t>(row * 3 + column)] +
    estimate_covariance_[static_cast<std::size_t>(column * 3 + row)]);
   estimate_covariance_[static_cast<std::size_t>(row * 3 + column)] = value;
   estimate_covariance_[static_cast<std::size_t>(column * 3 + row)] = value;
  }
 }
 estimate_covariance_[0] = std::clamp(
  estimate_covariance_[0], config_.pos_sigma_min_m * config_.pos_sigma_min_m,
  config_.pos_sigma_max_m * config_.pos_sigma_max_m);
 estimate_covariance_[4] = std::clamp(
  estimate_covariance_[4], config_.pos_sigma_min_m * config_.pos_sigma_min_m,
  config_.pos_sigma_max_m * config_.pos_sigma_max_m);
 estimate_covariance_[8] = std::clamp(
  estimate_covariance_[8], config_.yaw_sigma_min_rad * config_.yaw_sigma_min_rad,
  config_.yaw_sigma_max_rad * config_.yaw_sigma_max_rad);
 estimate_.stamp_ns = stamp_ns;
 publish_covariance(estimate_covariance_, config_, estimate_);
}

void FusedOdometryEstimator::seed_pose(double x_m, double y_m, double yaw_rad,
                                       double pos_sigma_m, double yaw_sigma_rad,
                                       core::MonotonicNs stamp_ns) {
  if (!std::isfinite(x_m) || !std::isfinite(y_m) ||
      !std::isfinite(yaw_rad) || !(pos_sigma_m > 0.0) ||
      !(yaw_sigma_rad > 0.0)) {
    return;  // fail closed on malformed seeds
  }
 // A seed defines map_T_base. The odom origin remains where continuous
 // wheel/IMU integration placed it; on cold start it begins at identity.
 ensure_odom_origin(stamp_ns);
 estimate_.valid = true;
 estimate_.x_m = x_m;
  estimate_.y_m = y_m;
  estimate_.z_m = odom_estimate_.z_m;
  estimate_.yaw_rad = core::wrap_angle(yaw_rad);
  estimate_.forward_mps = odom_estimate_.forward_mps;
 estimate_.lateral_mps = odom_estimate_.lateral_mps;
  estimate_.yaw_rate_radps = odom_estimate_.yaw_rate_radps;
  estimate_.roll_rad = odom_estimate_.roll_rad;
  estimate_.pitch_rad = odom_estimate_.pitch_rad;
  estimate_.attitude_valid = odom_estimate_.attitude_valid;
  estimate_.position_sigma_m =
      std::clamp(pos_sigma_m, config_.pos_sigma_min_m,
                 config_.pos_sigma_max_m);
  estimate_.heading_sigma_rad = std::clamp(
      yaw_sigma_rad, config_.yaw_sigma_min_rad, config_.yaw_sigma_max_rad);
 estimate_covariance_ = diagonal_covariance(
 estimate_.position_sigma_m, estimate_.heading_sigma_rad);
  estimate_.stamp_ns = stamp_ns;
  estimate_.confidence =
      1.0 / (1.0 + estimate_.position_sigma_m + estimate_.heading_sigma_rad);
  has_prior_ = true;
  prior_ = iface::ObservationFrame{};
 prior_.stamp_ns = stamp_ns;
  attitude_ = quaternion_from_euler(
  estimate_.roll_rad, estimate_.pitch_rad, estimate_.yaw_rad);
}

}  // namespace lost_signal::robot::autonomy
