#include "robot/autonomy/localization/estimation_system.hpp"
#include "robot/config/motion_policy.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sim {
namespace {

constexpr double kGravityMps2 = 9.81;
constexpr double kAttitudeCorrectionTimeConstantS = 0.50;
constexpr double kPoseVelocityWindowS = 0.10;
constexpr double kPoseVelocityFilterTimeConstantS = 0.25;
constexpr double kMinimumPositionSigmaM = 0.005;
constexpr double kMaximumPositionSigmaM = 100.0;
constexpr double kMinimumHeadingSigmaRad = 0.002;
constexpr double kMaximumHeadingSigmaRad = 3.141592653589793;

} // namespace

EstimationSystem::Quaternion EstimationSystem::quaternion_from_euler(double roll, double pitch,
 double yaw) {
 const double cr = std::cos(roll * 0.5);
 const double sr = std::sin(roll * 0.5);
 const double cp = std::cos(pitch * 0.5);
 const double sp = std::sin(pitch * 0.5);
 const double cy = std::cos(yaw * 0.5);
 const double sy = std::sin(yaw * 0.5);
 return {
 cr * cp * cy + sr * sp * sy,
 sr * cp * cy - cr * sp * sy,
 cr * sp * cy + sr * cp * sy,
 cr * cp * sy - sr * sp * cy,
 };
}

EstimationSystem::Vector3 EstimationSystem::euler_from_quaternion(
 const EstimationSystem::Quaternion& value) {
 const double sin_roll_cos_pitch = 2.0 * (value.w * value.x + value.y * value.z);
 const double cos_roll_cos_pitch = 1.0 - 2.0 * (value.x * value.x + value.y * value.y);
 const double sin_pitch = 2.0 * (value.w * value.y - value.z * value.x);
 const double sin_yaw_cos_pitch = 2.0 * (value.w * value.z + value.x * value.y);
 const double cos_yaw_cos_pitch = 1.0 - 2.0 * (value.y * value.y + value.z * value.z);
 return {
 std::atan2(sin_roll_cos_pitch, cos_roll_cos_pitch),
 std::asin(std::clamp(sin_pitch, -1.0, 1.0)),
 std::atan2(sin_yaw_cos_pitch, cos_yaw_cos_pitch),
 };
}

void EstimationSystem::normalize(EstimationSystem::Quaternion& value) {
 const double norm = std::sqrt(value.w * value.w + value.x * value.x +
 value.y * value.y + value.z * value.z);
 if (!(norm > 1e-12)) {
 value = {};
 return;
 }
 value.w /= norm;
 value.x /= norm;
 value.y /= norm;
 value.z /= norm;
}

void EstimationSystem::reset(double x, double y, double heading_rad, std::uint32_t seed) {
 rng_ = Rng(seed);
 odometry_.reset(x, y, heading_rad);
 map_from_odom_translation_ = {};
 map_from_odom_heading_rad_ = 0.0;
 state_ = {};
 state_.position = {x, y};
 state_.heading_rad = heading_rad;
 state_.position_sigma_m = 0.05;
 state_.heading_sigma_rad = 0.01;
 state_.confidence = 1.0;
 state_.localization_confidence = 1.0;
 state_.localization_valid = true;
 state_.localization_age_s = 0.0;
 attitude_ = {};
 attitude_initialized_ = false;
 velocity_pose_anchor_.reset();
 absolute_velocity_world_mps_ = {};
 absolute_yaw_rate_radps_ = 0.0;
 absolute_velocity_valid_ = false;
 pose_velocity_innovation_samples_ = 0;
 pose_yaw_rate_innovation_samples_ = 0;
}

void EstimationSystem::set_imu_mount_rpy(double roll_in_base_rad,
 double pitch_in_base_rad, double yaw_in_base_rad) {
 if (!std::isfinite(roll_in_base_rad) || !std::isfinite(pitch_in_base_rad) ||
 !std::isfinite(yaw_in_base_rad)) {
 throw std::invalid_argument("IMU mount calibration must be finite");
 }
 imu_mount_roll_rad_ = roll_in_base_rad;
 imu_mount_pitch_rad_ = pitch_in_base_rad;
 imu_mount_yaw_rad_ = yaw_in_base_rad;
}

void EstimationSystem::initialize_attitude(double roll_rad, double pitch_rad) {
 if (!std::isfinite(roll_rad) || !std::isfinite(pitch_rad))
 throw std::invalid_argument("initial attitude must be finite");
 attitude_ = quaternion_from_euler(roll_rad, pitch_rad, 0.0);
 attitude_initialized_ = true;
 state_.roll_rad = roll_rad;
 state_.pitch_rad = pitch_rad;
 state_.attitude_valid = true;
}

EstimationSystem::Vector3 EstimationSystem::imu_to_base(Vector3 value) const {
 // Rz(yaw) * Ry(pitch) * Rx(roll), matching the robot-description mount
 // convention. A vector measured in imu_link is thereby expressed in
 // base_link before it is consumed by localization.
 const double cr = std::cos(imu_mount_roll_rad_);
 const double sr = std::sin(imu_mount_roll_rad_);
 const double cp = std::cos(imu_mount_pitch_rad_);
 const double sp = std::sin(imu_mount_pitch_rad_);
 const double cy = std::cos(imu_mount_yaw_rad_);
 const double sy = std::sin(imu_mount_yaw_rad_);
 return {
 cy * cp * value.x + (cy * sp * sr - sy * cr) * value.y +
 (cy * sp * cr + sy * sr) * value.z,
 sy * cp * value.x + (sy * sp * sr + cy * cr) * value.y +
 (sy * sp * cr - cy * sr) * value.z,
 -sp * value.x + cp * sr * value.y + cp * cr * value.z,
 };
}

void EstimationSystem::update_attitude(const ImuObservation& imu, double dt_s) {
 const Vector3 omega = imu_to_base(
 {imu.roll_rate_radps, imu.pitch_rate_radps, imu.yaw_rate_radps});
 const Vector3 force = imu_to_base(
 {imu.forward_acceleration_mps2, imu.left_acceleration_mps2,
 imu.up_acceleration_mps2});
 const double force_norm = std::sqrt(
 force.x * force.x + force.y * force.y + force.z * force.z);
 const double gravity_tolerance = std::max(
 0.12 * kGravityMps2, 4.0 * imu.acceleration_noise_sigma_mps2);
 const bool gravity_reliable = std::isfinite(force_norm) && force_norm > 1e-6 &&
 std::abs(force_norm - kGravityMps2) <= gravity_tolerance;
 const double gravity_roll = std::atan2(force.y, force.z);
 const double gravity_pitch = std::atan2(-force.x,
 std::hypot(force.y, force.z));

 if (!attitude_initialized_) {
 if (!gravity_reliable)
 return;
 initialize_attitude(gravity_roll, gravity_pitch);
 return;
 }

 // Integrate calibrated body rates as a quaternion. This avoids treating
 // roll/pitch/yaw rates as independent Euler derivatives on steep terrain.
 const Quaternion previous = attitude_;
 attitude_.w += -0.5 * (previous.x * omega.x + previous.y * omega.y +
 previous.z * omega.z) * dt_s;
 attitude_.x += 0.5 * (previous.w * omega.x + previous.y * omega.z -
 previous.z * omega.y) * dt_s;
 attitude_.y += 0.5 * (previous.w * omega.y + previous.z * omega.x -
 previous.x * omega.z) * dt_s;
 attitude_.z += 0.5 * (previous.w * omega.z + previous.x * omega.y -
 previous.y * omega.x) * dt_s;
 normalize(attitude_);

 auto angles = euler_from_quaternion(attitude_);
 if (gravity_reliable) {
 // Accelerometers observe gravity plus linear acceleration. A bounded
 // complementary correction removes gyro drift without turning a one-frame
 // acceleration impulse into an instantaneous chassis tilt.
 const double correction = 1.0 - std::exp(
 -dt_s / kAttitudeCorrectionTimeConstantS);
 angles.x = wrap_angle(angles.x + correction * wrap_angle(gravity_roll - angles.x));
 angles.y += correction * (gravity_pitch - angles.y);
 attitude_ = quaternion_from_euler(angles.x, angles.y, angles.z);
 normalize(attitude_);
 }
 state_.roll_rad = angles.x;
 state_.pitch_rad = angles.y;
 state_.attitude_valid = true;
}

void EstimationSystem::update_map_state() {
 const double cosine = std::cos(map_from_odom_heading_rad_);
 const double sine = std::sin(map_from_odom_heading_rad_);
 state_.position = {
 map_from_odom_translation_.x + cosine * odometry_.p.x - sine * odometry_.p.y,
 map_from_odom_translation_.y + sine * odometry_.p.x + cosine * odometry_.p.y,
 };
 state_.heading_rad = wrap_angle(map_from_odom_heading_rad_ + odometry_.th);
}

void EstimationSystem::update(const ObservationFrame& observation, double dt_s) {
	const auto& policy = motion_policy_;
 if (!observation.encoders) {
  throw std::invalid_argument("estimation update requires encoder observation");
 }
 const auto& encoder = *observation.encoders;
 if (encoder.header.frame_id != "base_link" ||
 std::abs(encoder.header.stamp_s - observation.time_s) > 1e-9) {
 throw std::invalid_argument("encoder measurement header does not match observation frame");
 }
 const bool has_imu = observation.imu.has_value();
 if (has_imu && (observation.imu->header.frame_id != "imu_link" ||
 std::abs(observation.imu->header.stamp_s - observation.time_s) > 1e-9)) {
 throw std::invalid_argument("imu measurement header does not match observation frame");
 }
 const auto calibrated_angular_rate = has_imu
 ? imu_to_base({observation.imu->roll_rate_radps,
 observation.imu->pitch_rate_radps, observation.imu->yaw_rate_radps})
 : Vector3{};
 const double gyro_rate = calibrated_angular_rate.z;
 const double gyro_sigma = has_imu ? observation.imu->noise_sigma_radps : 0.0;
 odometry_.update(encoder.linear_velocity_mps, encoder.lateral_velocity_mps,
 encoder.angular_velocity_radps, dt_s, rng_, encoder.slip_scale,
 0.003 * encoder.slip_scale, has_imu, gyro_rate, gyro_sigma);
 if (has_imu)
 update_attitude(*observation.imu, dt_s);
 // Propagate a bounded diagonal SE(2) covariance approximation. The dead
 // reckoner owns the sampled motion; this is the estimator-owned uncertainty
 // envelope used by recovery policy, so it includes travelled distance,
 // heading uncertainty and provider-reported slip without pretending that
 // simulation truth is a measurement.
 const double safe_dt = std::max(0.0, dt_s);
 const double travelled_m = std::hypot(
 encoder.linear_velocity_mps, encoder.lateral_velocity_mps) * safe_dt;
 const double slip = std::max(1.0, encoder.slip_scale);
 const double distance_noise_m =
 (0.02 * travelled_m + 0.0005 * std::sqrt(safe_dt)) * slip;
 const double heading_increment = std::abs(encoder.angular_velocity_radps) * safe_dt;
 const double heading_noise_rad =
 (0.05 * heading_increment + 0.0008 * std::sqrt(safe_dt)) * slip;
 const double heading_sigma_from_imu = has_imu
 ? std::max(0.0, observation.imu->noise_sigma_radps) * safe_dt : 0.0;
 const double heading_process_sigma = std::hypot(
 heading_noise_rad, heading_sigma_from_imu);
 const double translation_from_heading = travelled_m * state_.heading_sigma_rad;
 const double position_variance = state_.position_sigma_m * state_.position_sigma_m +
 distance_noise_m * distance_noise_m +
 translation_from_heading * translation_from_heading;
 const double heading_variance = state_.heading_sigma_rad * state_.heading_sigma_rad +
 heading_process_sigma * heading_process_sigma;
 state_.position_sigma_m = std::clamp(
 std::sqrt(std::max(0.0, position_variance)),
 kMinimumPositionSigmaM, kMaximumPositionSigmaM);
 state_.heading_sigma_rad = std::clamp(
 std::sqrt(std::max(0.0, heading_variance)),
 kMinimumHeadingSigmaRad, kMaximumHeadingSigmaRad);
 update_map_state();
 state_.linear_velocity_mps = encoder.linear_velocity_mps;
 state_.lateral_velocity_mps = encoder.lateral_velocity_mps;
 // Wheel differential is the low-noise nominal rate. Once the provider marks
 // its uncertainty above the nominal slip scale, it is no longer a body-rate
 // observation (notably for skid steering), so a mounted gyro owns the live
 // angular velocity. Heading propagation above already blends both using
 // their variances; this selection prevents ordinary wheel motion from being
 // made noisier while refusing a known-slipping wheel rate as body truth.
 state_.angular_velocity_radps =
   has_imu && encoder.slip_scale > 2.0 ? gyro_rate : encoder.angular_velocity_radps;
 state_.confidence = std::clamp(1.0 / std::max(1.0, encoder.slip_scale), 0.05, 1.0);
 state_.timestamp_s = observation.time_s;
 state_.localization_age_s = std::max(0.0, state_.localization_age_s + safe_dt);
 const double covariance_quality = 1.0 /
 (1.0 + state_.position_sigma_m / policy.localization_quality.position_half_quality_sigma_m +
 state_.heading_sigma_rad / policy.localization_quality.heading_half_quality_sigma_rad);
 const double quality_evidence = std::clamp(
 std::min(state_.localization_score, state_.localization_overlap), 0.0, 1.0);
 const double age_factor = std::exp(
 -state_.localization_age_s / policy.localization_quality.confidence_decay_time_constant_s);
 state_.localization_confidence = std::clamp(
 (policy.localization_quality.covariance_weight * covariance_quality +
 policy.localization_quality.evidence_weight * quality_evidence) * age_factor,
 0.0, 1.0);
 state_.localization_valid = std::isfinite(state_.position_sigma_m) &&
 std::isfinite(state_.heading_sigma_rad) &&
 state_.position_sigma_m < policy.localization_quality.untrusted_position_sigma_m &&
 state_.heading_sigma_rad < policy.localization_quality.untrusted_heading_sigma_rad;
}

void EstimationSystem::correct(const PoseObservation& observation, double time_s) {
 if (observation.header.frame_id != "map" || std::abs(observation.header.stamp_s - time_s) > 1e-9) {
 throw std::invalid_argument("pose measurement header does not match correction time");
 }
 const double measurement_position_sigma = std::clamp(
 std::isfinite(observation.position_sigma_m) ? observation.position_sigma_m : 1.0,
 kMinimumPositionSigmaM, kMaximumPositionSigmaM);
 const double measurement_heading_sigma = std::clamp(
 std::isfinite(observation.heading_sigma_rad) ? observation.heading_sigma_rad : 1.0,
 kMinimumHeadingSigmaRad, kMaximumHeadingSigmaRad);
 const double position_gain = std::clamp(0.65 / (1.0 + measurement_position_sigma * 4.0), 0.15, 0.65);
 const double heading_gain = std::clamp(0.65 / (1.0 + measurement_heading_sigma * 10.0), 0.15, 0.65);
 const double position_innovation_m = std::hypot(
 observation.position.x - state_.position.x,
 observation.position.y - state_.position.y);
 const double heading_innovation_rad = std::abs(
 wrap_angle(observation.heading_rad - state_.heading_rad));
 const Vec2 corrected_position{
 state_.position.x + (observation.position.x - state_.position.x) * position_gain,
 state_.position.y + (observation.position.y - state_.position.y) * position_gain,
 };
 const double corrected_heading = wrap_angle(
 state_.heading_rad + wrap_angle(observation.heading_rad - state_.heading_rad) * heading_gain);
 std::optional<Vec2> measured_pose_velocity_world_mps;
 std::optional<double> measured_pose_yaw_rate_radps;

 // A pose source also observes body translation independently of wheel
 // rotation. Differentiate over a finite window, then low-pass the result so
 // centimetre-level position noise is not amplified by the 20 ms control
 // tick. This makes fused localization retain observability when locked or
 // slipping wheels report zero speed while the chassis still moves.
 if (!velocity_pose_anchor_) {
 velocity_pose_anchor_ = observation;
 } else {
 const double velocity_dt_s =
 observation.header.stamp_s - velocity_pose_anchor_->header.stamp_s;
 if (velocity_dt_s >= kPoseVelocityWindowS - 1e-12) {
 const Vec2 measured_world_velocity{
 (observation.position.x - velocity_pose_anchor_->position.x) / velocity_dt_s,
 (observation.position.y - velocity_pose_anchor_->position.y) / velocity_dt_s,
 };
 const double measured_yaw_rate = wrap_angle(
 observation.heading_rad - velocity_pose_anchor_->heading_rad) / velocity_dt_s;
 measured_pose_velocity_world_mps = measured_world_velocity;
 measured_pose_yaw_rate_radps = measured_yaw_rate;
 const double blend = absolute_velocity_valid_
 ? 1.0 - std::exp(-velocity_dt_s / kPoseVelocityFilterTimeConstantS)
 : 1.0;
 absolute_velocity_world_mps_.x +=
 blend * (measured_world_velocity.x - absolute_velocity_world_mps_.x);
 absolute_velocity_world_mps_.y +=
 blend * (measured_world_velocity.y - absolute_velocity_world_mps_.y);
 absolute_yaw_rate_radps_ +=
 blend * (measured_yaw_rate - absolute_yaw_rate_radps_);
 absolute_velocity_valid_ = true;
 velocity_pose_anchor_ = observation;
 }
 }
 // Keep odom -> base_footprint continuous. Global corrections move map ->
 // odom, matching the standard SLAM/localization TF contract.
 map_from_odom_heading_rad_ = wrap_angle(corrected_heading - odometry_.th);
 const double cosine = std::cos(map_from_odom_heading_rad_);
 const double sine = std::sin(map_from_odom_heading_rad_);
 map_from_odom_translation_ = {
 corrected_position.x - (cosine * odometry_.p.x - sine * odometry_.p.y),
 corrected_position.y - (sine * odometry_.p.x + cosine * odometry_.p.y),
 };
 update_map_state();
 // Fuse the accepted map observation into the estimator's uncertainty. The
 // pose update above deliberately retains its bounded tracking gain; the
 // covariance update is conservative and never reports more confidence than
 // the measurement can support.
 const double prior_position_variance = state_.position_sigma_m * state_.position_sigma_m;
 const double prior_heading_variance = state_.heading_sigma_rad * state_.heading_sigma_rad;
 const double measurement_position_variance = measurement_position_sigma * measurement_position_sigma;
 const double measurement_heading_variance = measurement_heading_sigma * measurement_heading_sigma;
 state_.position_sigma_m = std::clamp(
 std::max(std::sqrt((prior_position_variance * measurement_position_variance) /
 (prior_position_variance + measurement_position_variance)),
 position_innovation_m * 1.5),
 kMinimumPositionSigmaM, kMaximumPositionSigmaM);
 state_.heading_sigma_rad = std::clamp(
 std::max(std::sqrt((prior_heading_variance * measurement_heading_variance) /
 (prior_heading_variance + measurement_heading_variance)),
 heading_innovation_rad * 1.5),
 kMinimumHeadingSigmaRad, kMaximumHeadingSigmaRad);
 state_.localization_age_s = 0.0;
 state_.localization_score = observation.quality_score;
 state_.localization_overlap = observation.quality_overlap;
 state_.localization_score_margin = observation.quality_margin;
 const double quality_evidence = std::clamp(
 std::min(observation.quality_score, observation.quality_overlap), 0.0, 1.0);
	const auto& policy = motion_policy_;
 const double covariance_quality = 1.0 /
 (1.0 + state_.position_sigma_m / policy.localization_quality.position_half_quality_sigma_m +
 state_.heading_sigma_rad / policy.localization_quality.heading_half_quality_sigma_rad);
 state_.localization_confidence = std::clamp(
 policy.localization_quality.covariance_weight * covariance_quality +
 policy.localization_quality.evidence_weight * quality_evidence, 0.0, 1.0);
 state_.localization_valid = true;
 if (absolute_velocity_valid_ && measured_pose_velocity_world_mps &&
 measured_pose_yaw_rate_radps) {
 const double cosine_heading = std::cos(corrected_heading);
 const double sine_heading = std::sin(corrected_heading);
 const double pose_forward_mps =
 cosine_heading * absolute_velocity_world_mps_.x +
 sine_heading * absolute_velocity_world_mps_.y;
 const double pose_lateral_mps =
 -sine_heading * absolute_velocity_world_mps_.x +
 cosine_heading * absolute_velocity_world_mps_.y;
 // Weight a centimetre-accuracy pose source strongly enough to expose gross
 // wheel slip while retaining the smoother encoder estimate during ordinary
 // rolling. The measurement sigma remains part of the contract.
 const double nominal_velocity_gain = std::clamp(
 0.8 / (1.0 + 20.0 * observation.position_sigma_m), 0.25, 0.8);
 const double velocity_innovation_mps = std::hypot(
 cosine_heading * measured_pose_velocity_world_mps->x +
 sine_heading * measured_pose_velocity_world_mps->y -
 state_.linear_velocity_mps,
 -sine_heading * measured_pose_velocity_world_mps->x +
 cosine_heading * measured_pose_velocity_world_mps->y -
 state_.lateral_velocity_mps);
 const double velocity_innovation_threshold_mps = std::max(
 0.25, 3.0 * std::sqrt(2.0) * observation.position_sigma_m /
 kPoseVelocityWindowS);
 const double velocity_gain =
 velocity_innovation_mps > velocity_innovation_threshold_mps &&
 ++pose_velocity_innovation_samples_ >= 2
 ? nominal_velocity_gain : 0.0;
 if (velocity_innovation_mps <= velocity_innovation_threshold_mps)
 pose_velocity_innovation_samples_ = 0;
 state_.linear_velocity_mps +=
 velocity_gain * (pose_forward_mps - state_.linear_velocity_mps);
 state_.lateral_velocity_mps +=
 velocity_gain * (pose_lateral_mps - state_.lateral_velocity_mps);
 const double nominal_yaw_rate_gain = std::clamp(
 0.8 / (1.0 + 20.0 * observation.heading_sigma_rad), 0.25, 0.8);
 const double yaw_rate_innovation_threshold_radps = std::max(
 0.25, 3.0 * std::sqrt(2.0) * observation.heading_sigma_rad /
 kPoseVelocityWindowS);
 const double yaw_rate_gain =
 std::abs(*measured_pose_yaw_rate_radps - state_.angular_velocity_radps) >
 yaw_rate_innovation_threshold_radps &&
 ++pose_yaw_rate_innovation_samples_ >= 2
 ? nominal_yaw_rate_gain : 0.0;
 if (std::abs(*measured_pose_yaw_rate_radps - state_.angular_velocity_radps) <=
 yaw_rate_innovation_threshold_radps) {
 pose_yaw_rate_innovation_samples_ = 0;
 }
 state_.angular_velocity_radps +=
 yaw_rate_gain * (absolute_yaw_rate_radps_ - state_.angular_velocity_radps);
 }
 state_.confidence = std::max(state_.confidence, 0.9);
 state_.timestamp_s = time_s;
}

} // namespace sim
