#include "autonomy/localization/estimation_system.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sim {

void EstimationSystem::reset(double x, double y, double heading_rad, std::uint32_t seed) {
 rng_ = Rng(seed);
 odometry_.reset(x, y, heading_rad);
 map_from_odom_translation_ = {};
 map_from_odom_heading_rad_ = 0.0;
 state_ = {{x, y}, heading_rad, 0.0, 0.0, 1.0, 0.0};
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
 const double gyro_rate = has_imu ? observation.imu->yaw_rate_radps : 0.0;
 const double gyro_sigma = has_imu ? observation.imu->noise_sigma_radps : 0.0;
 odometry_.update(encoder.linear_velocity_mps, encoder.angular_velocity_radps, dt_s, rng_, encoder.slip_scale,
                  0.003 * encoder.slip_scale, has_imu, gyro_rate, gyro_sigma);
 update_map_state();
 state_.linear_velocity_mps = encoder.linear_velocity_mps;
 state_.angular_velocity_radps = encoder.angular_velocity_radps;
 state_.confidence = std::clamp(1.0 / std::max(1.0, encoder.slip_scale), 0.05, 1.0);
 state_.timestamp_s = observation.time_s;
}

void EstimationSystem::correct(const PoseObservation& observation, double time_s) {
 if (observation.header.frame_id != "map" || std::abs(observation.header.stamp_s - time_s) > 1e-9) {
 throw std::invalid_argument("pose measurement header does not match correction time");
 }
 const double position_gain = std::clamp(0.65 / (1.0 + observation.position_sigma_m * 4.0), 0.15, 0.65);
 const double heading_gain = std::clamp(0.65 / (1.0 + observation.heading_sigma_rad * 10.0), 0.15, 0.65);
 const Vec2 corrected_position{
 state_.position.x + (observation.position.x - state_.position.x) * position_gain,
 state_.position.y + (observation.position.y - state_.position.y) * position_gain,
 };
 const double corrected_heading = wrap_angle(
 state_.heading_rad + wrap_angle(observation.heading_rad - state_.heading_rad) * heading_gain);
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
 state_.confidence = std::max(state_.confidence, 0.9);
 state_.timestamp_s = time_s;
}

} // namespace sim
