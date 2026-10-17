#include "robot/autonomy/localization/dead_reckon.hpp"

#include <algorithm>
#include <cmath>

namespace sim {

void DeadReckon::update(double forward_velocity, double lateral_velocity,
 double angular_velocity, double dt,
 Rng& rng, double slip_scale, double yaw_bias, bool has_gyro,
 double gyro_rate, double gyro_sigma) {
 const double forward_distance = forward_velocity * dt;
 const double forward_sigma = 0.02 * std::abs(forward_distance) + 0.0005;
 const double noisy_forward_distance =
 forward_distance + rng.gauss(forward_sigma) * slip_scale;
 double noisy_lateral_distance = 0.0;
 if (std::abs(lateral_velocity) > 1e-12) {
 const double lateral_distance = lateral_velocity * dt;
 const double lateral_sigma = 0.02 * std::abs(lateral_distance) + 0.0005;
 noisy_lateral_distance = lateral_distance + rng.gauss(lateral_sigma) * slip_scale;
 }
 double gyro_weight = 0.0;
 if (has_gyro) {
 const double wheel_increment_sigma =
 (0.05 * std::abs(angular_velocity * dt) + 0.0008) *
 std::max(0.0, slip_scale);
 const double gyro_increment_sigma = std::max(0.0, gyro_sigma) * dt;
 const double wheel_variance = wheel_increment_sigma * wheel_increment_sigma;
 const double gyro_variance = gyro_increment_sigma * gyro_increment_sigma;
 const double total_variance = wheel_variance + gyro_variance;
 gyro_weight = total_variance > 0.0 ? wheel_variance / total_variance : 0.0;
 }
 const double heading_delta = angular_velocity * dt;
 const double heading_sigma = 0.05 * std::abs(heading_delta) + 0.0008;
 const double wheel_part = heading_delta + rng.gauss(heading_sigma) * slip_scale +
 yaw_bias * forward_distance;
 const double gyro_part = gyro_rate * dt;
 th += (1.0 - gyro_weight) * wheel_part + gyro_weight * gyro_part;
 p.x += noisy_forward_distance * std::cos(th) - noisy_lateral_distance * std::sin(th);
 p.y += noisy_forward_distance * std::sin(th) + noisy_lateral_distance * std::cos(th);
}

double DeadReckon::driftErr(Vec2 true_position) const {
 return std::hypot(true_position.x - p.x, true_position.y - p.y);
}

} // namespace sim
