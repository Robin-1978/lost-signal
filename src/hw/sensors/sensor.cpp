#include "sensor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace sim {
namespace {

double ray_circle_distance(Vec2 origin, double direction_x, double direction_y, const DynamicObjectState& object,
                           double max_range) {
 const double offset_x = object.p.x - origin.x;
 const double offset_y = object.p.y - origin.y;
 const double along = offset_x * direction_x + offset_y * direction_y;
 const double center_distance_sq = offset_x * offset_x + offset_y * offset_y;
 const double perpendicular_sq = center_distance_sq - along * along;
 const double radius_sq = object.radius * object.radius;
 if (along < 0.0 || perpendicular_sq > radius_sq)
  return max_range;
 const double entry = along - std::sqrt(std::max(0.0, radius_sq - perpendicular_sq));
 if (entry < 0.0)
  return 0.0;
 return entry < max_range ? entry : max_range;
}

} // namespace

Scan apply_lidar_noise(Scan ideal_scan, const LidarNoiseModel& model, Rng& rng) {
 if (model.range_stddev_m < 0.0 || model.range_resolution_m < 0.0 ||
  model.dropout_probability < 0.0 || model.dropout_probability > 1.0 ||
 !std::isfinite(model.range_bias_m) ||
  ideal_scan.ranges.size() != ideal_scan.hit.size() ||
 (!ideal_scan.intensities.empty() && ideal_scan.intensities.size() != ideal_scan.ranges.size()) ||
 ideal_scan.ranges.size() != static_cast<std::size_t>(ideal_scan.n_rays) ||
 ideal_scan.n_rays < 0 || ideal_scan.max_range <= 0.0) {
 throw std::invalid_argument("invalid ideal lidar scan");
 }
 for (std::size_t index = 0; index < ideal_scan.ranges.size(); ++index) {
 double& range = ideal_scan.ranges[index];
 if (!ideal_scan.hit[index]) {
 range = ideal_scan.max_range;
 continue;
 }
  if (model.dropout_probability > 0.0 && rng.uniform01() < model.dropout_probability) {
   ideal_scan.hit[index] = 0;
   range = ideal_scan.max_range;
 if (!ideal_scan.intensities.empty()) {
 ideal_scan.intensities[index] = 0.0;
 }
   continue;
  }
 range += model.range_bias_m + rng.gauss(model.range_stddev_m);
  if (model.range_resolution_m > 0.0) {
   range = std::round(range / model.range_resolution_m) * model.range_resolution_m;
  }
 // A measurement driven outside the device interval is a no-return, not a
 // valid obstacle invented exactly at the range boundary. The minimum-range
 // blind zone is applied by SensorSystem because it belongs to the selected
 // device profile; this layer owns the universal maximum-range behavior.
 if (range < 0.0 || range >= ideal_scan.max_range) {
 ideal_scan.hit[index] = 0;
 range = ideal_scan.max_range;
 if (!ideal_scan.intensities.empty())
 ideal_scan.intensities[index] = 0.0;
 }
 }
 return ideal_scan;
}

Scan apply_lidar_noise(Scan ideal_scan, double noise_std, Rng& rng) {
 return apply_lidar_noise(std::move(ideal_scan), {noise_std, 0.0, 0.0, 0.0}, rng);
}

Scan lidar_raycast(const GridMap& map, Vec2 position, double heading, int n_rays,
 double max_range, const std::vector<DynamicObjectState>& dynamic_objects) {
 if (n_rays <= 0 || max_range <= 0.0) {
  throw std::invalid_argument("invalid lidar raycast configuration");
 }
 Scan scan;
 scan.n_rays = n_rays;
 scan.max_range = max_range;
 scan.angle_increment_rad = 2.0 * kPi / static_cast<double>(n_rays);
 scan.ranges.reserve(n_rays);
 scan.hit.reserve(n_rays);
 scan.intensities.reserve(n_rays);
 const double step = map.res() * 0.5;
 for (int ray_index = 0; ray_index < n_rays; ++ray_index) {
  const double angle = heading + 2 * kPi * ray_index / n_rays;
  const double direction_x = std::cos(angle);
  const double direction_y = std::sin(angle);
  double range = max_range;
  bool hit = false;
  for (double distance = step; distance < max_range; distance += step) {
   const double x = position.x + direction_x * distance;
   const double y = position.y + direction_y * distance;
   const auto [grid_x, grid_y] = map.worldToGrid(x, y);
   if (!map.inside(grid_x, grid_y)) {
    range = max_range;
    break;
   }
   if (!map.isFreeCell(grid_x, grid_y)) {
    range = distance;
    hit = true;
    break;
   }
  }
  for (std::size_t object_index = 0; object_index < dynamic_objects.size(); ++object_index) {
   const double object_range =
     ray_circle_distance(position, direction_x, direction_y, dynamic_objects[object_index], max_range);
   if (object_range < range) {
    range = object_range;
    hit = true;
   }
  }
  range = std::clamp(range, 0.0, max_range);
  scan.ranges.push_back(range);
  scan.hit.push_back(hit ? 1 : 0);
 // The grid backend has no surface normal or optical material. Keep a
 // conservative distance-only first-return model for deterministic Domain
 // comparisons; Bullet supplies incidence-aware strength in production.
 scan.intensities.push_back(hit ? 0.55 / (1.0 + 0.015 * range * range) : 0.0);
 }
 return scan;
}

Scan lidar_scan(const GridMap& map, Vec2 position, double heading, int n_rays, double max_range,
 double noise_std, Rng& rng, const std::vector<DynamicObjectState>& dynamic_objects) {
 return apply_lidar_noise(
  lidar_raycast(map, position, heading, n_rays, max_range, dynamic_objects), noise_std, rng);
}

void DeadReckon::update(double velocity, double angular_velocity, double dt, Rng& rng, double slip_scale,
                        double yaw_bias, bool has_gyro, double gyro_rate, double gyro_sigma) {
 const double distance = velocity * dt;
 const double distance_sigma = 0.02 * std::abs(distance) + 0.0005;
 const double noisy_distance = distance + rng.gauss(distance_sigma) * slip_scale;
 // Complementary heading, weighted by the variance of each increment. Wheel
 // heading uncertainty grows with the contact/slip scale. Gyro rate noise is
 // converted to an increment uncertainty over this sample. The IMU hardware
 // model has already applied bias, random walk and white noise to gyro_rate;
 // sampling gyro_sigma again here would count the same sensor noise twice.
 double gyro_weight = 0.0;
 if (has_gyro) {
 const double wheel_increment_sigma =
 (0.05 * std::abs(angular_velocity * dt) + 0.0008) * std::max(0.0, slip_scale);
 const double gyro_increment_sigma = std::max(0.0, gyro_sigma) * dt;
 const double wheel_variance = wheel_increment_sigma * wheel_increment_sigma;
 const double gyro_variance = gyro_increment_sigma * gyro_increment_sigma;
 const double total_variance = wheel_variance + gyro_variance;
 gyro_weight = total_variance > 0.0 ? wheel_variance / total_variance : 0.0;
 }
 const double heading_delta = angular_velocity * dt;
 const double heading_sigma = 0.05 * std::abs(heading_delta) + 0.0008;
 const double wheel_part = heading_delta + rng.gauss(heading_sigma) * slip_scale + yaw_bias * distance;
 const double gyro_part = gyro_rate * dt;
 th += (1.0 - gyro_weight) * wheel_part + gyro_weight * gyro_part;
 p.x += noisy_distance * std::cos(th);
 p.y += noisy_distance * std::sin(th);
}

double DeadReckon::driftErr(Vec2 true_position) const {
 return std::hypot(true_position.x - p.x, true_position.y - p.y);
}

} // namespace sim
