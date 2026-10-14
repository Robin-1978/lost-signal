#include "sensor.hpp"

#include <algorithm>
#include <cmath>

namespace sim {
namespace {

double ray_circle_distance(Vec2 origin, double direction_x, double direction_y, const DynState& object,
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

Scan lidar_scan(const GridMap& map, Vec2 position, double heading, int n_rays, double max_range,
                double noise_std, Rng& rng, const std::vector<DynState>& dynamic_objects) {
 Scan scan;
 scan.n_rays = n_rays;
 scan.max_range = max_range;
 scan.ranges.reserve(n_rays);
 scan.hit.reserve(n_rays);
 scan.dynamic_hit.reserve(n_rays);
 const double step = map.res() * 0.5;
 for (int ray_index = 0; ray_index < n_rays; ++ray_index) {
  const double angle = heading + 2 * kPi * ray_index / n_rays;
  const double direction_x = std::cos(angle);
  const double direction_y = std::sin(angle);
  double range = max_range;
  bool hit = false;
  int dynamic_index = -1;
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
    dynamic_index = static_cast<int>(object_index);
   }
  }
  range += rng.gauss(noise_std);
  range = std::clamp(range, 0.0, max_range);
  scan.ranges.push_back(range);
  scan.hit.push_back(hit ? 1 : 0);
  scan.dynamic_hit.push_back(dynamic_index);
 }
 return scan;
}

void DeadReckon::update(double velocity, double angular_velocity, double dt, Rng& rng, double slip_scale,
                        double yaw_bias, bool has_gyro, double gyro_rate, double gyro_sigma) {
 const double distance = velocity * dt;
 const double distance_sigma = 0.02 * std::abs(distance) + 0.0005;
 const double noisy_distance = distance + rng.gauss(distance_sigma) * slip_scale;
 // Complementary heading, weighted by slip: wheels are excellent on grip and
 // lie when sliding, the gyro is mediocre everywhere. Trust each source where
 // it deserves it, like any real fusion filter does with covariances.
 // Authority cap 0.5: even on ice the wheels keep half the vote, so soft-tire
 // evidence is never fully masked by the gyro. L5-recommended (grip, low slip)
 // never engages the gyro at all and is unaffected by this number.
 double gyro_weight = 0.0;
 if (has_gyro) {
  gyro_weight = std::clamp((slip_scale - 1.0) * 0.5, 0.0, 0.8);
 }
 const double heading_delta = angular_velocity * dt;
 const double heading_sigma = 0.05 * std::abs(heading_delta) + 0.0008;
 const double wheel_part = heading_delta + rng.gauss(heading_sigma) * slip_scale + yaw_bias * distance;
 // Zero weight draws nothing: gyro-less runs reproduce the old stream bit-for-bit.
 double gyro_part = 0.0;
 if (gyro_weight > 0.0) {
  gyro_part = gyro_rate * dt + rng.gauss(gyro_sigma) * std::sqrt(dt);
 }
 th += (1.0 - gyro_weight) * wheel_part + gyro_weight * gyro_part;
 p.x += noisy_distance * std::cos(th);
 p.y += noisy_distance * std::sin(th);
}

double DeadReckon::driftErr(Vec2 true_position) const {
 return std::hypot(true_position.x - p.x, true_position.y - p.y);
}

} // namespace sim
