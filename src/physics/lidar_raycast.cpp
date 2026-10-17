#include "physics/lidar_raycast.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sim {
namespace {

double ray_circle_distance(Vec2 origin, double direction_x, double direction_y,
 const DynamicObjectState& object, double max_range_m) {
 const double offset_x = object.p.x - origin.x;
 const double offset_y = object.p.y - origin.y;
 const double along = offset_x * direction_x + offset_y * direction_y;
 const double center_distance_sq = offset_x * offset_x + offset_y * offset_y;
 const double perpendicular_sq = center_distance_sq - along * along;
 const double radius_sq = object.radius * object.radius;
 if (along < 0.0 || perpendicular_sq > radius_sq)
 return max_range_m;
 const double entry = along - std::sqrt(std::max(0.0, radius_sq - perpendicular_sq));
 if (entry < 0.0)
 return 0.0;
 return entry < max_range_m ? entry : max_range_m;
}

} // namespace

Scan planar_lidar_raycast(const GridMap& map, Vec2 position, double heading,
 int ray_count, double max_range_m,
 const std::vector<DynamicObjectState>& dynamic_objects) {
 if (ray_count <= 0 || max_range_m <= 0.0)
 throw std::invalid_argument("invalid lidar raycast configuration");

 Scan scan;
 scan.n_rays = ray_count;
 scan.max_range = max_range_m;
 scan.angle_increment_rad = 2.0 * kPi / static_cast<double>(ray_count);
 scan.ranges.reserve(ray_count);
 scan.hit.reserve(ray_count);
 scan.intensities.reserve(ray_count);
 const double step = map.res() * 0.5;
 for (int ray_index = 0; ray_index < ray_count; ++ray_index) {
 const double angle = heading + 2.0 * kPi * ray_index / ray_count;
 const double direction_x = std::cos(angle);
 const double direction_y = std::sin(angle);
 double range = max_range_m;
 bool hit = false;
 for (double distance = step; distance < max_range_m; distance += step) {
 const auto [grid_x, grid_y] = map.worldToGrid(
 position.x + direction_x * distance,
 position.y + direction_y * distance);
 if (!map.inside(grid_x, grid_y)) {
 range = max_range_m;
 break;
 }
 if (!map.isFreeCell(grid_x, grid_y)) {
 range = distance;
 hit = true;
 break;
 }
 }
 for (const auto& object : dynamic_objects) {
 const double object_range = ray_circle_distance(
 position, direction_x, direction_y, object, max_range_m);
 if (object_range < range) {
 range = object_range;
 hit = true;
 }
 }
 range = std::clamp(range, 0.0, max_range_m);
 scan.ranges.push_back(range);
 scan.hit.push_back(hit ? 1 : 0);
 scan.intensities.push_back(hit ? 0.55 / (1.0 + 0.015 * range * range) : 0.0);
 }
 return scan;
}

LidarRaycastTruth planar_lidar_truth(const GridMap& map, Vec2 position,
 double heading, int ray_count, double max_range_m,
 const std::vector<DynamicObjectState>& dynamic_objects) {
 return {
 planar_lidar_raycast(map, position, heading, ray_count, max_range_m,
 dynamic_objects),
 planar_lidar_raycast(map, position, heading, ray_count, max_range_m),
 };
}

} // namespace sim
