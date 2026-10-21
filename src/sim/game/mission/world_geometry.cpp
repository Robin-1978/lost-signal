#include "world_geometry.hpp"

#include "robot/core/math.hpp"

#include <algorithm>
#include <cmath>

namespace mission {

std::vector<ResolvedWorldCollision> resolve_world_collisions(
 const MapDefinition& map, std::string_view level_id) {
 std::vector<ResolvedWorldCollision> result;
 constexpr double kDegreesToRadians = sim::kPi / 180.0;
 for (const auto& object : map.world_objects) {
 if (!object.active_in.empty() &&
 std::find(object.active_in.begin(), object.active_in.end(), level_id) == object.active_in.end()) {
 continue;
 }
 const double object_yaw = object.yaw_degrees * kDegreesToRadians;
 const double object_cos = std::cos(object_yaw);
 const double object_sin = std::sin(object_yaw);
 for (const auto& collision : object.collisions) {
 const Vec3 scaled_offset{
 collision.position_m.x * object.scale.x,
 collision.position_m.y * object.scale.y,
 collision.position_m.z * object.scale.z,
 };
 const Vec3 center{
 object.position_m.x + object_cos * scaled_offset.x - object_sin * scaled_offset.y,
 object.position_m.y + object_sin * scaled_offset.x + object_cos * scaled_offset.y,
 object.position_m.z + scaled_offset.z,
 };
 const Vec3 size{
 collision.size_m.x * object.scale.x,
 collision.size_m.y * object.scale.y,
 collision.size_m.z * object.scale.z,
 };
 const double yaw = object_yaw + collision.yaw_degrees * kDegreesToRadians;
 const double half_x = size.x * 0.5;
 const double half_y = size.y * 0.5;
 const double extent_x = std::abs(std::cos(yaw)) * half_x + std::abs(std::sin(yaw)) * half_y;
 const double extent_y = std::abs(std::sin(yaw)) * half_x + std::abs(std::cos(yaw)) * half_y;
 result.push_back({
 object.id,
 center,
 size,
 yaw,
 collision.material,
 collision.friction_mu,
 collision.rolling_drag,
 collision.contact_time_constant_s,
 collision.contact_damping_ratio,
 collision.lidar_reflectivity,
 {center.x - extent_x, center.y - extent_y},
 {center.x + extent_x, center.y + extent_y},
 });
 }
 }
 return result;
}

std::vector<WorldBoundaryWall> world_boundary_walls(const MapDefinition& map) {
 constexpr double kWallThicknessM = 0.10;
 const sim::Vec2 size = map.size_m;
 return {
 {{size.x * 0.5, -kWallThicknessM}, {size.x * 0.5 + kWallThicknessM, kWallThicknessM}},
 {{size.x * 0.5, size.y + kWallThicknessM}, {size.x * 0.5 + kWallThicknessM, kWallThicknessM}},
 {{-kWallThicknessM, size.y * 0.5}, {kWallThicknessM, size.y * 0.5 + kWallThicknessM}},
 {{size.x + kWallThicknessM, size.y * 0.5}, {kWallThicknessM, size.y * 0.5 + kWallThicknessM}},
 };
}

} // namespace mission
