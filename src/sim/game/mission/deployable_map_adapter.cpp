#include "sim/game/mission/deployable_map_adapter.hpp"
#include "sim/game/mission/world_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <string>

namespace mission {
namespace {

bool active(const std::vector<std::string>& levels, std::string_view level_id) {
 return levels.empty() || std::find(levels.begin(), levels.end(), level_id) != levels.end();
}

bool contains(sim::Vec2 point, sim::Vec2 min, sim::Vec2 max) {
 return point.x >= min.x && point.x < max.x && point.y >= min.y && point.y < max.y;
}

bool overlaps_cell(const ResolvedWorldCollision& collision,
 double cell_min_x, double cell_min_y, double cell_max_x, double cell_max_y) {
 const double cell_half_x = (cell_max_x - cell_min_x) * 0.5;
 const double cell_half_y = (cell_max_y - cell_min_y) * 0.5;
 const double dx = collision.center_m.x - (cell_min_x + cell_half_x);
 const double dy = collision.center_m.y - (cell_min_y + cell_half_y);
 const double box_half_x = collision.size_m.x * 0.5;
 const double box_half_y = collision.size_m.y * 0.5;
 const double cosine = std::cos(collision.yaw_rad);
 const double sine = std::sin(collision.yaw_rad);

 // Separating-axis test for an oriented collision box and an axis-aligned
 // grid cell. Rasterizing the full world AABB would close valid passages
 // around rotated rails and props.
 if (std::abs(dx) > cell_half_x + std::abs(cosine) * box_half_x +
 std::abs(sine) * box_half_y ||
 std::abs(dy) > cell_half_y + std::abs(sine) * box_half_x +
 std::abs(cosine) * box_half_y) {
 return false;
 }
 const double local_x = cosine * dx + sine * dy;
 const double local_y = -sine * dx + cosine * dy;
 return std::abs(local_x) <= box_half_x + std::abs(cosine) * cell_half_x +
 std::abs(sine) * cell_half_y &&
 std::abs(local_y) <= box_half_y + std::abs(sine) * cell_half_x +
 std::abs(cosine) * cell_half_y;
}

} // namespace

sim::runtime::MapSnapshot make_deployable_map_snapshot(
 const MapDefinition& map, std::string_view level_id,
 sim::runtime::TimePoint sample_time, std::uint64_t revision) {
 sim::runtime::MapSnapshot result;
 result.map_id = map.id + "/" + std::string(level_id);
 result.revision = revision;
 result.sample_time = sample_time;
 result.provenance = {"authored", "lost-signal.map-editor", map.id};
 auto& grid = result.grid.emplace();
 grid.width = static_cast<std::uint32_t>(std::ceil(map.size_m.x / map.grid_resolution_m));
 grid.height = static_cast<std::uint32_t>(std::ceil(map.size_m.y / map.grid_resolution_m));
 grid.resolution_m = map.grid_resolution_m;
 const auto cell_count = static_cast<std::size_t>(grid.width) * grid.height;
 grid.occupancy.assign(cell_count, 0);
 grid.elevation_m.assign(cell_count, 0.0F);
 grid.surface_class.assign(cell_count, 0);
 grid.surface_classes = {"default"};
 const auto world_collisions = resolve_world_collisions(map, level_id);

 std::map<std::string, std::uint16_t> surface_indices{{"default", 0}};
 for (const auto& surface : map.surfaces) {
 if (!active(surface.active_in, level_id) || surface_indices.count(surface.material))
 continue;
 const auto index = static_cast<std::uint16_t>(grid.surface_classes.size());
 surface_indices.emplace(surface.material, index);
 grid.surface_classes.push_back(surface.material);
 }

 for (std::uint32_t y = 0; y < grid.height; ++y) {
 for (std::uint32_t x = 0; x < grid.width; ++x) {
 const auto index = static_cast<std::size_t>(y) * grid.width + x;
 const auto centre = sim::runtime::grid_cell_center(grid, {x, y});
 const double cell_min_x = static_cast<double>(x) * grid.resolution_m;
 const double cell_min_y = static_cast<double>(y) * grid.resolution_m;
 const double cell_max_x = cell_min_x + grid.resolution_m;
 const double cell_max_y = cell_min_y + grid.resolution_m;
 for (const auto& obstacle : map.obstacles) {
 const bool overlaps = cell_min_x < obstacle.max.x && cell_max_x > obstacle.min.x &&
 cell_min_y < obstacle.max.y && cell_max_y > obstacle.min.y;
 if (active(obstacle.active_in, level_id) && overlaps) {
 grid.occupancy[index] = 100;
 break;
 }
 }
 if (grid.occupancy[index] < 50) {
 for (const auto& collision : world_collisions) {
 if (overlaps_cell(collision, cell_min_x, cell_min_y, cell_max_x, cell_max_y)) {
 grid.occupancy[index] = 100;
 break;
 }
 }
 }
 for (const auto& surface : map.surfaces) {
 if (active(surface.active_in, level_id) && contains(centre, surface.min, surface.max)) {
 grid.surface_class[index] = surface_indices.at(surface.material);
 break;
 }
 }
 for (auto patch = map.terrain.rbegin(); patch != map.terrain.rend(); ++patch) {
 if (!active(patch->active_in, level_id) || !contains(centre, patch->min, patch->max))
 continue;
 grid.elevation_m[index] = static_cast<float>(
 terrain_patch_height(*patch, centre.x, centre.y));
 break;
 }
 }
 }

 for (const auto& [id, pose] : map.markers)
 result.places.emplace(id, sim::runtime::SemanticPlace{{pose.x, pose.y}, pose.theta});

 for (const auto& zone : map.conflict_zones) {
 if (!active(zone.active_in, level_id))
 continue;
 sim::runtime::SemanticFeature feature;
 feature.id = zone.id;
 feature.kind = "conflict_zone";
 feature.geometry = sim::runtime::SemanticGeometry::Polygon;
 feature.vertices_m = {
 zone.min,
 {zone.max.x, zone.min.y},
 zone.max,
 {zone.min.x, zone.max.y},
 };
 feature.attributes.emplace("authority", zone.authority);
 result.features.push_back(std::move(feature));
 for (std::size_t index = 0; index < zone.observation_approaches.size(); ++index) {
 const auto& approach = zone.observation_approaches[index];
 sim::runtime::SemanticFeature approach_feature;
 approach_feature.id = zone.id + ".approach." + std::to_string(index);
 approach_feature.kind = "conflict_approach";
 approach_feature.geometry = sim::runtime::SemanticGeometry::Polyline;
 approach_feature.vertices_m = approach.centerline;
 approach_feature.attributes.emplace("conflict_zone_id", zone.id);
 approach_feature.attributes.emplace(
 "half_width_m", std::to_string(approach.half_width_m));
 result.features.push_back(std::move(approach_feature));
 }
 }
 return result;
}

} // namespace mission
