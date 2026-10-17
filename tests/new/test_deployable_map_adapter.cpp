#include "mission/deployable_map_adapter.hpp"

#include <cmath>
#include <iostream>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

int main() {
 mission::MapDefinition source;
 source.id = "ramp-fixture";
 source.size_m = {4.0, 2.0};
 source.grid_resolution_m = 0.5;
 source.obstacles.push_back({"thin-wall", {1.01, 0.0}, {1.08, 0.5}, {"ramp-test"}});
 source.surfaces.push_back({
 "steel", {0.0, 0.0}, {2.0, 2.0}, "steel_deck", {"ramp-test"}, "Steel",
 });
 source.terrain.push_back({
 "ramp", {0.0, 0.0}, {2.0, 1.0}, 0.2, 0.25, -0.10, 0.0, {"ramp-test"},
 });
 source.terrain.push_back({
 "inactive-ramp", {0.0, 0.0}, {2.0, 1.0}, 9.0, 0.0, 0.0, 0.0, {"other-level"},
 });
 source.markers.emplace("dock", mission::Pose{0.5, 1.5, 0.75});
 source.conflict_zones.push_back({
 "crossing", {2.0, 0.0}, {3.0, 1.0}, "control", {"ramp-test"},
 });

 const auto map = mission::make_deployable_map_snapshot(source, "ramp-test");
 std::string reason;
 CHECK(sim::runtime::valid_map_snapshot(map, &reason));
 CHECK(map.provenance.source_kind == "authored");
 CHECK(map.grid->width == 8 && map.grid->height == 4);
 CHECK(map.grid->elevation_m.size() == 32);
 const auto cell = sim::runtime::world_to_grid(*map.grid, {0.75, 0.25});
 CHECK(cell);
 const auto centre = sim::runtime::grid_cell_center(*map.grid, *cell);
 const double expected_height = 0.2 + 0.25 * centre.x - 0.10 * centre.y;
 const auto index = static_cast<std::size_t>(cell->y) * map.grid->width + cell->x;
 CHECK(std::abs(map.grid->elevation_m[index] - expected_height) < 1e-6);
 CHECK(map.grid->surface_classes[map.grid->surface_class[index]] == "steel_deck");

 // Rasterization marks any cell touched by a thin static obstacle; checking
 // only the cell centre would silently erase this wall.
 const auto wall = sim::runtime::world_to_grid(*map.grid, {1.02, 0.25});
 CHECK(wall);
 const auto wall_index = static_cast<std::size_t>(wall->y) * map.grid->width + wall->x;
 CHECK(map.grid->occupancy[wall_index] == 100);
 CHECK(map.places.at("dock").heading_rad == 0.75);
 CHECK(map.features.size() == 1);
 CHECK(map.features.front().kind == "conflict_zone");
 CHECK(map.features.front().attributes.at("authority") == "control");
 return 0;
}
