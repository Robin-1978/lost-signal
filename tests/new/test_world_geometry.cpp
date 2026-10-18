#include "sim/game/mission/deployable_map_adapter.hpp"
#include "sim/game/mission/world_geometry.hpp"

#include "robot/core/math.hpp"

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
 mission::MapDefinition map;
 map.id = "world-geometry-fixture";
 map.size_m = {4.0, 4.0};
 map.grid_resolution_m = 0.25;

 mission::WorldCollisionDefinition collision;
 collision.position_m = {0.5, 0.0, 0.4};
 collision.size_m = {2.0, 0.4, 0.8};
 collision.yaw_degrees = 15.0;
 collision.material = "steel_deck";
 collision.friction_mu = 0.7;
 collision.lidar_reflectivity = 0.82;
 mission::WorldObjectDefinition object;
 object.id = "rail";
 object.prefab = "prefab.rail";
 object.position_m = {2.0, 2.0, 0.0};
 object.scale = {1.5, 0.5, 2.0};
 object.yaw_degrees = 30.0;
 object.collisions.push_back(collision);
 map.world_objects.push_back(object);

 const auto resolved = mission::resolve_world_collisions(map, "fixture");
 CHECK(resolved.size() == 1);
 const auto& box = resolved.front();
 CHECK(std::abs(box.center_m.x - (2.0 + std::cos(sim::kPi / 6.0) * 0.75)) < 1e-12);
 CHECK(std::abs(box.center_m.y - (2.0 + std::sin(sim::kPi / 6.0) * 0.75)) < 1e-12);
 CHECK(std::abs(box.center_m.z - 0.8) < 1e-12);
 CHECK(std::abs(box.size_m.x - 3.0) < 1e-12);
 CHECK(std::abs(box.size_m.y - 0.2) < 1e-12);
 CHECK(std::abs(box.size_m.z - 1.6) < 1e-12);
 CHECK(std::abs(box.yaw_rad - sim::kPi / 4.0) < 1e-12);
 CHECK(std::abs(box.friction_mu - 0.7) < 1e-12);
 CHECK(std::abs(box.lidar_reflectivity - 0.82) < 1e-12);

 map.world_objects.front().active_in = {"other-level"};
 CHECK(mission::resolve_world_collisions(map, "fixture").empty());
 map.world_objects.front().active_in.clear();

 // A long rail rotated by 45 degrees occupies cells along its real shape.
 // A corner of its world AABB must remain free.
 map.world_objects.front().position_m = {2.0, 2.0, 0.0};
 map.world_objects.front().scale = {1.0, 1.0, 1.0};
 map.world_objects.front().yaw_degrees = 45.0;
 map.world_objects.front().collisions.front().position_m = {0.0, 0.0, 0.5};
 map.world_objects.front().collisions.front().size_m = {2.0, 0.4, 1.0};
 map.world_objects.front().collisions.front().yaw_degrees = 0.0;
 const auto snapshot = mission::make_deployable_map_snapshot(map, "fixture");
 const auto occupied = sim::runtime::world_to_grid(*snapshot.grid, {2.375, 2.375});
 const auto free_corner = sim::runtime::world_to_grid(*snapshot.grid, {2.625, 1.875});
 CHECK(occupied && free_corner);
 const auto index_of = [&](sim::runtime::GridIndex cell) {
 return static_cast<std::size_t>(cell.y) * snapshot.grid->width + cell.x;
 };
 CHECK(snapshot.grid->occupancy[index_of(*occupied)] == 100);
 CHECK(snapshot.grid->occupancy[index_of(*free_corner)] == 0);
 return 0;
}
