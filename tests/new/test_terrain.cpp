#include "robot/autonomy/planning/global/astar.hpp"
#include "robot/autonomy/mapping/grid_map.hpp"
#include "robot/autonomy/mapping/terrain.hpp"

#include <cmath>
#include <iostream>

#define CHECK(condition)                                                                                     \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n";                           \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

int main() {
 sim::TerrainModel terrain({
   {"steep", {2.0, 1.0}, {8.0, 4.0}, 0.0, 0.36, 0.0, 0.1},
 });
 const auto sample = terrain.sample({3.0, 2.0});
 CHECK(std::abs(sample.height_m - 0.36) < 1e-12);
 CHECK(std::abs(sim::longitudinal_grade(sample, 0.0) - 0.36) < 1e-12);
 CHECK(std::abs(sim::lateral_grade(sample, sim::kPi * 0.5) + 0.36) < 1e-12);

 sim::TerrainCapability weak;
 weak.mass_kg = 24.0;
 weak.available_drive_force_N = 55.0;
 weak.friction_mu = 0.8;
 CHECK(!std::isfinite(terrain.traversal_cost({3.0, 2.0}, {4.0, 2.0}, weak)));
 sim::TerrainCapability strong = weak;
 strong.available_drive_force_N = 120.0;
 CHECK(std::isfinite(terrain.traversal_cost({3.0, 2.0}, {4.0, 2.0}, strong)));
 CHECK(sim::static_rollover_angle(0.6, 0.6) < sim::static_rollover_angle(0.6, 0.3));
 CHECK(sim::rollover_risk(0.4, 0.6, 0.6) > sim::rollover_risk(0.4, 0.6, 0.3));
 const double static_risk = sim::dynamic_rollover_risk(0.4, 0.0, 0.6, 0.4);
 CHECK(std::abs(static_risk - sim::rollover_risk(0.4, 0.6, 0.4)) < 1e-12);
 CHECK(sim::dynamic_rollover_risk(0.4, 1.5, 0.6, 0.4) > static_risk);
 CHECK(sim::dynamic_rollover_risk(0.4, -1.5, 0.6, 0.4) < static_risk);
 CHECK(sim::roughness_lateral_acceleration(0.05, 2.0, 0.5) >
       sim::roughness_lateral_acceleration(0.05, 1.0, 0.5));
 CHECK(sim::roughness_lateral_acceleration(0.0, 2.0, 0.5) == 0.0);

 sim::TerrainModel platform({
 {"deck", {5.0, 1.0}, {9.0, 4.0}, 2.0, 0.0, 0.0, 0.0},
 });
 CHECK(!std::isfinite(platform.traversal_cost({4.5, 2.0}, {5.0, 2.0}, strong)));
 CHECK(!std::isfinite(platform.traversal_cost({5.0, 2.0}, {4.5, 2.0}, strong)));

 sim::GridMap map(20, 10, 0.5);
 const auto route = sim::astar_plan(map, {0.5, 2.0}, {9.0, 2.0}, 1.0, [&](sim::Vec2 from, sim::Vec2 to) {
  return terrain.traversal_cost(from, to, weak);
 });
 CHECK(route.ok);
 bool used_detour = false;
 for (const auto point : route.path) {
 if (point.position.y < 1.0 || point.position.y >= 4.0) {
   used_detour = true;
  }
 }
 CHECK(used_detour);
 return 0;
}
