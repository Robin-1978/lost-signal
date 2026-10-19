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

 // A mixed derivative represents a continuous cross-slope transition. The
 // entry edge is flat; four metres later it exactly matches a 16% side slope.
 sim::TerrainModel cross_slope_transition({
 {"entry", {0.0, 0.0}, {4.0, 6.0}, 0.0, 0.0, 0.0, 0.0, 0.04},
 });
 const auto entry = cross_slope_transition.sample({0.0, 3.0});
 const auto middle = cross_slope_transition.sample({2.0, 3.0});
 const auto exit = cross_slope_transition.sample({3.999999, 3.0});
 CHECK(std::abs(entry.height_m) < 1e-12);
 CHECK(std::abs(entry.grade_y) < 1e-12);
 CHECK(std::abs(middle.height_m - 0.24) < 1e-12);
 CHECK(std::abs(middle.grade_y - 0.08) < 1e-12);
 CHECK(std::abs(exit.height_m - 0.47999988) < 1e-8);
 CHECK(std::abs(exit.grade_y - 0.15999996) < 1e-8);

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

 // A front-heavy/asymmetric support polygon has less braking margin than
 // acceleration margin. A downhill grade consumes still more of that front
 // margin, while an uphill grade consumes rearward acceleration margin.
 const double level_limit = sim::longitudinal_stability_acceleration_limit(
  0.0, 0.12, 0.31, 0.40, 1.0);
 const double downhill_limit = sim::longitudinal_stability_acceleration_limit(
  -0.11, 0.12, 0.31, 0.40, 1.0);
 const double uphill_limit = sim::longitudinal_stability_acceleration_limit(
  0.11, 0.12, 0.31, 0.40, 1.0);
 CHECK(std::abs(level_limit - 9.81 * 0.12 / 0.40) < 1e-9);
 CHECK(downhill_limit < level_limit);
 CHECK(uphill_limit < 9.81 * 0.31 / 0.40);
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

 // An eight-connected point path cannot pass through the zero-width corner
 // shared by two occupied cells. Both cardinal neighbours block the only
 // exit, so the diagonal goal is unreachable.
 sim::GridMap corner_map(2, 2, 1.0);
 corner_map.setOccupied(1.5, 0.5);
 corner_map.setOccupied(0.5, 1.5);
 const auto corner_route = sim::astar_plan(corner_map, {0.5, 0.5}, {1.5, 1.5});
 CHECK(!corner_route.ok);
 return 0;
}
