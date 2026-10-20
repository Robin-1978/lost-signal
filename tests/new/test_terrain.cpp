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
 // Plane-only patch: the relief contract is exercised separately below, so
 // this sample pins the exact plane gradient the planner relies on.
 {"steep", {2.0, 1.0}, {8.0, 4.0}, 0.0, 0.36, 0.0, 0.0},
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
 const double downhill_margin_limit = sim::longitudinal_stability_acceleration_limit(
 -0.11, 0.12, 0.31, 0.40, 0.70);
 const double downhill_angle = std::atan(-0.11);
 const double expected_downhill_margin = std::max(
 0.0, 0.70 * 9.81 * std::cos(downhill_angle) * 0.12 / 0.40 +
 9.81 * std::sin(downhill_angle));
 CHECK(std::abs(downhill_margin_limit - expected_downhill_margin) < 1e-9);
 CHECK(sim::roughness_lateral_acceleration(0.05, 2.0, 0.5) >
       sim::roughness_lateral_acceleration(0.05, 1.0, 0.5));
 CHECK(sim::roughness_lateral_acceleration(0.0, 2.0, 0.5) == 0.0);

 // A three-point chassis has a triangular support polygon. The same lateral
 // load projection can be safe near the axle and outside near the caster;
 // independent width/length bounds would incorrectly accept both.
 const std::vector<sim::Vec2> triangular_support{
 {-0.28, 0.0}, {0.12, -0.29}, {0.12, 0.29},
 };
 CHECK(sim::support_polygon_risk(
 {0.0, -0.10}, {0.0, 0.0}, triangular_support) < 1.0);
 CHECK(sim::support_polygon_risk(
 {-0.15, -0.10}, {0.0, 0.0}, triangular_support) > 1.0);

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

 // Rough relief is part of the deployable map, not a cosmetic terrain value.
 // The physics mesh and this sample are authored from one function, so the
 // robot plans over the surface it actually drives on. Height and gradient
 // must describe that same surface; a sampled plane with hidden relief would
 // make every stability and stopping decision optimistic exactly where the
 // ground is worst.
 sim::TerrainModel rough({
 {"rough", {0.0, 0.0}, {10.0, 10.0}, 0.0, 0.0, 0.0, 0.04},
 });
 const sim::Vec2 probe{4.1, 5.3};
 const auto rough_sample = rough.sample(probe);
 CHECK(std::abs(rough_sample.height_m) > 1e-3);
 CHECK(std::abs(rough_sample.height_m) <= 0.04 + 1e-9);
 constexpr double kStep = 1e-4;
 const double finite_x =
 (rough.sample({probe.x + kStep, probe.y}).height_m -
 rough.sample({probe.x - kStep, probe.y}).height_m) /
 (2.0 * kStep);
 const double finite_y =
 (rough.sample({probe.x, probe.y + kStep}).height_m -
 rough.sample({probe.x, probe.y - kStep}).height_m) /
 (2.0 * kStep);
 CHECK(std::abs(finite_x - rough_sample.grade_x) < 1e-3);
 CHECK(std::abs(finite_y - rough_sample.grade_y) < 1e-3);
 // Local planner rotation shims can advance only a few millimetres around an
 // offset axle. Roughness must be evaluated from the local surface gradient,
 // not an exaggerated secant slope over that tiny segment.
 sim::TerrainCapability rough_cap;
 rough_cap.mass_kg = 24.0;
 rough_cap.available_drive_force_N = 240.0;
 rough_cap.friction_mu = 0.8;
 CHECK(std::isfinite(rough.traversal_cost(
 {probe.x, probe.y}, {probe.x + 0.003, probe.y - 0.002}, rough_cap)));
 // Relief the chassis cannot tilt to follow becomes a vertical excitation.
 // The ride limit is the speed at which that excitation consumes the stated
 // share of gravity, and it must tighten with rougher ground.
 CHECK(!std::isfinite(sim::roughness_ride_speed_limit(0.0, 0.435, 0.5)));
 const double smooth_ride = sim::roughness_ride_speed_limit(0.01, 0.435, 0.5);
 const double rough_ride = sim::roughness_ride_speed_limit(0.04, 0.435, 0.5);
 CHECK(std::isfinite(rough_ride));
 CHECK(rough_ride < smooth_ride);
 CHECK(std::abs(sim::roughness_vertical_acceleration(0.04, rough_ride, 0.435) -
 0.5 * 9.81) < 1e-6);
 CHECK(sim::roughness_ride_speed_limit(0.04, 0.435, 0.25) < rough_ride);
 return 0;
}
