#include "game/content_catalog.hpp"
#include "game/physics_scene.hpp"
#include "physics/physics_world.hpp"

#include <algorithm>
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
 CHECK(sim::physics::bullet_physics_available());
 const auto catalog = game::ContentCatalog::load(LOST_SIGNAL_TEST_ASSETS);
 const auto& level = catalog.level("slope_launch");
 const auto& map = catalog.map(level.map);
 const auto& robot = catalog.robot(catalog.campaign().companion_robot);
 std::vector<sim::HeightPatch> patches;
 for (const auto& patch : map.terrain) {
 if (game::active_in_level(patch.active_in, level.id)) {
 patches.push_back({patch.id, patch.min, patch.max, patch.base_height_m, patch.grade_x,
 patch.grade_y, patch.roughness});
 }
 }
 const sim::TerrainModel terrain(std::move(patches));
 const auto& spawn = map.markers.at(level.objective.spawn);
 const double mass = robot.mass_kg + level.payload_mass_kg;
 const double center_height = std::max(0.05,
 (robot.mass_kg * robot.collision_height_m * 0.5 +
 level.payload_mass_kg * level.payload_cog_height_m) /
 mass);
 sim::ChassisCaps chassis_caps;
 chassis_caps.geometry.support_width_m = robot.track_width_m;
 chassis_caps.geometry.support_length_m = std::max(robot.wheel_base_m, robot.collision_radius_m * 1.8);
 game::PhysicsScene physics(map, level, robot, chassis_caps, terrain,
 {spawn.x, spawn.y}, spawn.theta, mass,
 center_height);
 CHECK(std::string(physics.backend_name()) == "bullet");
 CHECK(physics.lookup_transform("map", "odom").has_value());
 CHECK(physics.lookup_transform("odom", "base_footprint").has_value());
 CHECK(physics.lookup_transform("base_footprint", "base_link").has_value());
 CHECK(physics.lookup_transform("base_link", "lidar_link").has_value());
 const auto base_from_imu = physics.lookup_transform("base_link", "imu_link");
 CHECK(base_from_imu.has_value());
 CHECK(std::abs(base_from_imu->translation_m.z - 0.24) < 1e-12);
 CHECK(physics.lookup_transform("base_link", "left_wheel_link").has_value());
 physics.sync_actors({{"test_actor", {spawn.x + 1.0, spawn.y}, {}, 0.40}});
 const auto scan = physics.scan_lidar(360, 8.0, {spawn.x, spawn.y}, spawn.theta, 1.0, 0.1);
 CHECK(scan.combined.header.frame_id == "lidar_link");
 CHECK(std::abs(scan.combined.angle_increment_rad - 2.0 * sim::kPi / 360.0) < 1e-12);
 CHECK(std::abs(scan.combined.time_increment_s - 0.1 / 360.0) < 1e-12);
 CHECK(scan.combined.intensities.size() == scan.combined.ranges.size());
 CHECK(scan.combined.hit[0]);
 CHECK(scan.combined.ranges[0] > robot.collision_radius_m);
 CHECK(scan.combined.ranges[0] < 0.8);
 CHECK(scan.combined.intensities[0] > 0.0);
 CHECK(scan.static_environment.ranges[0] > scan.combined.ranges[0] + 0.5);
 const auto initial = physics.state();
 CHECK(std::isfinite(initial.height_m));
 sim::ChassisTelemetry actuation;
 actuation.differential = true;
 actuation.left.longitudinal_force_N = 35.0;
 actuation.right.longitudinal_force_N = 35.0;
 sim::GroundContact ground;
 game::RigidVehicleState current;
 for (int step = 0; step < 100; ++step) {
 current = physics.step(actuation, ground, 0.02);
 }
 CHECK(current.planar.x > initial.planar.x + 0.2);
 CHECK(current.contact_count > 0);
 CHECK(std::isfinite(current.roll_rad));
 CHECK(std::isfinite(current.pitch_rad));
 const auto rolling = physics.scan_lidar(360, 8.0,
 {current.planar.x, current.planar.y}, current.planar.theta, 3.0, 0.1);
 const auto instantaneous = physics.scan_lidar(360, 8.0,
 {current.planar.x, current.planar.y}, current.planar.theta, 3.0, 0.0);
 double max_motion_distortion_m = 0.0;
 for (std::size_t index = 0; index < rolling.combined.ranges.size(); ++index) {
 max_motion_distortion_m = std::max(max_motion_distortion_m,
 std::abs(rolling.combined.ranges[index] - instantaneous.combined.ranges[index]));
 }
 CHECK(max_motion_distortion_m > 0.001);
 return 0;
}
