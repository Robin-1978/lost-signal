#include "sim/game/mission/component_registry.hpp"
#include "sim/game/mission/content_catalog.hpp"
#include "sim/game/scenario/scenario_session.hpp"
#include "sim/hardware/sensors/sensor_system.hpp"
#include "sim/hardware/sensors/deterministic_delivery_queue.hpp"
#include "sim/game/platform/simulated_platform.hpp"
#include "robot/runtime/runtime_channel.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <set>
#include <vector>

#define REQUIRE(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": requirement failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

namespace {

mission::MapDefinition empty_map(const mission::MapDefinition& source) {
 auto map = source;
 map.id = "ro1_lidar_acceptance";
 map.size_m = {40.0, 30.0};
 map.obstacles.clear();
 map.world_objects.clear();
 map.surfaces.clear();
 map.terrain.clear();
 map.actors.clear();
 return map;
}

sim::ChassisCaps caps_for(const mission::RobotDefinition& robot) {
 sim::ChassisCaps caps;
 caps.geometry.support_width_m = robot.wheel_base_m;
 caps.geometry.support_length_m = 0.435;
 caps.geometry.drive_axle_x_m = 0.12;
 caps.tread_slip_observable = true;
 return caps;
}

} // namespace

int main() {
 REQUIRE(sim::physics::mujoco_physics_available());
 const auto catalog = mission::ContentCatalog::load(LOST_SIGNAL_TEST_ASSETS);
 mission::BuildSelection build;
 build.parts["sensor"] = "sensor.standard_lidar";
 const auto resolved = mission::ComponentRegistry::builtins().resolve(catalog, build);
 const auto* lidar = resolved.lidar();
 REQUIRE(lidar);

 // These are the sourced, shipped RO-1 reference-profile limits. Keep this
 // gate independent of any mission or renderer configuration.
 REQUIRE(resolved.definition("sensor").model_profile.fidelity == "typical");
 REQUIRE(!resolved.definition("sensor").model_profile.source_refs.empty());
 REQUIRE(lidar->min_range_m == 0.2);
 REQUIRE(lidar->range_m == 12.0);
 REQUIRE(lidar->rays == 800);
 REQUIRE(lidar->scan_hz == 10.0);
 REQUIRE(lidar->range_resolution_m == 0.005);
 REQUIRE(lidar->power_w == 3.0);

 mission::BuildSelection multibeam_build;
 multibeam_build.parts["sensor"] = "sensor.sixteen_line_lidar";
 const auto multibeam_resolved =
 mission::ComponentRegistry::builtins().resolve(catalog, multibeam_build);
 const auto* multibeam = multibeam_resolved.lidar();
 REQUIRE(multibeam);
 REQUIRE(multibeam->vertical_channels == 16);
 REQUIRE(multibeam->latency_s == 0.04);
 REQUIRE(multibeam->vertical_angles_rad.size() == 16);
 REQUIRE(std::abs(multibeam->vertical_angles_rad.front() +
 15.0 * sim::kPi / 180.0) < 1e-12);
 REQUIRE(std::abs(multibeam->vertical_angles_rad[1] -
 1.0 * sim::kPi / 180.0) < 1e-12);

 const auto& source_level = catalog.level("motor_trial");
 const auto& source_map = catalog.map(source_level.map);
 const auto map = empty_map(source_map);
 const auto& robot = catalog.robot(catalog.campaign().companion_robot);
 const sim::TerrainModel terrain(std::vector<sim::HeightPatch>{});
 const sim::Vec2 spawn{10.0, 10.0};
 sim::platform::SimulatedPlatform platform(
 map, source_level, robot, caps_for(robot), terrain, spawn, 0.0,
 robot.mass_kg, robot.center_of_mass_height_m, robot.yaw_inertia_kgm2);
 const auto world_from_lidar = platform.lookup_transform("map", "lidar_link");
 REQUIRE(world_from_lidar);
 const double lidar_x = world_from_lidar->translation_m.x;
 const double lidar_y = world_from_lidar->translation_m.y;

 sim::LidarReturnModel return_model;
 return_model.actor_reflectivity = lidar->actor_reflectivity;
 return_model.incidence_floor = lidar->incidence_floor;
 return_model.distance_attenuation_per_m2 = lidar->distance_attenuation_per_m2;

 // A thin cylinder directly on ray zero has a closed-form first intersection.
 // A farther, larger object on the same ray must remain occluded.
 platform.sync_actors({
 {"thin_pole", {lidar_x + 1.50, lidar_y}, {}, 0.03},
 {"far_actor", {lidar_x + 3.00, lidar_y}, {}, 0.40},
 });
 constexpr double kStampS = 5.0;
 const double scan_period_s = 1.0 / lidar->scan_hz;
 const auto scan = platform.scan_lidar(lidar->rays, lidar->range_m, spawn, 0.0,
 kStampS, scan_period_s, return_model);
 REQUIRE(scan.combined.header.frame_id == "lidar_link");
 REQUIRE(scan.combined.header.stamp_s == kStampS);
 REQUIRE(scan.combined.n_rays == lidar->rays);
 REQUIRE(scan.combined.angle_min_rad == 0.0);
 REQUIRE(std::abs(scan.combined.angle_increment_rad -
 2.0 * sim::kPi / static_cast<double>(lidar->rays)) < 1e-12);
 REQUIRE(scan.combined.angle_increment_rad > 0.0); // +Z / counter-clockwise
 REQUIRE(std::abs(scan.combined.scan_period_s - scan_period_s) < 1e-12);
 REQUIRE(std::abs(scan.combined.time_increment_s -
 scan_period_s / static_cast<double>(lidar->rays)) < 1e-12);
 REQUIRE(scan.combined.hit[0]);
 REQUIRE(std::abs(scan.combined.ranges[0] - 1.47) < 1e-6);
 REQUIRE(scan.combined.intensities[0] > 0.0);
 REQUIRE(!scan.static_environment.hit[0]);
 REQUIRE(scan.static_environment.ranges[0] == lidar->range_m);

 // One rotating 3D scan owns all sixteen calibrated rings. Returns carry
 // their ring and firing time; this is not sixteen unrelated planar scans.
 const auto vertical_angles = multibeam->vertical_angles_rad;
 platform.sync_actors({{"cloud_actor", {lidar_x + 2.0, lidar_y}, {}, 0.45}});
 const auto cloud_truth = platform.scan_lidar_point_cloud(
 multibeam->rays, vertical_angles, multibeam->range_m, spawn, 0.0, kStampS,
 1.0 / multibeam->scan_hz, return_model);
 REQUIRE(cloud_truth.combined.rings == 16);
 REQUIRE(cloud_truth.combined.azimuth_samples ==
 static_cast<std::uint32_t>(multibeam->rays));
 REQUIRE(!cloud_truth.combined.points.empty());
 REQUIRE(std::all_of(cloud_truth.combined.points.begin(),
 cloud_truth.combined.points.end(), [](const auto& point) {
 return point.ring < 16 && point.time_offset_s <= 0.0 &&
 point.time_offset_s >= -0.1;
 }));
 REQUIRE(std::any_of(cloud_truth.combined.points.begin(),
 cloud_truth.combined.points.end(), [](const auto& point) {
 return point.z_m < -0.1;
 }));

 // Static-reference geometry is rendered from the estimator pose rather than
 // simulator truth. A +0.5 m estimated x offset makes the same wall appear
 // 0.5 m closer in the reference cloud while the measured cloud is unchanged.
 auto reference_map = map;
 reference_map.obstacles.push_back(
 {"reference_wall", {lidar_x + 2.0, lidar_y - 0.5},
 {lidar_x + 2.2, lidar_y + 0.5}, {}});
 sim::platform::SimulatedPlatform reference_platform(
 reference_map, source_level, robot, caps_for(robot), terrain, spawn, 0.0,
 robot.mass_kg, robot.center_of_mass_height_m, robot.yaw_inertia_kgm2);
 const auto reference_truth = reference_platform.scan_lidar_point_cloud(
 multibeam->rays, vertical_angles, multibeam->range_m,
 {spawn.x + 0.5, spawn.y}, 0.0, kStampS,
 1.0 / multibeam->scan_hz, return_model);
 const auto ring_zero_azimuth = [](const auto& cloud) -> const sim::LidarPointReturn* {
 const auto found = std::find_if(cloud.points.begin(), cloud.points.end(),
 [](const auto& point) {
 return point.ring == 14 && point.x_m > 0.0 && std::abs(point.y_m) < 1e-8;
 });
 return found == cloud.points.end() ? nullptr : &*found;
 };
 const auto* measured_wall = ring_zero_azimuth(reference_truth.combined);
 const auto* expected_wall = ring_zero_azimuth(reference_truth.static_environment);
 REQUIRE(measured_wall && expected_wall);
 REQUIRE(std::abs((measured_wall->x_m - expected_wall->x_m) - 0.5) < 1e-4);

 sim::LidarConfig multibeam_config;
 multibeam_config.rays = multibeam->rays;
 multibeam_config.min_range_m = multibeam->min_range_m;
 multibeam_config.range_m = multibeam->range_m;
 multibeam_config.scan_hz = multibeam->scan_hz;
 multibeam_config.vertical_angles_rad = vertical_angles;
 multibeam_config.projection_min_height_m =
 multibeam->projection_min_height_m;
 multibeam_config.projection_max_height_m =
 multibeam->projection_max_height_m;
 multibeam_config.projection_ground_seed_max_height_m =
 multibeam->projection_ground_seed_max_height_m;
 multibeam_config.projection_ground_max_slope_rad =
 multibeam->projection_ground_max_slope_rad;
 multibeam_config.latency_s = multibeam->latency_s;
 sim::WorldTruth cloud_world;
 cloud_world.robot = {spawn.x, spawn.y, 0.0, 0.0, 0.0};
 sim::SensorSystem cloud_sensor(77, 9);
 const auto cloud_observation = cloud_sensor.capture_lidar_point_cloud(
 cloud_world, kStampS, multibeam_config, cloud_truth);
 REQUIRE(cloud_observation.lidar_point_cloud.has_value());
 REQUIRE(cloud_observation.lidar.has_value());
 REQUIRE(cloud_observation.lidar_static_reference.has_value());
 REQUIRE(cloud_observation.lidar->n_rays == multibeam->rays);
 REQUIRE(std::count(cloud_observation.lidar->hit.begin(),
 cloud_observation.lidar->hit.end(), 1) > 0);
 REQUIRE(sim::runtime::valid_observation_frame(cloud_observation));
 REQUIRE(cloud_observation.lidar_point_cloud->header.stamp_s == kStampS);
 REQUIRE(cloud_observation.lidar_point_cloud->header.latency_s == 0.04);

 // Device latency changes availability, never acquisition time. The fixed-step
 // queue also preserves production order for equal delivery timestamps.
 sim::DeterministicDeliveryQueue<sim::ObservationFrame> delivery_queue(4);
 REQUIRE(delivery_queue.enqueue(
 kStampS + multibeam_config.latency_s, cloud_observation));
 REQUIRE(!delivery_queue.pop_ready(kStampS + 0.039).has_value());
 const auto delayed_cloud = delivery_queue.pop_ready(kStampS + 0.04);
 REQUIRE(delayed_cloud.has_value());
 REQUIRE(delayed_cloud->time_s == kStampS);
 REQUIRE(delayed_cloud->lidar_point_cloud->header.stamp_s == kStampS);

 // A range-image ground classifier follows adjacent rings over flat and
 // sloped ground, while a vertical face, low obstacle and overhang remain in
 // the 2D obstacle projection. These are synthetic geometric oracles and do
 // not duplicate the raycaster implementation.
 sim::LidarPointCloud projection_cloud;
 projection_cloud.rings = 8;
 projection_cloud.azimuth_samples = 4;
 projection_cloud.max_range_m = 12.0;
 projection_cloud.scan_period_s = 0.1;
 projection_cloud.points = {
 // azimuth 0: a 11-degree rising ground surface, all removed.
 {2.0, 0.0, -0.30, 0.3, -0.10, 0, 0},
 {3.0, 0.0, -0.10, 0.3, -0.09, 1, 0},
 {4.0, 0.0, 0.10, 0.3, -0.08, 2, 0},
 // azimuth 1: a vertical wall. Its upper return must survive.
 {0.0, 3.0, -0.30, 0.8, -0.07, 0, 0},
 {0.0, 3.0, 0.10, 0.8, -0.06, 1, 0},
 // azimuth 2: a low obstacle.
 {-2.0, 0.0, 0.05, 0.6, -0.05, 4, 0},
 // azimuth 3: an overhanging return below robot clearance.
 {0.0, -2.5, 0.60, 0.7, -0.04, 6, 0},
 // Same azimuth, above configured obstacle height and ignored.
 {0.0, -1.5, 2.0, 0.9, -0.03, 7, 0},
 };
 const auto projection = sim::project_point_cloud_to_planar(
 projection_cloud, {-0.2, 1.5, -0.25, 30.0 * sim::kPi / 180.0});
 REQUIRE(!projection.hit[0]);
 REQUIRE(projection.hit[1] && std::abs(projection.ranges[1] - 3.0) < 1e-12);
 REQUIRE(projection.hit[2] && std::abs(projection.ranges[2] - 2.0) < 1e-12);
 REQUIRE(projection.hit[3] && std::abs(projection.ranges[3] - 2.5) < 1e-12);

 // Integration gate: the scenario provider does not expose a configured
 // 40 ms scan at t=0 during the first two 20 ms ticks. It becomes visible on
 // the third tick, while the acquisition timestamp remains owned by LSRI.
 auto delayed_build = catalog.presentation_for_level("first_sight").recommended_build;
 delayed_build.parts["tires"] = "tires.standard";
 delayed_build.parts["sensor"] = "sensor.sixteen_line_lidar";
 mission::ScenarioSession delayed_session(catalog, "first_sight", delayed_build);
 delayed_session.start();
 delayed_session.step();
 REQUIRE(!delayed_session.snapshot().lidar_point_cloud);
 delayed_session.step();
 REQUIRE(!delayed_session.snapshot().lidar_point_cloud);
 delayed_session.step();
 REQUIRE(delayed_session.snapshot().lidar_point_cloud &&
 !delayed_session.snapshot().lidar_point_cloud->empty());
 delayed_session.step();
 REQUIRE(!delayed_session.replay().empty());
 REQUIRE(delayed_session.replay().back().lidar_point_cloud ==
 delayed_session.snapshot().lidar_point_cloud);

 // Ray zero belongs to the acquisition start. A +1 m/s actor was 0.1 m
 // behind its end-of-scan pose, so its measured entry range changes by 0.1 m.
 platform.sync_actors({{"moving_actor", {lidar_x + 2.10, lidar_y}, {1.0, 0.0}, 0.20}});
 const auto moving = platform.scan_lidar(lidar->rays, lidar->range_m, spawn, 0.0,
 kStampS, scan_period_s, return_model);
 REQUIRE(moving.combined.hit[0]);
 REQUIRE(std::abs(moving.combined.ranges[0] - 1.80) < 1e-6);

 // Device range limits are observation semantics layered after ideal MuJoCo
 // geometry. A true return in the blind zone becomes an explicit no-return.
 sim::SensorSystem sensors(42, 7);
 sim::LidarConfig config;
 config.rays = lidar->rays;
 config.min_range_m = lidar->min_range_m;
 config.range_m = lidar->range_m;
 config.scan_hz = lidar->scan_hz;
 config.noise = {};
 config.returns = return_model;
 platform.sync_actors({{"near_actor", {lidar_x + 0.25, lidar_y}, {}, 0.10}});
 const auto near_truth = platform.scan_lidar(lidar->rays, lidar->range_m, spawn, 0.0,
 kStampS, scan_period_s, return_model);
 sim::WorldTruth truth;
 truth.robot = {spawn.x, spawn.y, 0.0, 0.0, 0.0};
 const auto near_observation = sensors.capture_lidar(truth, kStampS, config, near_truth);
 REQUIRE(!near_observation.lidar->hit[0]);
 REQUIRE(near_observation.lidar->ranges[0] == lidar->range_m);
 REQUIRE(near_observation.lidar->intensities[0] == 0.0);

 // Noise, bias, quantization and dropout are deterministic for a fixed seed.
 sim::Scan ideal;
 ideal.n_rays = 128;
 ideal.max_range = lidar->range_m;
 ideal.ranges.assign(128, 4.0);
 ideal.hit.assign(128, 1);
 ideal.intensities.assign(128, 0.5);
 const sim::LidarNoiseModel noisy_model{0.05, 0.012, 0.005, 0.20};
 sim::Rng rng_a(1234);
 sim::Rng rng_b(1234);
 const auto noisy_a = sim::apply_lidar_noise(ideal, noisy_model, rng_a);
 const auto noisy_b = sim::apply_lidar_noise(ideal, noisy_model, rng_b);
 REQUIRE(noisy_a.ranges == noisy_b.ranges);
 REQUIRE(noisy_a.hit == noisy_b.hit);
 REQUIRE(noisy_a.intensities == noisy_b.intensities);
 REQUIRE(std::count(noisy_a.hit.begin(), noisy_a.hit.end(), 0) > 0);
 for (std::size_t index = 0; index < noisy_a.ranges.size(); ++index) {
 if (!noisy_a.hit[index]) {
 REQUIRE(noisy_a.ranges[index] == lidar->range_m);
 REQUIRE(noisy_a.intensities[index] == 0.0);
 continue;
 }
 REQUIRE(std::abs(noisy_a.ranges[index] / 0.005 -
 std::round(noisy_a.ranges[index] / 0.005)) < 1e-9);
 }

 // Positive error beyond maximum range must not manufacture a valid hit at
 // exactly max_range.
 sim::Scan edge = ideal;
 edge.n_rays = 1;
 edge.ranges = {11.99};
 edge.hit = {1};
 edge.intensities = {0.7};
 sim::Rng edge_rng(9);
 const auto out_of_range = sim::apply_lidar_noise(
 edge, {0.0, 0.02, 0.005, 0.0}, edge_rng);
 REQUIRE(!out_of_range.hit[0]);
 REQUIRE(out_of_range.ranges[0] == lidar->range_m);
 REQUIRE(out_of_range.intensities[0] == 0.0);

 // A data-authored world object is one physical entity: MuJoCo and both the
 // planar and multibeam LiDAR paths observe the same collision proxy. The
 // material reflectivity also owns the return intensity.
 auto world_map = empty_map(source_map);
 mission::WorldCollisionDefinition world_collision;
 world_collision.position_m = {0.0, 0.0, 1.0};
 world_collision.size_m = {0.4, 1.0, 2.0};
 world_collision.material = "steel_deck";
 world_collision.lidar_reflectivity = 0.82;
 mission::WorldObjectDefinition world_object;
 world_object.id = "lidar-rail";
 world_object.prefab = "prefab.test";
 world_object.position_m = {lidar_x + 2.0, lidar_y, 0.0};
 world_object.collisions.push_back(world_collision);
 world_map.world_objects.push_back(world_object);
 sim::platform::SimulatedPlatform world_platform(
 world_map, source_level, robot, caps_for(robot), terrain, spawn, 0.0,
 robot.mass_kg, robot.center_of_mass_height_m, robot.yaw_inertia_kgm2);
 const auto world_scan = world_platform.scan_lidar(
 lidar->rays, lidar->range_m, spawn, 0.0, kStampS, scan_period_s, return_model);
 REQUIRE(world_scan.static_environment.hit[0]);
 const auto world_lidar = world_platform.lookup_transform("map", "lidar_link");
 REQUIRE(world_lidar);
 const double world_object_range = world_object.position_m.x - 0.2 -
 world_lidar->translation_m.x;
 REQUIRE(std::abs(world_scan.static_environment.ranges[0] - world_object_range) < 1e-3);
 const double measured_world_object_range = world_scan.static_environment.ranges[0];
 const double expected_intensity = 0.82 /
 (1.0 + return_model.distance_attenuation_per_m2 *
 measured_world_object_range * measured_world_object_range);
 REQUIRE(std::abs(world_scan.static_environment.intensities[0] -
 expected_intensity) < 1e-6);

 const auto world_cloud = world_platform.scan_lidar_point_cloud(
 multibeam->rays, multibeam->vertical_angles_rad, multibeam->range_m,
 spawn, 0.0, kStampS, 1.0 / multibeam->scan_hz, return_model);
 std::set<std::uint16_t> rail_rings;
 for (const auto& point : world_cloud.static_environment.points) {
 if (std::abs(point.x_m - measured_world_object_range) < 0.01 &&
 std::abs(point.y_m) < 1e-3)
 rail_rings.insert(point.ring);
 }
 REQUIRE(rail_rings.size() >= 4);

 std::cout << "RO-1 planar LiDAR acceptance: " << lidar->rays << " rays @ "
 << lidar->scan_hz << " Hz, sample cadence="
 << lidar->rays * lidar->scan_hz << " Hz, range=[" << lidar->min_range_m
 << ',' << lidar->range_m << "] m\n";
 return 0;
}
