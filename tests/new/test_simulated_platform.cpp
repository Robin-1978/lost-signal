#include "sim/game/mission/content_catalog.hpp"
#include "sim/game/platform/simulated_platform.hpp"
#include "sim/hardware/physics/physics_world.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <tuple>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

int main() {
 CHECK(sim::physics::mujoco_physics_available());
 const auto catalog = mission::ContentCatalog::load(LOST_SIGNAL_TEST_ASSETS);
 // The reference drive acceptance fixture is flat so straight-line geometry
 // and support stability are isolated from slope-transition impulses.
 const auto& level = catalog.level("motor_trial");
 const auto& map = catalog.map(level.map);
 const auto& robot = catalog.robot(catalog.campaign().companion_robot);
 std::vector<sim::HeightPatch> patches;
 for (const auto& patch : map.terrain) {
 if (mission::active_in_level(patch.active_in, level.id)) {
 patches.push_back({patch.id, patch.min, patch.max, patch.base_height_m, patch.grade_x,
 patch.grade_y, patch.roughness, patch.grade_xy_per_m});
 }
 }
 const sim::TerrainModel terrain(std::move(patches));
 const auto& spawn = map.markers.at(level.objective.spawn);
 const double mass = robot.mass_kg + level.payload_mass_kg;
 const double center_height = std::max(0.05,
 (robot.mass_kg * robot.center_of_mass_height_m +
 level.payload_mass_kg * level.payload_cog_height_m) /
 mass);
 const double wheel_loop_mass = robot.motor.wheel_equiv_mass_kg +
 std::max(mass * 0.5, 2.0 * robot.yaw_inertia_kgm2 /
 (robot.wheel_base_m * robot.wheel_base_m));
 sim::ChassisCaps chassis_caps;
 chassis_caps.geometry.support_width_m = robot.track_width_m;
 chassis_caps.geometry.support_length_m = std::max(robot.wheel_base_m, robot.collision_radius_m * 1.8);
 sim::platform::SimulatedPlatform physics(map, level, robot, chassis_caps, terrain,
 {spawn.x, spawn.y}, spawn.theta, mass,
 center_height, robot.yaw_inertia_kgm2);
 CHECK(std::string(physics.backend_name()) == "mujoco");
 CHECK(physics.lookup_transform("map", "odom").has_value());
 CHECK(physics.lookup_transform("odom", "base_footprint").has_value());
 CHECK(physics.lookup_transform("base_footprint", "base_link").has_value());
 CHECK(physics.lookup_transform("base_link", "lidar_link").has_value());
 const auto base_from_imu = physics.lookup_transform("base_link", "imu_link");
 CHECK(base_from_imu.has_value());
CHECK(std::abs(base_from_imu->translation_m.z) < 1e-12);
 CHECK(physics.lookup_transform("base_link", "left_wheel_link").has_value());
 physics.sync_actors({{"test_actor", {spawn.x + 1.0, spawn.y}, {}, 0.40}});
 const auto scan = physics.scan_lidar(360, 8.0, {spawn.x, spawn.y}, spawn.theta, 1.0, 0.1);
 CHECK(scan.combined.header.frame_id == "lidar_link");
 CHECK(std::abs(scan.combined.angle_increment_rad - 2.0 * sim::kPi / 360.0) < 1e-12);
 CHECK(std::abs(scan.combined.time_increment_s - 0.1 / 360.0) < 1e-12);
 CHECK(scan.combined.intensities.size() == scan.combined.ranges.size());
 CHECK(scan.combined.hit[0]);
 const auto world_from_lidar = physics.lookup_transform("map", "lidar_link");
 CHECK(world_from_lidar.has_value());
 const auto ray_direction = sim::physics::rotate_vector(
 world_from_lidar->rotation, {1.0, 0.0, 0.0});
 const auto cylinder_range = [&](double center_x) {
 const double ox = world_from_lidar->translation_m.x - center_x;
 const double oy = world_from_lidar->translation_m.y - spawn.y;
 const double a = ray_direction.x * ray_direction.x + ray_direction.y * ray_direction.y;
 const double b = 2.0 * (ox * ray_direction.x + oy * ray_direction.y);
 const double c = ox * ox + oy * oy - 0.40 * 0.40;
 return (-b - std::sqrt(b * b - 4.0 * a * c)) / (2.0 * a);
 };
 const double expected_actor_range = cylinder_range(spawn.x + 1.0);
 CHECK(std::abs(scan.combined.ranges[0] - expected_actor_range) < 1e-6);
 const double hit_x = world_from_lidar->translation_m.x +
 ray_direction.x * expected_actor_range;
 const double hit_y = world_from_lidar->translation_m.y +
 ray_direction.y * expected_actor_range;
 const double normal_x = (hit_x - (spawn.x + 1.0)) / 0.40;
 const double normal_y = (hit_y - spawn.y) / 0.40;
 const double incidence = std::clamp(
 -(ray_direction.x * normal_x + ray_direction.y * normal_y), 0.0, 1.0);
 const double expected_actor_intensity =
 0.52 * (0.15 + 0.85 * incidence) /
 (1.0 + 0.018 * expected_actor_range * expected_actor_range);
 CHECK(std::abs(scan.combined.intensities[0] - expected_actor_intensity) < 1e-6);
 sim::LidarReturnModel bright_actor;
 bright_actor.actor_reflectivity = 0.80;
 bright_actor.incidence_floor = 0.25;
 bright_actor.distance_attenuation_per_m2 = 0.0;
 const auto profiled_scan = physics.scan_lidar(
 360, 8.0, {spawn.x, spawn.y}, spawn.theta, 1.0, 0.1, bright_actor);
 const double expected_profiled_intensity =
 0.80 * (0.25 + 0.75 * incidence);
 CHECK(std::abs(profiled_scan.combined.intensities[0] -
 expected_profiled_intensity) < 1e-6);
 CHECK(scan.static_environment.ranges[0] > scan.combined.ranges[0] + 0.5);
 // Acquisition ends at timestamp_s. Ray zero belongs to one full scan period
 // earlier, so a +1 m/s actor is 0.10 m closer to its prior position.
 physics.sync_actors({{"test_actor", {spawn.x + 1.0, spawn.y}, {1.0, 0.0}, 0.40}});
 const auto moving_scan = physics.scan_lidar(
 4, 8.0, {spawn.x, spawn.y}, spawn.theta, 1.0, 0.1);
 const auto instantaneous_scan = physics.scan_lidar(
 4, 8.0, {spawn.x, spawn.y}, spawn.theta, 1.0, 0.0);
 CHECK(std::abs(moving_scan.combined.ranges[0] - cylinder_range(spawn.x + 0.9)) < 1e-6);
 CHECK(std::abs(instantaneous_scan.combined.ranges[0] - expected_actor_range) < 1e-6);
 CHECK(std::abs(moving_scan.combined.time_increment_s - 0.025) < 1e-12);
 // The following drivetrain fixture isolates wheel/ground dynamics; remove
 // the LiDAR target so it does not become a physical collision obstacle.
 physics.sync_actors({});
 const auto initial = physics.state();
 CHECK(std::isfinite(initial.height_m));
 CHECK(std::abs(initial.imu_angular_velocity_radps.x) < 1e-9);
 CHECK(std::abs(initial.imu_angular_velocity_radps.y) < 1e-9);
 CHECK(std::abs(initial.imu_angular_velocity_radps.z) < 1e-9);
 CHECK(std::abs(initial.imu_specific_force_mps2.x) < 0.05);
 CHECK(std::abs(initial.imu_specific_force_mps2.y) < 0.05);
 CHECK(std::abs(initial.imu_specific_force_mps2.z - 9.81) < 0.05);
 CHECK(physics.articulated());
 const auto initial_feedback = physics.actuator_feedback();
 CHECK(initial_feedback.valid());
 const auto* initial_left = initial_feedback.find("left");
 const auto* initial_right = initial_feedback.find("right");
 CHECK(initial_left);
 CHECK(initial_right);
 CHECK(physics.joint_state_for_child("left_wheel_link").has_value());
 CHECK(physics.joint_state_for_child("rear_caster_link").has_value());
 CHECK(physics.joint_state_for_child("rear_caster_wheel_link").has_value());
 const auto left_link = physics.link_transform_in_base("left_wheel_link");
 const auto right_link = physics.link_transform_in_base("right_wheel_link");
 const auto caster_fork_link = physics.link_transform_in_base("rear_caster_link");
 const auto caster_wheel_link = physics.link_transform_in_base("rear_caster_wheel_link");
 CHECK(left_link && std::abs(left_link->translation_m.y - 0.29) < 0.003);
 CHECK(left_link && std::abs(left_link->translation_m.z + 0.14) < 0.003);
 CHECK(right_link && std::abs(right_link->translation_m.y + 0.29) < 0.003);
 // Cylinder and visual assets share one link-frame contract: local +Z is the
 // axle. The articulated transform must place that axis laterally in base.
 // This independently catches the former double 90-degree visual rotation
 // that laid all three wheels flat while their MuJoCo bodies still moved.
 const auto left_axle = sim::physics::rotate_vector(left_link->rotation, {0.0, 0.0, 1.0});
 const auto right_axle = sim::physics::rotate_vector(right_link->rotation, {0.0, 0.0, 1.0});
 CHECK(std::abs(left_axle.x) < 0.01 && std::abs(std::abs(left_axle.y) - 1.0) < 0.01 &&
 std::abs(left_axle.z) < 0.01);
 CHECK(std::abs(right_axle.x) < 0.01 && std::abs(std::abs(right_axle.y) - 1.0) < 0.01 &&
 std::abs(right_axle.z) < 0.01);
 const auto left_mount = std::find_if(robot.mounts.begin(), robot.mounts.end(),
 [](const auto& mount) { return mount.frame_id == "left_wheel_link"; });
 CHECK(left_mount != robot.mounts.end());
 const auto preview_axle = sim::physics::rotate_vector(
 sim::physics::quaternion_from_euler(left_mount->rotation_deg.x * sim::kPi / 180.0,
 left_mount->rotation_deg.y * sim::kPi / 180.0,
 left_mount->rotation_deg.z * sim::kPi / 180.0),
 {0.0, 0.0, 1.0});
 CHECK(std::abs(preview_axle.x - left_axle.x) < 0.01);
 CHECK(std::abs(preview_axle.y - left_axle.y) < 0.01);
 CHECK(std::abs(preview_axle.z - left_axle.z) < 0.01);
 CHECK(caster_fork_link && caster_wheel_link);
 const auto fork_from_wheel = sim::physics::compose(
 sim::physics::inverse(*caster_fork_link), *caster_wheel_link);
 CHECK(std::abs(fork_from_wheel.translation_m.x + 0.035) < 0.003);
 CHECK(std::abs(fork_from_wheel.translation_m.z + 0.065) < 0.003);
 const auto caster_axle = sim::physics::rotate_vector(caster_wheel_link->rotation, {0.0, 0.0, 1.0});
 CHECK(std::abs(caster_axle.x) < 0.01 && std::abs(std::abs(caster_axle.y) - 1.0) < 0.01 &&
 std::abs(caster_axle.z) < 0.01);

 // Static-load oracle on a horizontal plane. MuJoCo may distribute each
 // wheel over several manifold points, so compare the time-averaged sum and
 // require all three supports to carry a positive share.
 {
 sim::platform::SimulatedPlatform resting(map, level, robot, chassis_caps, terrain,
 {spawn.x, spawn.y}, spawn.theta, mass, center_height, robot.yaw_inertia_kgm2);
 sim::ChassisTelemetry no_effort;
 no_effort.differential = true;
 double force_sum_N = 0.0;
 double left_force_N = 0.0;
 double right_force_N = 0.0;
 double caster_force_N = 0.0;
 for (int step = 0; step < 200; ++step) {
 const auto state = resting.step(no_effort, sim::GroundContact{}, 0.02);
 if (step < 100)
 continue;
 for (const auto& load : state.ground_contact_loads) {
 force_sum_N += load.normal_force_N;
 if (load.link == "left_wheel_link") left_force_N += load.normal_force_N;
 if (load.link == "right_wheel_link") right_force_N += load.normal_force_N;
 if (load.link == "rear_caster_wheel_link") caster_force_N += load.normal_force_N;
 }
 }
 const double average_force_N = force_sum_N / 100.0;
 CHECK(std::abs(average_force_N - mass * 9.81) < mass * 9.81 * 0.25);
 CHECK(left_force_N / 100.0 > 1.0);
 CHECK(right_force_N / 100.0 > 1.0);
 CHECK(caster_force_N / 100.0 > 1.0);
 }

 sim::ChassisTelemetry actuation;
 actuation.differential = true;
 actuation.joint_efforts = {{"left", 3.5}, {"right", 3.5}};
 sim::GroundContact ground;
 sim::platform::RigidVehicleState current;
 auto previous = initial;
 double max_height_step_m = 0.0;
 double max_roll_step_rad = 0.0;
 double max_pitch_step_rad = 0.0;
 bool caster_supported = false;
 bool hull_touched_ground = false;
 double peak_forward_specific_force_mps2 = 0.0;
 double max_motion_distortion_m = 0.0;
 for (int step = 0; step < 100; ++step) {
 if (step == 75)
 actuation.joint_efforts = {{"left", 0.0}, {"right", 0.0}};
 current = physics.step(actuation, ground, 0.02);
 peak_forward_specific_force_mps2 = std::max(peak_forward_specific_force_mps2,
 current.imu_specific_force_mps2.x);
 caster_supported = caster_supported ||
 std::find(current.ground_contact_links.begin(), current.ground_contact_links.end(),
 "rear_caster_wheel_link") != current.ground_contact_links.end();
 hull_touched_ground = hull_touched_ground ||
 std::find(current.ground_contact_links.begin(), current.ground_contact_links.end(),
 "base_link") != current.ground_contact_links.end() ||
 std::find(current.ground_contact_links.begin(), current.ground_contact_links.end(),
 "rear_caster_link") != current.ground_contact_links.end();
 max_height_step_m = std::max(max_height_step_m,
 std::abs(current.height_m - previous.height_m));
 max_roll_step_rad = std::max(max_roll_step_rad,
 std::abs(current.roll_rad - previous.roll_rad));
 max_pitch_step_rad = std::max(max_pitch_step_rad,
 std::abs(current.pitch_rad - previous.pitch_rad));
 previous = current;
 if (step == 20) {
 const auto rolling = physics.scan_lidar(360, 8.0,
 {current.planar.x, current.planar.y}, current.planar.theta, 0.42, 0.1);
 const auto instantaneous = physics.scan_lidar(360, 8.0,
 {current.planar.x, current.planar.y}, current.planar.theta, 0.42, 0.0);
 for (std::size_t index = 0; index < rolling.combined.ranges.size(); ++index) {
 max_motion_distortion_m = std::max(max_motion_distortion_m,
 std::abs(rolling.combined.ranges[index] - instantaneous.combined.ranges[index]));
 }
 }
 }
 const auto final_feedback = physics.actuator_feedback();
 const auto* final_left = final_feedback.find("left");
 const auto* final_right = final_feedback.find("right");
 CHECK(final_left);
 CHECK(final_right);
 const auto caster_roll = physics.joint_state_for_child("rear_caster_wheel_link");
 CHECK(final_left->position_rad > initial_left->position_rad);
 CHECK(final_right->position_rad > initial_right->position_rad);
 CHECK(current.planar.x > initial.planar.x + 0.2);
 CHECK(current.contact_count > 0);
 CHECK(caster_roll && std::abs(caster_roll->position_rad) > 0.1);
 CHECK(std::isfinite(current.roll_rad));
 CHECK(std::isfinite(current.pitch_rad));
 CHECK(std::find(current.ground_contact_links.begin(), current.ground_contact_links.end(),
 "left_wheel_link") != current.ground_contact_links.end());
 CHECK(std::find(current.ground_contact_links.begin(), current.ground_contact_links.end(),
 "right_wheel_link") != current.ground_contact_links.end());
 CHECK(caster_supported);
 CHECK(!hull_touched_ground);
 CHECK(peak_forward_specific_force_mps2 > 0.2);
 CHECK(std::find(current.ground_contact_links.begin(), current.ground_contact_links.end(),
 "base_link") == current.ground_contact_links.end());
 CHECK(std::find(current.ground_contact_links.begin(), current.ground_contact_links.end(),
 "rear_caster_link") == current.ground_contact_links.end());
 // A wheeled hull may follow terrain, but it must not hop or flip between
 // fixed steps. These limits catch the visible ground penetration/jitter
 // caused by the former cylinder-on-edge proxy.
 CHECK(max_height_step_m < 0.015);
 CHECK(max_roll_step_rad < 0.025);
CHECK(max_pitch_step_rad < 0.025);
 CHECK(max_motion_distortion_m > 0.001);

 // IMU lever-arm oracle: two otherwise identical worlds differ only by an
 // x-offset sensor. For planar rotation, the offset adds -omega^2*r forward
 // and alpha*r left. Gravity and root linear acceleration cancel in the
 // comparison, so this does not repeat the platform implementation.
 {
 auto offset_robot = robot;
 const auto imu_mount = std::find_if(offset_robot.mounts.begin(), offset_robot.mounts.end(),
 [](const auto& mount) { return mount.frame_id == "imu_link"; });
 CHECK(imu_mount != offset_robot.mounts.end());
 imu_mount->position_m = {0.20, 0.0, 0.0};
 sim::platform::SimulatedPlatform center_imu(map, level, robot, chassis_caps, terrain,
 {spawn.x, spawn.y}, spawn.theta, mass, center_height, robot.yaw_inertia_kgm2);
 sim::platform::SimulatedPlatform offset_imu(map, level, offset_robot, chassis_caps, terrain,
 {spawn.x, spawn.y}, spawn.theta, mass, center_height, robot.yaw_inertia_kgm2);
 sim::ChassisTelemetry spin_effort;
 spin_effort.differential = true;
 spin_effort.joint_efforts = {{"left", -2.0}, {"right", 2.0}};
 auto offset_before = offset_imu.state();
 sim::platform::RigidVehicleState center_after;
 sim::platform::RigidVehicleState offset_after;
 for (int index = 0; index < 20; ++index) {
 offset_before = offset_imu.state();
 center_after = center_imu.step(spin_effort, ground, 0.02);
 offset_after = offset_imu.step(spin_effort, ground, 0.02);
 }
 CHECK(std::abs(center_after.planar.w - offset_after.planar.w) < 1e-9);
 const double alpha_radps2 = (offset_after.planar.w - offset_before.planar.w) / 0.02;
 const double expected_forward_delta = -offset_after.planar.w * offset_after.planar.w * 0.20;
 const double expected_left_delta = alpha_radps2 * 0.20;
 CHECK(std::abs((offset_after.imu_specific_force_mps2.x -
 center_after.imu_specific_force_mps2.x) - expected_forward_delta) < 0.02);
 CHECK(std::abs((offset_after.imu_specific_force_mps2.y -
 center_after.imu_specific_force_mps2.y) - expected_left_delta) < 0.02);
 }

 // Independent differential geometry checks use direct, symmetric shaft
 // efforts so controller tuning cannot hide a joint-axis or sign error.
 {
 sim::platform::SimulatedPlatform spin(map, level, robot, chassis_caps, terrain,
 {spawn.x, spawn.y}, spawn.theta, mass, center_height, robot.yaw_inertia_kgm2);
 const auto start = spin.state();
 sim::ChassisTelemetry effort;
 effort.differential = true;
 effort.joint_efforts = {{"left", -4.0}, {"right", 4.0}};
 sim::platform::RigidVehicleState end;
 for (int index = 0; index < 20; ++index)
 end = spin.step(effort, ground, 0.02);
 const auto wheels = spin.actuator_feedback();
 const auto* left = wheels.find("left");
 const auto* right = wheels.find("right");
 CHECK(left);
 CHECK(right);
 const double encoder_yaw = (right->position_rad - left->position_rad) *
 robot.wheel_radius_m / robot.wheel_base_m;
 CHECK(std::abs(encoder_yaw - sim::wrap_angle(end.planar.theta - start.planar.theta)) < 1.0);
CHECK(sim::wrap_angle(end.planar.theta - start.planar.theta) > 0.20);
 CHECK(std::hypot(end.planar.x - start.planar.x, end.planar.y - start.planar.y) < 0.20);
 CHECK(left->position_rad < 0.0);
 CHECK(right->position_rad > 0.0);
 CHECK(end.imu_angular_velocity_radps.z > 0.0);
 }
 {
 sim::platform::SimulatedPlatform arc(map, level, robot, chassis_caps, terrain,
 {spawn.x, spawn.y}, spawn.theta, mass, center_height, robot.yaw_inertia_kgm2);
 const auto start = arc.state();
 sim::ChassisTelemetry effort;
 effort.differential = true;
 effort.joint_efforts = {{"left", 1.5}, {"right", 3.0}};
 sim::platform::RigidVehicleState end;
 double peak_arc_slip_ratio = 0.0;
 for (int index = 0; index < 40; ++index) {
 end = arc.step(effort, ground, 0.02);
 peak_arc_slip_ratio = std::max(peak_arc_slip_ratio, end.max_drive_slip_ratio);
 }
 CHECK(end.planar.x > start.planar.x + 0.15);
 const auto wheels = arc.actuator_feedback();
 const auto* left = wheels.find("left");
 const auto* right = wheels.find("right");
 CHECK(left);
 CHECK(right);
 CHECK(right->position_rad > left->position_rad);
 const double encoder_yaw = (right->position_rad - left->position_rad) *
 robot.wheel_radius_m / robot.wheel_base_m;
 const double body_yaw = sim::wrap_angle(end.planar.theta - start.planar.theta);
 std::cerr << "RO1 direct arc body-yaw=" << body_yaw
 << " encoder-yaw=" << encoder_yaw
 << " slip=" << peak_arc_slip_ratio << '\n';
 CHECK(body_yaw > 0.005);
 // Encoder geometry is exact only under no-slip rolling. The compliant
 // caster is free to align during this sustained arc, so compare both
 // independent measurements within a bounded slip error instead of
 // requiring an artificial transient skid.
 CHECK(std::abs(encoder_yaw - body_yaw) < 0.10);
 CHECK(peak_arc_slip_ratio < 0.25);
 }

 // Full RO-1 authority chain: body command -> IK -> encoder-count PID -> PWM
 // -> motor torque -> MuJoCo joints -> physical body state. Run two fresh
 // worlds to prove that the complete articulated trajectory is deterministic.
 const auto run_reference = [&]() {
 auto reference = std::make_unique<sim::platform::SimulatedPlatform>(map, level, robot, chassis_caps, terrain,
 sim::Vec2{spawn.x, spawn.y}, spawn.theta, mass, center_height, robot.yaw_inertia_kgm2);
 sim::MotorDriveParams params;
 params.wheel_base_m = robot.wheel_base_m;
 params.wheel_radius_m = robot.wheel_radius_m;
 params.drive_axle_x_m = 0.12;
 params.max_wheel_speed_mps = robot.motor.max_wheel_speed_mps;
 params.max_wheel_accel_mps2 = robot.motor.max_wheel_accel_mps2;
 params.stall_force_N = robot.motor.stall_force_N;
 params.free_speed_mps = robot.motor.free_speed_mps;
 params.mass_kg = mass;
 params.yaw_inertia_kgm2 = robot.yaw_inertia_kgm2;
 params.encoder_counts_per_revolution = robot.encoder.counts_per_revolution;
 params.pwm_limit = robot.motor.pwm_limit;
 params.integral_limit = robot.motor.integral_limit;
 const auto gains = sim::tune_wheel_pid(wheel_loop_mass, robot.motor.stall_force_N,
 robot.motor.free_speed_mps, robot.motor.max_wheel_speed_mps * 0.5);
 sim::DifferentialMotorDrive controller(params, gains);
 auto physical = reference->state();
 for (int index = 0; index < 200; ++index) {
 controller.set_actuator_feedback(reference->actuator_feedback());
 (void)controller.step(physical.planar, {0.70, 0.0}, 0.02, ground);
 physical = reference->step(controller.telemetry(), ground, 0.02);
 }
 const auto cruise = controller.telemetry();
 const auto before_brake_speed = std::abs(physical.planar.v);
 for (int index = 0; index < 100; ++index) {
 controller.set_actuator_feedback(reference->actuator_feedback());
 (void)controller.step(physical.planar, {0.0, 0.0}, 0.02, ground);
 physical = reference->step(controller.telemetry(), ground, 0.02);
 }
 return std::tuple{physical, cruise, before_brake_speed, controller.telemetry()};
 };
 const auto [reference_a, cruise_a, before_brake_a, stopped_a] = run_reference();
 const auto [reference_b, cruise_b, before_brake_b, stopped_b] = run_reference();
 CHECK(before_brake_a > 0.25);
 CHECK(std::abs(reference_a.planar.v) < before_brake_a * 0.35);
 CHECK(cruise_a.left.actual_mps > 0.0);
 CHECK(cruise_a.left.measured_mps > 0.0);
 CHECK(std::abs(cruise_a.left.actual_mps - cruise_a.right.actual_mps) < 0.08);
 CHECK(std::abs(cruise_a.left.pwm) <= 1.0);
 CHECK(std::abs(stopped_a.left.joint_effort_Nm) <= robot.motor.stall_force_N * robot.wheel_radius_m);
 CHECK(std::abs(reference_a.planar.x - reference_b.planar.x) < 1e-10);
 CHECK(std::abs(reference_a.planar.y - reference_b.planar.y) < 1e-10);
 CHECK(std::abs(reference_a.planar.theta - reference_b.planar.theta) < 1e-10);
 CHECK(std::abs(reference_a.imu_specific_force_mps2.x -
 reference_b.imu_specific_force_mps2.x) < 1e-10);
 CHECK(std::abs(before_brake_a - before_brake_b) < 1e-10);
 CHECK(std::abs(cruise_a.left.measured_mps - cruise_b.left.measured_mps) < 1e-10);
 CHECK(std::abs(stopped_a.left.pwm - stopped_b.left.pwm) < 1e-10);

 // Normal-operation envelope: settle the passive caster straight, then run
 // a moderate closed-loop arc. Encoder geometry is an independent oracle;
 // startup caster scrub is excluded from both deltas.
 {
 sim::platform::SimulatedPlatform maneuver(map, level, robot, chassis_caps, terrain,
 {spawn.x, spawn.y}, spawn.theta, mass, center_height, robot.yaw_inertia_kgm2);
 sim::MotorDriveParams params;
 params.wheel_base_m = robot.wheel_base_m;
 params.wheel_radius_m = robot.wheel_radius_m;
 params.drive_axle_x_m = 0.12;
 params.max_wheel_speed_mps = robot.motor.max_wheel_speed_mps;
 params.max_wheel_accel_mps2 = robot.motor.max_wheel_accel_mps2;
 params.stall_force_N = robot.motor.stall_force_N;
 params.free_speed_mps = robot.motor.free_speed_mps;
 params.mass_kg = mass;
 params.yaw_inertia_kgm2 = robot.yaw_inertia_kgm2;
 params.encoder_counts_per_revolution = robot.encoder.counts_per_revolution;
 params.pwm_limit = robot.motor.pwm_limit;
 params.integral_limit = robot.motor.integral_limit;
 const auto gains = sim::tune_wheel_pid(wheel_loop_mass,
 robot.motor.stall_force_N, robot.motor.free_speed_mps,
 robot.motor.max_wheel_speed_mps * 0.5);
 sim::DifferentialMotorDrive controller(params, gains);
 auto physical = maneuver.state();
 const auto drive = [&](sim::DiffCmd command, int steps, const sim::GroundContact& contact) {
 for (int index = 0; index < steps; ++index) {
 controller.set_actuator_feedback(maneuver.actuator_feedback());
 (void)controller.step(physical.planar, command, 0.02, contact);
 physical = maneuver.step(controller.telemetry(), contact, 0.02);
 }
 };
 drive({0.40, 0.0}, 100, ground);
 const auto arc_start = physical;
 const auto wheels_start = maneuver.actuator_feedback();
 drive({0.45, 0.30}, 150, ground);
 const auto wheels_end = maneuver.actuator_feedback();
 const auto* left_start = wheels_start.find("left");
 const auto* right_start = wheels_start.find("right");
 const auto* left_end = wheels_end.find("left");
 const auto* right_end = wheels_end.find("right");
 CHECK(left_start);
 CHECK(right_start);
 CHECK(left_end);
 CHECK(right_end);
 const double body_yaw = sim::wrap_angle(physical.planar.theta - arc_start.planar.theta);
 const double encoder_yaw =
 ((right_end->position_rad - right_start->position_rad) -
 (left_end->position_rad - left_start->position_rad)) *
 robot.wheel_radius_m / robot.wheel_base_m;
 CHECK(body_yaw > 0.25);
 CHECK(std::abs(encoder_yaw - body_yaw) < 0.35);
 CHECK(physical.planar.v > 0.25);
 const auto forward_end = physical;
 drive({-0.35, 0.0}, 180, ground);
 CHECK(physical.planar.v < -0.18);
 CHECK(controller.telemetry().left.measured_mps < 0.0);
 CHECK(controller.telemetry().right.measured_mps < 0.0);
 CHECK(std::hypot(physical.planar.x - forward_end.planar.x,
 physical.planar.y - forward_end.planar.y) > 0.35);
 }

 // The same PWM-speed loop on low friction must spin more wheel for less
 // chassis travel. This demonstrates that MuJoCo contact, rather than an
 // analytic traction clamp, owns slip in the reference platform.
 const auto traction_run = [&](double friction_mu) {
 sim::platform::SimulatedPlatform trial(map, level, robot, chassis_caps, terrain,
 {spawn.x, spawn.y}, spawn.theta, mass, center_height, robot.yaw_inertia_kgm2);
 sim::MotorDriveParams params;
 params.wheel_base_m = robot.wheel_base_m;
 params.wheel_radius_m = robot.wheel_radius_m;
 params.drive_axle_x_m = 0.12;
 params.max_wheel_speed_mps = robot.motor.max_wheel_speed_mps;
 params.max_wheel_accel_mps2 = robot.motor.max_wheel_accel_mps2;
 params.stall_force_N = robot.motor.stall_force_N;
 params.free_speed_mps = robot.motor.free_speed_mps;
 params.mass_kg = mass;
 params.yaw_inertia_kgm2 = robot.yaw_inertia_kgm2;
 params.encoder_counts_per_revolution = robot.encoder.counts_per_revolution;
 params.pwm_limit = robot.motor.pwm_limit;
 params.integral_limit = robot.motor.integral_limit;
 const auto gains = sim::tune_wheel_pid(wheel_loop_mass,
 robot.motor.stall_force_N, robot.motor.free_speed_mps,
 robot.motor.max_wheel_speed_mps * 0.5);
 sim::DifferentialMotorDrive controller(params, gains);
 auto physical = trial.state();
 const auto start = physical;
 const sim::GroundContact contact{friction_mu, 0.01, 6.0};
 bool slipping = false;
 double max_slip_ratio = 0.0;
 for (int index = 0; index < 100; ++index) {
 controller.set_actuator_feedback(trial.actuator_feedback());
 (void)controller.step(physical.planar, {0.9, 0.0}, 0.02, contact);
 physical = trial.step(controller.telemetry(), contact, 0.02);
 slipping = slipping || physical.drive_slipping;
 max_slip_ratio = std::max(max_slip_ratio, physical.max_drive_slip_ratio);
 }
 const auto wheels = trial.actuator_feedback();
 const auto* left = wheels.find("left");
 const auto* right = wheels.find("right");
 if (!left || !right)
 throw std::runtime_error("reference platform did not publish both drive joint samples");
return std::tuple{std::hypot(physical.planar.x - start.planar.x,
 physical.planar.y - start.planar.y),
 0.5 * (std::abs(left->position_rad) + std::abs(right->position_rad)) *
 robot.wheel_radius_m,
 slipping,
 max_slip_ratio};
 };
 const auto high_grip = traction_run(1.0);
 const auto low_grip = traction_run(0.08);
CHECK(std::get<0>(high_grip) > std::get<0>(low_grip) + 0.15);
CHECK(std::get<1>(low_grip) > std::get<0>(low_grip) + 0.20);
CHECK(std::get<2>(low_grip));
CHECK(std::get<3>(low_grip) > 0.25);

 // Cross-slope plant probe: isolate passive lateral holding and symmetric
 // drive from planning/controller effects while the reference gate is being
 // calibrated.
 {
 const auto& cross_level = catalog.level("cross_slope");
 const auto& cross_map = catalog.map(cross_level.map);
 std::vector<sim::HeightPatch> cross_patches;
 for (const auto& patch : cross_map.terrain) {
 if (mission::active_in_level(patch.active_in, cross_level.id))
 cross_patches.push_back({patch.id, patch.min, patch.max, patch.base_height_m,
 patch.grade_x, patch.grade_y, patch.roughness, patch.grade_xy_per_m});
 }
 const sim::TerrainModel cross_terrain(std::move(cross_patches));
 const auto& cross_spawn = cross_map.markers.at(cross_level.objective.spawn);
 const double cross_mass = robot.mass_kg + cross_level.payload_mass_kg;
 const double cross_center =
 (robot.mass_kg * robot.center_of_mass_height_m +
 cross_level.payload_mass_kg * cross_level.payload_cog_height_m) / cross_mass;
 // Coast is an explicit physical state: no command means no hidden parking
 // brake. A backdrivable differential platform with a passive caster can
 // align with the fall line and roll downhill.
 sim::platform::SimulatedPlatform coasting(cross_map, cross_level, robot, chassis_caps,
 cross_terrain, {cross_spawn.x, cross_spawn.y}, cross_spawn.theta,
 cross_mass, cross_center, robot.yaw_inertia_kgm2);
 const auto coast_start = coasting.state();
 sim::ChassisTelemetry coast_command;
 coast_command.differential = true;
 sim::platform::RigidVehicleState coast_state = coast_start;
 for (int index = 0; index < 150; ++index)
 coast_state = coasting.step(coast_command, {1.2, 0.014, 6.0}, 0.02);
 CHECK(std::hypot(coast_state.planar.x - coast_start.planar.x,
 coast_state.planar.y - coast_start.planar.y) > 0.10);
 CHECK(std::abs(sim::wrap_angle(coast_state.planar.theta - coast_start.planar.theta)) > 0.10);

 sim::platform::SimulatedPlatform cross(cross_map, cross_level, robot, chassis_caps,
 cross_terrain, {cross_spawn.x, cross_spawn.y}, cross_spawn.theta,
 cross_mass, cross_center, robot.yaw_inertia_kgm2);
 const auto cross_start = cross.state();
 sim::MotorDriveParams cross_params;
 cross_params.wheel_base_m = robot.wheel_base_m;
 cross_params.wheel_radius_m = robot.wheel_radius_m;
 cross_params.drive_axle_x_m = 0.12;
 cross_params.max_wheel_speed_mps = robot.motor.max_wheel_speed_mps;
 cross_params.max_wheel_accel_mps2 = robot.motor.max_wheel_accel_mps2;
 cross_params.stall_force_N = robot.motor.stall_force_N;
 cross_params.free_speed_mps = robot.motor.free_speed_mps;
 cross_params.mass_kg = cross_mass;
 cross_params.yaw_inertia_kgm2 = robot.yaw_inertia_kgm2;
 cross_params.encoder_counts_per_revolution = robot.encoder.counts_per_revolution;
 cross_params.pwm_limit = robot.motor.pwm_limit;
 cross_params.integral_limit = robot.motor.integral_limit;
 const auto cross_gains = sim::tune_wheel_pid(
 robot.motor.wheel_equiv_mass_kg +
 std::max(cross_mass * 0.5, 2.0 * robot.yaw_inertia_kgm2 /
 (robot.wheel_base_m * robot.wheel_base_m)),
 robot.motor.stall_force_N, robot.motor.free_speed_mps,
 robot.motor.max_wheel_speed_mps * 0.5);
 sim::DifferentialMotorDrive cross_controller(cross_params, cross_gains);
 const sim::GroundContact cross_contact{1.2, 0.014, 6.0};
 sim::platform::RigidVehicleState cross_state;
 cross_state = cross_start;
 for (int index = 0; index < 500; ++index) {
 cross_controller.set_actuator_feedback(cross.actuator_feedback());
 const double heading_error = sim::wrap_angle(cross_state.planar.theta - cross_start.planar.theta);
 const double position_error_x = cross_state.planar.x - cross_start.planar.x;
 const double position_error_y = cross_state.planar.y - cross_start.planar.y;
 const double forward_error = position_error_x * std::cos(cross_start.planar.theta) +
 position_error_y * std::sin(cross_start.planar.theta);
 (void)cross_controller.step(cross_state.planar,
 {std::clamp(-2.0 * forward_error, -0.5, 0.5),
 std::clamp(-4.0 * heading_error, -1.0, 1.0)}, 0.02, cross_contact);
 cross_state = cross.step(cross_controller.telemetry(), cross_contact, 0.02);
 }
 CHECK(std::hypot(cross_state.planar.x - cross_start.planar.x,
 cross_state.planar.y - cross_start.planar.y) < 0.03);
 CHECK(std::abs(cross_state.roll_rad - std::atan(0.36)) < 0.03);
 CHECK(std::find(cross_state.ground_contact_links.begin(),
 cross_state.ground_contact_links.end(), "left_wheel_link") !=
 cross_state.ground_contact_links.end());
 CHECK(std::find(cross_state.ground_contact_links.begin(),
 cross_state.ground_contact_links.end(), "right_wheel_link") !=
 cross_state.ground_contact_links.end());
 CHECK(std::find(cross_state.ground_contact_links.begin(),
 cross_state.ground_contact_links.end(), "rear_caster_wheel_link") !=
 cross_state.ground_contact_links.end());
 sim::ChassisTelemetry uphill;
 uphill.differential = true;
 uphill.joint_efforts = {{"left", 2.0}, {"right", 2.0}};
 for (int index = 0; index < 50; ++index)
 cross_state = cross.step(uphill, cross_contact, 0.02);
 std::cerr << "RO1 cross-slope raw torque delta="
 << cross_state.planar.x - cross_start.planar.x << ','
 << cross_state.planar.y - cross_start.planar.y << '\n';
 // Raw equal shaft torque must move the chassis forward. It is intentionally
 // not a cross-slope lane-hold controller, so bounded downhill drift remains
 // a plant output rather than a controller acceptance criterion.
 CHECK(cross_state.planar.x > cross_start.planar.x + 0.10);
CHECK(std::abs(cross_state.planar.y - cross_start.planar.y) < 0.60);
 }

 // The proving-ground cross slope has a physical approach, not a vertical
 // height-patch wall. A direct plant command must be able to enter it from
 // the adjacent flat floor; autonomy success from a spawn already on the
 // slope would not prove this geometry contract.
 {
 const auto& proving_level = catalog.level("vehicle_proving_ground");
 const auto& proving_map = catalog.map(proving_level.map);
 std::vector<sim::HeightPatch> proving_patches;
 for (const auto& patch : proving_map.terrain) {
 proving_patches.push_back({patch.id, patch.min, patch.max, patch.base_height_m,
 patch.grade_x, patch.grade_y, patch.roughness, patch.grade_xy_per_m});
 }
 const sim::TerrainModel proving_terrain(std::move(proving_patches));
 sim::platform::SimulatedPlatform transition(
 proving_map, proving_level, robot, chassis_caps, proving_terrain,
 {23.25, 24.0}, 0.0, robot.mass_kg, robot.center_of_mass_height_m,
 robot.yaw_inertia_kgm2);
 const auto transition_start = transition.state();
 sim::MotorDriveParams transition_params;
 transition_params.wheel_base_m = robot.wheel_base_m;
 transition_params.wheel_radius_m = robot.wheel_radius_m;
 transition_params.drive_axle_x_m = 0.12;
 transition_params.max_wheel_speed_mps = robot.motor.max_wheel_speed_mps;
 transition_params.max_wheel_accel_mps2 = robot.motor.max_wheel_accel_mps2;
 transition_params.stall_force_N = robot.motor.stall_force_N;
 transition_params.free_speed_mps = robot.motor.free_speed_mps;
 transition_params.mass_kg = robot.mass_kg;
 transition_params.yaw_inertia_kgm2 = robot.yaw_inertia_kgm2;
 transition_params.encoder_counts_per_revolution = robot.encoder.counts_per_revolution;
 transition_params.pwm_limit = robot.motor.pwm_limit;
 transition_params.integral_limit = robot.motor.integral_limit;
 const auto transition_gains = sim::tune_wheel_pid(
 robot.motor.wheel_equiv_mass_kg +
 std::max(robot.mass_kg * 0.5, 2.0 * robot.yaw_inertia_kgm2 /
 (robot.wheel_base_m * robot.wheel_base_m)),
 robot.motor.stall_force_N, robot.motor.free_speed_mps,
 robot.motor.max_wheel_speed_mps * 0.5);
 sim::DifferentialMotorDrive transition_controller(transition_params, transition_gains);
 auto transition_state = transition_start;
 for (int index = 0; index < 700; ++index) {
 transition_controller.set_actuator_feedback(transition.actuator_feedback());
 const double heading_error = sim::wrap_angle(transition_state.planar.theta);
 (void)transition_controller.step(
 transition_state.planar, {0.55, std::clamp(-4.0 * heading_error, -1.0, 1.0)},
 0.02, {1.2, 0.014, 6.0});
 transition_state = transition.step(
 transition_controller.telemetry(), {1.2, 0.014, 6.0}, 0.02);
 }
 CHECK(transition_state.planar.x > 28.0);
 CHECK(!transition_state.static_collision);
 CHECK(std::abs(transition_state.roll_rad) < 0.25);
 }

 // Smooth longitudinal-grade probe: bypass navigation and the motor speed
 // loop so the articulated plant sees a known symmetric shaft torque.
 {
 const auto& slope_level = catalog.level("slope_launch");
 const auto& slope_map = catalog.map(slope_level.map);
 std::vector<sim::HeightPatch> slope_patches;
 for (const auto& patch : slope_map.terrain) {
 if (mission::active_in_level(patch.active_in, slope_level.id))
 slope_patches.push_back({patch.id, patch.min, patch.max, patch.base_height_m,
 patch.grade_x, patch.grade_y, patch.roughness, patch.grade_xy_per_m});
 }
 const sim::TerrainModel slope_terrain(std::move(slope_patches));
 const double slope_mass = robot.mass_kg + slope_level.payload_mass_kg;
 const double slope_center =
 (robot.mass_kg * robot.center_of_mass_height_m +
 slope_level.payload_mass_kg * slope_level.payload_cog_height_m) / slope_mass;
 // A commanded effort below the gravity requirement cannot hold the grade.
 // This is motor authority, not a tire/contact tuning failure.
 sim::platform::SimulatedPlatform weak_slope(slope_map, slope_level, robot, chassis_caps,
 slope_terrain, {6.0, 2.9}, 0.0, slope_mass, slope_center,
 robot.yaw_inertia_kgm2);
 const auto weak_start = weak_slope.state();
 sim::ChassisTelemetry weak_effort;
 weak_effort.differential = true;
 weak_effort.joint_efforts = {{"left", 2.0}, {"right", 2.0}};
 sim::platform::RigidVehicleState weak_state = weak_start;
 for (int index = 0; index < 50; ++index)
 weak_state = weak_slope.step(weak_effort, {1.2, 0.0, 6.0}, 0.02);
 CHECK(weak_state.planar.x < weak_start.planar.x - 0.05);

 sim::platform::SimulatedPlatform slope(slope_map, slope_level, robot, chassis_caps,
 slope_terrain, {6.0, 2.9}, 0.0, slope_mass, slope_center,
 robot.yaw_inertia_kgm2);
 const auto slope_start = slope.state();
 sim::ChassisTelemetry effort;
 effort.differential = true;
 effort.joint_efforts = {{"left", 8.0}, {"right", 8.0}};
 sim::platform::RigidVehicleState slope_state;
 for (int index = 0; index < 50; ++index)
 slope_state = slope.step(effort, {1.2, 0.0, 6.0}, 0.02);
 const auto left_joint = slope.joint_state_for_child("left_wheel_link");
 const auto right_joint = slope.joint_state_for_child("right_wheel_link");
 const auto caster_joint = slope.joint_state_for_child("rear_caster_wheel_link");
 const auto swivel_joint = slope.joint_state_for_child("rear_caster_link");
 CHECK(slope_state.planar.x > slope_start.planar.x + 0.20);
 CHECK(std::abs(slope_state.planar.y - slope_start.planar.y) < 0.03);
 CHECK(std::abs(slope_state.pitch_rad + std::atan(0.364)) < 0.03);
 CHECK(left_joint && right_joint && caster_joint && swivel_joint);
 CHECK(std::abs(left_joint->velocity_radps - right_joint->velocity_radps) < 1.0);
 CHECK(slope_state.max_drive_slip_ratio < 0.30);
 }

 // Deterministic rough-ground fixture. This validates the configured
 // wheel-ground compliance and articulated caster without inventing a
 // suspension joint. Small height disturbances must excite the IMU while
 // the chassis and caster fork remain clear of the terrain.
 {
 auto rough_map = map;
 rough_map.terrain.push_back({
 "reference_rough_strip", {4.4, 3.8}, {7.2, 4.75},
 0.015, 0.0, 0.0, 0.015, {},
 });
 std::vector<sim::HeightPatch> rough_patches;
 for (const auto& patch : rough_map.terrain) {
 if (mission::active_in_level(patch.active_in, level.id))
 rough_patches.push_back({patch.id, patch.min, patch.max, patch.base_height_m,
 patch.grade_x, patch.grade_y, patch.roughness, patch.grade_xy_per_m});
 }
 const sim::TerrainModel rough_terrain(std::move(rough_patches));
 const auto run_rough = [&]() {
 sim::platform::SimulatedPlatform trial(rough_map, level, robot, chassis_caps,
 rough_terrain, {spawn.x, spawn.y}, spawn.theta, mass, center_height,
 robot.yaw_inertia_kgm2);
 sim::MotorDriveParams params;
 params.wheel_base_m = robot.wheel_base_m;
 params.wheel_radius_m = robot.wheel_radius_m;
 params.drive_axle_x_m = 0.12;
 params.max_wheel_speed_mps = robot.motor.max_wheel_speed_mps;
 params.max_wheel_accel_mps2 = robot.motor.max_wheel_accel_mps2;
 params.stall_force_N = robot.motor.stall_force_N;
 params.free_speed_mps = robot.motor.free_speed_mps;
 params.mass_kg = mass;
 params.yaw_inertia_kgm2 = robot.yaw_inertia_kgm2;
 params.encoder_counts_per_revolution = robot.encoder.counts_per_revolution;
 params.pwm_limit = robot.motor.pwm_limit;
 params.integral_limit = robot.motor.integral_limit;
 params.coulomb_smoothing_speed_mps = robot.motor.coulomb_smoothing_speed_mps;
 params.viscous_drag_N_per_mps = robot.motor.viscous_drag_N_per_mps;
 const sim::WheelControllerDesign design{
 robot.motor.controller_bandwidth_radps,
 robot.motor.controller_integral_time_s,
 robot.motor.controller_derivative_time_s,
 };
 const auto gains = sim::tune_wheel_pid(wheel_loop_mass,
 robot.motor.stall_force_N, robot.motor.free_speed_mps,
 robot.motor.max_wheel_speed_mps * 0.5, design);
 sim::DifferentialMotorDrive controller(params, gains);
 auto state = trial.state();
 const auto initial = state;
 double peak_specific_force = 0.0;
 double peak_pitch = 0.0;
 double peak_roll = 0.0;
 double peak_height_step = 0.0;
 int supported_steps = 0;
 bool chassis_and_fork_clear = true;
 for (int index = 0; index < 550; ++index) {
 controller.set_actuator_feedback(trial.actuator_feedback());
 const sim::DiffCmd command = index < 450 ? sim::DiffCmd{0.38, 0.0} : sim::DiffCmd{};
 (void)controller.step(state.planar, command, 0.02, ground);
 const auto before = state;
 state = trial.step(controller.telemetry(), ground, 0.02);
 peak_specific_force = std::max(peak_specific_force,
 std::sqrt(state.imu_specific_force_mps2.x * state.imu_specific_force_mps2.x +
 state.imu_specific_force_mps2.y * state.imu_specific_force_mps2.y +
 state.imu_specific_force_mps2.z * state.imu_specific_force_mps2.z));
 peak_pitch = std::max(peak_pitch, std::abs(state.pitch_rad));
 peak_roll = std::max(peak_roll, std::abs(state.roll_rad));
 peak_height_step = std::max(peak_height_step, std::abs(state.height_m - before.height_m));
 supported_steps += state.ground_contact_links.empty() ? 0 : 1;
 chassis_and_fork_clear = chassis_and_fork_clear &&
 std::find(state.ground_contact_links.begin(), state.ground_contact_links.end(),
 "base_link") == state.ground_contact_links.end() &&
 std::find(state.ground_contact_links.begin(), state.ground_contact_links.end(),
 "rear_caster_link") == state.ground_contact_links.end();
 }
 return std::tuple{state, initial, peak_specific_force, peak_pitch, peak_roll,
 peak_height_step, supported_steps, chassis_and_fork_clear};
 };
 const auto rough_a = run_rough();
 const auto rough_b = run_rough();
 CHECK(std::abs(std::get<0>(rough_a).planar.x - std::get<0>(rough_b).planar.x) < 1e-10);
 CHECK(std::abs(std::get<0>(rough_a).planar.y - std::get<0>(rough_b).planar.y) < 1e-10);
 CHECK(std::abs(std::get<0>(rough_a).planar.theta - std::get<0>(rough_b).planar.theta) < 1e-10);
 CHECK(std::get<0>(rough_a).planar.x > std::get<1>(rough_a).planar.x + 1.4);
 CHECK(std::get<6>(rough_a) > 500);
 CHECK(std::get<7>(rough_a));
 CHECK(std::get<2>(rough_a) > 9.82);
 CHECK(std::get<2>(rough_a) < 30.0);
 CHECK(std::get<3>(rough_a) < 0.35);
 CHECK(std::get<4>(rough_a) < 0.20);
 CHECK(std::get<5>(rough_a) < 0.04);
 CHECK(std::get<2>(rough_a) == std::get<2>(rough_b));
 CHECK(std::get<3>(rough_a) == std::get<3>(rough_b));
 CHECK(std::get<4>(rough_a) == std::get<4>(rough_b));
 CHECK(std::get<5>(rough_a) == std::get<5>(rough_b));
 }
 return 0;
}
