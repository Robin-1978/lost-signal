#include "robot/control/manipulator_joint_controller.hpp"
#include "robot/platform/serial_manipulator.hpp"
#include "sim/game/mission/content_catalog.hpp"
#include "sim/game/platform/simulated_platform.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <string_view>

#define REQUIRE(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": requirement failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

namespace {
constexpr double kDt = 0.02;

mission::MapDefinition empty_map(const mission::MapDefinition& source) {
 auto map = source;
 map.id = "ro6_manipulator_acceptance";
 map.size_m = {30.0, 20.0};
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
 return caps;
}

bool touching(const sim::platform::RigidVehicleState& state, std::string_view link) {
 return std::find(state.ground_contact_links.begin(), state.ground_contact_links.end(), link) !=
 state.ground_contact_links.end();
}

struct ArmRig {
 const mission::ManipulatorDefinition& definition;
 sim::ManipulatorJointController controller;

 explicit ArmRig(const mission::ManipulatorDefinition& arm)
 : definition(arm), controller({arm.kp, arm.kd, arm.maximum_effort_Nm,
 arm.maximum_velocity_radps, arm.gravity_moment_Nm}) {}

 sim::ChassisTelemetry step(sim::platform::SimulatedPlatform& platform,
 const std::array<double, 4>& target_rad, bool hold_wheels = true) {
 const auto feedback = platform.actuator_feedback();
 sim::ManipulatorJointFeedback arm_feedback;
 arm_feedback.valid = true;
 for (std::size_t index = 0; index < definition.channels.size(); ++index) {
 const auto* sample = feedback.find(definition.channels[index]);
 if (!sample)
 throw std::runtime_error("RO-6 arm feedback is incomplete");
 arm_feedback.position_rad[index] = sample->position_rad;
 arm_feedback.velocity_radps[index] = sample->velocity_radps;
 }
 const auto output = controller.update(target_rad, arm_feedback, kDt);
 sim::ChassisTelemetry telemetry;
 telemetry.differential = true;
 for (std::size_t index = 0; index < definition.channels.size(); ++index)
 telemetry.joint_efforts.push_back({definition.channels[index], output.effort_Nm[index]});
 if (hold_wheels) {
 telemetry.joint_brakes = {
 {"left", 10.0, 0.2},
 {"right", 10.0, 0.2},
 };
 }
 return telemetry;
 }
};

std::array<double, 4> arm_positions(sim::platform::SimulatedPlatform& platform,
 const mission::ManipulatorDefinition& arm) {
 const auto feedback = platform.actuator_feedback();
 std::array<double, 4> positions{};
 for (std::size_t index = 0; index < arm.channels.size(); ++index) {
 const auto* sample = feedback.find(arm.channels[index]);
 if (!sample)
 throw std::runtime_error("RO-6 arm feedback is incomplete");
 positions[index] = sample->position_rad;
 }
 return positions;
}
} // namespace

int main() {
 REQUIRE(sim::physics::mujoco_physics_available());
 const auto catalog = mission::ContentCatalog::load(LOST_SIGNAL_TEST_ASSETS);
 const auto& robot = catalog.robot("robot.ro6");
 const auto& level = catalog.level("motor_trial");
 const auto source_map = catalog.map(level.map);
 const auto map = empty_map(source_map);
 REQUIRE(robot.manipulator.has_value());
 const auto& arm = *robot.manipulator;
 const sim::ManipulatorGeometry geometry{
 arm.shoulder_height_m, arm.upper_arm_length_m, arm.forearm_length_m,
 arm.tool_length_m, arm.lower_rad, arm.upper_rad};
 REQUIRE(sim::valid_manipulator_geometry(geometry));
 REQUIRE(robot.articulation.links.size() == 9);
 REQUIRE(robot.articulation.joints.size() == 8);
 REQUIRE(robot.articulation.actuators.size() == 6);

 const sim::TerrainModel terrain;
 const sim::GroundContact dry{1.0, 0.01, 6.0, 0.006, 1.0};
 const sim::Vec2 spawn{10.0, 10.0};
 sim::platform::SimulatedPlatform platform(
 map, level, robot, caps_for(robot), terrain, spawn, 0.0, robot.mass_kg,
 robot.center_of_mass_height_m, robot.yaw_inertia_kgm2);
 ArmRig rig(arm);
 auto state = platform.state();
 const auto base_start = state.planar;
 const std::array<double, 4> target{0.35, 0.80, -0.50, -0.30};
 for (int step = 0; step < 350; ++step)
 state = platform.step(rig.step(platform, target), dry, kDt);

 const auto measured = arm_positions(platform, arm);
 for (std::size_t index = 0; index < measured.size(); ++index) {
  if (std::abs(measured[index] - target[index]) >= 0.035) {
   std::cerr << "joint " << index << " target=" << target[index]
             << " measured=" << measured[index]
             << " error=" << measured[index] - target[index] << '\n';
  }
 REQUIRE(measured[index] >= arm.lower_rad[index] - 1e-3);
 REQUIRE(measured[index] <= arm.upper_rad[index] + 1e-3);
 REQUIRE(std::abs(measured[index] - target[index]) < 0.035);
 }
 const auto expected_tool = sim::manipulator_forward_kinematics(
 geometry, sim::ManipulatorJointPosition{measured});
 const auto tool = platform.link_transform_in_base("tool_link");
 REQUIRE(tool.has_value());
 const auto physical_tip = sim::physics::transform_point(
 *tool, {0.0, 0.0, arm.tool_length_m * 0.5});
 REQUIRE(std::abs(physical_tip.x - expected_tool.x_m) < 0.01);
 REQUIRE(std::abs(physical_tip.y - expected_tool.y_m) < 0.01);
 REQUIRE(std::abs(physical_tip.z - expected_tool.z_m) < 0.01);
 REQUIRE(touching(state, "left_wheel_link"));
 REQUIRE(touching(state, "right_wheel_link"));
 REQUIRE(touching(state, "rear_caster_wheel_link"));
 REQUIRE(!touching(state, "base_link"));
 REQUIRE(std::hypot(state.planar.x - base_start.x, state.planar.y - base_start.y) < 0.05);

 // Removing actuator effort must expose gravity; joint positions cannot stay
 // frozen by the simulator. The wheel brakes isolate arm/base reaction from
 // free rolling without pinning the chassis pose.
 const auto held = measured;
 sim::ChassisTelemetry released;
 released.differential = true;
 released.joint_brakes = {{"left", 10.0, 0.2}, {"right", 10.0, 0.2}};
 for (int step = 0; step < 100; ++step)
 state = platform.step(released, dry, kDt);
 const auto fallen = arm_positions(platform, arm);
 REQUIRE(std::abs(fallen[1] - held[1]) > 0.08 ||
 std::abs(fallen[2] - held[2]) > 0.08 ||
 std::abs(fallen[3] - held[3]) > 0.08);

 // A wall outside the hull but inside maximum arm reach must constrain the
 // articulated links through MuJoCo contact rather than a gameplay reach
 // test. Rebuild an independent fixture so prior gravity release cannot
 // affect the result.
 auto contact_map = map;
 contact_map.obstacles.push_back(
 {"arm_contact_wall", {spawn.x + 0.44, spawn.y - 0.25},
 {spawn.x + 0.54, spawn.y + 0.25}, {}});
 sim::platform::SimulatedPlatform contact_platform(
 contact_map, level, robot, caps_for(robot), terrain, spawn, 0.0,
 robot.mass_kg, robot.center_of_mass_height_m, robot.yaw_inertia_kgm2);
 ArmRig contact_rig(arm);
 sim::platform::RigidVehicleState contact_state;
 const std::array<double, 4> wall_target{0.0, 1.40, -0.45, -0.95};
 bool arm_contact = false;
 for (int step = 0; step < 350; ++step) {
 contact_state = contact_platform.step(
 contact_rig.step(contact_platform, wall_target), dry, kDt);
 arm_contact = arm_contact || contact_state.static_collision;
 }
 REQUIRE(arm_contact);
 REQUIRE(std::hypot(contact_state.planar.x - spawn.x,
 contact_state.planar.y - spawn.y) < 0.08);
 REQUIRE(!touching(contact_state, "base_link"));
 return 0;
}
