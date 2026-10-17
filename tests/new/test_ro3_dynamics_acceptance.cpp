#include "robot/control/differential_wheel_controller.hpp"
#include "sim/game/mission/content_catalog.hpp"
#include "sim/game/platform/simulated_platform.hpp"
#include "sim/hardware/vehicle/tracked.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string_view>
#include <vector>

#define REQUIRE(condition)                                                                                   \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": requirement failed: " #condition "\n";                     \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

namespace {
constexpr double kDt = 0.02;

mission::MapDefinition empty_map(const mission::MapDefinition& source) {
 auto map = source;
 map.id = "ro3_dynamics_acceptance";
 map.size_m = {40.0, 30.0};
 map.obstacles.clear();
 map.surfaces.clear();
 map.terrain.clear();
 map.actors.clear();
 return map;
}

mission::MapDefinition slope_map(const mission::MapDefinition& source,
 double grade_x, double grade_y = 0.0, double roughness = 0.0) {
 auto map = empty_map(source);
 map.terrain.push_back({"ro3_acceptance_grade", {0.5, 0.5}, {39.5, 29.5},
 0.0, grade_x, grade_y, roughness, {}});
 return map;
}

sim::TerrainModel terrain_for(const mission::MapDefinition& map) {
 std::vector<sim::HeightPatch> patches;
 for (const auto& patch : map.terrain) {
 patches.push_back({patch.id, patch.min, patch.max, patch.base_height_m,
 patch.grade_x, patch.grade_y, patch.roughness});
 }
 return sim::TerrainModel(std::move(patches));
}

bool touching(const sim::platform::RigidVehicleState& state, std::string_view link) {
 return std::find(state.ground_contact_links.begin(), state.ground_contact_links.end(), link) !=
        state.ground_contact_links.end();
}

struct Rig {
 sim::TrackedDrive plant;
 sim::DifferentialWheelController controller;
 explicit Rig(const mission::RobotDefinition& robot)
     : plant({robot.wheel_base_m, robot.effective_track_width_m, robot.wheel_radius_m, robot.track_length_m,
              robot.motor.max_wheel_speed_mps, robot.motor.max_wheel_accel_mps2, robot.motor.stall_force_N,
              robot.motor.free_speed_mps, robot.mass_kg, robot.yaw_inertia_kgm2,
              robot.tracked_max_yaw_rate_rads, robot.motor.pwm_limit, robot.motor.integral_limit,
              robot.encoder.counts_per_revolution, robot.motor.wheel_equiv_mass_kg, robot.track_width_m},
             sim::tune_wheel_pid(
               robot.motor.wheel_equiv_mass_kg + robot.mass_kg * 0.5, robot.motor.stall_force_N * 3.0,
               robot.motor.free_speed_mps, robot.motor.max_wheel_speed_mps * 0.5,
               {robot.motor.controller_bandwidth_radps, robot.motor.controller_integral_time_s,
                robot.motor.controller_derivative_time_s})),
       controller({robot.effective_track_width_m, robot.wheel_radius_m, robot.motor.max_wheel_speed_mps,
                   robot.encoder.counts_per_revolution, robot.motor.pwm_limit, robot.motor.integral_limit},
                  sim::tune_wheel_pid(
                    robot.motor.wheel_equiv_mass_kg + robot.mass_kg * 0.5, robot.motor.stall_force_N * 3.0,
                    robot.motor.free_speed_mps, robot.motor.max_wheel_speed_mps * 0.5,
                    {robot.motor.controller_bandwidth_radps, robot.motor.controller_integral_time_s,
                     robot.motor.controller_derivative_time_s})) {}

 sim::ChassisTelemetry step(sim::platform::SimulatedPlatform& platform, const sim::DiffState& state,
                            const sim::DiffCmd& command, const sim::GroundContact& ground, double dt = kDt) {
  const auto feedback = platform.actuator_feedback();
  const auto average = [&](const char* a, const char* b, const char* c, bool position) {
   const auto* x = feedback.find(a);
   const auto* y = feedback.find(b);
   const auto* z = feedback.find(c);
   if (!x || !y || !z)
    throw std::runtime_error("RO-3 feedback incomplete");
   return position ? (x->position_rad + y->position_rad + z->position_rad) / 3.0
                   : (x->velocity_radps + y->velocity_radps + z->velocity_radps) / 3.0;
  };
  const auto output =
    controller.update(command,
                      {average("front_left_drive", "middle_left_drive", "rear_left_drive", true),
                       average("front_right_drive", "middle_right_drive", "rear_right_drive", true),
                       average("front_left_drive", "middle_left_drive", "rear_left_drive", false),
                       average("front_right_drive", "middle_right_drive", "rear_right_drive", false), true},
                      dt);
  plant.set_actuator_feedback(feedback);
  (void)plant.step_actuator_duty(
    state, command, {{{"left", output.left.normalized_duty}, {"right", output.right.normalized_duty}}}, dt,
    ground);
  return plant.telemetry();
 }
};
} // namespace

int main() {
 REQUIRE(sim::physics::bullet_physics_available());
 const auto catalog = mission::ContentCatalog::load(LOST_SIGNAL_TEST_ASSETS);
 const auto& robot = catalog.robot("robot.ro3");
 const auto& level = catalog.level("motor_trial");
 const auto map = empty_map(catalog.map(level.map));
 sim::ChassisCaps caps;
 caps.geometry.support_width_m = robot.track_width_m;
 caps.geometry.support_length_m = robot.track_length_m;
 const sim::TerrainModel terrain;
 const sim::GroundContact dry{1.0, 0.01, 6.0, 40000.0, 1000.0};
 const sim::Vec2 spawn{10.0, 10.0};

 REQUIRE(robot.articulation.links.size() == 7);
 REQUIRE(robot.articulation.joints.size() == 6);
 REQUIRE(robot.articulation.actuators.size() == 6);
 {
  sim::platform::SimulatedPlatform platform(map, level, robot, caps, terrain, spawn, 0.0, robot.mass_kg,
                                            robot.center_of_mass_height_m, robot.yaw_inertia_kgm2);
  sim::ChassisTelemetry coast;
  auto state = platform.state();
  const auto start = state;
  for (int i = 0; i < 250; ++i)
   state = platform.step(coast, dry, kDt);
  for (const auto* link : {"front_left_wheel_link", "middle_left_wheel_link", "rear_left_wheel_link",
                           "front_right_wheel_link", "middle_right_wheel_link", "rear_right_wheel_link"})
   REQUIRE(touching(state, link));
  REQUIRE(!touching(state, "base_link"));
  REQUIRE(std::hypot(state.planar.x - start.planar.x, state.planar.y - start.planar.y) < 0.03);
  REQUIRE(std::abs(state.roll_rad) < 0.03);
  REQUIRE(std::abs(state.pitch_rad) < 0.03);
 }
 {
  sim::platform::SimulatedPlatform platform(map, level, robot, caps, terrain, spawn, 0.0, robot.mass_kg,
                                            robot.center_of_mass_height_m, robot.yaw_inertia_kgm2);
  Rig rig(robot);
  auto state = platform.state();
  const auto start = state;
  for (int i = 0; i < 250; ++i)
   state = platform.step(rig.step(platform, state.planar, {1.0, 0.0}, dry), dry, kDt);
  std::cerr << "RO3 straight dx=" << state.planar.x - start.planar.x
            << " dy=" << state.planar.y - start.planar.y << " v=" << state.planar.v << '\n';
  REQUIRE(state.planar.x > start.planar.x + 1.5);
  REQUIRE(std::abs(state.planar.y - start.planar.y) < 0.25);
  REQUIRE(std::abs(sim::wrap_angle(state.planar.theta - start.planar.theta)) < 0.15);
 }
 // Signed drive is symmetric: reverse must move the same physical joints and
 // may not be emulated by changing the chassis pose directly.
 {
  sim::platform::SimulatedPlatform platform(map, level, robot, caps, terrain, spawn, 0.0, robot.mass_kg,
                                            robot.center_of_mass_height_m, robot.yaw_inertia_kgm2);
  Rig rig(robot);
  auto state = platform.state();
  const auto start = state;
  for (int i = 0; i < 200; ++i)
   state = platform.step(rig.step(platform, state.planar, {-0.8, 0.0}, dry), dry, kDt);
  std::cerr << "RO3 reverse dx=" << state.planar.x - start.planar.x << '\n';
  REQUIRE(state.planar.x < start.planar.x - 0.8);
  REQUIRE(std::abs(state.planar.y - start.planar.y) < 0.2);
 }
 {
  sim::platform::SimulatedPlatform platform(map, level, robot, caps, terrain, spawn, 0.0, robot.mass_kg,
                                            robot.center_of_mass_height_m, robot.yaw_inertia_kgm2);
  Rig rig(robot);
  auto state = platform.state();
  const auto start = state;
  double yaw = 0.0;
  for (int i = 0; i < 300; ++i) {
   const auto before = state;
   state = platform.step(rig.step(platform, state.planar, {0.0, 1.0}, dry), dry, kDt);
   yaw += sim::wrap_angle(state.planar.theta - before.planar.theta);
  }
  std::cerr << "RO3 pivot yaw=" << yaw
            << " drift=" << std::hypot(state.planar.x - start.planar.x, state.planar.y - start.planar.y)
            << '\n';
  // The calibrated skid-steer separation must make a 1 rad/s request track
  // the body response, rather than merely proving that the chassis can twitch.
  // A broad physical band admits contact-solver variation while rejecting the
  // old 0.15 rad/s understeer that stranded the local planner near shelving.
  REQUIRE(yaw > 4.5);
  REQUIRE(yaw < 8.0);
  REQUIRE(std::hypot(state.planar.x - start.planar.x, state.planar.y - start.planar.y) < 0.5);
 }
 // Fixed-step convergence and adhesion are model evidence, independent of
 // whether a game mission happens to complete.
 const auto launch = [&](double dt, const sim::GroundContact& contact) {
  sim::platform::SimulatedPlatform platform(map, level, robot, caps, terrain, spawn, 0.0, robot.mass_kg,
                                            robot.center_of_mass_height_m, robot.yaw_inertia_kgm2);
  Rig rig(robot);
  auto state = platform.state();
  const auto start = state;
  for (int i = 0; i < static_cast<int>(2.0 / dt); ++i)
   state = platform.step(rig.step(platform, state.planar, {1.0, 0.0}, contact, dt), contact, dt);
  return state.planar.x - start.planar.x;
 };
 const double coarse = launch(.02, dry), fine = launch(.01, dry);
 REQUIRE(std::abs(coarse - fine) < .2);
 const sim::GroundContact low_grip{.12, .01, 6.0, 40000.0, 1000.0};
 REQUIRE(coarse > launch(.02, low_grip) + .08);
 {
 const auto grade_map = slope_map(catalog.map(level.map), 0.18);
 const auto grade_terrain = terrain_for(grade_map);
 sim::platform::SimulatedPlatform platform(
 grade_map, level, robot, caps, grade_terrain, spawn, 0.0,
 robot.mass_kg, robot.center_of_mass_height_m, robot.yaw_inertia_kgm2);
 Rig rig(robot);
 auto state = platform.state();
 const auto start = state;
 for (int i = 0; i < 400; ++i)
 state = platform.step(
 rig.step(platform, state.planar, {0.8, 0.0}, dry), dry, kDt);
 std::cerr << "RO3 grade dx=" << state.planar.x - start.planar.x
 << " odom=" << rig.plant.telemetry().measured_forward_mps << '\n';
 REQUIRE(state.planar.x > start.planar.x + 2.0);
 }
 {
 const auto cross_map = slope_map(catalog.map(level.map), 0.0, 0.16);
 const auto cross_terrain = terrain_for(cross_map);
 sim::platform::SimulatedPlatform platform(
 cross_map, level, robot, caps, cross_terrain, spawn, 0.0,
 robot.mass_kg, robot.center_of_mass_height_m, robot.yaw_inertia_kgm2);
 sim::ChassisTelemetry coast;
 auto state = platform.state();
 for (int i = 0; i < 200; ++i)
 state = platform.step(coast, dry, kDt);
 REQUIRE(std::abs(state.roll_rad - std::atan(0.16)) < 0.06);
 REQUIRE(!touching(state, "base_link"));
 int contacts = 0;
 for (const auto* link : {"front_left_wheel_link", "middle_left_wheel_link",
 "rear_left_wheel_link", "front_right_wheel_link",
 "middle_right_wheel_link", "rear_right_wheel_link"})
 contacts += touching(state, link);
 REQUIRE(contacts >= 5);
 }
 {
 const auto rough_map = slope_map(catalog.map(level.map), 0.0, 0.0, 0.025);
 const auto rough_terrain = terrain_for(rough_map);
 sim::platform::SimulatedPlatform platform(
 rough_map, level, robot, caps, rough_terrain, spawn, 0.0,
 robot.mass_kg, robot.center_of_mass_height_m, robot.yaw_inertia_kgm2);
 Rig rig(robot);
 auto state = platform.state();
 const auto start = state;
 int supported_steps = 0;
 bool hull_contact = false;
 for (int i = 0; i < 300; ++i) {
 state = platform.step(
 rig.step(platform, state.planar, {0.6, 0.0}, dry), dry, kDt);
 int contacts = 0;
 for (const auto* link : {"front_left_wheel_link", "middle_left_wheel_link",
 "rear_left_wheel_link", "front_right_wheel_link",
 "middle_right_wheel_link", "rear_right_wheel_link"})
 contacts += touching(state, link);
 supported_steps += contacts >= 2;
 hull_contact = hull_contact || touching(state, "base_link");
 }
 REQUIRE(state.planar.x > start.planar.x + 1.5);
 std::cerr << "RO3 rough dx=" << state.planar.x - start.planar.x
 << " supported=" << supported_steps << '\n';
 REQUIRE(supported_steps > 290);
 REQUIRE(!hull_contact);
 }
 {
 sim::platform::SimulatedPlatform platform(
 map, level, robot, caps, terrain, spawn, 0.0,
 robot.mass_kg, robot.center_of_mass_height_m, robot.yaw_inertia_kgm2);
 Rig rig(robot);
 auto state = platform.state();
 for (int i = 0; i < 100; ++i)
 state = platform.step(
 rig.step(platform, state.planar, {1.0, 0.0}, dry), dry, kDt);
 REQUIRE(state.planar.v > 0.5);
 double minimum_speed = state.planar.v;
 for (int i = 0; i < 150; ++i) {
 state = platform.step(
 rig.step(platform, state.planar, {0.0, 0.0}, dry), dry, kDt);
 minimum_speed = std::min(minimum_speed, state.planar.v);
 }
 REQUIRE(std::abs(state.planar.v) < 0.15);
 REQUIRE(minimum_speed > -0.2);
 }
 std::cout << "RO-3 articulated tracked acceptance passed\n";
 return 0;
}
