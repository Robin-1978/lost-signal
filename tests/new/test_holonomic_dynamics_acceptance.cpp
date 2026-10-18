#include "robot/control/holonomic_actuator_controller.hpp"
#include "sim/game/mission/content_catalog.hpp"
#include "sim/game/platform/simulated_platform.hpp"
#include "sim/hardware/vehicle/holonomic.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <string_view>

#define REQUIRE(c)                                                                                           \
 do {                                                                                                        \
  if (!(c)) {                                                                                                \
   std::cerr << __FILE__ << ':' << __LINE__ << ": requirement failed: " #c "\n";                             \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)
namespace {
constexpr double kDt = .02;
constexpr std::array<const char*, 4> drives{"front_left_drive", "front_right_drive", "rear_left_drive",
                                            "rear_right_drive"};
constexpr std::array<const char*, 4> steers{"front_left_steer", "front_right_steer", "rear_left_steer",
                                            "rear_right_steer"};
mission::MapDefinition empty_map(const mission::MapDefinition& source) {
 auto m = source;
 m.id = "holonomic_acceptance";
 m.size_m = {50, 40};
 m.obstacles.clear();
 m.world_objects.clear();
 m.surfaces.clear();
 m.terrain.clear();
 m.actors.clear();
 return m;
}
mission::MapDefinition slope_map(const mission::MapDefinition& source, double grade_x,
 double grade_y = 0.0, double roughness = 0.0) {
 auto m = empty_map(source);
 m.terrain.push_back({"acceptance_grade", {0.5, 0.5}, {49.5, 39.5}, 0.0,
 grade_x, grade_y, roughness, {}});
 return m;
}
sim::TerrainModel terrain_for(const mission::MapDefinition& map) {
 std::vector<sim::HeightPatch> patches;
 for (const auto& patch : map.terrain) {
  patches.push_back({patch.id, patch.min, patch.max, patch.base_height_m,
                     patch.grade_x, patch.grade_y, patch.roughness});
 }
 return sim::TerrainModel(std::move(patches));
}
bool touching(const sim::platform::RigidVehicleState& s, std::string_view l) {
 return std::find(s.ground_contact_links.begin(), s.ground_contact_links.end(), l) !=
        s.ground_contact_links.end();
}
struct Rig {
 sim::HolonomicDrive plant;
 sim::HolonomicActuatorController controller;
 bool swerve;
 explicit Rig(const mission::RobotDefinition& r, bool sw)
     : plant({sw ? sim::HolonomicKind::Swerve : sim::HolonomicKind::Mecanum, r.wheel_base_m, r.track_width_m,
              r.motor.max_wheel_speed_mps, r.motor.max_wheel_accel_mps2, r.max_yaw_accel_radps2, r.mass_kg,
              r.yaw_inertia_kgm2, 2.0, r.wheel_radius_m, r.motor.stall_force_N, r.motor.free_speed_mps,
              r.motor.pwm_limit, r.steering.stall_torque_Nm}),
       controller({sw ? sim::HolonomicControlKind::Swerve : sim::HolonomicControlKind::Mecanum,
                   r.wheel_base_m, r.track_width_m, r.wheel_radius_m, r.motor.max_wheel_speed_mps,
                   r.motor.pwm_limit, r.motor.integral_limit,
                   1.0 / std::max(.05, r.steering.servo_response_s)},
                  sim::tune_wheel_pid(r.motor.wheel_equiv_mass_kg + r.mass_kg * .25, r.motor.stall_force_N,
                                      r.motor.free_speed_mps, r.motor.max_wheel_speed_mps * .5,
                                      {r.motor.controller_bandwidth_radps, r.motor.controller_integral_time_s,
                                       r.motor.controller_derivative_time_s})),
       swerve(sw) {}
 sim::ChassisTelemetry step(sim::platform::SimulatedPlatform& p, const sim::DiffState& s,
                            const sim::DiffCmd& cmd, const sim::GroundContact& g, double dt = kDt) {
  const auto f = p.actuator_feedback();
  sim::HolonomicActuatorFeedback in;
  for (size_t i = 0; i < 4; ++i) {
   auto* d = f.find(drives[i]);
   if (!d)
    throw std::runtime_error("drive feedback");
   in.drive_velocity_radps[i] = d->velocity_radps;
   if (swerve) {
    auto* t = f.find(steers[i]);
    if (!t)
     throw std::runtime_error("steer feedback");
    in.steer_position_rad[i] = t->position_rad;
 in.steer_velocity_radps[i] = t->velocity_radps;
   }
  }
  const auto out = controller.update(cmd, in, dt);
  sim::ChassisActuatorDutyCommand duty;
  for (size_t i = 0; i < 4; ++i) {
   duty.setpoints.push_back({drives[i], out.drive_duty[i]});
   if (swerve)
    duty.setpoints.push_back({steers[i], out.steer_duty[i]});
  }
  plant.set_actuator_feedback(f);
  (void)plant.step_actuator_duty(s, cmd, duty, dt, g);
  return plant.telemetry();
 }
};
} // namespace
int main() {
 REQUIRE(sim::physics::bullet_physics_available());
 auto catalog = mission::ContentCatalog::load(LOST_SIGNAL_TEST_ASSETS);
 auto& level = catalog.level("motor_trial");
 auto map = empty_map(catalog.map(level.map));
 sim::TerrainModel terrain;
 sim::GroundContact dry{1, .01, 6, 40000, 1000};
 for (const auto id : {"robot.ro4", "robot.ro5"}) {
  const auto& r = catalog.robot(id);
  const bool sw = r.drive_type == "swerve";
  sim::ChassisCaps caps;
  caps.command_space = sim::ChassisCommandSpace::HolonomicTwist;
  caps.geometry.support_width_m = r.track_width_m;
  caps.geometry.support_length_m = r.wheel_base_m;
  REQUIRE(r.articulation.links.size() == (sw ? 9 : 5));
  REQUIRE(r.articulation.actuators.size() == (sw ? 8 : 4));
  {
   sim::platform::SimulatedPlatform p(map, level, r, caps, terrain, {10, 10}, 0, r.mass_kg,
                                      r.center_of_mass_height_m, r.yaw_inertia_kgm2);
   sim::ChassisTelemetry coast;
   auto s = p.state(), start = s;
   for (int i = 0; i < 200; ++i)
    s = p.step(coast, dry, kDt);
   for (auto* l :
        {"front_left_wheel_link", "front_right_wheel_link", "rear_left_wheel_link", "rear_right_wheel_link"})
    REQUIRE(touching(s, l));
   REQUIRE(!touching(s, "base_link"));
   REQUIRE(std::hypot(s.planar.x - start.planar.x, s.planar.y - start.planar.y) < .03);
  }
  auto maneuver = [&](sim::DiffCmd cmd, const sim::GroundContact& ground, double dt, double duration) {
   sim::platform::SimulatedPlatform p(map, level, r, caps, terrain, {10, 10}, 0, r.mass_kg,
                                      r.center_of_mass_height_m, r.yaw_inertia_kgm2);
   Rig rig(r, sw);
   auto s = p.state(), start = s;
   double yaw = 0;
   for (int i = 0; i < static_cast<int>(duration / dt); ++i) {
    auto b = s;
    s = p.step(rig.step(p, s.planar, cmd, ground, dt), ground, dt);
    yaw += sim::wrap_angle(s.planar.theta - b.planar.theta);
   }
   return std::array<double, 3>{s.planar.x - start.planar.x, s.planar.y - start.planar.y, yaw};
  };
  auto forward = maneuver({1, 0, 0}, dry, kDt, 6.0);
  std::cerr << id << " forward " << forward[0] << ',' << forward[1] << ',' << forward[2] << '\n';
  REQUIRE(forward[0] > 2.0);
  REQUIRE(std::abs(forward[1]) < .35);
 auto reverse = maneuver({-.8, 0, 0}, dry, kDt, 5.0);
 REQUIRE(reverse[0] < -1.5);
 REQUIRE(std::abs(reverse[1]) < .35);
  auto lateral = maneuver({0, 0, 1}, dry, kDt, 6.0);
  std::cerr << id << " lateral " << lateral[0] << ',' << lateral[1] << ',' << lateral[2] << '\n';
  REQUIRE(lateral[1] > 1.0);
  REQUIRE(std::abs(lateral[0]) < .5);
  auto spin = maneuver({0, 1, 0}, dry, kDt, 6.0);
  std::cerr << id << " spin " << spin[0] << ',' << spin[1] << ',' << spin[2] << '\n';
  REQUIRE(spin[2] > .5);
  REQUIRE(std::hypot(spin[0], spin[1]) < .6);
 if (sw) {
 // A direction change is a coupled steering/drive transient. After the
 // modules have had time to realign, straight driving must not retain the
 // rear-body fishtail that full drive-through-steer-error used to excite.
 sim::platform::SimulatedPlatform transition_platform(
 map, level, r, caps, terrain, {10, 10}, 0,
 r.mass_kg, r.center_of_mass_height_m, r.yaw_inertia_kgm2);
 Rig transition_rig(r, true);
 auto transition_state = transition_platform.state();
 for (int i = 0; i < 150; ++i) {
 transition_state = transition_platform.step(
 transition_rig.step(transition_platform, transition_state.planar, {.8, .8, 0}, dry),
 dry, kDt);
 }
 sim::DiffState settled_start;
 double unwrapped_yaw = transition_state.planar.theta;
 double min_settled_yaw = 0.0;
 double max_settled_yaw = 0.0;
 for (int i = 0; i < 300; ++i) {
 const double before_yaw = transition_state.planar.theta;
 transition_state = transition_platform.step(
 transition_rig.step(transition_platform, transition_state.planar, {1, 0, 0}, dry),
 dry, kDt);
 unwrapped_yaw += sim::wrap_angle(transition_state.planar.theta - before_yaw);
 if (i == 149) {
 settled_start = transition_state.planar;
 min_settled_yaw = unwrapped_yaw;
 max_settled_yaw = unwrapped_yaw;
 } else if (i > 149) {
 min_settled_yaw = std::min(min_settled_yaw, unwrapped_yaw);
 max_settled_yaw = std::max(max_settled_yaw, unwrapped_yaw);
 }
 }
 const double dx = transition_state.planar.x - settled_start.x;
 const double dy = transition_state.planar.y - settled_start.y;
 const double lateral_drift =
 -std::sin(settled_start.theta) * dx + std::cos(settled_start.theta) * dy;
 std::cerr << id << " turn-to-straight yaw-range "
 << (max_settled_yaw - min_settled_yaw)
 << " lateral-drift " << lateral_drift << '\n';
 REQUIRE(max_settled_yaw - min_settled_yaw < .08);
 REQUIRE(std::abs(lateral_drift) < .18);

 // A raw operator release is an immediate zero chassis-speed request, but
 // it must remain a physical stop: finite motor torque and tire contact
 // decelerate the moving rigid body over time. This case deliberately adds
 // no cosmetic command smoothing or body damping.
 sim::platform::SimulatedPlatform braking_platform(
 map, level, r, caps, terrain, {10, 10}, 0,
 r.mass_kg, r.center_of_mass_height_m, r.yaw_inertia_kgm2);
 Rig braking_rig(r, true);
 auto braking_state = braking_platform.state();
 for (int i = 0; i < 220; ++i) {
 braking_state = braking_platform.step(
 braking_rig.step(braking_platform, braking_state.planar, {1.0, 0, .65}, dry),
 dry, kDt);
 }
 const auto release = braking_state.planar;
 const double release_speed = std::hypot(release.v, release.lateral_v);
 braking_state = braking_platform.step(
 braking_rig.step(braking_platform, braking_state.planar, {0, 0, 0}, dry),
 dry, kDt);
 const double first_step_speed = std::hypot(braking_state.planar.v, braking_state.planar.lateral_v);
 double braking_unwrapped_yaw = braking_state.planar.theta;
 double min_braking_settle_yaw = 0.0;
 double max_braking_settle_yaw = 0.0;
 for (int i = 0; i < 300; ++i) {
 const double before_yaw = braking_state.planar.theta;
 braking_state = braking_platform.step(
 braking_rig.step(braking_platform, braking_state.planar, {0, 0, 0}, dry),
 dry, kDt);
 braking_unwrapped_yaw += sim::wrap_angle(braking_state.planar.theta - before_yaw);
 if (i == 199) {
 min_braking_settle_yaw = braking_unwrapped_yaw;
 max_braking_settle_yaw = braking_unwrapped_yaw;
 } else if (i > 199) {
 min_braking_settle_yaw = std::min(min_braking_settle_yaw, braking_unwrapped_yaw);
 max_braking_settle_yaw = std::max(max_braking_settle_yaw, braking_unwrapped_yaw);
 }
 }
 const double stop_distance = std::hypot(braking_state.planar.x - release.x,
 braking_state.planar.y - release.y);
 const double stopped_speed =
 std::hypot(braking_state.planar.v, braking_state.planar.lateral_v);
 std::cerr << id << " turning release speed " << release_speed
 << " first-step " << first_step_speed << " stop-distance " << stop_distance
 << " settled-yaw-range " << (max_braking_settle_yaw - min_braking_settle_yaw) << '\n';
 REQUIRE(release_speed > .7);
 REQUIRE(first_step_speed > release_speed * .8);
 REQUIRE(stop_distance > .08);
 REQUIRE(stopped_speed < .15);
 REQUIRE(std::abs(braking_state.planar.w) < .10);
 REQUIRE(max_braking_settle_yaw - min_braking_settle_yaw < .06);
 }
  const auto fine = maneuver({.8, .35, .3}, dry, .01, 4.0);
  const auto coarse = maneuver({.8, .35, .3}, dry, .02, 4.0);
 std::cerr << id << " convergence fine " << fine[0] << ',' << fine[1] << ',' << fine[2]
 << " coarse " << coarse[0] << ',' << coarse[1] << ',' << coarse[2] << '\n';
  REQUIRE(std::hypot(fine[0] - coarse[0], fine[1] - coarse[1]) < .35);
  REQUIRE(std::abs(fine[2] - coarse[2]) < .35);
  const sim::GroundContact low_grip{.12, .01, 6, 40000, 1000};
  const auto dry_launch = maneuver({1, 0, 0}, dry, .02, 1.0);
  const auto wet_launch = maneuver({1, 0, 0}, low_grip, .02, 1.0);
  REQUIRE(dry_launch[0] > wet_launch[0] + .08);

 const auto grade_map = slope_map(catalog.map(level.map), 0.18);
 const auto grade_terrain = terrain_for(grade_map);
 sim::platform::SimulatedPlatform grade_platform(grade_map, level, r, caps, grade_terrain,
                                                  {10, 10}, 0, r.mass_kg,
                                                  r.center_of_mass_height_m,
                                                  r.yaw_inertia_kgm2);
 Rig grade_rig(r, sw);
 auto grade_state = grade_platform.state();
 const auto grade_start = grade_state;
 double min_progress_m = 0.0;
 double max_progress_m = 0.0;
 double mean_normal_N = 0.0;
 int normal_samples = 0;
 for (int i = 0; i < 400; ++i) {
  grade_state = grade_platform.step(grade_rig.step(grade_platform, grade_state.planar,
                                                   {0.8, 0, 0}, dry),
                                    dry, kDt);
  const double progress_m = grade_state.planar.x - grade_start.planar.x;
  min_progress_m = std::min(min_progress_m, progress_m);
  max_progress_m = std::max(max_progress_m, progress_m);
  for (const auto& load : grade_state.ground_contact_loads) {
   if (load.link.ends_with("_wheel_link")) {
    mean_normal_N += load.normal_force_N;
    ++normal_samples;
   }
  }
 }
 mean_normal_N /= std::max(1, normal_samples);
 std::cerr << id << " grade progress " << (grade_state.planar.x - grade_start.planar.x)
           << " range " << min_progress_m << ',' << max_progress_m
           << " mean-wheel-load " << mean_normal_N << '\n';
 REQUIRE(grade_state.planar.x > grade_start.planar.x + 2.0);
 REQUIRE(min_progress_m > -0.25);
 REQUIRE(mean_normal_N > r.mass_kg * 9.81 / 8.0);

 const auto cross_map = slope_map(catalog.map(level.map), 0.0, 0.16);
 const auto cross_terrain = terrain_for(cross_map);
 sim::platform::SimulatedPlatform cross_platform(
 cross_map, level, r, caps, cross_terrain, {10, 10}, 0,
 r.mass_kg, r.center_of_mass_height_m, r.yaw_inertia_kgm2);
 sim::ChassisTelemetry coast;
 auto cross_state = cross_platform.state();
 for (int i = 0; i < 200; ++i)
 cross_state = cross_platform.step(coast, dry, kDt);
 REQUIRE(std::abs(cross_state.roll_rad - std::atan(0.16)) < 0.06);
 REQUIRE(!touching(cross_state, "base_link"));
 int cross_contacts = 0;
 for (auto* link : {"front_left_wheel_link", "front_right_wheel_link",
 "rear_left_wheel_link", "rear_right_wheel_link"})
 cross_contacts += touching(cross_state, link);
 std::cerr << id << " cross y " << cross_state.planar.y
 << " roll " << cross_state.roll_rad << " contacts " << cross_contacts << '\n';
 REQUIRE(cross_contacts >= (sw ? 4 : 2));
 if (sw)
 REQUIRE(std::abs(cross_state.planar.y - 10.0) < 0.20);
 else
 REQUIRE(cross_state.planar.y < 9.5);

 const auto rough_map = slope_map(catalog.map(level.map), 0.0, 0.0, 0.025);
 const auto rough_terrain = terrain_for(rough_map);
 sim::platform::SimulatedPlatform rough_platform(
 rough_map, level, r, caps, rough_terrain, {10, 10}, 0,
 r.mass_kg, r.center_of_mass_height_m, r.yaw_inertia_kgm2);
 Rig rough_rig(r, sw);
 auto rough_state = rough_platform.state();
 const auto rough_start = rough_state;
 int supported_steps = 0;
 bool hull_contact = false;
 for (int i = 0; i < 300; ++i) {
 rough_state = rough_platform.step(
 rough_rig.step(rough_platform, rough_state.planar, {.6, 0, 0}, dry), dry, kDt);
 int contacts = 0;
 for (auto* link : {"front_left_wheel_link", "front_right_wheel_link",
 "rear_left_wheel_link", "rear_right_wheel_link"})
 contacts += touching(rough_state, link);
 supported_steps += contacts >= 2;
 hull_contact = hull_contact || touching(rough_state, "base_link");
 }
 REQUIRE(rough_state.planar.x > rough_start.planar.x + 1.5);
 std::cerr << id << " rough dx " << (rough_state.planar.x - rough_start.planar.x)
 << " supported " << supported_steps << '\n';
 REQUIRE(supported_steps > 290);
 REQUIRE(!hull_contact);

 sim::platform::SimulatedPlatform brake_platform(
 map, level, r, caps, terrain, {10, 10}, 0,
 r.mass_kg, r.center_of_mass_height_m, r.yaw_inertia_kgm2);
 Rig brake_rig(r, sw);
 auto brake_state = brake_platform.state();
 for (int i = 0; i < 100; ++i)
 brake_state = brake_platform.step(
 brake_rig.step(brake_platform, brake_state.planar, {1.0, 0, 0}, dry), dry, kDt);
 REQUIRE(brake_state.planar.v > 0.5);
 double minimum_brake_speed = brake_state.planar.v;
 for (int i = 0; i < 150; ++i) {
 brake_state = brake_platform.step(
 brake_rig.step(brake_platform, brake_state.planar, {0, 0, 0}, dry), dry, kDt);
 minimum_brake_speed = std::min(minimum_brake_speed, brake_state.planar.v);
 }
 REQUIRE(std::abs(brake_state.planar.v) < 0.15);
 REQUIRE(minimum_brake_speed > -0.2);
 }
 std::cout << "RO-4/RO-5 articulated holonomic acceptance passed\n";
 return 0;
}
