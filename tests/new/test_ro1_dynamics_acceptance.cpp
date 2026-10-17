#include "mission/content_catalog.hpp"
#include "sim/simulated_platform.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string_view>
#include <vector>

#define REQUIRE(condition)                                                                        \
  do {                                                                                            \
    if (!(condition)) {                                                                           \
      std::cerr << __FILE__ << ':' << __LINE__ << ": requirement failed: " #condition "\n";    \
      return 1;                                                                                   \
    }                                                                                             \
  } while (false)

namespace {

constexpr double kGravityMps2 = 9.81;
constexpr double kStepS = 0.02;

sim::ChassisCaps caps_for(const mission::RobotDefinition& robot) {
  sim::ChassisCaps caps;
  caps.geometry.support_width_m = robot.wheel_base_m;
  caps.geometry.support_length_m = 0.435; // driven axle x=+0.120, caster contact x=-0.315
  caps.geometry.drive_axle_x_m = 0.12;
  caps.max_yaw_rate_radps = 2.0;
  caps.tread_slip_observable = true;
  return caps;
}

sim::TerrainModel terrain_for(const mission::MapDefinition& map,
                              const mission::LevelDefinition& level) {
  std::vector<sim::HeightPatch> patches;
  for (const auto& patch : map.terrain) {
    if (mission::active_in_level(patch.active_in, level.id)) {
      patches.push_back({patch.id, patch.min, patch.max, patch.base_height_m,
                         patch.grade_x, patch.grade_y, patch.roughness});
    }
  }
  return sim::TerrainModel(std::move(patches));
}

mission::MapDefinition empty_map(const mission::MapDefinition& source) {
  auto map = source;
  map.id = "ro1_dynamics_acceptance";
  map.size_m = {40.0, 20.0};
  map.obstacles.clear();
  map.surfaces.clear();
  map.terrain.clear();
  map.actors.clear();
  return map;
}

mission::MapDefinition slope_map(const mission::MapDefinition& source, double grade_x,
                                 double grade_y) {
  auto map = empty_map(source);
  map.terrain.push_back({"acceptance_plane", {0.5, 0.5}, {39.5, 19.5}, 0.0,
                         grade_x, grade_y, 0.0, {}});
  return map;
}

double load_on(const sim::platform::RigidVehicleState& state, std::string_view link) {
  const auto found = std::find_if(state.ground_contact_loads.begin(),
                                  state.ground_contact_loads.end(),
                                  [&](const auto& value) { return value.link == link; });
  return found == state.ground_contact_loads.end() ? 0.0 : found->normal_force_N;
}

bool touching(const sim::platform::RigidVehicleState& state, std::string_view link) {
  return std::find(state.ground_contact_links.begin(), state.ground_contact_links.end(), link) !=
         state.ground_contact_links.end();
}

std::unique_ptr<sim::DifferentialMotorDrive> controller_for(
    const mission::RobotDefinition& robot, double mass_kg) {
  sim::MotorDriveParams params;
  params.wheel_base_m = robot.wheel_base_m;
  params.wheel_radius_m = robot.wheel_radius_m;
  params.drive_axle_x_m = 0.12;
  params.max_wheel_speed_mps = robot.motor.max_wheel_speed_mps;
  params.max_wheel_accel_mps2 = robot.motor.max_wheel_accel_mps2;
  params.stall_force_N = robot.motor.stall_force_N;
  params.free_speed_mps = robot.motor.free_speed_mps;
  params.mass_kg = mass_kg;
  params.yaw_inertia_kgm2 = robot.yaw_inertia_kgm2;
  params.encoder_counts_per_revolution = robot.encoder.counts_per_revolution;
  params.pwm_limit = robot.motor.pwm_limit;
  params.integral_limit = robot.motor.integral_limit;
  params.support_width_m = robot.wheel_base_m;
  params.support_length_m = 0.435;
  params.max_yaw_rate_radps = 2.0;
  params.coulomb_smoothing_speed_mps = robot.motor.coulomb_smoothing_speed_mps;
  params.viscous_drag_N_per_mps = robot.motor.viscous_drag_N_per_mps;
  const double reflected_mass = robot.motor.wheel_equiv_mass_kg +
      std::max(mass_kg * 0.5, 2.0 * robot.yaw_inertia_kgm2 /
                                     (robot.wheel_base_m * robot.wheel_base_m));
  const sim::WheelControllerDesign design{
      robot.motor.controller_bandwidth_radps,
      robot.motor.controller_integral_time_s,
      robot.motor.controller_derivative_time_s,
  };
  const auto gains = sim::tune_wheel_pid(reflected_mass, robot.motor.stall_force_N,
                                         robot.motor.free_speed_mps,
                                         robot.motor.max_wheel_speed_mps * 0.5, design);
  return std::make_unique<sim::DifferentialMotorDrive>(params, gains);
}

} // namespace

int main() {
  REQUIRE(sim::physics::bullet_physics_available());
  const auto catalog = mission::ContentCatalog::load(LOST_SIGNAL_TEST_ASSETS);
  const auto& source_level = catalog.level("motor_trial");
  const auto& source_map = catalog.map(source_level.map);
  const auto& robot = catalog.robot(catalog.campaign().companion_robot);
  const auto caps = caps_for(robot);
  const auto map = empty_map(source_map);
  const auto terrain = terrain_for(map, source_level);
  const sim::Vec2 spawn{10.0, 10.0};
  const double mass = robot.mass_kg;
  const double center_height = robot.center_of_mass_height_m;
 constexpr double kDriveAxleX = 0.12;
 constexpr double kCasterContactX = -0.315;
 double assembly_com_x_m = 0.0;
 for (const auto& link : robot.articulation.links)
 assembly_com_x_m += link.mass_kg * link.position_m.x;
 assembly_com_x_m /= robot.mass_kg;
  const sim::GroundContact dry{1.0, 0.01, 6.0, 40000.0, 1000.0};

  // Static equilibrium has an analytic three-point-support answer. This
  // checks the authored centre of mass against independent force and moment
  // balances, rather than merely checking that three contacts exist.
  double measured_left_N = 0.0;
  double measured_right_N = 0.0;
  double measured_caster_N = 0.0;
  double measured_total_N = 0.0;
 double rest_peak_height_step_m = 0.0;
 double rest_peak_planar_speed_mps = 0.0;
 double rest_peak_yaw_rate_radps = 0.0;
 double rest_peak_specific_force_mps2 = 0.0;
  {
    sim::platform::SimulatedPlatform platform(map, source_level, robot, caps, terrain,
                                               spawn, 0.0, mass, center_height,
                                               robot.yaw_inertia_kgm2);
    sim::ChassisTelemetry coast;
    coast.differential = true;
    auto state = platform.state();
 const auto initial = state;
 auto previous = state;
    double settled_height_m = 0.0;
 bool hull_contact = false;
    for (int step = 0; step < 300; ++step) {
      state = platform.step(coast, dry, kStepS);
 rest_peak_height_step_m = std::max(rest_peak_height_step_m,
 std::abs(state.height_m - previous.height_m));
 rest_peak_planar_speed_mps = std::max(rest_peak_planar_speed_mps,
 std::abs(state.planar.v));
 rest_peak_yaw_rate_radps = std::max(rest_peak_yaw_rate_radps,
 std::abs(state.planar.w));
 rest_peak_specific_force_mps2 = std::max(rest_peak_specific_force_mps2,
 std::sqrt(state.imu_specific_force_mps2.x * state.imu_specific_force_mps2.x +
 state.imu_specific_force_mps2.y * state.imu_specific_force_mps2.y +
 state.imu_specific_force_mps2.z * state.imu_specific_force_mps2.z));
 hull_contact = hull_contact || touching(state, "base_link") ||
 touching(state, "rear_caster_link");
      if (step == 149)
        settled_height_m = state.height_m;
 if (step < 150) {
 previous = state;
        continue;
 }
      measured_left_N += load_on(state, "left_wheel_link");
      measured_right_N += load_on(state, "right_wheel_link");
      measured_caster_N += load_on(state, "rear_caster_wheel_link");
 measured_total_N += load_on(state, "left_wheel_link") +
                          load_on(state, "right_wheel_link") +
                          load_on(state, "rear_caster_wheel_link");
 previous = state;
    }
    measured_left_N /= 150.0;
    measured_right_N /= 150.0;
    measured_caster_N /= 150.0;
    measured_total_N /= 150.0;

    const double expected_caster_N = mass * kGravityMps2 *
 (kDriveAxleX - assembly_com_x_m) / (kDriveAxleX - kCasterContactX);
    const double expected_each_drive_N = (mass * kGravityMps2 - expected_caster_N) * 0.5;
    REQUIRE(std::abs(measured_total_N - mass * kGravityMps2) < mass * kGravityMps2 * 0.12);
 REQUIRE(std::abs(measured_left_N - measured_right_N) < mass * kGravityMps2 * 0.06);
    REQUIRE(std::abs(measured_left_N - expected_each_drive_N) < mass * kGravityMps2 * 0.12);
    REQUIRE(std::abs(measured_caster_N - expected_caster_N) < mass * kGravityMps2 * 0.12);
    REQUIRE(std::abs(state.height_m - settled_height_m) < 0.003);
    REQUIRE(std::abs(state.roll_rad) < 0.01);
    REQUIRE(std::abs(state.pitch_rad) < 0.01);
    REQUIRE(std::abs(state.planar.v) < 0.01);
    REQUIRE(std::abs(state.planar.w) < 0.01);
 // Construction includes a deterministic contact settle followed by a
 // velocity reset. With no command the first public step must therefore not
 // inject a visible launch impulse or create horizontal kinetic energy.
 REQUIRE(rest_peak_height_step_m < 0.0025);
 REQUIRE(rest_peak_planar_speed_mps < 0.025);
 REQUIRE(rest_peak_yaw_rate_radps < 0.025);
 REQUIRE(rest_peak_specific_force_mps2 < 13.0);
 REQUIRE(std::hypot(state.planar.x - initial.planar.x,
 state.planar.y - initial.planar.y) < 0.015);
 REQUIRE(!hull_contact);
  }

  // A full-torque command from exactly zero speed must accelerate through the
  // contact patch without a vertical launch, hull strike, or lost support.
  double launch_peak_accel_mps2 = 0.0;
  {
    sim::platform::SimulatedPlatform platform(map, source_level, robot, caps, terrain,
                                               spawn, 0.0, mass, center_height,
                                               robot.yaw_inertia_kgm2);
    sim::ChassisTelemetry effort;
    effort.differential = true;
    const double stall_torque_Nm = robot.motor.stall_force_N * robot.wheel_radius_m;
    effort.joint_efforts = {{"left", stall_torque_Nm}, {"right", stall_torque_Nm}};
    const auto start = platform.state();
    auto previous = start;
    auto state = start;
    double max_height_delta_m = 0.0;
    double max_pitch_rad = 0.0;
    int full_support_steps = 0;
    bool hull_contact = false;
 // Stall torque is a short-duration peak. Probe only the 300 ms launch
 // transient; holding it for seconds would be an impossible motor command.
 for (int step = 0; step < 15; ++step) {
      state = platform.step(effort, dry, kStepS);
      launch_peak_accel_mps2 = std::max(launch_peak_accel_mps2,
                                        (state.planar.v - previous.planar.v) / kStepS);
      max_height_delta_m = std::max(max_height_delta_m,
                                    std::abs(state.height_m - start.height_m));
      max_pitch_rad = std::max(max_pitch_rad, std::abs(state.pitch_rad));
      full_support_steps += touching(state, "left_wheel_link") &&
                            touching(state, "right_wheel_link") &&
                            touching(state, "rear_caster_wheel_link");
      hull_contact = hull_contact || touching(state, "base_link") ||
                     touching(state, "rear_caster_link");
      previous = state;
    }
    const double force_limited_accel = 2.0 * robot.motor.stall_force_N / mass;
    const double traction_limited_accel = dry.mu_eff * kGravityMps2;
 REQUIRE(state.planar.x > start.planar.x + 0.05);
 REQUIRE(std::abs(state.planar.y - start.planar.y) < 0.01);
 REQUIRE(std::abs(sim::wrap_angle(state.planar.theta - start.planar.theta)) < 0.03);
    REQUIRE(launch_peak_accel_mps2 > 0.5);
    REQUIRE(launch_peak_accel_mps2 < 1.35 * std::min(force_limited_accel,
                                                     traction_limited_accel));
    REQUIRE(max_height_delta_m < 0.025);
    REQUIRE(max_pitch_rad < 0.12);
 REQUIRE(full_support_steps >= 14);
    REQUIRE(!hull_contact);
  }

 // A fixed public step may contain several contact substeps, but the plant
 // must still show convergence when the caller halves that public step. This
 // catches results that are artifacts of one particular frame cadence.
 {
 const auto run_step_size = [&](double dt_s) {
 sim::platform::SimulatedPlatform platform(map, source_level, robot, caps, terrain,
 spawn, 0.0, mass, center_height,
 robot.yaw_inertia_kgm2);
 sim::ChassisTelemetry effort;
 effort.differential = true;
 effort.joint_efforts = {{"left", 3.0}, {"right", 3.0}};
 auto state = platform.state();
 const int steps = static_cast<int>(std::lround(1.0 / dt_s));
 for (int step = 0; step < steps; ++step)
 state = platform.step(effort, dry, dt_s);
 return state;
 };
 const auto coarse = run_step_size(0.02);
 const auto fine = run_step_size(0.01);
 REQUIRE(std::abs(coarse.planar.x - fine.planar.x) < 0.03);
 REQUIRE(std::abs(coarse.planar.y - fine.planar.y) < 0.01);
 REQUIRE(std::abs(coarse.planar.v - fine.planar.v) < 0.08);
 REQUIRE(std::abs(sim::wrap_angle(coarse.planar.theta - fine.planar.theta)) < 0.05);
 REQUIRE(std::abs(coarse.height_m - fine.height_m) < 0.003);
 REQUIRE(std::abs(coarse.pitch_rad - fine.pitch_rad) < 0.02);
 }

 // Maximum-speed straight motion is a caster/joint stability test. A steady
 // circle cannot expose caster shimmy or a left/right contact bias that grows
 // only when the requested curvature is zero.
 double cruise_pitch_range_rad = 0.0;
 double cruise_height_range_m = 0.0;
 double cruise_speed_range_mps = 0.0;
 {
 sim::platform::SimulatedPlatform platform(map, source_level, robot, caps, terrain,
 spawn, 0.0, mass, center_height,
 robot.yaw_inertia_kgm2);
 auto controller = controller_for(robot, mass);
 auto state = platform.state();
 double min_pitch = std::numeric_limits<double>::infinity();
 double max_pitch = -std::numeric_limits<double>::infinity();
 double min_height = std::numeric_limits<double>::infinity();
 double max_height = -std::numeric_limits<double>::infinity();
 double min_speed = std::numeric_limits<double>::infinity();
 double max_speed = -std::numeric_limits<double>::infinity();
 for (int step = 0; step < 300; ++step) {
 controller->set_actuator_feedback(platform.actuator_feedback());
 (void)controller->step(state.planar, {1.0, 0.0}, kStepS, dry);
 state = platform.step(controller->telemetry(), dry, kStepS);
 if (step < 200)
 continue;
 min_pitch = std::min(min_pitch, state.pitch_rad);
 max_pitch = std::max(max_pitch, state.pitch_rad);
 min_height = std::min(min_height, state.height_m);
 max_height = std::max(max_height, state.height_m);
 min_speed = std::min(min_speed, state.planar.v);
 max_speed = std::max(max_speed, state.planar.v);
 }
 cruise_pitch_range_rad = max_pitch - min_pitch;
 cruise_height_range_m = max_height - min_height;
 cruise_speed_range_mps = max_speed - min_speed;
 // These are physical-state limits, independent of rendering. If the client
 // still appears to rock while they hold, the defect is presentation cadence
 // rather than wheel/contact dynamics.
 REQUIRE(cruise_pitch_range_rad < 0.002);
 REQUIRE(cruise_height_range_m < 0.001);
 REQUIRE(cruise_speed_range_mps < 0.02);
 }
 double high_speed_straight_v_mps = 0.0;
 double high_speed_straight_lateral_drift_m = 0.0;
 {
 sim::platform::SimulatedPlatform platform(map, source_level, robot, caps, terrain,
 spawn, 0.0, mass, center_height,
 robot.yaw_inertia_kgm2);
 auto controller = controller_for(robot, mass);
 const auto start = platform.state();
 auto state = start;
 int full_support_steps = 0;
 double peak_yaw_rad = 0.0;
 double peak_roll_rad = 0.0;
 for (int step = 0; step < 300; ++step) {
 controller->set_actuator_feedback(platform.actuator_feedback());
 (void)controller->step(state.planar, {robot.motor.max_wheel_speed_mps, 0.0},
 kStepS, dry);
 state = platform.step(controller->telemetry(), dry, kStepS);
 if (step >= 200) {
 high_speed_straight_v_mps += state.planar.v;
 full_support_steps += touching(state, "left_wheel_link") &&
 touching(state, "right_wheel_link") &&
 touching(state, "rear_caster_wheel_link");
 }
 peak_yaw_rad = std::max(peak_yaw_rad,
 std::abs(sim::wrap_angle(state.planar.theta - start.planar.theta)));
 peak_roll_rad = std::max(peak_roll_rad, std::abs(state.roll_rad));
 }
 high_speed_straight_v_mps /= 100.0;
 high_speed_straight_lateral_drift_m = std::abs(state.planar.y - start.planar.y);
 REQUIRE(high_speed_straight_v_mps > robot.motor.max_wheel_speed_mps * 0.75);
 REQUIRE(high_speed_straight_v_mps < robot.motor.max_wheel_speed_mps * 1.08);
 REQUIRE(high_speed_straight_lateral_drift_m < 0.12);
 REQUIRE(peak_yaw_rad < 0.08);
 REQUIRE(peak_roll_rad < 0.08);
 REQUIRE(full_support_steps == 100);
 REQUIRE(!touching(state, "base_link"));
 REQUIRE(!touching(state, "rear_caster_link"));
 }

 // Differential spin is a coupled joint/contact test: equal opposite shaft
 // efforts should create yaw about the driven axle without translating or
 // ejecting the passive caster from the support set.
 double spin_yaw_rad = 0.0;
 {
 sim::platform::SimulatedPlatform platform(map, source_level, robot, caps, terrain,
 spawn, 0.0, mass, center_height,
 robot.yaw_inertia_kgm2);
 const auto start = platform.state();
 auto controller = controller_for(robot, mass);
 auto state = start;
 auto previous = start;
 int full_support_steps = 0;
 double max_height_delta_m = 0.0;
 for (int step = 0; step < 150; ++step) {
 controller->set_actuator_feedback(platform.actuator_feedback());
 (void)controller->step(state.planar, {0.0, 0.80}, kStepS, dry);
 state = platform.step(controller->telemetry(), dry, kStepS);
 spin_yaw_rad += sim::wrap_angle(state.planar.theta - previous.planar.theta);
 full_support_steps += touching(state, "left_wheel_link") &&
 touching(state, "right_wheel_link") &&
 touching(state, "rear_caster_wheel_link");
 max_height_delta_m = std::max(max_height_delta_m,
 std::abs(state.height_m - start.height_m));
 previous = state;
 }
 const auto feedback = platform.actuator_feedback();
 const auto* left = feedback.find("left");
 const auto* right = feedback.find("right");
 REQUIRE(left);
 REQUIRE(right);
 const sim::Vec2 start_axle{
 start.planar.x + 0.12 * std::cos(start.planar.theta),
 start.planar.y + 0.12 * std::sin(start.planar.theta),
 };
 const sim::Vec2 end_axle{
 state.planar.x + 0.12 * std::cos(state.planar.theta),
 state.planar.y + 0.12 * std::sin(state.planar.theta),
 };
 const double axle_drift_m = std::hypot(end_axle.x - start_axle.x,
 end_axle.y - start_axle.y);
 REQUIRE(spin_yaw_rad > 1.5);
 REQUIRE(axle_drift_m < 0.20);
 REQUIRE(left->position_rad < 0.0);
 REQUIRE(right->position_rad > 0.0);
 REQUIRE(full_support_steps >= 145);
 REQUIRE(max_height_delta_m < 0.025);
 REQUIRE(!touching(state, "base_link"));
 REQUIRE(!touching(state, "rear_caster_link"));
 }

 // Friction acceptance has both a behavioral oracle and a contact-force
 // oracle. Low mu must produce more tread travel than chassis travel and the
 // resolved tangent force must remain inside Bullet's two-axis friction
 // pyramid. Its vector magnitude is bounded by sqrt(2) * mu * normal load.
 struct TractionResult {
 double travel_m{0.0};
 double tread_travel_m{0.0};
 double max_friction_utilization{0.0};
 double max_slip_ratio{0.0};
 bool slipping{false};
 };
 const auto traction_run = [&](double friction_mu) {
 sim::platform::SimulatedPlatform platform(map, source_level, robot, caps, terrain,
 spawn, 0.0, mass, center_height,
 robot.yaw_inertia_kgm2);
 const auto start = platform.state();
 sim::ChassisTelemetry effort;
 effort.differential = true;
 const double stall_torque_Nm = robot.motor.stall_force_N * robot.wheel_radius_m;
 effort.joint_efforts = {{"left", stall_torque_Nm}, {"right", stall_torque_Nm}};
 const sim::GroundContact contact{friction_mu, 0.01, 6.0, 40000.0, 1000.0};
 TractionResult result;
 auto state = start;
 for (int step = 0; step < 75; ++step) {
 state = platform.step(effort, contact, kStepS);
 double normal_N = 0.0;
 double tangent_N = 0.0;
 for (const auto& load : state.ground_contact_loads) {
 normal_N += load.normal_force_N;
 tangent_N += load.tangent_force_N;
 }
 if (normal_N > 1.0 && step > 5) {
 result.max_friction_utilization = std::max(result.max_friction_utilization,
 tangent_N / (friction_mu * normal_N));
 }
 result.slipping = result.slipping || state.drive_slipping;
 result.max_slip_ratio = std::max(result.max_slip_ratio, state.max_drive_slip_ratio);
 }
 const auto feedback = platform.actuator_feedback();
 const auto* left = feedback.find("left");
 const auto* right = feedback.find("right");
 if (!left || !right)
 throw std::runtime_error("RO-1 traction test lost drive feedback");
 result.travel_m = std::hypot(state.planar.x - start.planar.x,
 state.planar.y - start.planar.y);
 result.tread_travel_m = 0.5 * (std::abs(left->position_rad) +
 std::abs(right->position_rad)) * robot.wheel_radius_m;
 return result;
 };
 const auto dry_traction = traction_run(1.0);
 const auto low_traction = traction_run(0.08);
 REQUIRE(dry_traction.travel_m > low_traction.travel_m + 0.12);
 REQUIRE(low_traction.tread_travel_m > low_traction.travel_m + 0.15);
 REQUIRE(low_traction.slipping);
 REQUIRE(low_traction.max_slip_ratio > 0.25);
 REQUIRE(low_traction.max_friction_utilization <= std::sqrt(2.0) * 1.05);

  // Emergency braking: stopping distance cannot beat the motor/traction
  // energy bound, and longitudinal load transfer must unload the rear caster.
  double brake_initial_speed_mps = 0.0;
  double brake_distance_m = 0.0;
  {
    sim::platform::SimulatedPlatform platform(map, source_level, robot, caps, terrain,
                                               spawn, 0.0, mass, center_height,
                                               robot.yaw_inertia_kgm2);
    sim::ChassisTelemetry drive;
    drive.differential = true;
    drive.joint_efforts = {{"left", 3.0}, {"right", 3.0}};
    auto state = platform.state();
    double cruise_caster_N = 0.0;
    for (int step = 0; step < 80; ++step) {
      state = platform.step(drive, dry, kStepS);
      if (step >= 60)
        cruise_caster_N += load_on(state, "rear_caster_wheel_link");
    }
    cruise_caster_N /= 20.0;
    brake_initial_speed_mps = state.planar.v;
    const double brake_start_x = state.planar.x;
    const double brake_torque_Nm = robot.motor.stall_force_N * robot.wheel_radius_m;
    drive.joint_efforts = {{"left", -brake_torque_Nm}, {"right", -brake_torque_Nm}};
    double braking_caster_N = 0.0;
    int braking_load_samples = 0;
    bool stopped = false;
    for (int step = 0; step < 100; ++step) {
      state = platform.step(drive, dry, kStepS);
      if (step < 10) {
        braking_caster_N += load_on(state, "rear_caster_wheel_link");
        ++braking_load_samples;
      }
      if (state.planar.v <= 0.02) {
        brake_distance_m = state.planar.x - brake_start_x;
        stopped = true;
        break;
      }
    }
    braking_caster_N /= std::max(1, braking_load_samples);
    const double max_decel_mps2 = std::min(2.0 * robot.motor.stall_force_N / mass,
                                           dry.mu_eff * kGravityMps2);
    const double ideal_min_distance_m = brake_initial_speed_mps * brake_initial_speed_mps /
                                        (2.0 * max_decel_mps2);
    REQUIRE(brake_initial_speed_mps > 0.7);
    REQUIRE(stopped);
    REQUIRE(brake_distance_m > ideal_min_distance_m * 0.70);
    REQUIRE(brake_distance_m < ideal_min_distance_m * 3.5 + 0.15);
    REQUIRE(braking_caster_N < cruise_caster_N);
    REQUIRE(!touching(state, "base_link"));
    REQUIRE(!touching(state, "rear_caster_link"));
  }

  // ISO 4138 uses a steady circular maneuver to expose the relationship
  // between speed, curvature and lateral acceleration. For a differential
  // robot the same independent invariant is a_y = v * yaw_rate after the
  // caster transient has settled.
  double circle_v_mps = 0.0;
  double circle_w_radps = 0.0;
  double circle_ay_mps2 = 0.0;
 double circle_encoder_w_radps = 0.0;
 double circle_peak_slip_ratio = 0.0;
  {
    sim::platform::SimulatedPlatform platform(map, source_level, robot, caps, terrain,
                                               spawn, 0.0, mass, center_height,
                                               robot.yaw_inertia_kgm2);
    auto controller = controller_for(robot, mass);
    auto state = platform.state();
    double sum_v = 0.0;
    double sum_w = 0.0;
    double sum_ay = 0.0;
 double sum_encoder_w = 0.0;
    int samples = 0;
    int full_support_steps = 0;
    double peak_roll_rad = 0.0;
    for (int step = 0; step < 300; ++step) {
      controller->set_actuator_feedback(platform.actuator_feedback());
 (void)controller->step(state.planar, {1.70, 1.10}, kStepS, dry);
      state = platform.step(controller->telemetry(), dry, kStepS);
      if (step < 200)
        continue;
      sum_v += state.planar.v;
      sum_w += state.planar.w;
      sum_ay += state.imu_specific_force_mps2.y;
 const auto feedback = platform.actuator_feedback();
 const auto* left = feedback.find("left");
 const auto* right = feedback.find("right");
 REQUIRE(left);
 REQUIRE(right);
 sum_encoder_w += (right->velocity_radps - left->velocity_radps) *
 robot.wheel_radius_m / robot.wheel_base_m;
 circle_peak_slip_ratio = std::max(circle_peak_slip_ratio, state.max_drive_slip_ratio);
      ++samples;
      full_support_steps += touching(state, "left_wheel_link") &&
                            touching(state, "right_wheel_link") &&
                            touching(state, "rear_caster_wheel_link");
      peak_roll_rad = std::max(peak_roll_rad, std::abs(state.roll_rad));
    }
    circle_v_mps = sum_v / samples;
    circle_w_radps = sum_w / samples;
    circle_ay_mps2 = sum_ay / samples;
 circle_encoder_w_radps = sum_encoder_w / samples;
 REQUIRE(circle_v_mps > 1.10);
 REQUIRE(circle_w_radps > 0.65);
 REQUIRE(std::abs(circle_ay_mps2 - circle_v_mps * circle_w_radps) < 0.60);
    REQUIRE(std::abs(circle_ay_mps2) < dry.mu_eff * kGravityMps2);
 REQUIRE(std::abs(circle_w_radps - circle_encoder_w_radps) < 0.55);
    REQUIRE(full_support_steps >= 95);
    REQUIRE(peak_roll_rad < 0.12);
  }

  // Grade capability is a boundary, not a single successful 20-degree run.
  // The same stall effort must climb below the analytic motor limit and roll
  // back above it. A no-effort plant must also backdrive on the lower grade.
  double low_grade_progress_m = 0.0;
  double high_grade_progress_m = 0.0;
  double coast_progress_m = 0.0;
 const auto run_grade = [&](double grade, double torque_Nm,
 const sim::GroundContact& contact) {
    const auto grade_map = slope_map(source_map, grade, 0.0);
    const auto grade_terrain = terrain_for(grade_map, source_level);
    sim::platform::SimulatedPlatform platform(grade_map, source_level, robot, caps,
                                               grade_terrain, {8.0, 10.0}, 0.0,
                                               mass, center_height,
                                               robot.yaw_inertia_kgm2);
    const auto start = platform.state();
    sim::ChassisTelemetry effort;
    effort.differential = true;
    if (std::abs(torque_Nm) > 0.0)
      effort.joint_efforts = {{"left", torque_Nm}, {"right", torque_Nm}};
    auto state = start;
    for (int step = 0; step < 75; ++step)
 state = platform.step(effort, contact, kStepS);
    return state.planar.x - start.planar.x;
  };
  const double stall_torque_Nm = robot.motor.stall_force_N * robot.wheel_radius_m;
 low_grade_progress_m = run_grade(0.20, stall_torque_Nm, dry);
 high_grade_progress_m = run_grade(0.65, stall_torque_Nm, dry);
 coast_progress_m = run_grade(0.20, 0.0, dry);
  const double analytic_motor_limit_rad =
      std::asin(std::clamp(2.0 * robot.motor.stall_force_N /
                           (mass * kGravityMps2), 0.0, 1.0));
 // Probe close to both sides of the motor-only boundary with deliberately
 // high grip. This separates torque authority from the tire-adhesion limit.
 const sim::GroundContact grade_high_grip{2.0, 0.0, 6.0, 40000.0, 1000.0};
 const double below_motor_limit_grade = std::tan(analytic_motor_limit_rad - 0.06);
 const double above_motor_limit_grade = std::tan(analytic_motor_limit_rad + 0.06);
 const double near_limit_climb_m =
 run_grade(below_motor_limit_grade, stall_torque_Nm, grade_high_grip);
 const double beyond_limit_rollback_m =
 run_grade(above_motor_limit_grade, stall_torque_Nm, grade_high_grip);
 // On the default surface only the two front wheels are driven. Static
 // equilibrium gives their uphill normal load as
 // mg*((x_com-x_caster)*cos(theta)-h*sin(theta))/support_length.
 // Equating mu*N_drive to mg*sin(theta) gives an independent traction-grade
 // boundary. Probe either side without changing the authored plant.
 constexpr double kSupportLengthM = kDriveAxleX - kCasterContactX;
 const double drive_axle_arm_m = assembly_com_x_m - kCasterContactX;
 const double analytic_traction_limit_grade =
 dry.mu_eff * drive_axle_arm_m /
 (kSupportLengthM + dry.mu_eff * center_height);
 const double traction_limit_rad = std::atan(analytic_traction_limit_grade);
 const double below_traction_limit_grade = std::tan(traction_limit_rad - 0.06);
 const double above_traction_limit_grade = std::tan(traction_limit_rad + 0.06);
 const double near_traction_climb_m =
 run_grade(below_traction_limit_grade, stall_torque_Nm, dry);
 const double beyond_traction_rollback_m =
 run_grade(above_traction_limit_grade, stall_torque_Nm, dry);
  REQUIRE(std::atan(0.20) < analytic_motor_limit_rad);
  REQUIRE(std::atan(0.65) > analytic_motor_limit_rad);
  REQUIRE(low_grade_progress_m > 0.20);
  REQUIRE(high_grade_progress_m < -0.05);
  REQUIRE(coast_progress_m < -0.05);
 REQUIRE(near_limit_climb_m > 0.015);
 REQUIRE(beyond_limit_rollback_m < -0.015);
 REQUIRE(traction_limit_rad < analytic_motor_limit_rad);
 REQUIRE(near_traction_climb_m > 0.015);
 REQUIRE(beyond_traction_rollback_m < -0.015);

 // A discrete curb and drop exercise clearance and short contact loss in a
 // way the smooth rough-strip fixture cannot. The 30 mm boundary is an
 // explicit tested envelope, not a claim that RO-1 can climb any obstacle
 // below its nominal hull clearance.
 double curb_peak_specific_force_mps2 = 0.0;
 {
 auto curb_map = empty_map(source_map);
 curb_map.terrain.push_back({"curb_deck", {11.0, 9.0}, {13.0, 11.0}, 0.03,
 0.0, 0.0, 0.0, {}});
 const auto curb_terrain = terrain_for(curb_map, source_level);
 sim::platform::SimulatedPlatform platform(curb_map, source_level, robot, caps,
 curb_terrain, {10.0, 10.0}, 0.0,
 mass, center_height,
 robot.yaw_inertia_kgm2);
 auto controller = controller_for(robot, mass);
 const auto start = platform.state();
 auto previous = start;
 auto state = start;
 double max_height_step_m = 0.0;
 double max_pitch_rad = 0.0;
 int unsupported_steps = 0;
 int partial_support_steps = 0;
 bool hull_contact = false;
 bool reached_deck = false;
 bool left_deck = false;
 for (int step = 0; step < 500; ++step) {
 controller->set_actuator_feedback(platform.actuator_feedback());
 (void)controller->step(state.planar, {0.45, 0.0}, kStepS, dry);
 state = platform.step(controller->telemetry(), dry, kStepS);
 max_height_step_m = std::max(max_height_step_m,
 std::abs(state.height_m - previous.height_m));
 max_pitch_rad = std::max(max_pitch_rad, std::abs(state.pitch_rad));
 curb_peak_specific_force_mps2 = std::max(curb_peak_specific_force_mps2,
 std::sqrt(state.imu_specific_force_mps2.x * state.imu_specific_force_mps2.x +
 state.imu_specific_force_mps2.y * state.imu_specific_force_mps2.y +
 state.imu_specific_force_mps2.z * state.imu_specific_force_mps2.z));
 const bool any_support = touching(state, "left_wheel_link") ||
 touching(state, "right_wheel_link") ||
 touching(state, "rear_caster_wheel_link");
 const bool full_support = touching(state, "left_wheel_link") &&
 touching(state, "right_wheel_link") &&
 touching(state, "rear_caster_wheel_link");
 unsupported_steps += !any_support;
 partial_support_steps += !full_support;
 hull_contact = hull_contact || touching(state, "base_link") ||
 touching(state, "rear_caster_link");
 reached_deck = reached_deck || state.planar.x > 11.4;
 left_deck = left_deck || state.planar.x > 13.4;
 previous = state;
 }
 REQUIRE(reached_deck);
 REQUIRE(left_deck);
 REQUIRE(state.planar.x > 13.8);
 REQUIRE(!hull_contact);
 REQUIRE(unsupported_steps <= 2);
 REQUIRE(partial_support_steps > 0);
 REQUIRE(touching(state, "left_wheel_link"));
 REQUIRE(touching(state, "right_wheel_link"));
 REQUIRE(touching(state, "rear_caster_wheel_link"));
 REQUIRE(max_height_step_m < 0.02);
 REQUIRE(max_pitch_rad < 0.30);
 REQUIRE(curb_peak_specific_force_mps2 > 9.82);
 REQUIRE(curb_peak_specific_force_mps2 < 35.0);
 REQUIRE(std::abs(state.height_m - start.height_m) < 0.02);
 }

 // Static cross-slope boundary probe. High grip isolates support-polygon
 // tipping from simple downhill sliding; the uphill wheel should retain load
 // below the analytic boundary and unload above it.
 const auto run_cross_slope = [&](double grade_y) {
 auto grip_robot = robot;
 for (auto& link : grip_robot.articulation.links) {
 if (link.id == "left_wheel_link" || link.id == "right_wheel_link" ||
 link.id == "rear_caster_wheel_link")
 link.friction = 2.0;
 }
 const auto cross_map = slope_map(source_map, 0.0, grade_y);
 const auto cross_terrain = terrain_for(cross_map, source_level);
 sim::platform::SimulatedPlatform platform(cross_map, source_level, grip_robot, caps,
 cross_terrain, {8.0, 10.0}, 0.0,
 mass, center_height,
 robot.yaw_inertia_kgm2);
 sim::ChassisTelemetry coast;
 coast.differential = true;
 sim::platform::RigidVehicleState state;
 double left_N = 0.0;
 double right_N = 0.0;
 int samples = 0;
 bool hull_contact = false;
 double probe_roll_rad = 0.0;
 for (int step = 0; step < 50; ++step) {
 state = platform.step(coast, {1.0, 0.0, 6.0, 40000.0, 1000.0}, kStepS);
 if (step >= 10 && step < 20) {
 left_N += load_on(state, "left_wheel_link");
 right_N += load_on(state, "right_wheel_link");
 ++samples;
 }
 if (step == 19)
 probe_roll_rad = state.roll_rad;
 hull_contact = hull_contact || touching(state, "base_link") ||
 touching(state, "rear_caster_link");
 }
 return std::tuple{left_N / samples, right_N / samples,
 probe_roll_rad, state.roll_rad, hull_contact, state.planar.y};
 };
 const auto cross_below = run_cross_slope(0.60);
 const auto cross_above = run_cross_slope(1.20);
 const double lateral_margin_m = 0.29 *
 (assembly_com_x_m - (-0.315)) / (0.12 - (-0.315));
 const double static_tip_grade = lateral_margin_m / center_height;
 REQUIRE(0.60 < static_tip_grade);
 REQUIRE(1.20 > static_tip_grade);
 REQUIRE(std::abs(std::get<2>(cross_below) - std::atan(0.60)) < 0.04);
 REQUIRE(std::get<0>(cross_below) > 20.0);
 REQUIRE(std::get<1>(cross_below) > 20.0);
 REQUIRE(!std::get<4>(cross_below));
 REQUIRE(std::get<0>(cross_above) < std::get<0>(cross_below) * 0.30);
 REQUIRE(std::abs(std::get<3>(cross_above) - std::get<2>(cross_above)) > 0.70);

 // Cross-slope adhesion is distinct from the geometric tip boundary. Keep
 // the angle below the support-polygon limit and vary only surface friction;
 // high grip should hold while low grip must slide measurably downhill.
 const auto cross_adhesion = [&](double friction_scale) {
 const auto cross_map = slope_map(source_map, 0.0, 0.35);
 const auto cross_terrain = terrain_for(cross_map, source_level);
 sim::platform::SimulatedPlatform platform(cross_map, source_level, robot, caps,
 cross_terrain, {8.0, 10.0}, 0.0,
 mass, center_height,
 robot.yaw_inertia_kgm2);
 const auto start = platform.state();
 sim::ChassisTelemetry coast;
 coast.differential = true;
 sim::platform::RigidVehicleState state;
 const sim::GroundContact contact{friction_scale, 0.0, 6.0, 40000.0, 1000.0};
 double peak_slip = 0.0;
 // Keep this a short contact probe. Over a longer interval the passive caster
 // can legitimately yaw the unbraked robot toward the fall line, after which
 // the freely backdrivable drive wheels roll rather than slide laterally.
 for (int step = 0; step < 25; ++step) {
 state = platform.step(coast, contact, kStepS);
 peak_slip = std::max(peak_slip, state.max_drive_slip_ratio);
 }
 return std::tuple{
 std::hypot(state.planar.x - start.planar.x, state.planar.y - start.planar.y),
 state.planar.y - start.planar.y,
 peak_slip,
 touching(state, "left_wheel_link") || touching(state, "right_wheel_link") ||
 touching(state, "rear_caster_wheel_link")};
 };
 const auto cross_grip = cross_adhesion(1.0);
 const auto cross_slide = cross_adhesion(0.10);
 REQUIRE(std::get<0>(cross_grip) < 0.10);
 REQUIRE(std::get<0>(cross_slide) > std::get<0>(cross_grip) + 0.20);
 REQUIRE(std::get<1>(cross_slide) < -0.20);
 REQUIRE(std::get<3>(cross_grip));
 REQUIRE(std::get<3>(cross_slide));

  std::cout << "RO-1 dynamics acceptance: load=" << measured_total_N
            << " N (L/R/caster=" << measured_left_N << '/' << measured_right_N << '/'
 << measured_caster_N << "), launch_peak=" << launch_peak_accel_mps2
 << " m/s^2, rest=(dz=" << rest_peak_height_step_m
 << ", v=" << rest_peak_planar_speed_mps
 << ", imu=" << rest_peak_specific_force_mps2
 << "), straight=(v=" << high_speed_straight_v_mps
 << ", drift=" << high_speed_straight_lateral_drift_m
 << "), cruise-ripple=(pitch=" << cruise_pitch_range_rad
 << ", height=" << cruise_height_range_m
 << ", speed=" << cruise_speed_range_mps
 << "), spin=" << spin_yaw_rad << " rad, low-mu utilization="
 << low_traction.max_friction_utilization
 << ", brake=" << brake_initial_speed_mps << " m/s in "
            << brake_distance_m << " m, circle=(v=" << circle_v_mps << ", w="
 << circle_w_radps << ", encoder_w=" << circle_encoder_w_radps
 << ", ay=" << circle_ay_mps2 << ", slip=" << circle_peak_slip_ratio
 << "), curb_peak=" << curb_peak_specific_force_mps2
 << " m/s^2, cross_tip_grade=" << static_tip_grade
 << ", grades=(low="
            << low_grade_progress_m << ", high=" << high_grade_progress_m
 << ", coast=" << coast_progress_m << ", near=" << near_limit_climb_m
 << ", beyond=" << beyond_limit_rollback_m
 << ", traction_near=" << near_traction_climb_m
 << ", traction_beyond=" << beyond_traction_rollback_m
 << "), cross_slide=" << std::get<0>(cross_slide) << " m\n";
  return 0;
}
