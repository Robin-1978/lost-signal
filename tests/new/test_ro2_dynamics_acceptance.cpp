#include "robot/control/ackermann_actuator_controller.hpp"
#include "sim/game/mission/content_catalog.hpp"
#include "sim/game/platform/simulated_platform.hpp"
#include "sim/hardware/vehicle/ackermann.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string_view>
#include <vector>

#define REQUIRE(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": requirement failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

namespace {

constexpr double kStepS = 0.02;
constexpr double kGravityMps2 = 9.81;

mission::MapDefinition empty_map(const mission::MapDefinition& source) {
 auto map = source;
 map.id = "ro2_dynamics_acceptance";
 map.size_m = {40.0, 20.0};
 map.obstacles.clear();
 map.surfaces.clear();
 map.terrain.clear();
 map.actors.clear();
 return map;
}

mission::MapDefinition slope_map(const mission::MapDefinition& source,
 double grade_x, double grade_y, double roughness = 0.0) {
 auto map = empty_map(source);
 map.terrain.push_back({"ro2_acceptance_surface", {0.5, 0.5}, {39.5, 19.5},
 0.0, grade_x, grade_y, roughness, {}});
 return map;
}

bool touching(const sim::platform::RigidVehicleState& state,
 std::string_view link) {
 return std::find(state.ground_contact_links.begin(),
 state.ground_contact_links.end(), link) != state.ground_contact_links.end();
}

double load_on(const sim::platform::RigidVehicleState& state,
 std::string_view link) {
 const auto found = std::find_if(state.ground_contact_loads.begin(),
 state.ground_contact_loads.end(),
 [&](const auto& value) { return value.link == link; });
 return found == state.ground_contact_loads.end() ? 0.0 : found->normal_force_N;
}

double tangent_force_on(const sim::platform::RigidVehicleState& state,
 std::string_view link) {
 const auto found = std::find_if(state.ground_contact_loads.begin(),
 state.ground_contact_loads.end(),
 [&](const auto& value) { return value.link == link; });
 return found == state.ground_contact_loads.end() ? 0.0 : found->tangent_force_N;
}

sim::ChassisCaps caps_for(const mission::RobotDefinition& robot) {
 sim::ChassisCaps caps;
 caps.can_spin = false;
 caps.command_space = sim::ChassisCommandSpace::SteeringAngle;
 caps.geometry.support_width_m = robot.track_width_m;
 caps.geometry.support_length_m = robot.wheel_base_m;
 caps.geometry.steering_wheel_base_m = robot.wheel_base_m;
 caps.geometry.max_steer_rad = robot.steering.max_steer_rad;
 caps.geometry.max_steer_rate_radps = robot.steering.max_steer_rate_radps;
 return caps;
}

struct Rig {
 sim::AckermannDrive plant;
 sim::AckermannActuatorController controller;

 explicit Rig(const mission::RobotDefinition& robot)
 : plant({robot.wheel_base_m, robot.wheel_radius_m, robot.track_width_m,
 robot.steering.max_steer_rad, robot.steering.max_steer_rate_radps,
 robot.steering.servo_response_s, robot.motor.max_wheel_speed_mps,
 robot.motor.max_wheel_accel_mps2, robot.motor.stall_force_N,
 robot.motor.free_speed_mps, robot.mass_kg, robot.yaw_inertia_kgm2,
 robot.motor.pwm_limit, robot.motor.integral_limit,
 robot.encoder.counts_per_revolution, robot.motor.wheel_equiv_mass_kg,
 robot.steering.stall_torque_Nm,
 robot.service_brake.enabled ? robot.service_brake.maximum_torque_Nm : 0.0,
 robot.service_brake.apply_time_s, robot.service_brake.release_time_s,
 robot.service_brake.low_speed_smoothing_radps}, {}, 1.0),
 controller({robot.wheel_base_m, robot.track_width_m, robot.wheel_radius_m,
 robot.motor.max_wheel_speed_mps, robot.steering.max_steer_rad,
 robot.steering.max_steer_rate_radps,
 robot.encoder.counts_per_revolution, robot.motor.pwm_limit,
 robot.motor.integral_limit, 1.0, robot.steering.integral_limit,
 1.0 / robot.steering.servo_response_s, 0.35,
 robot.service_brake.enabled,
 robot.service_brake.low_speed_smoothing_radps * robot.wheel_radius_m},
 sim::tune_wheel_pid(robot.motor.wheel_equiv_mass_kg + robot.mass_kg * 0.5,
 robot.motor.stall_force_N, robot.motor.free_speed_mps,
 robot.motor.max_wheel_speed_mps * 0.5,
 {robot.motor.controller_bandwidth_radps,
 robot.motor.controller_integral_time_s,
 robot.motor.controller_derivative_time_s})) {}

 sim::ChassisTelemetry step(sim::platform::SimulatedPlatform& platform,
 const sim::DiffState& state, sim::DiffCmd command,
 const sim::GroundContact& ground, double dt_s = kStepS) {
 const auto feedback = platform.actuator_feedback();
 const auto* rear_left = feedback.find("rear_left_drive");
 const auto* rear_right = feedback.find("rear_right_drive");
 const auto* front_left = feedback.find("front_left_steer");
 const auto* front_right = feedback.find("front_right_steer");
 if (!rear_left || !rear_right || !front_left || !front_right)
 throw std::runtime_error("RO-2 feedback incomplete");
 const auto output = controller.update(command, {
 rear_left->position_rad, rear_right->position_rad,
 rear_left->velocity_radps, rear_right->velocity_radps,
 front_left->position_rad, front_right->position_rad,
 front_left->velocity_radps, front_right->velocity_radps, true}, dt_s);
 plant.set_actuator_feedback(feedback);
 (void)plant.step_actuator_duty(state, command, {{
 {"rear_left_drive", output.rear_left.normalized_duty},
 {"rear_right_drive", output.rear_right.normalized_duty},
 {"front_left_steer", output.front_left_steer.normalized_duty},
 {"front_right_steer", output.front_right_steer.normalized_duty},
 {"rear_left_brake", output.rear_left_brake_request},
 {"rear_right_brake", output.rear_right_brake_request},
 }}, dt_s, ground);
 return plant.telemetry();
 }
};

} // namespace

int main() {
 REQUIRE(sim::physics::bullet_physics_available());
 const auto catalog = mission::ContentCatalog::load(LOST_SIGNAL_TEST_ASSETS);
 const auto& level = catalog.level("motor_trial");
 const auto map = empty_map(catalog.map(level.map));
 const auto& robot = catalog.robot("robot.ro2");
 const auto caps = caps_for(robot);
 const sim::TerrainModel terrain;
 const sim::GroundContact dry{1.0, 0.01, 6.0, 40000.0, 1000.0};
 const sim::Vec2 spawn{10.0, 10.0};

 REQUIRE(robot.articulation.links.size() == 7);
 REQUIRE(robot.articulation.joints.size() == 6);
 REQUIRE(robot.articulation.actuators.size() == 6);
 REQUIRE(robot.service_brake.enabled);
 REQUIRE(robot.service_brake.maximum_torque_Nm > 0.0);

 // RO-2 is a small 0.62 m-wheelbase teaching car. Its optional high-speed
 // gearing is still a physical configuration: prove that it exceeds the
 // standard profile while retaining straight-line support and closed-loop
 // braking instead of merely increasing a UI speed limit.
 {
 auto fast_robot = robot;
 const auto& fast = catalog.component("motor.speed").behavior;
 fast_robot.motor.max_wheel_speed_mps = fast.at("max_wheel_speed_mps");
 fast_robot.motor.stall_force_N = fast.at("stall_force_N");
 fast_robot.motor.free_speed_mps = fast.at("free_speed_mps");
 fast_robot.motor.controller_bandwidth_radps = fast.at("controller_bandwidth_radps");
 fast_robot.motor.controller_integral_time_s = fast.at("controller_integral_time_s");
 fast_robot.motor.controller_derivative_time_s = fast.at("controller_derivative_time_s");
 fast_robot.motor.coulomb_smoothing_speed_mps = fast.at("coulomb_smoothing_speed_mps");
 fast_robot.motor.viscous_drag_N_per_mps = fast.at("viscous_drag_N_per_mps");
 sim::platform::SimulatedPlatform platform(map, level, fast_robot, caps, terrain,
 spawn, 0.0, fast_robot.mass_kg, fast_robot.center_of_mass_height_m,
 fast_robot.yaw_inertia_kgm2);
 Rig rig(fast_robot);
 auto state = platform.state();
 const auto start = state;
 int full_support_steps = 0;
 double peak_yaw_rad = 0.0;
 for (int step = 0; step < 250; ++step) {
 const auto actuation = rig.step(platform, state.planar,
 {fast_robot.motor.max_wheel_speed_mps, 0.0}, dry);
 state = platform.step(actuation, dry, kStepS);
 full_support_steps += touching(state, "front_left_wheel_link") &&
 touching(state, "front_right_wheel_link") &&
 touching(state, "rear_left_wheel_link") &&
 touching(state, "rear_right_wheel_link");
 peak_yaw_rad = std::max(peak_yaw_rad,
 std::abs(sim::wrap_angle(state.planar.theta - start.planar.theta)));
 }
 const double brake_start_x = state.planar.x;
 const double speed_before_brake_mps = state.planar.v;
 double minimum_braking_speed_mps = speed_before_brake_mps;
 for (int step = 0; step < 200; ++step) {
 const auto actuation = rig.step(platform, state.planar, {0.0, 0.0}, dry);
 state = platform.step(actuation, dry, kStepS);
 minimum_braking_speed_mps = std::min(minimum_braking_speed_mps, state.planar.v);
 }
 std::cerr << "RO2 high-speed v=" << speed_before_brake_mps
 << " brake=" << state.planar.x - brake_start_x << '\n';
 REQUIRE(speed_before_brake_mps > robot.motor.max_wheel_speed_mps * 1.15);
 REQUIRE(std::abs(state.planar.y - start.planar.y) < 0.25);
 REQUIRE(peak_yaw_rad < 0.10);
 REQUIRE(full_support_steps >= 245);
 REQUIRE(std::abs(state.planar.v) < 0.10);
 REQUIRE(minimum_braking_speed_mps > -0.20);
 const double peak_motor_decel_mps2 =
 2.0 * fast_robot.motor.stall_force_N / fast_robot.mass_kg;
 const double ideal_min_brake_m = speed_before_brake_mps * speed_before_brake_mps /
 (2.0 * peak_motor_decel_mps2);
 REQUIRE(state.planar.x - brake_start_x > ideal_min_brake_m * 0.85);
 REQUIRE(state.planar.x - brake_start_x < ideal_min_brake_m * 1.6);
 }

 {
 sim::platform::SimulatedPlatform platform(map, level, robot, caps, terrain,
 spawn, 0.0, robot.mass_kg, robot.center_of_mass_height_m,
 robot.yaw_inertia_kgm2);
 sim::ChassisTelemetry coast;
 sim::platform::RigidVehicleState state;
const auto initial = platform.state();
 double front_left_load_N = 0.0;
 double front_right_load_N = 0.0;
 double rear_left_load_N = 0.0;
 double rear_right_load_N = 0.0;
 for (int step = 0; step < 300; ++step) {
 state = platform.step(coast, dry, kStepS);
 if (step >= 200) {
 front_left_load_N += load_on(state, "front_left_wheel_link");
 front_right_load_N += load_on(state, "front_right_wheel_link");
 rear_left_load_N += load_on(state, "rear_left_wheel_link");
 rear_right_load_N += load_on(state, "rear_right_wheel_link");
 }
 }
 front_left_load_N /= 100.0;
 front_right_load_N /= 100.0;
 rear_left_load_N /= 100.0;
 rear_right_load_N /= 100.0;
 REQUIRE(touching(state, "front_left_wheel_link"));
 REQUIRE(touching(state, "front_right_wheel_link"));
 REQUIRE(touching(state, "rear_left_wheel_link"));
 REQUIRE(touching(state, "rear_right_wheel_link"));
 REQUIRE(!touching(state, "base_link"));
 REQUIRE(std::abs(state.roll_rad) < 0.02);
 REQUIRE(std::abs(state.pitch_rad) < 0.02);
 REQUIRE(std::hypot(state.planar.x - initial.planar.x,
 state.planar.y - initial.planar.y) < 0.02);
 const double measured_weight_N = front_left_load_N + front_right_load_N +
 rear_left_load_N + rear_right_load_N;
 REQUIRE(std::abs(measured_weight_N - robot.mass_kg * kGravityMps2) <
 robot.mass_kg * kGravityMps2 * 0.15);
 REQUIRE(std::abs(front_left_load_N - front_right_load_N) <
 robot.mass_kg * kGravityMps2 * 0.08);
 REQUIRE(std::abs(rear_left_load_N - rear_right_load_N) <
 robot.mass_kg * kGravityMps2 * 0.08);
 REQUIRE(front_left_load_N > 20.0);
 REQUIRE(front_right_load_N > 20.0);
 REQUIRE(rear_left_load_N > 20.0);
 REQUIRE(rear_right_load_N > 20.0);
 }

 {
 sim::platform::SimulatedPlatform platform(map, level, robot, caps, terrain,
 spawn, 0.0, robot.mass_kg, robot.center_of_mass_height_m,
 robot.yaw_inertia_kgm2);
 Rig rig(robot);
 auto state = platform.state();
 const auto start = state;
 for (int step = 0; step < 250; ++step) {
 const auto actuation = rig.step(platform, state.planar, {1.0, 0.0}, dry);
 state = platform.step(actuation, dry, kStepS);
 }
 std::cerr << "RO2 straight x=" << state.planar.x - start.planar.x
 << " y=" << state.planar.y - start.planar.y
 << " yaw=" << sim::wrap_angle(state.planar.theta - start.planar.theta)
 << " v=" << state.planar.v << '\n';
 REQUIRE(state.planar.x > start.planar.x + 2.0);
 REQUIRE(std::abs(state.planar.y - start.planar.y) < 0.2);
 REQUIRE(std::abs(sim::wrap_angle(state.planar.theta - start.planar.theta)) < 0.1);
 }

 // Reverse uses the same signed wheel-speed contract and must not invert the
 // steering convention or introduce a lateral bias.
 {
 sim::platform::SimulatedPlatform platform(map, level, robot, caps, terrain,
 spawn, 0.0, robot.mass_kg, robot.center_of_mass_height_m,
 robot.yaw_inertia_kgm2);
 Rig rig(robot);
 auto state = platform.state();
 const auto start = state;
 for (int step = 0; step < 200; ++step) {
 const auto actuation = rig.step(platform, state.planar, {-0.7, 0.0}, dry);
 state = platform.step(actuation, dry, kStepS);
 }
 REQUIRE(state.planar.x < start.planar.x - 1.5);
 REQUIRE(std::abs(state.planar.y - start.planar.y) < 0.12);
 REQUIRE(std::abs(sim::wrap_angle(state.planar.theta - start.planar.theta)) < 0.08);
 }

 {
 sim::platform::SimulatedPlatform platform(map, level, robot, caps, terrain,
 spawn, 0.0, robot.mass_kg, robot.center_of_mass_height_m,
 robot.yaw_inertia_kgm2);
 Rig rig(robot);
 auto state = platform.state();
 const auto start = state;
 double travelled_m = 0.0;
 double unwrapped_yaw_rad = 0.0;
 for (int step = 0; step < 200; ++step) {
 const auto previous = state;
 const auto actuation = rig.step(platform, state.planar, {0.8, 0.35}, dry);
 state = platform.step(actuation, dry, kStepS);
 travelled_m += std::hypot(state.planar.x - previous.planar.x,
 state.planar.y - previous.planar.y);
 unwrapped_yaw_rad += sim::wrap_angle(state.planar.theta - previous.planar.theta);
 }
 const auto feedback = platform.actuator_feedback();
 const auto* left = feedback.find("front_left_steer");
 const auto* right = feedback.find("front_right_steer");
 std::cerr << "RO2 turn dx=" << state.planar.x - start.planar.x
 << " dy=" << state.planar.y - start.planar.y
 << " yaw=" << sim::wrap_angle(state.planar.theta - start.planar.theta)
 << " steer=" << left->position_rad << ',' << right->position_rad << '\n';
 REQUIRE(left->position_rad > right->position_rad);
 REQUIRE(right->position_rad > 0.0);
 REQUIRE(sim::wrap_angle(state.planar.theta - start.planar.theta) > 0.6);
 const auto target = sim::ackermann_steering_angles(0.35,
 robot.wheel_base_m, robot.track_width_m);
 REQUIRE(std::abs(left->position_rad - target.left_rad) < 0.04);
 REQUIRE(std::abs(right->position_rad - target.right_rad) < 0.04);
 const double left_icr_m = robot.wheel_base_m / std::tan(left->position_rad) +
 robot.track_width_m * 0.5;
 const double right_icr_m = robot.wheel_base_m / std::tan(right->position_rad) -
 robot.track_width_m * 0.5;
 REQUIRE(std::abs(left_icr_m - right_icr_m) < 0.12);
 const double measured_radius_m = travelled_m / std::abs(unwrapped_yaw_rad);
 const double geometric_radius_m = robot.wheel_base_m / std::tan(0.35);
 REQUIRE(std::abs(measured_radius_m - geometric_radius_m) < geometric_radius_m * 0.25);
 }

 // Steering step, authored joint limit and return-to-centre are measured at
 // the physical knuckles. The maximum equivalent angle must never be treated
 // as a yaw-rate command.
 {
 sim::platform::SimulatedPlatform platform(map, level, robot, caps, terrain,
 spawn, 0.0, robot.mass_kg, robot.center_of_mass_height_m,
 robot.yaw_inertia_kgm2);
 Rig rig(robot);
 auto state = platform.state();
 double peak_rate_radps = 0.0;
 for (int step = 0; step < 150; ++step) {
 const auto actuation = rig.step(platform, state.planar,
 {0.2, robot.steering.max_steer_rad}, dry);
 state = platform.step(actuation, dry, kStepS);
 const auto feedback = platform.actuator_feedback();
 peak_rate_radps = std::max({peak_rate_radps,
 std::abs(feedback.find("front_left_steer")->velocity_radps),
 std::abs(feedback.find("front_right_steer")->velocity_radps)});
 }
 auto feedback = platform.actuator_feedback();
 const auto limit_target = sim::ackermann_steering_angles(
 robot.steering.max_steer_rad, robot.wheel_base_m, robot.track_width_m);
 std::cerr << "RO2 steer step target=" << limit_target.left_rad << ','
 << limit_target.right_rad << " actual="
 << feedback.find("front_left_steer")->position_rad << ','
 << feedback.find("front_right_steer")->position_rad
 << " peak_rate=" << peak_rate_radps << '\n';
 REQUIRE(std::abs(feedback.find("front_left_steer")->position_rad -
 limit_target.left_rad) < 0.05);
 REQUIRE(std::abs(feedback.find("front_right_steer")->position_rad -
 limit_target.right_rad) < 0.05);
 REQUIRE(peak_rate_radps < robot.steering.max_steer_rate_radps * 1.35);
 for (int step = 0; step < 150; ++step) {
 const auto actuation = rig.step(platform, state.planar, {0.0, 0.0}, dry);
 state = platform.step(actuation, dry, kStepS);
 }
 feedback = platform.actuator_feedback();
 REQUIRE(std::abs(feedback.find("front_left_steer")->position_rad) < 0.035);
 REQUIRE(std::abs(feedback.find("front_right_steer")->position_rad) < 0.035);
 }

 // Closed-loop braking must dissipate forward motion without reversing or
 // relying on an analytic body brake force.
 {
 sim::platform::SimulatedPlatform platform(map, level, robot, caps, terrain,
 spawn, 0.0, robot.mass_kg, robot.center_of_mass_height_m,
 robot.yaw_inertia_kgm2);
 Rig rig(robot);
 auto state = platform.state();
 for (int step = 0; step < 150; ++step) {
 const auto actuation = rig.step(platform, state.planar, {1.2, 0.0}, dry);
 state = platform.step(actuation, dry, kStepS);
 }
 REQUIRE(state.planar.v > 0.8);
 const double brake_start_x = state.planar.x;
 double minimum_speed_mps = state.planar.v;
 for (int step = 0; step < 150; ++step) {
 const auto actuation = rig.step(platform, state.planar, {0.0, 0.0}, dry);
 if (step == 0) {
 REQUIRE(actuation.service_brake_count == 2);
 REQUIRE(actuation.service_brakes[0].requested == 1.0);
 REQUIRE(actuation.service_brakes[0].applied > 0.0);
 REQUIRE(actuation.service_brakes[0].delivered_torque_Nm < 0.0);
 }
 state = platform.step(actuation, dry, kStepS);
 minimum_speed_mps = std::min(minimum_speed_mps, state.planar.v);
 }
 REQUIRE(std::abs(state.planar.v) < 0.08);
 REQUIRE(state.planar.x - brake_start_x < 1.8);
 REQUIRE(minimum_speed_mps > -0.15);
 }

 // Brake torque is independent of tire adhesion. The same request on a
 // low-grip surface must produce more wheel/body slip and a longer stop;
 // otherwise the brake plant has incorrectly duplicated the contact limit.
 const auto brake_on = [&](const sim::GroundContact& contact) {
 sim::platform::SimulatedPlatform platform(map, level, robot, caps, terrain,
 spawn, 0.0, robot.mass_kg, robot.center_of_mass_height_m,
 robot.yaw_inertia_kgm2);
 Rig rig(robot);
 auto state = platform.state();
 for (int step = 0; step < 180; ++step) {
 const auto actuation = rig.step(platform, state.planar, {1.6, 0.0}, contact);
 state = platform.step(actuation, contact, kStepS);
 }
 const double start_x = state.planar.x;
 double peak_slip = 0.0;
 int stop_steps = 240;
 for (int step = 0; step < 240; ++step) {
 const auto actuation = rig.step(platform, state.planar, {0.0, 0.0}, contact);
 state = platform.step(actuation, contact, kStepS);
 peak_slip = std::max(peak_slip, state.max_drive_slip_ratio);
 if (std::abs(state.planar.v) < 0.05 && stop_steps == 240)
 stop_steps = step;
 }
 return std::array<double, 3>{state.planar.x - start_x, peak_slip,
 stop_steps * kStepS};
 };
 const auto dry_brake = brake_on(dry);
 const sim::GroundContact wet_brake_contact{0.18, 0.01, 6.0, 40000.0, 1000.0};
 const auto wet_brake = brake_on(wet_brake_contact);
 REQUIRE(wet_brake[0] > dry_brake[0] * 1.15);
 REQUIRE(wet_brake[1] > dry_brake[1] + 0.05);
 REQUIRE(wet_brake[2] > dry_brake[2]);

 // Each articulated wheel resolves contact at its own world position. A
 // left/right split-friction stop must therefore create a yaw response even
 // though both brake actuators receive the same request.
 {
 sim::platform::SimulatedPlatform platform(map, level, robot, caps, terrain,
 spawn, 0.0, robot.mass_kg, robot.center_of_mass_height_m,
 robot.yaw_inertia_kgm2);
 Rig rig(robot);
 auto state = platform.state();
 for (int step = 0; step < 180; ++step) {
 const auto actuation = rig.step(platform, state.planar, {1.5, 0.0}, dry);
 state = platform.step(actuation, dry, kStepS);
 }
 const double heading_before = state.planar.theta;
 const sim::GroundContact split_low{0.10, 0.01, 6.0, 40000.0, 1000.0};
 double peak_rear_force_delta_N = 0.0;
 for (int step = 0; step < 160; ++step) {
 const auto actuation = rig.step(platform, state.planar, {0.0, 0.0}, dry);
 state = platform.step(actuation,
 [&](sim::Vec2 contact_position) {
 return contact_position.y >= state.planar.y ? dry : split_low;
 }, kStepS);
 peak_rear_force_delta_N = std::max(peak_rear_force_delta_N,
 std::abs(tangent_force_on(state, "rear_left_wheel_link") -
 tangent_force_on(state, "rear_right_wheel_link")));
 }
 std::cerr << "RO2 split brake yaw="
 << sim::wrap_angle(state.planar.theta - heading_before)
 << " v=" << state.planar.v << " force_delta=" << peak_rear_force_delta_N << '\n';
 REQUIRE(peak_rear_force_delta_N > 5.0);
 REQUIRE(std::abs(state.planar.v) < 0.25);
 }

 // Halving the public physics step must converge to the same straight-line
 // solution; this catches a controller that only works at one frame rate.
 const auto run_step_size = [&](double dt_s) {
 sim::platform::SimulatedPlatform platform(map, level, robot, caps, terrain,
 spawn, 0.0, robot.mass_kg, robot.center_of_mass_height_m,
 robot.yaw_inertia_kgm2);
 Rig rig(robot);
 auto state = platform.state();
 for (int step = 0; step < static_cast<int>(std::lround(3.0 / dt_s)); ++step) {
 const auto actuation = rig.step(platform, state.planar, {0.8, 0.2}, dry, dt_s);
 state = platform.step(actuation, dry, dt_s);
 }
 return state;
 };
 const auto coarse = run_step_size(0.02);
 const auto fine = run_step_size(0.01);
 REQUIRE(std::hypot(coarse.planar.x - fine.planar.x,
 coarse.planar.y - fine.planar.y) < 0.25);
 REQUIRE(std::abs(sim::wrap_angle(coarse.planar.theta - fine.planar.theta)) < 0.12);

 // Tire force, rather than an injected body yaw moment, limits cornering on
 // a low-friction surface. Reduced grip must increase slip and reduce the
 // achieved heading change for the same speed/steering command.
 const auto corner_on = [&](const sim::GroundContact& contact) {
 sim::platform::SimulatedPlatform platform(map, level, robot, caps, terrain,
 spawn, 0.0, robot.mass_kg, robot.center_of_mass_height_m,
 robot.yaw_inertia_kgm2);
 Rig rig(robot);
 auto state = platform.state();
 double yaw_rad = 0.0;
 double peak_slip = 0.0;
 for (int step = 0; step < 180; ++step) {
 const auto previous = state;
 const auto actuation = rig.step(platform, state.planar, {1.4, 0.4}, contact);
 state = platform.step(actuation, contact, kStepS);
 yaw_rad += sim::wrap_angle(state.planar.theta - previous.planar.theta);
 peak_slip = std::max(peak_slip, state.max_drive_slip_ratio);
 }
 return std::pair{yaw_rad, peak_slip};
 };
 const auto dry_corner = corner_on(dry);
 const sim::GroundContact low_grip{0.12, 0.01, 6.0, 40000.0, 1000.0};
 const auto low_grip_corner = corner_on(low_grip);
 std::cerr << "RO2 grip yaw/slip dry=" << dry_corner.first << '/'
 << dry_corner.second << " low=" << low_grip_corner.first << '/'
 << low_grip_corner.second << '\n';
 REQUIRE(std::abs(low_grip_corner.first) < std::abs(dry_corner.first) * 0.9);
 REQUIRE(low_grip_corner.second > dry_corner.second + 0.05);

 // With no hill-hold command, gravity must move the car downhill. A cross
 // slope must tilt the rigid assembly while all four tires retain support.
 {
 const auto grade_map = slope_map(catalog.map(level.map), 0.12, 0.0);
 const auto grade_terrain = sim::TerrainModel({{"ro2_acceptance_surface",
 {0.5, 0.5}, {39.5, 19.5}, 0.0, 0.12, 0.0, 0.0}});
 sim::platform::SimulatedPlatform platform(grade_map, level, robot, caps,
 grade_terrain, spawn, 0.0, robot.mass_kg,
 robot.center_of_mass_height_m, robot.yaw_inertia_kgm2);
 sim::ChassisTelemetry coast;
 const auto start = platform.state();
 auto state = start;
 for (int step = 0; step < 150; ++step)
 state = platform.step(coast, dry, kStepS);
 std::cerr << "RO2 grade dx=" << state.planar.x - start.planar.x
 << " pitch=" << state.pitch_rad << " expected=" << -std::atan(0.12) << '\n';
 REQUIRE(state.planar.x < start.planar.x - 0.15);
 REQUIRE(std::abs(state.pitch_rad + std::atan(0.12)) < 0.055);
 }

 // Gradeability is a measured boundary, not a single successful ramp. With
 // deliberately high grip, the independent peak-force limit is
 // sin(theta) = 2 F_stall / (m g). The same closed-loop drive must climb
 // below that boundary and roll back above it.
 const auto run_driven_grade = [&](double grade) {
 const auto grade_map = slope_map(catalog.map(level.map), grade, 0.0);
 const auto grade_terrain = sim::TerrainModel({{"ro2_acceptance_surface",
 {0.5, 0.5}, {39.5, 19.5}, 0.0, grade, 0.0, 0.0}});
 sim::platform::SimulatedPlatform platform(grade_map, level, robot, caps,
 grade_terrain, spawn, 0.0, robot.mass_kg,
 robot.center_of_mass_height_m, robot.yaw_inertia_kgm2);
 Rig rig(robot);
 const auto start = platform.state();
 auto state = start;
 const sim::GroundContact high_grip{2.0, 0.0, 6.0, 40000.0, 1000.0};
 for (int step = 0; step < 100; ++step) {
 const auto actuation = rig.step(platform, state.planar,
 {robot.motor.max_wheel_speed_mps, 0.0}, high_grip);
 state = platform.step(actuation, high_grip, kStepS);
 }
 return state.planar.x - start.planar.x;
 };
 const double motor_limit_rad = std::asin(std::clamp(
 2.0 * robot.motor.stall_force_N / (robot.mass_kg * kGravityMps2), 0.0, 1.0));
 const double below_motor_limit_grade = std::tan(motor_limit_rad - 0.07);
 const double above_motor_limit_grade = std::tan(motor_limit_rad + 0.07);
 const double grade_climb_m = run_driven_grade(below_motor_limit_grade);
 const double grade_rollback_m = run_driven_grade(above_motor_limit_grade);
 std::cerr << "RO2 grade boundary theta=" << motor_limit_rad
 << " climb=" << grade_climb_m << " rollback=" << grade_rollback_m << '\n';
 REQUIRE(grade_climb_m > 0.05);
 REQUIRE(grade_rollback_m < -0.05);

 {
 const auto cross_map = slope_map(catalog.map(level.map), 0.0, 0.15);
 const auto cross_terrain = sim::TerrainModel({{"ro2_acceptance_surface",
 {0.5, 0.5}, {39.5, 19.5}, 0.0, 0.0, 0.15, 0.0}});
 sim::platform::SimulatedPlatform platform(cross_map, level, robot, caps,
 cross_terrain, spawn, 0.0, robot.mass_kg,
 robot.center_of_mass_height_m, robot.yaw_inertia_kgm2);
 sim::ChassisTelemetry coast;
 auto state = platform.state();
 for (int step = 0; step < 100; ++step)
 state = platform.step(coast, dry, kStepS);
 REQUIRE(std::abs(std::abs(state.roll_rad) - std::atan(0.15)) < 0.05);
 REQUIRE(touching(state, "front_left_wheel_link"));
 REQUIRE(touching(state, "front_right_wheel_link"));
 REQUIRE(touching(state, "rear_left_wheel_link"));
 REQUIRE(touching(state, "rear_right_wheel_link"));
 REQUIRE(!touching(state, "base_link"));
 }

 // The same articulated assembly must cross a bounded rough patch without
 // hull-ground contact or losing every tire contact. This validates the
 // rigid contact model; suspension/compliant tires remain a later fidelity
 // provider rather than a hidden mission-specific correction.
 {
 const auto rough_map = slope_map(catalog.map(level.map), 0.0, 0.0, 0.025);
 const auto rough_terrain = sim::TerrainModel({{"ro2_acceptance_surface",
 {0.5, 0.5}, {39.5, 19.5}, 0.0, 0.0, 0.0, 0.025}});
 sim::platform::SimulatedPlatform platform(rough_map, level, robot, caps,
 rough_terrain, spawn, 0.0, robot.mass_kg,
 robot.center_of_mass_height_m, robot.yaw_inertia_kgm2);
 Rig rig(robot);
 auto state = platform.state();
 bool hull_contact = false;
 int supported_steps = 0;
 for (int step = 0; step < 250; ++step) {
 const auto actuation = rig.step(platform, state.planar, {0.6, 0.0}, dry);
 state = platform.step(actuation, dry, kStepS);
 hull_contact = hull_contact || touching(state, "base_link");
 supported_steps += !state.ground_contact_links.empty();
 }
 REQUIRE(state.planar.x > spawn.x + 1.5);
 REQUIRE(supported_steps > 230);
 REQUIRE(!hull_contact);
 REQUIRE(std::abs(state.roll_rad) < 0.12);
 REQUIRE(std::abs(state.pitch_rad) < 0.12);
 }
 return 0;
}
