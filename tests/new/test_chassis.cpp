#include "sim/hardware/vehicle/ackermann.hpp"
#include "sim/hardware/actuators/electric_drive.hpp"
#include "robot/platform/chassis_caps.hpp"
#include "sim/hardware/vehicle/chassis_factory.hpp"
#include "sim/hardware/vehicle/motor_drive.hpp"
#include "sim/hardware/vehicle/tracked.hpp"
#include "robot/autonomy/control/follow.hpp"
#include "robot/control/pid.hpp"

#include <cmath>
#include <iostream>
#include <utility>

#define CHECK(condition)                                                                                     \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n";                           \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

namespace {

sim::AckermannDrive make_car() {
 sim::AckermannParams params;
 params.wheel_base_m = 2.0;
 params.wheel_radius_m = 0.1;
 params.max_steer_rad = 0.6;
 params.max_steer_rate_radps = 2.5;
 params.servo_response_s = 0.4;
 params.max_drive_speed_mps = 2.4;
 params.stall_force_N = 54.0;
 params.free_speed_mps = 3.0;
 params.mass_kg = 24.0;
 const sim::WheelPidGains gains =
   sim::tune_wheel_pid(params.mass_kg, params.stall_force_N, params.free_speed_mps, 1.2);
 return sim::AckermannDrive(params, gains, 8.0);
}

const sim::GroundContact kGround{0.7, 0.01, 4.0};

} // namespace

int main() {
 // Motoring loses authority near no-load speed, while an opposing H-bridge
 // command keeps bounded braking authority. Zero duty remains coast.
 CHECK(std::abs(sim::hardware::dc_motor_force_N(1.0, 2.5, 50.0, 3.0) -
 50.0 / 6.0) < 1e-12);
 CHECK(sim::hardware::dc_motor_force_N(-1.0, 2.5, 50.0, 3.0) == -50.0);
 CHECK(sim::hardware::dc_motor_force_N(1.0, -2.5, 50.0, 3.0) == 50.0);
 CHECK(sim::hardware::dc_motor_force_N(0.0, 2.5, 50.0, 3.0) == 0.0);

 // Actuator speed loops need their own anti-windup policy. A prolonged stall
 // must not store a full positive integral that erases differential torque
 // when the next command asks one wheel to reverse. Navigation heading PIDs
 // retain the legacy integration behavior through the default constructor.
 {
 sim::PID motor_pid(2.0, 1.0, 0.0, 1.0, 10.0,
 sim::AntiWindupMode::ConditionalIntegration);
 for (int step = 0; step < 100; ++step)
 (void)motor_pid.update(2.0, 0.02);
 CHECK(std::abs(motor_pid.i()) < 1e-12);
 CHECK(motor_pid.update(-0.25, 0.02) < 0.0);

 sim::MotorDriveParams params;
 params.wheel_base_m = 0.58;
 sim::DifferentialMotorDrive drive(params, {2.0, 1.0, 0.0});
 sim::ChassisActuatorFeedback stalled_feedback;
 stalled_feedback.joints = {{"left", 0.0, 0.0}, {"right", 0.0, 0.0}};
 sim::DiffState stalled;
for (int step = 0; step < 100; ++step) {
 drive.set_actuator_feedback(stalled_feedback);
 stalled = drive.step(stalled, {1.0, 0.0}, 0.02, kGround);
 }
 drive.set_actuator_feedback(stalled_feedback);
 (void)drive.step(stalled, {0.0, 1.0}, 0.02, kGround);
 CHECK(drive.telemetry().left.pwm < 0.0);
 CHECK(drive.telemetry().right.pwm > 0.0);
 }

 // Path follower boundaries: empty and exhausted paths are valid terminal
 // states. These cases used to read path[idx] before checking the bound.
 {
  sim::GridMap open(20, 20, 0.1);
 std::vector<sim::RoutePoint> empty;
  std::size_t empty_index = 0;
  sim::advance_wp(empty, empty_index, 0.0, 0.0, 0.0, 0.6, 1.0);
  CHECK(empty_index == 0);
  const auto empty_lookahead = sim::lookahead_pt(empty, empty_index, 2.0, 3.0, 1.0, open);
  CHECK(empty_lookahead.x == 2.0 && empty_lookahead.y == 3.0);
  sim::GuideState empty_state;
  sim::PID empty_pid(1.0, 0.0, 0.0, 1.0, 1.0);
  CHECK(
    sim::guide_step(empty, empty_state, 0.0, 0.0, 0.0, empty_pid, 1.0, 1.0, 0.02, open, sim::ChassisCaps{})
      .reached);

 const std::vector<sim::RoutePoint> finished{
 {{0.0, 0.0}, 0.0, sim::TravelDirection::Unspecified}};
  std::size_t terminal_index = finished.size();
  sim::advance_wp(finished, terminal_index, 0.0, 0.0, 0.0, 0.6, 1.0);
  CHECK(terminal_index == finished.size());
 }

 // Typed car paths preserve gear and keep lookahead on the current side of a
 // forward/reverse cusp. Reverse steering follows signed bicycle kinematics.
 {
 sim::GridMap route_map(200, 200, 0.1);
 const std::vector<sim::RoutePoint> cusp_path{
 {{5.0, 5.0}, 0.0, sim::TravelDirection::Unspecified},
 {{6.0, 5.0}, 0.0, sim::TravelDirection::Forward},
 {{7.0, 5.0}, 0.0, sim::TravelDirection::Forward},
 {{6.0, 5.0}, 0.0, sim::TravelDirection::Reverse},
 };
 std::size_t route_index = 0;
 auto section_end = sim::path_direction_section_end(cusp_path, route_index);
 sim::advance_wp_until(cusp_path, route_index, section_end, 5.0, 5.0, 0.0, 0.55);
 section_end = sim::path_direction_section_end(cusp_path, route_index);
 const auto target =
 sim::lookahead_pt_until(cusp_path, route_index, section_end, 5.0, 5.0, 10.0, route_map);
 CHECK(target.x == 7.0 && target.y == 5.0);

 const std::vector<sim::RoutePoint> reverse_path{
 {{5.0, 5.0}, 0.0, sim::TravelDirection::Unspecified},
 {{3.0, 6.0}, 0.0, sim::TravelDirection::Reverse},
 };
 sim::GuideState reverse_state;
 const auto reverse = sim::ackermann_guide_step(
 reverse_path, reverse_state, 5.0, 5.0, 0.0, 2.0, 0.6,
 1.0, 1.0, 0.02, route_map);
 CHECK(reverse.tv < 0.0);
 CHECK(reverse.steer > 0.0);
 }

 // Straight: drive reaches target, steering stays centered.
 sim::AckermannDrive car = make_car();
 sim::DiffState straight;
 for (int step = 0; step < 200; ++step) {
  straight = car.step(straight, {1.0, 0.0}, 0.02, kGround);
 }
 CHECK(straight.v > 0.9);
 CHECK(std::abs(car.steer_angle()) < 0.01);
 CHECK(std::abs(straight.w) < 0.05);
 CHECK(!car.telemetry().differential);
 CHECK(std::abs(car.telemetry().drive.pwm) <= 1.0);

 // Turn: the SteeringAngle command reaches the servo unchanged, yaw follows
 // bicycle kinematics, and limits hold.
 sim::AckermannDrive turning = make_car();
 sim::DiffState pose;
 for (int step = 0; step < 200; ++step) {
  pose = turning.step(pose, {1.0, 0.5}, 0.02, kGround);
 }
 CHECK(turning.steer_angle() > 0.1);
 CHECK(turning.steer_angle() <= 0.6);
 CHECK(std::abs(turning.telemetry().steer.target_rad - 0.5) < 1e-12);
 CHECK(pose.w > 0.1);
 CHECK(pose.theta > 0.2);
 // Linkage output: left turn puts the larger angle on the left wheel, and
 // both satisfy the Ackermann condition (shared turn center on the rear axle:
 // L/tan(inner) + T/2 == L/tan(outer) - T/2).
 {
 CHECK(turning.telemetry().wheel_steer_count == 2);
 const double left = turning.telemetry().wheel_steer_rad[0];
 const double right = turning.telemetry().wheel_steer_rad[1];
  CHECK(left > right);
  CHECK(right > 0.0);
  const double base = 2.0;
  const double track = 0.58;
  CHECK(std::abs(base / std::tan(left) + track / 2.0 - (base / std::tan(right) - track / 2.0)) < 1e-9);
 }

 // Standstill twist: a car cannot spin like a differential drive.
 sim::AckermannDrive parked = make_car();
 sim::DiffState still;
 for (int step = 0; step < 100; ++step) {
  still = parked.step(still, {0.0, 1.0}, 0.02, kGround);
 }
 CHECK(std::abs(still.v) < 0.05);
 CHECK(std::abs(still.w) < 0.05);

 // Servo rate limit respected across a full-lock step.
 sim::AckermannDrive servo = make_car();
 sim::DiffState held;
 double previous = 0.0;
 for (int step = 0; step < 50; ++step) {
  held = servo.step(held, {1.0, 2.0}, 0.02, kGround);
  CHECK(std::abs(servo.steer_angle() - previous) <= 2.5 * 0.02 + 1e-9);
  previous = servo.steer_angle();
 }

 // Determinism: same commands reproduce the pose bit-for-bit.
 sim::AckermannDrive first = make_car();
 sim::AckermannDrive second = make_car();
 sim::DiffState a;
 sim::DiffState b;
 for (int step = 0; step < 120; ++step) {
  a = first.step(a, {1.2, -0.3}, 0.02, kGround);
  b = second.step(b, {1.2, -0.3}, 0.02, kGround);
 }
 CHECK(std::abs(a.x - b.x) < 1e-12);
 CHECK(std::abs(a.theta - b.theta) < 1e-12);

 // Capability branching: a spinning chassis recovers by spinning in place,
 // a non-spinning one keeps reversing (ackermann recovery is steered reverse).
 {
  sim::GridMap corridor(200, 200, 0.1);
 const std::vector<sim::RoutePoint> path{
 {{0.0, 0.0}, 0.0, sim::TravelDirection::Unspecified},
 {{10.0, 0.0}, 0.0, sim::TravelDirection::Forward}};
  sim::PID pid(2.0, 0.0, 0.0, 2.0, 1.0);
  sim::GuideState spin_state;
 sim::ChassisCaps spin;
 spin.can_spin = true;
 // Face away from the path: latches reverse first.
  sim::GuideOut r1 =
    sim::guide_step(path, spin_state, 0.0, 0.0, 3.14159, pid, 1.6, 1.0, 0.02, corridor, spin);
 CHECK(r1.tv < 0.0);
 // Hold the error past the watchdog: spinner switches to spin-in-place.
 bool spun = false;
 for (int i = 0; i < 250; ++i) {
 sim::GuideOut r =
 sim::guide_step(path, spin_state, 0.0, 0.0, 3.14159, pid, 1.6, 1.0, 0.02, corridor, spin);
 if (r.tv == 0.0) {
 spun = true;
 break;
 }
 }
 CHECK(spun);
  // Non-spinner never spins: still reversing after the watchdog period.
  sim::GuideState stiff;
  sim::PID pid3(2.0, 0.0, 0.0, 2.0, 1.0);
 sim::ChassisCaps no_spin;
 no_spin.can_spin = false;
  sim::GuideOut last{0, 0, 0, false};
  for (int i = 0; i < 250; ++i) {
   last = sim::guide_step(path, stiff, 0.0, 0.0, 3.14159, pid3, 1.6, 1.0, 0.02, corridor, no_spin);
  }
  CHECK(last.tv < 0.0);
 }
 // Corner preview: straight path returns cruise; a bend inside the window
 // caps by lateral physics v <= sqrt(a_lat * leg / turn); a bend beyond the
 // window changes nothing; degenerate inputs never crash.
 {
 const std::vector<sim::RoutePoint> straight{
 {{0.0, 0.0}, 0.0, sim::TravelDirection::Unspecified},
 {{5.0, 0.0}, 0.0, sim::TravelDirection::Forward},
 {{10.0, 0.0}, 0.0, sim::TravelDirection::Forward}};
  CHECK(sim::corner_speed_cap(straight, 0, 1.0, 3.0, 2.0) == 1.0);
  // 90-degree bend with a 0.3 m exit leg under a_lat 3: sqrt(3*0.3/1.5708).
 const std::vector<sim::RoutePoint> bend{
 {{0.0, 0.0}, 0.0, sim::TravelDirection::Unspecified},
 {{1.0, 0.0}, 0.0, sim::TravelDirection::Forward},
 {{1.0, 0.3}, sim::kPi * 0.5, sim::TravelDirection::Forward}};
  const double capped = sim::corner_speed_cap(bend, 0, 1.0, 3.0, 2.0);
  CHECK(capped < 1.0);
  CHECK(std::abs(capped - std::sqrt(3.0 * 0.3 / 1.5707963)) < 1e-6);
  // Same bend beyond the window: cruise untouched.
  CHECK(sim::corner_speed_cap(bend, 0, 1.0, 3.0, 0.5) == 1.0);
  // Gentle bend needs no slowdown: 0.2 rad over 2 m at a_lat 3 allows 5.5.
 const std::vector<sim::RoutePoint> gentle{
 {{0.0, 0.0}, 0.0, sim::TravelDirection::Unspecified},
 {{2.0, 0.0}, 0.0, sim::TravelDirection::Forward},
 {{4.0, 0.4}, std::atan2(0.4, 2.0), sim::TravelDirection::Forward}};
  CHECK(sim::corner_speed_cap(gentle, 0, 1.0, 3.0, 5.0) == 1.0);
  CHECK(sim::corner_speed_cap(straight, 99, 1.0, 3.0, 2.0) == 1.0);
  CHECK(sim::corner_speed_cap({}, 0, 1.0, 3.0, 2.0) == 1.0);
  CHECK(sim::corner_speed_cap(straight, 0, 0.0, 3.0, 2.0) == 0.0);
 }
 // Tracked drive: twin-track layout on the shared contact, scrub-limited
 // spin, tread-reading encoders, deterministic twin runs.
 {
  sim::TrackedParams params;
  params.mass_kg = 30.0;
  params.yaw_inertia_kgm2 = 3.0;
  const sim::WheelPidGains gains = sim::tune_wheel_pid(3.0, params.stall_force_N, params.free_speed_mps, 1.0);
  sim::TrackedDrive first(params, gains);
  sim::TrackedDrive second(params, gains);
  CHECK(first.caps().can_spin);
  CHECK(first.telemetry().differential);
  sim::DiffState a;
  sim::DiffState b;
  for (int step = 0; step < 100; ++step) {
   a = first.step(a, {1.0, 0.0}, 0.02, kGround);
   b = second.step(b, {1.0, 0.0}, 0.02, kGround);
  }
  CHECK(std::abs(a.x - b.x) < 1e-12);
  CHECK(a.v > 0.5);
  CHECK(std::abs(a.w) < 0.05);
  // Spin in place (slower than wheels: scrub eats torque).
 sim::TrackedDrive spinner(params, gains);
 sim::DiffState spun;
 for (int step = 0; step < 500; ++step) {
 spun = spinner.step(spun, {0.0, 1.0}, 0.02, kGround);
  }
 const double drive_yaw_torque =
 (spinner.telemetry().right.longitudinal_force_N -
 spinner.telemetry().left.longitudinal_force_N) * params.wheel_base_m * 0.5;
 const double scrub_capacity =
 kGround.mu_eff * params.mass_kg * 9.81 * params.track_length_m * 0.25;
 const double steady_residual = drive_yaw_torque -
 scrub_capacity * std::tanh(spun.w / 0.05) - 2.0 * spun.w;
 CHECK(spun.w > 0.04);
 CHECK(std::abs(steady_residual) < 0.2);
  CHECK(std::abs(spun.v) < 0.1);
CHECK(spinner.telemetry().steer.angle_rad == 0.0);
  // Ice: longitudinal breakaway flags slipping on the shared contact.
  const sim::GroundContact ice{0.05, 0.01, 4.0};
  sim::TrackedDrive sled(params, gains);
  sim::DiffState sliding;
  bool slipped = false;
  for (int step = 0; step < 200; ++step) {
   sliding = sled.step(sliding, {2.0, 0.0}, 0.02, ice);
   slipped = sled.telemetry().left.slipping || sled.telemetry().right.slipping || slipped;
  }
  CHECK(slipped);
 }
 // Kinematic pairs are mutual inverses (plan §4 item 14): differential twist
 // round-trips exactly; ackermann round-trips wherever the standstill floor
 // does not intervene (documented control policy, not kinematics).
 {
  for (const auto& [v, w] : {std::make_pair(1.0, 0.0), std::make_pair(1.0, 0.5), std::make_pair(-0.5, -1.2),
                            std::make_pair(0.0, 0.0)}) {
   double left = 0.0;
   double right = 0.0;
   sim::DifferentialKinematics::twist_to_wheels(v, w, 0.52, left, right);
   double back_v = 0.0;
   double back_w = 0.0;
   sim::DifferentialKinematics::wheels_to_twist(left, right, 0.52, back_v, back_w);
   CHECK(std::abs(back_v - v) < 1e-12);
   CHECK(std::abs(back_w - w) < 1e-12);
  }
  for (const auto& [v, w] : {std::make_pair(1.0, 0.3), std::make_pair(2.0, -0.4), std::make_pair(0.5, 0.0)}) {
   const auto solution = sim::steer_for_twist(w, v, 0.62, 0.55);
   CHECK(solution.feasible);
   CHECK(std::abs(sim::yaw_for_steer(v, solution.delta_rad, 0.62) - w) < 1e-9);
  }
  // Beyond steer limits the demand says infeasible instead of faking it.
  const auto clamped = sim::steer_for_twist(3.0, 1.0, 0.62, 0.55);
  CHECK(!clamped.feasible);
 }
 // Factory contract: every builtin builds, unknown types throw loudly
 // (content typos die at construction, never silent fallback).
 {
  sim::ChassisFactory::register_builtin_chassis();
  const sim::ChassisBuildParams params;
  const sim::WheelPidGains gains = sim::tune_wheel_pid(3.0, 54.0, 3.0, 1.0);
  for (const auto* type : {"differential", "ackermann", "tracked"}) {
   auto built = sim::ChassisFactory::create(type, params, gains);
   CHECK(built.model != nullptr);
  }
  bool threw = false;
  try {
   sim::ChassisFactory::create("hoverboard", params, gains);
  } catch (const std::runtime_error&) {
   threw = true;
  }
  CHECK(threw);
  threw = false;
  try {
   sim::ChassisFactory::create("Differential", params, gains);
  } catch (const std::runtime_error&) {
   threw = true;
  }
  CHECK(threw);
 }
 return 0;
}
