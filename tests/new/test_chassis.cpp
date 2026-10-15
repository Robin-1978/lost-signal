#include "hw/chassis/ackermann.hpp"
#include "hw/chassis/caps.hpp"
#include "hw/chassis/chassis_factory.hpp"
#include "hw/chassis/tracked.hpp"
#include "algo/follow.hpp"
#include "algo/pid.hpp"

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

 // Turn: steering takes an angle, yaw follows bicycle kinematics, limits hold.
 sim::AckermannDrive turning = make_car();
 sim::DiffState pose;
 for (int step = 0; step < 200; ++step) {
  pose = turning.step(pose, {1.0, 0.5}, 0.02, kGround);
 }
 CHECK(turning.steer_angle() > 0.1);
 CHECK(turning.steer_angle() <= 0.6);
 CHECK(pose.w > 0.1);
 CHECK(pose.theta > 0.2);
 // Linkage output: left turn puts the larger angle on the left wheel, and
 // both satisfy the Ackermann condition (shared turn center on the rear axle:
 // L/tan(inner) + T/2 == L/tan(outer) - T/2).
 {
  const double left = turning.telemetry().front_left_steer_rad;
  const double right = turning.telemetry().front_right_steer_rad;
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
  const std::vector<sim::Vec2> path{{0.0, 0.0}, {10.0, 0.0}};
  sim::PID pid(2.0, 0.0, 0.0, 2.0, 1.0);
  sim::GuideState spin_state;
  sim::ChassisCaps spin{true};
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
  sim::ChassisCaps no_spin{false};
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
  const std::vector<sim::Vec2> straight{{0.0, 0.0}, {5.0, 0.0}, {10.0, 0.0}};
  CHECK(sim::corner_speed_cap(straight, 0, 1.0, 3.0, 2.0) == 1.0);
  // 90-degree bend with a 0.3 m exit leg under a_lat 3: sqrt(3*0.3/1.5708).
  const std::vector<sim::Vec2> bend{{0.0, 0.0}, {1.0, 0.0}, {1.0, 0.3}};
  const double capped = sim::corner_speed_cap(bend, 0, 1.0, 3.0, 2.0);
  CHECK(capped < 1.0);
  CHECK(std::abs(capped - std::sqrt(3.0 * 0.3 / 1.5707963)) < 1e-6);
  // Same bend beyond the window: cruise untouched.
  CHECK(sim::corner_speed_cap(bend, 0, 1.0, 3.0, 0.5) == 1.0);
  // Gentle bend needs no slowdown: 0.2 rad over 2 m at a_lat 3 allows 5.5.
  const std::vector<sim::Vec2> gentle{{0.0, 0.0}, {2.0, 0.0}, {4.0, 0.4}};
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
  sim::DiffState spun;
  for (int step = 0; step < 100; ++step) {
   spun = first.step(spun, {0.0, 1.0}, 0.02, kGround);
  }
  CHECK(spun.w > 0.1);
  CHECK(std::abs(spun.v) < 0.1);
  CHECK(first.telemetry().steer.angle_rad == 0.0);
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
  for (const auto [v, w] : {std::make_pair(1.0, 0.0), std::make_pair(1.0, 0.5), std::make_pair(-0.5, -1.2),
                            std::make_pair(0.0, 0.0)}) {
   double left = 0.0;
   double right = 0.0;
   sim::DiffDrive::twistToWheels(v, w, 0.52, left, right);
   double back_v = 0.0;
   double back_w = 0.0;
   sim::DiffDrive::wheelsToTwist(left, right, 0.52, back_v, back_w);
   CHECK(std::abs(back_v - v) < 1e-12);
   CHECK(std::abs(back_w - w) < 1e-12);
  }
  for (const auto [v, w] : {std::make_pair(1.0, 0.3), std::make_pair(2.0, -0.4), std::make_pair(0.5, 0.0)}) {
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
