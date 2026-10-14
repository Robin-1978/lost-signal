#include "sim/ackermann.hpp"

#include <cmath>
#include <iostream>

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
 return 0;
}
