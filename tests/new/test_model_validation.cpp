#include "game/content_catalog.hpp"
#include "hw/chassis/ackermann.hpp"
#include "hw/chassis/motor_drive.hpp"

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

sim::MotorDriveParams drive_params(const game::RobotDefinition& robot) {
 sim::MotorDriveParams params;
 params.wheel_base_m = robot.wheel_base_m;
 params.wheel_radius_m = robot.wheel_radius_m;
 params.max_wheel_speed_mps = robot.motor.max_wheel_speed_mps;
 params.max_wheel_accel_mps2 = robot.motor.max_wheel_accel_mps2;
 params.stall_force_N = robot.motor.stall_force_N;
 params.free_speed_mps = robot.motor.free_speed_mps;
 params.mass_kg = robot.mass_kg;
 params.yaw_inertia_kgm2 = robot.yaw_inertia_kgm2;
 params.pwm_limit = robot.motor.pwm_limit;
 params.integral_limit = robot.motor.integral_limit;
 params.encoder_counts_per_revolution = robot.encoder.counts_per_revolution;
 return params;
}

const sim::GroundContact kDry{0.7, 0.01, 6.0};
const sim::GroundContact kIce{0.1, 0.005, 6.0};

} // namespace

// Model validation: physics assertions DERIVED from content parameters, so DIY
// robots and retuned numbers stay covered. Levels are validated separately;
// this test proves the model itself is sane before any level runs.
int main() {
 const auto catalog = game::ContentCatalog::load(LOST_SIGNAL_TEST_ASSETS);
 const auto& robot = catalog.robot(catalog.campaign().companion_robot);
 CHECK(robot.drive_type == "differential");
 CHECK(robot.mass_kg > 0.0);
 CHECK(robot.yaw_inertia_kgm2 > 0.0);
 const auto params = drive_params(robot);
 const sim::WheelPidGains gains =
   sim::tune_wheel_pid(robot.mass_kg / 2.0, robot.motor.stall_force_N, robot.motor.free_speed_mps,
                       0.5 * robot.motor.max_wheel_speed_mps);
 CHECK(gains.kp > 0.0 && gains.ki > 0.0 && gains.kd >= 0.0);

 // Full throttle from standstill: F=ma per wheel pair, within caps.
 {
  sim::DifferentialMotorDrive drive(params, gains);
  sim::DiffState state;
  state = drive.step(state, {2.4, 0.0}, 0.02, kDry);
  const double accel = state.v / 0.02;
  const double expected = 2.0 * robot.motor.stall_force_N / robot.mass_kg;
  CHECK(accel > 0.5 * expected);
  CHECK(accel <= robot.motor.max_wheel_accel_mps2 + 1e-9);
 }

 // Top speed asymptotes at or below the motor limit.
 {
  sim::DifferentialMotorDrive drive(params, gains);
  sim::DiffState state;
  for (int i = 0; i < 1000; ++i) {
   state = drive.step(state, {9.9, 0.0}, 0.02, kDry);
  }
  CHECK(state.v <= robot.motor.max_wheel_speed_mps + 1e-9);
  CHECK(state.v > 0.75 * robot.motor.max_wheel_speed_mps);
 }

 // Braking from cruise stops quickly and stays bounded.
 {
  sim::DifferentialMotorDrive drive(params, gains);
  sim::DiffState state{0.0, 0.0, 0.0, 2.0, 0.0};
  double distance = 0.0;
  for (int i = 0; i < 500; ++i) {
   const sim::DiffState next = drive.step(state, {0.0, 0.0}, 0.02, kDry);
   distance += std::hypot(next.x - state.x, next.y - state.y);
   state = next;
  }
  CHECK(std::abs(state.v) < 0.05);
  CHECK(distance < 2.0);
 }

 // Traction saturation fires on ice under full power (torque-heavy setup).
 {
  sim::MotorDriveParams hot = params;
  hot.stall_force_N = 75.0;
  const sim::WheelPidGains hot_gains = sim::tune_wheel_pid(robot.mass_kg / 2.0, hot.stall_force_N,
                                                           hot.free_speed_mps, 0.5 * hot.max_wheel_speed_mps);
  sim::DifferentialMotorDrive drive(hot, hot_gains);
  sim::DiffState state;
  bool slipped = false;
  for (int i = 0; i < 100; ++i) {
   state = drive.step(state, {2.4, 0.0}, 0.02, kIce);
   slipped = slipped || drive.telemetry().left.slipping || drive.telemetry().right.slipping;
  }
  CHECK(slipped);
 }

 // Encoder quantization: readings are always integer multiples of one count.
 {
  sim::DifferentialMotorDrive drive(params, gains);
  sim::DiffState state;
  const double quantum = 2.0 * sim::kPi * robot.wheel_radius_m / robot.encoder.counts_per_revolution / 0.02;
  for (int i = 0; i < 50; ++i) {
   state = drive.step(state, {1.0, 0.0}, 0.02, kDry);
   const double ratio = drive.telemetry().left.measured_mps / quantum;
   CHECK(std::abs(ratio - std::round(ratio)) < 1e-6);
  }
 }

 // Ackermann: no spinning at standstill; yaw follows bicycle kinematics.
 {
  sim::AckermannParams acar;
  acar.wheel_base_m = 2.0;
  acar.mass_kg = 24.0;
  const sim::WheelPidGains agains =
    sim::tune_wheel_pid(acar.mass_kg, acar.stall_force_N, acar.free_speed_mps, 1.2);
  sim::AckermannDrive car(acar, agains, 8.0);
  sim::DiffState still;
  for (int i = 0; i < 100; ++i) {
   still = car.step(still, {0.0, 1.0}, 0.02, kDry);
  }
  CHECK(std::abs(still.v) < 0.05);
  CHECK(std::abs(still.w) < 0.05);
  sim::DiffState turning;
  for (int i = 0; i < 200; ++i) {
   turning = car.step(turning, {1.0, 0.5}, 0.02, kDry);
  }
  CHECK(std::abs(car.steer_angle()) <= acar.max_steer_rad + 1e-9);
  const double expected_w = turning.v * std::tan(car.steer_angle()) / acar.wheel_base_m;
  CHECK(std::abs(turning.w - expected_w) < 1e-9);
 }
 return 0;
}
