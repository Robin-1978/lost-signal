#include "motor_drive.hpp"

#include "math.hpp"

#include <algorithm>
#include <cmath>

namespace sim {

DifferentialMotorDrive::DifferentialMotorDrive(MotorDriveParams params)
 : params_(params),
 left_pid_(params.speed_pid_kp, params.speed_pid_ki, params.speed_pid_kd, 1.0, 2.0),
 right_pid_(params.speed_pid_kp, params.speed_pid_ki, params.speed_pid_kd, 1.0, 2.0) {}

void DifferentialMotorDrive::reset() {
 left_pid_.reset();
 right_pid_.reset();
 telemetry_ = {};
}

double DifferentialMotorDrive::encoder_measure(double wheel_mps, double dt) const {
 const double circumference = 2.0 * kPi * params_.wheel_radius_m;
 const double revolutions = wheel_mps * dt / circumference;
 const double counts = std::round(
 revolutions * static_cast<double>(params_.encoder_counts_per_revolution));
 return counts * circumference
 / (static_cast<double>(params_.encoder_counts_per_revolution) * dt);
}

DiffState DifferentialMotorDrive::step(
 const DiffState& state, const DiffCmd& command, double dt) {
 DiffDrive::twistToWheels(
 command.target_v, command.target_w, params_.wheel_base_m,
 telemetry_.left.target_mps, telemetry_.right.target_mps);
 telemetry_.left.target_mps = clamp(
 telemetry_.left.target_mps, -params_.max_wheel_speed_mps, params_.max_wheel_speed_mps);
 telemetry_.right.target_mps = clamp(
 telemetry_.right.target_mps, -params_.max_wheel_speed_mps, params_.max_wheel_speed_mps);

 double actual_left = 0.0;
 double actual_right = 0.0;
 DiffDrive::twistToWheels(
 state.v, state.w, params_.wheel_base_m, actual_left, actual_right);
 telemetry_.left.measured_mps = encoder_measure(actual_left, dt);
 telemetry_.right.measured_mps = encoder_measure(actual_right, dt);

 telemetry_.left.pwm = left_pid_.update(
 telemetry_.left.target_mps - telemetry_.left.measured_mps, dt);
 telemetry_.right.pwm = right_pid_.update(
 telemetry_.right.target_mps - telemetry_.right.measured_mps, dt);
 telemetry_.left.pid_p = left_pid_.p();
 telemetry_.left.pid_i = left_pid_.i();
 telemetry_.left.pid_d = left_pid_.d();
 telemetry_.right.pid_p = right_pid_.p();
 telemetry_.right.pid_i = right_pid_.i();
 telemetry_.right.pid_d = right_pid_.d();

 actual_left += telemetry_.left.pwm * params_.max_wheel_accel_mps2 * dt;
 actual_right += telemetry_.right.pwm * params_.max_wheel_accel_mps2 * dt;
 telemetry_.left.actual_mps = clamp(
 actual_left, -params_.max_wheel_speed_mps, params_.max_wheel_speed_mps);
 telemetry_.right.actual_mps = clamp(
 actual_right, -params_.max_wheel_speed_mps, params_.max_wheel_speed_mps);

 DiffState next = state;
 DiffDrive::wheelsToTwist(
 telemetry_.left.actual_mps, telemetry_.right.actual_mps,
 params_.wheel_base_m, next.v, next.w);
 next.x = state.x + next.v * std::cos(state.theta) * dt;
 next.y = state.y + next.v * std::sin(state.theta) * dt;
 next.theta = wrap_angle(state.theta + next.w * dt);
 return next;
}

} // namespace sim
