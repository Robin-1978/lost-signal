#include "motor_drive.hpp"

#include "algo/math.hpp"

#include <algorithm>
#include <cmath>

namespace sim {
namespace {

// Gravity and contact constants. g is physics, the 0.05 m/s Coulomb smoothing
// width and the small viscous term keep standstill stable without stick-slip
// chatter at the 0.02 s fixed step.
constexpr double kGravity = 9.81;
constexpr double kCoulombWidthMps = 0.05;
constexpr double kViscousNPerMps = 0.6;
constexpr double kClosedLoopBandwidthRadps = 6.0;

} // namespace

WheelPidGains tune_wheel_pid(double wheel_mass_kg, double stall_force_N, double free_speed_mps,
                             double linearize_at_mps) {
 const double speed = std::max(0.0, linearize_at_mps);
 const double slope = std::max(0.05, 1.0 - speed / std::max(0.05, free_speed_mps));
 const double k_lin = std::max(1.0, stall_force_N * slope);
 const double kp = wheel_mass_kg * kClosedLoopBandwidthRadps / k_lin;
 return {kp, kp * kClosedLoopBandwidthRadps / 30.0, kp * 0.005};
}

DifferentialMotorDrive::DifferentialMotorDrive(MotorDriveParams params, WheelPidGains gains)
    : params_(params), left_pid_(gains.kp, gains.ki, gains.kd, params.pwm_limit, params.integral_limit),
      right_pid_(gains.kp, gains.ki, gains.kd, params.pwm_limit, params.integral_limit) {}

void DifferentialMotorDrive::reset() {
 left_pid_.reset();
 right_pid_.reset();
 telemetry_ = {};
}

double DifferentialMotorDrive::encoder_measure(double wheel_mps, double dt) const {
 const double circumference = 2.0 * kPi * params_.wheel_radius_m;
 const double revolutions = wheel_mps * dt / circumference;
 const double counts = std::round(revolutions * static_cast<double>(params_.encoder_counts_per_revolution));
 return counts * circumference / (static_cast<double>(params_.encoder_counts_per_revolution) * dt);
}

DiffState DifferentialMotorDrive::step(const DiffState& state, const DiffCmd& command, double dt,
                                       const GroundContact& ground) {
 DiffDrive::twistToWheels(command.target_v, command.target_w, params_.wheel_base_m,
                          telemetry_.left.target_mps, telemetry_.right.target_mps);
 telemetry_.left.target_mps =
   clamp(telemetry_.left.target_mps, -params_.max_wheel_speed_mps, params_.max_wheel_speed_mps);
 telemetry_.right.target_mps =
   clamp(telemetry_.right.target_mps, -params_.max_wheel_speed_mps, params_.max_wheel_speed_mps);

 double actual_left = 0.0;
 double actual_right = 0.0;
 DiffDrive::twistToWheels(state.v, state.w, params_.wheel_base_m, actual_left, actual_right);
 telemetry_.left.measured_mps = encoder_measure(actual_left, dt);
 telemetry_.right.measured_mps = encoder_measure(actual_right, dt);

 telemetry_.left.pwm = left_pid_.update(telemetry_.left.target_mps - telemetry_.left.measured_mps, dt);
 telemetry_.right.pwm = right_pid_.update(telemetry_.right.target_mps - telemetry_.right.measured_mps, dt);
 telemetry_.left.pid_p = left_pid_.p();
 telemetry_.left.pid_i = left_pid_.i();
 telemetry_.left.pid_d = left_pid_.d();
 telemetry_.right.pid_p = right_pid_.p();
 telemetry_.right.pid_i = right_pid_.i();
 telemetry_.right.pid_d = right_pid_.d();

 const double weight_per_wheel = params_.mass_kg * kGravity / 2.0;
 const double traction_limit = std::max(0.0, ground.mu_eff) * weight_per_wheel;
 const double resist_coeff = std::max(0.0, ground.rolling_resist);
 const double wheel_mass = std::max(0.5, params_.mass_kg / 2.0);
 auto wheel_force = [&](double pwm, double wheel_mps, WheelMotorTelemetry& telemetry) {
  const double curve = std::max(0.0, 1.0 - std::abs(wheel_mps) / std::max(0.05, params_.free_speed_mps));
  double force = pwm * params_.stall_force_N * curve;
  const double resist =
    resist_coeff * weight_per_wheel * std::tanh(wheel_mps / kCoulombWidthMps) + kViscousNPerMps * wheel_mps;
  force -= resist;
  telemetry.slipping = std::abs(force) > traction_limit;
  if (telemetry.slipping) {
   force = std::copysign(traction_limit, force);
  }
  return force;
 };
 const double force_left = wheel_force(telemetry_.left.pwm, actual_left, telemetry_.left);
 const double force_right = wheel_force(telemetry_.right.pwm, actual_right, telemetry_.right);

 const double accel = (force_left + force_right) / std::max(0.5, params_.mass_kg);
 const double alpha =
   (force_right - force_left) * params_.wheel_base_m / 2.0 / std::max(0.05, params_.yaw_inertia_kgm2);
 const double accel_cap = std::min(params_.max_wheel_accel_mps2, std::max(0.5, ground.accel_limit_mps2));
 DiffState next = state;
 next.v = state.v + std::clamp(accel * dt, -accel_cap * dt, accel_cap * dt);
 next.w = state.w + alpha * dt;
 DiffDrive::twistToWheels(next.v, next.w, params_.wheel_base_m, telemetry_.left.actual_mps,
                          telemetry_.right.actual_mps);
 telemetry_.left.actual_mps =
   clamp(telemetry_.left.actual_mps, -params_.max_wheel_speed_mps, params_.max_wheel_speed_mps);
 telemetry_.right.actual_mps =
   clamp(telemetry_.right.actual_mps, -params_.max_wheel_speed_mps, params_.max_wheel_speed_mps);
 DiffDrive::wheelsToTwist(telemetry_.left.actual_mps, telemetry_.right.actual_mps, params_.wheel_base_m,
                          next.v, next.w);
 next.x = state.x + next.v * std::cos(state.theta) * dt;
 next.y = state.y + next.v * std::sin(state.theta) * dt;
 next.theta = wrap_angle(state.theta + next.w * dt);
 return next;
}

} // namespace sim
