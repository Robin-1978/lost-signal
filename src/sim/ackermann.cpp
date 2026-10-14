#include "ackermann.hpp"

#include "math.hpp"

#include <algorithm>
#include <cmath>

namespace sim {
namespace {

// Shared contact physics: gravity is physics, the Coulomb smoothing width and
// the small viscous term keep standstill stable at the 0.02 s fixed step. Same
// values as the differential model so tire and ground content means the same
// thing on every chassis.
constexpr double kGravity = 9.81;
constexpr double kCoulombWidthMps = 0.05;
constexpr double kViscousNPerMps = 0.6;
// Twist-to-curvature floor: below this speed the steering demand freezes at
// the low-speed geometry instead of dividing toward infinity. A real car
// cannot turn standing still either.
constexpr double kSteerSpeedFloorMps = 0.5;

} // namespace

AckermannDrive::AckermannDrive(AckermannParams params, WheelPidGains drive_gains, double servo_kp)
    : params_(params),
      drive_pid_(drive_gains.kp, drive_gains.ki, drive_gains.kd, params.pwm_limit, params.integral_limit),
      servo_pid_(servo_kp, 0.0, 0.0, 1.0, 0.5) {
 telemetry_.differential = false;
}

void AckermannDrive::reset() {
 drive_pid_.reset();
 servo_pid_.reset();
 delta_rad_ = 0.0;
 servo_rate_state_ = 0.0;
 drive_wheel_ = 0.0;
 telemetry_ = {};
 telemetry_.differential = false;
}

double AckermannDrive::encoder_measure(double wheel_mps, double dt) const {
 const double circumference = 2.0 * kPi * params_.wheel_radius_m;
 const double revolutions = wheel_mps * dt / circumference;
 const double counts = std::round(revolutions * static_cast<double>(params_.encoder_counts_per_revolution));
 return counts * circumference / (static_cast<double>(params_.encoder_counts_per_revolution) * dt);
}

DiffState AckermannDrive::step(const DiffState& state, const DiffCmd& command, double dt,
                               const GroundContact& ground) {
 auto& drive = telemetry_.drive;
 auto& servo = telemetry_.steer;

 drive.target_mps = clamp(command.target_v, -params_.max_drive_speed_mps, params_.max_drive_speed_mps);
 drive.measured_mps = encoder_measure(drive_wheel_, dt);

 // Curvature demand from the twist interface, then the servo position loop.
 // P-only by construction: a servo closes its own inner loop, the outer loop
 // only sets the angle demand.
 const double steer_demand = std::atan2(command.target_w * params_.wheel_base_m,
                                        std::max(std::abs(drive.target_mps), kSteerSpeedFloorMps));
 servo.target_rad = clamp(steer_demand, -params_.max_steer_rad, params_.max_steer_rad);
 servo.pwm = servo_pid_.update(servo.target_rad - delta_rad_, dt);
 servo.pid_p = servo_pid_.p();
 const double rate_demand = clamp(servo.pwm * params_.max_steer_rate_radps, -params_.max_steer_rate_radps,
                                  params_.max_steer_rate_radps);
 const double lag = params_.servo_response_s > 0.0 ? 1.0 - std::exp(-dt / params_.servo_response_s) : 1.0;
 servo_rate_state_ += (rate_demand - servo_rate_state_) * lag;
 delta_rad_ = clamp(delta_rad_ + servo_rate_state_ * dt, -params_.max_steer_rad, params_.max_steer_rad);
 servo.angle_rad = delta_rad_;

 // Drive unit: same shared contact as differential wheels (tire/ground content
 // means the same thing on every chassis). The drive wheel owns its speed
 // state; the body only feels the contact force.
 drive.pwm = drive_pid_.update(drive.target_mps - drive.measured_mps, dt);
 drive.pid_p = drive_pid_.p();
 drive.pid_i = drive_pid_.i();
 drive.pid_d = drive_pid_.d();
 const double weight = params_.mass_kg * kGravity;
 const WheelContactParams contact{params_.stall_force_N, params_.free_speed_mps,
                                  std::max(0.0, ground.mu_eff) * weight};
 const WheelContactResult contact_out = step_driven_wheel(drive_wheel_, state.v, drive.pwm, contact, dt);
 drive_wheel_ = clamp(contact_out.wheel_mps, -params_.max_drive_speed_mps, params_.max_drive_speed_mps);
 drive.actual_mps = drive_wheel_;
 drive.slipping = contact_out.slipping;
 const double resist = std::max(0.0, ground.rolling_resist) * weight * std::tanh(state.v / kCoulombWidthMps) +
                       kViscousNPerMps * state.v;

 const double accel_cap = params_.max_drive_accel_mps2;
 DiffState next = state;
 next.v = state.v + clamp(((contact_out.body_force_N - resist) / std::max(0.5, params_.mass_kg)) * dt,
                          -accel_cap * dt, accel_cap * dt);
 next.w = next.v * std::tan(delta_rad_) / std::max(0.05, params_.wheel_base_m);
 next.x = state.x + next.v * std::cos(state.theta) * dt;
 next.y = state.y + next.v * std::sin(state.theta) * dt;
 next.theta = wrap_angle(state.theta + next.w * dt);
 return next;
}

} // namespace sim
