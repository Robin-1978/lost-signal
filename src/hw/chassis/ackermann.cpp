#include "ackermann.hpp"
#include "hw/chassis/chassis_factory.hpp"
#include "hw/dynamics/contact.hpp"

#include "core/math.hpp"

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

SteerSolution steer_for_twist(double target_w, double target_v, double wheel_base_m, double max_steer_rad) {
 const double raw = std::atan2(target_w * wheel_base_m, std::max(std::abs(target_v), kSteerSpeedFloorMps));
 return {raw, std::abs(raw) <= max_steer_rad};
}

double yaw_for_steer(double body_v, double delta_rad, double wheel_base_m) {
 return body_v * std::tan(delta_rad) / std::max(0.05, wheel_base_m);
}

AckermannDrive::AckermannDrive(AckermannParams params, WheelPidGains drive_gains, double servo_kp)
    : params_(params),
 drive_pid_(drive_gains.kp, drive_gains.ki, drive_gains.kd, params.pwm_limit, params.integral_limit,
 AntiWindupMode::ConditionalIntegration),
      servo_pid_(servo_kp, 0.0, 0.0, 1.0, 0.5) {
 telemetry_.differential = false;
 telemetry_.steering_servo_present = true;
 telemetry_.wheel_steer_count = 2;
}

void AckermannDrive::reset() {
 drive_pid_.reset();
 servo_pid_.reset();
 delta_rad_ = 0.0;
 servo_rate_state_ = 0.0;
 drive_wheel_ = 0.0;
 telemetry_ = {};
 telemetry_.differential = false;
 telemetry_.steering_servo_present = true;
 telemetry_.wheel_steer_count = 2;
}

void AckermannDrive::set_body_dynamics(const ChassisBodyDynamics& dynamics) {
 params_.mass_kg = std::max(0.5, dynamics.mass_kg);
 params_.yaw_inertia_kgm2 = std::max(0.05, dynamics.yaw_inertia_kgm2);
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

 // Ackermann's declared command space is SteeringAngle, so target_w carries
 // the equivalent front-wheel angle at this internal compatibility boundary.
 // Converting it from yaw rate here would change the command's meaning after
 // LSRI validation and would make low-speed simulation disagree with a real
 // position-controlled steering drive. The P-only outer loop sets that angle;
 // a provider may map it to a servo's native position/current/PWM cascade.
 servo.target_rad = clamp(command.target_w, -params_.max_steer_rad, params_.max_steer_rad);
 servo.pwm = servo_pid_.update(servo.target_rad - delta_rad_, dt);
 servo.pid_p = servo_pid_.p();
 const double rate_demand = clamp(servo.pwm * params_.max_steer_rate_radps, -params_.max_steer_rate_radps,
                                  params_.max_steer_rate_radps);
 const double lag = params_.servo_response_s > 0.0 ? 1.0 - std::exp(-dt / params_.servo_response_s) : 1.0;
 servo_rate_state_ += (rate_demand - servo_rate_state_) * lag;
 delta_rad_ = clamp(delta_rad_ + servo_rate_state_ * dt, -params_.max_steer_rad, params_.max_steer_rad);
 servo.angle_rad = delta_rad_;
 // Linkage output (Ackermann condition): both front wheels share the turn
 // center on the rear-axle line, so the inner wheel steers more.
 // R = L/tan(delta); inner = atan(L/(R-T/2)), outer = atan(L/(R+T/2)).
 // Straight (delta ~ 0) keeps both at the servo angle; the denominator guard
 // can never trip inside steer limits (R_min >> T/2) but stays for safety.
 {
  const double track = std::max(0.05, params_.track_width_m);
  const double base = std::max(0.05, params_.wheel_base_m);
  if (std::abs(delta_rad_) < 1e-9) {
 telemetry_.wheel_steer_rad[0] = delta_rad_;
 telemetry_.wheel_steer_rad[1] = delta_rad_;
  } else {
   const double radius = base / std::tan(delta_rad_);
   const double inner = std::atan(base / std::max(0.05, std::abs(radius) - track / 2.0));
   const double outer = std::atan(base / (std::abs(radius) + track / 2.0));
   const double sign = delta_rad_ > 0.0 ? 1.0 : -1.0;
   // Positive delta is a left turn: the left wheel is inner.
 telemetry_.wheel_steer_rad[0] = sign * (delta_rad_ > 0.0 ? inner : outer);
 telemetry_.wheel_steer_rad[1] = sign * (delta_rad_ > 0.0 ? outer : inner);
  }
 }

 // Drive unit: same shared contact as differential wheels (tire/ground content
 // means the same thing on every chassis). The drive wheel owns its speed
 // state; the body only feels the contact force.
 drive.pwm = drive_pid_.update(drive.target_mps - drive.measured_mps, dt);
 drive.pid_p = drive_pid_.p();
 drive.pid_i = drive_pid_.i();
 drive.pid_d = drive_pid_.d();
 const double weight = params_.mass_kg * kGravity;
 const WheelContactParams contact{params_.stall_force_N, params_.free_speed_mps,
                                  std::max(0.0, ground.mu_eff) * weight, params_.wheel_equiv_mass_kg};
 const WheelContactResult contact_out = step_driven_wheel(drive_wheel_, state.v, drive.pwm, contact, dt);
 drive_wheel_ = clamp(contact_out.wheel_mps, -params_.max_drive_speed_mps, params_.max_drive_speed_mps);
 drive.actual_mps = drive_wheel_;
 drive.slipping = contact_out.slipping;
 const double resist = std::max(0.0, ground.rolling_resist) * weight * std::tanh(state.v / kCoulombWidthMps) +
                       kViscousNPerMps * state.v;
 const double forward_force = contact_out.body_force_N - resist;
 drive.longitudinal_force_N = forward_force;

 const double accel_cap = params_.max_drive_accel_mps2;
 DiffState next = state;
 next.v = state.v + clamp((forward_force / std::max(0.5, params_.mass_kg)) * dt,
                          -accel_cap * dt, accel_cap * dt);
 next.w = yaw_for_steer(next.v, delta_rad_, params_.wheel_base_m);
 telemetry_.desired_yaw_rate_radps = next.w;
 // The legacy planar result applies bicycle kinematics directly. A rigid
 // body consumes this moment through its own inertia and contact solver.
 telemetry_.extra_yaw_torque_Nm =
 (next.w - state.w) * params_.yaw_inertia_kgm2 / std::max(1e-6, dt);
 telemetry_.measured_forward_mps = telemetry_.drive.measured_mps;
 telemetry_.measured_lateral_mps = 0.0;
 telemetry_.measured_yaw_radps = yaw_for_steer(telemetry_.drive.measured_mps,
 telemetry_.steer.angle_rad, params_.wheel_base_m);
 telemetry_.traction_saturated = telemetry_.drive.slipping;
 next.x = state.x + next.v * std::cos(state.theta) * dt;
 next.y = state.y + next.v * std::sin(state.theta) * dt;
 next.theta = wrap_angle(state.theta + next.w * dt);
 return next;
}

} // namespace sim

// Chassis creator for the factory registry (see motor_drive.cpp).
namespace sim {

BuiltChassis create_ackermann_chassis(const sim::ChassisBuildParams& params,
                                   const sim::WheelPidGains& gains) {
  // The drive unit is a wheel: same autotune as a differential wheel. The
  // servo closes its own inner loop; the outer P only sets angle demand, so
  // kp = 1/(response * rate) targets the specified first-order settle
  // (dδ/dt ≈ kp·rate·err ⇒ τ = 1/(kp·rate)). Same rule for every car.
  const double servo_kp =
    1.0 / std::max(0.05, params.servo_response_s * params.max_steer_rate_radps);
  sim::BuiltChassis out;
  out.model = std::make_unique<sim::AckermannDrive>(
    sim::AckermannParams{
      params.wheel_base_m,
      params.wheel_radius_m,
      params.track_width_m,
      params.max_steer_rad,
      params.max_steer_rate_radps,
      params.servo_response_s,
      params.max_wheel_speed_mps,
      std::min(params.max_wheel_accel_mps2, params.behavior_accel_max),
      params.stall_force_N,
      params.free_speed_mps,
      params.mass_kg,
 params.yaw_inertia_kgm2,
      params.pwm_limit,
      params.integral_limit,
      params.encoder_counts_per_revolution,
    },
    gains, servo_kp);
  return out;
}

} // namespace sim

