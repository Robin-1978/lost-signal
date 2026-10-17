#include "sim/hardware/vehicle/ackermann.hpp"
#include "sim/hardware/vehicle/chassis_factory.hpp"
#include "sim/hardware/vehicle/contact.hpp"
#include "sim/hardware/actuators/electric_drive.hpp"

#include "robot/core/math.hpp"
#include "robot/core/actuator_response.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

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
 servo_pid_(servo_kp, 0.0, 0.0, 1.0, 0.5),
 rear_left_brake_({params.service_brake_max_torque_Nm,
 params.service_brake_apply_time_s, params.service_brake_release_time_s,
 params.service_brake_low_speed_radps}),
 rear_right_brake_({params.service_brake_max_torque_Nm,
 params.service_brake_apply_time_s, params.service_brake_release_time_s,
 params.service_brake_low_speed_radps}) {
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
 feedback_ = {};
 rear_left_brake_.reset();
 rear_right_brake_.reset();
}

void AckermannDrive::set_body_dynamics(const ChassisBodyDynamics& dynamics) {
 params_.mass_kg = std::max(0.5, dynamics.mass_kg);
 params_.yaw_inertia_kgm2 = std::max(0.05, dynamics.yaw_inertia_kgm2);
}

void AckermannDrive::set_actuator_feedback(
 const ChassisActuatorFeedback& feedback) {
 feedback_ = feedback;
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
 const double rate_demand = clamp(servo.pwm * params_.max_steer_rate_radps,
 -params_.max_steer_rate_radps, params_.max_steer_rate_radps);
 servo_rate_state_ = first_order_rate_step(
 servo_rate_state_, rate_demand, params_.servo_response_s, dt);
 delta_rad_ = clamp(delta_rad_ + servo_rate_state_ * dt, -params_.max_steer_rad, params_.max_steer_rad);
 servo.angle_rad = delta_rad_;
 servo.rate_radps = servo_rate_state_;
 // Linkage output (Ackermann condition): both front wheels share the turn
 // center on the rear-axle line, so the inner wheel steers more.
 // R = L/tan(delta); inner = atan(L/(R-T/2)), outer = atan(L/(R+T/2)).
 // Straight (delta ~ 0) keeps both at the servo angle; the denominator guard
 // can never trip inside steer limits (R_min >> T/2) but stays for safety.
 const auto wheel_angles = ackermann_steering_angles(delta_rad_,
 params_.wheel_base_m, params_.track_width_m);
 telemetry_.wheel_steer_rad[0] = wheel_angles.left_rad;
 telemetry_.wheel_steer_rad[1] = wheel_angles.right_rad;

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

DiffState AckermannDrive::step_actuator_duty(const DiffState& state,
 const DiffCmd& command, const ChassisActuatorDutyCommand& control,
 double dt, const GroundContact&) {
 const auto* rear_left = feedback_.find("rear_left_drive");
 const auto* rear_right = feedback_.find("rear_right_drive");
 const auto* front_left = feedback_.find("front_left_steer");
 const auto* front_right = feedback_.find("front_right_steer");
 if (!rear_left || !rear_right || !front_left || !front_right)
 throw std::runtime_error("articulated Ackermann plant requires four actuator feedback channels");

 const double rear_left_duty = clamp(control.value("rear_left_drive"),
 -params_.pwm_limit, params_.pwm_limit);
 const double rear_right_duty = clamp(control.value("rear_right_drive"),
 -params_.pwm_limit, params_.pwm_limit);
 const double front_left_duty = clamp(control.value("front_left_steer"), -1.0, 1.0);
 const double front_right_duty = clamp(control.value("front_right_steer"), -1.0, 1.0);
 const double rear_left_brake_request = clamp(
 control.value("rear_left_brake"), 0.0, 1.0);
 const double rear_right_brake_request = clamp(
 control.value("rear_right_brake"), 0.0, 1.0);
 const auto drive_torque = [&](double duty, double velocity_radps) {
 const double speed_mps = velocity_radps * params_.wheel_radius_m;
 return hardware::dc_motor_force_N(duty, speed_mps, params_.stall_force_N,
 params_.free_speed_mps) * params_.wheel_radius_m;
 };
 const auto steering_torque = [&](double duty, double velocity_radps) {
 const bool motoring = duty * velocity_radps > 0.0;
 const double motor_curve = motoring
 ? std::max(0.0, 1.0 - std::abs(velocity_radps) /
 std::max(0.05, params_.max_steer_rate_radps))
 : 1.0;
 return duty * params_.steering_stall_torque_Nm * motor_curve;
 };

 telemetry_ = {};
 telemetry_.differential = false;
 telemetry_.steering_servo_present = true;
 telemetry_.wheel_steer_count = 2;
 telemetry_.left.pwm = rear_left_duty;
 telemetry_.right.pwm = rear_right_duty;
 telemetry_.left.actual_mps = rear_left->velocity_radps * params_.wheel_radius_m;
 telemetry_.right.actual_mps = rear_right->velocity_radps * params_.wheel_radius_m;
 telemetry_.drive.pwm = 0.5 * (rear_left_duty + rear_right_duty);
 telemetry_.drive.actual_mps =
 0.5 * (telemetry_.left.actual_mps + telemetry_.right.actual_mps);
 telemetry_.drive.measured_mps = telemetry_.drive.actual_mps;
 telemetry_.drive.target_mps = command.target_v;
 telemetry_.steer.target_rad = command.target_w;
 telemetry_.steer.angle_rad = 0.5 * (front_left->position_rad + front_right->position_rad);
 telemetry_.steer.rate_radps = 0.5 * (front_left->velocity_radps + front_right->velocity_radps);
 telemetry_.steer.pwm = 0.5 * (front_left_duty + front_right_duty);
 telemetry_.wheel_steer_rad[0] = front_left->position_rad;
 telemetry_.wheel_steer_rad[1] = front_right->position_rad;
 telemetry_.measured_forward_mps = telemetry_.drive.actual_mps;
 telemetry_.measured_yaw_radps = state.w;
 const double rear_left_brake_torque = rear_left_brake_.step(
 rear_left_brake_request, rear_left->velocity_radps, dt);
 const double rear_right_brake_torque = rear_right_brake_.step(
 rear_right_brake_request, rear_right->velocity_radps, dt);
 telemetry_.service_brake_count = params_.service_brake_max_torque_Nm > 0.0 ? 2 : 0;
 const auto left_brake = rear_left_brake_.telemetry();
 const auto right_brake = rear_right_brake_.telemetry();
 telemetry_.service_brakes[0] = {
 left_brake.requested, left_brake.applied, left_brake.delivered_torque_Nm};
 telemetry_.service_brakes[1] = {
 right_brake.requested, right_brake.applied, right_brake.delivered_torque_Nm};
 telemetry_.joint_efforts = {
 {"rear_left_drive", drive_torque(rear_left_duty, rear_left->velocity_radps)},
 {"rear_right_drive", drive_torque(rear_right_duty, rear_right->velocity_radps)},
 {"front_left_steer", steering_torque(front_left_duty, front_left->velocity_radps)},
 {"front_right_steer", steering_torque(front_right_duty, front_right->velocity_radps)},
 {"rear_left_brake", rear_left_brake_torque},
 {"rear_right_brake", rear_right_brake_torque},
 };
 // Physical yaw is produced only by the steered tire contact. The returned
 // planar estimate is for domain/shadow comparison and never drives Bullet.
 DiffState next = state;
 next.v = telemetry_.measured_forward_mps;
 next.w = yaw_for_steer(next.v, command.target_w, params_.wheel_base_m);
 next.x += next.v * std::cos(state.theta) * dt;
 next.y += next.v * std::sin(state.theta) * dt;
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
 params.wheel_equiv_mass_kg,
 params.steering_stall_torque_Nm,
 params.service_brake_max_torque_Nm,
 params.service_brake_apply_time_s,
 params.service_brake_release_time_s,
 params.service_brake_low_speed_radps,
    },
    gains, servo_kp);
  return out;
}

} // namespace sim

