#include "motor_drive.hpp"
#include "hw/chassis/chassis_factory.hpp"

#include <memory>

#include "core/math.hpp"

#include <algorithm>
#include <cmath>

namespace sim {
namespace {

// Gravity and contact constants. g is physics, the 0.05 m/s Coulomb smoothing
// width and the small viscous term keep standstill stable without stick-slip
// chatter at the 0.02 s fixed step.
constexpr double kGravity = 9.81;

} // namespace

const ChassisActuatorFeedback::JointSample* ChassisActuatorFeedback::find(
 const std::string& channel) const {
 const auto found = std::find_if(joints.begin(), joints.end(), [&](const auto& sample) {
 return sample.channel == channel;
 });
 return found == joints.end() ? nullptr : &*found;
}

WheelPidGains tune_wheel_pid(double wheel_mass_kg, double stall_force_N, double free_speed_mps,
 double linearize_at_mps, const WheelControllerDesign& design) {
 const double speed = std::max(0.0, linearize_at_mps);
 const double slope = std::max(0.05, 1.0 - speed / std::max(0.05, free_speed_mps));
 const double k_lin = std::max(1.0, stall_force_N * slope);
 const double bandwidth = std::max(0.01, design.bandwidth_radps);
 const double integral_time = std::max(0.01, design.integral_time_s);
 const double derivative_time = std::max(0.0, design.derivative_time_s);
 const double kp = wheel_mass_kg * bandwidth / k_lin;
 return {kp, kp / integral_time, kp * derivative_time};
}

DifferentialMotorDrive::DifferentialMotorDrive(MotorDriveParams params, WheelPidGains gains)
 : params_(params),
 left_pid_(gains.kp, gains.ki, gains.kd, params.pwm_limit, params.integral_limit,
 AntiWindupMode::ConditionalIntegration),
 right_pid_(gains.kp, gains.ki, gains.kd, params.pwm_limit, params.integral_limit,
 AntiWindupMode::ConditionalIntegration) {}

void DifferentialMotorDrive::set_body_dynamics(const ChassisBodyDynamics& dynamics) {
 params_.mass_kg = std::max(0.5, dynamics.mass_kg);
 params_.yaw_inertia_kgm2 = std::max(0.05, dynamics.yaw_inertia_kgm2);
}

void DifferentialMotorDrive::set_actuator_feedback(const ChassisActuatorFeedback& feedback) {
 feedback_ = feedback;
}

void DifferentialMotorDrive::reset() {
 left_pid_.reset();
 right_pid_.reset();
 telemetry_ = {};
 feedback_ = {};
 encoder_initialized_ = false;
 previous_left_count_ = 0.0;
 previous_right_count_ = 0.0;
}

double DifferentialMotorDrive::encoder_measure(double position_rad, double& previous_count, double dt) {
 const double counts_per_rad =
 static_cast<double>(params_.encoder_counts_per_revolution) / (2.0 * kPi);
 const double count = std::round(position_rad * counts_per_rad);
 const double delta = count - previous_count;
 previous_count = count;
 return delta / counts_per_rad * params_.wheel_radius_m / dt;
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
 const auto* left_feedback = feedback_.find("left");
 const auto* right_feedback = feedback_.find("right");
 const bool physical_feedback = left_feedback && right_feedback;
 if (physical_feedback) {
 actual_left = left_feedback->velocity_radps * params_.wheel_radius_m;
 actual_right = right_feedback->velocity_radps * params_.wheel_radius_m;
 if (!encoder_initialized_) {
 previous_left_count_ = std::round(left_feedback->position_rad *
 static_cast<double>(params_.encoder_counts_per_revolution) / (2.0 * kPi));
 previous_right_count_ = std::round(right_feedback->position_rad *
 static_cast<double>(params_.encoder_counts_per_revolution) / (2.0 * kPi));
 encoder_initialized_ = true;
 telemetry_.left.measured_mps = 0.0;
 telemetry_.right.measured_mps = 0.0;
 } else {
 telemetry_.left.measured_mps =
 encoder_measure(left_feedback->position_rad, previous_left_count_, dt);
 telemetry_.right.measured_mps =
 encoder_measure(right_feedback->position_rad, previous_right_count_, dt);
 }
 } else {
 DiffDrive::twistToWheels(state.v, state.w, params_.wheel_base_m, actual_left, actual_right);
 // The analytic fallback has no joint position state. Quantize the distance
 // travelled during this control sample without claiming it is Bullet truth.
 telemetry_.left.measured_mps = std::round(actual_left * dt /
 (2.0 * kPi * params_.wheel_radius_m) * params_.encoder_counts_per_revolution) *
 (2.0 * kPi * params_.wheel_radius_m) / params_.encoder_counts_per_revolution / dt;
 telemetry_.right.measured_mps = std::round(actual_right * dt /
 (2.0 * kPi * params_.wheel_radius_m) * params_.encoder_counts_per_revolution) *
 (2.0 * kPi * params_.wheel_radius_m) / params_.encoder_counts_per_revolution / dt;
 }

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
 auto wheel_force = [&](double pwm, double wheel_mps, WheelMotorTelemetry& telemetry) {
  const double curve = std::max(0.0, 1.0 - std::abs(wheel_mps) / std::max(0.05, params_.free_speed_mps));
  double force = pwm * params_.stall_force_N * curve;
 if (physical_feedback) {
 // Articulated mode ends the electrical / motor model at shaft effort.
 // Bullet owns rolling resistance, available tire friction and slip; applying
 // those limits here as well would create a second contact plant.
 telemetry.slipping = false;
 return force;
 }
  const double resist =
 resist_coeff * weight_per_wheel *
 std::tanh(wheel_mps / std::max(1e-4, params_.coulomb_smoothing_speed_mps)) +
 params_.viscous_drag_N_per_mps * wheel_mps;
  force -= resist;
  telemetry.slipping = std::abs(force) > traction_limit;
  if (telemetry.slipping) {
   force = std::copysign(traction_limit, force);
  }
  return force;
 };
 const double force_left = wheel_force(telemetry_.left.pwm, actual_left, telemetry_.left);
 const double force_right = wheel_force(telemetry_.right.pwm, actual_right, telemetry_.right);
 telemetry_.left.longitudinal_force_N = force_left;
 telemetry_.right.longitudinal_force_N = force_right;
 telemetry_.left.joint_effort_Nm = force_left * params_.wheel_radius_m;
 telemetry_.right.joint_effort_Nm = force_right * params_.wheel_radius_m;
 telemetry_.joint_efforts = {
 {"left", telemetry_.left.joint_effort_Nm},
 {"right", telemetry_.right.joint_effort_Nm},
 };
 telemetry_.extra_yaw_torque_Nm = 0.0;
 telemetry_.desired_yaw_rate_radps = command.target_w;

 const double accel = (force_left + force_right) / std::max(0.5, params_.mass_kg);
 const double alpha =
   (force_right - force_left) * params_.wheel_base_m / 2.0 / std::max(0.05, params_.yaw_inertia_kgm2);
 const double accel_cap = std::min(params_.max_wheel_accel_mps2, std::max(0.5, ground.accel_limit_mps2));
 DiffState next = state;
 next.v = state.v + std::clamp(accel * dt, -accel_cap * dt, accel_cap * dt);
 next.w = state.w + alpha * dt;
 if (physical_feedback) {
 // Joint velocity is the only actual wheel state in articulated mode. The
 // analytic candidate above remains available for shadow comparison, but it
 // must never overwrite physical telemetry or encoder truth.
 telemetry_.left.actual_mps = actual_left;
 telemetry_.right.actual_mps = actual_right;
 } else {
 DiffDrive::twistToWheels(next.v, next.w, params_.wheel_base_m, telemetry_.left.actual_mps,
 telemetry_.right.actual_mps);
 telemetry_.left.actual_mps =
 clamp(telemetry_.left.actual_mps, -params_.max_wheel_speed_mps, params_.max_wheel_speed_mps);
 telemetry_.right.actual_mps =
 clamp(telemetry_.right.actual_mps, -params_.max_wheel_speed_mps, params_.max_wheel_speed_mps);
 DiffDrive::wheelsToTwist(telemetry_.left.actual_mps, telemetry_.right.actual_mps,
 params_.wheel_base_m, next.v, next.w);
 }
 DiffDrive::wheelsToTwist(telemetry_.left.measured_mps, telemetry_.right.measured_mps,
 params_.wheel_base_m, telemetry_.measured_forward_mps,
 telemetry_.measured_yaw_radps);
 next.lateral_v = -params_.drive_axle_x_m * next.w;
 telemetry_.measured_lateral_mps = -params_.drive_axle_x_m * telemetry_.measured_yaw_radps;
 telemetry_.traction_saturated = telemetry_.left.slipping || telemetry_.right.slipping;
 // A three-point differential platform commonly places its axle ahead of
 // base_link so the center of mass lies inside the wheel/caster triangle.
 // The axle has zero lateral velocity; the offset body reference therefore
 // has v_left = -x_axle*w. Integrate that rigid transform instead of silently
 // pretending the axle still passes through the body origin.
 const double heading_mid = state.theta + next.w * dt * 0.5;
 next.x = state.x +
 (next.v * std::cos(heading_mid) - next.lateral_v * std::sin(heading_mid)) * dt;
 next.y = state.y +
 (next.v * std::sin(heading_mid) + next.lateral_v * std::cos(heading_mid)) * dt;
 next.theta = wrap_angle(state.theta + next.w * dt);
 return next;
}

} // namespace sim

// Chassis creator for the factory registry: new types follow this exact shape
// (creator beside its implementation + one call line in
// register_builtin_chassis) instead of extending a switch elsewhere.
namespace sim {

BuiltChassis create_differential_chassis(const ChassisBuildParams& params, const WheelPidGains& gains) {
 sim::BuiltChassis out;
 out.model = std::make_unique<sim::DifferentialMotorDrive>(
   sim::MotorDriveParams{
     params.wheel_base_m,
     params.wheel_radius_m,
 params.drive_axle_x_m,
     params.max_wheel_speed_mps,
     std::min(params.max_wheel_accel_mps2, params.behavior_accel_max),
     params.stall_force_N,
     params.free_speed_mps,
     params.mass_kg,
     params.yaw_inertia_kgm2,
     params.encoder_counts_per_revolution,
     params.pwm_limit,
     params.integral_limit,
 params.support_width_m,
 params.support_length_m,
 params.behavior_w_max,
 params.coulomb_smoothing_speed_mps,
 params.viscous_drag_N_per_mps,
   },
   gains);
 return out;
}

} // namespace sim
