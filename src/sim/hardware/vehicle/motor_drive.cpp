#include "sim/hardware/vehicle/motor_drive.hpp"
#include "sim/hardware/vehicle/chassis_factory.hpp"

#include "robot/core/math.hpp"

#include <algorithm>
#include <cmath>
#include <memory>

namespace sim {
namespace {

constexpr double kGravity = 9.81;

void copy_control_telemetry(const WheelControlTelemetry& source,
 WheelMotorTelemetry& destination) {
 destination.target_mps = source.target_mps;
 destination.measured_mps = source.measured_mps;
 destination.actual_mps = source.actual_mps;
 destination.pwm = source.normalized_duty;
 destination.pid_p = source.pid_p;
 destination.pid_i = source.pid_i;
 destination.pid_d = source.pid_d;
}

} // namespace

const ChassisActuatorFeedback::JointSample* ChassisActuatorFeedback::find(
 const std::string& channel) const {
 const auto found = std::find_if(joints.begin(), joints.end(), [&](const auto& sample) {
 return sample.channel == channel;
 });
 return found == joints.end() ? nullptr : &*found;
}

DifferentialMotorDrive::DifferentialMotorDrive(MotorDriveParams params,
 WheelPidGains gains)
 : params_(params),
 controller_({params.wheel_base_m, params.wheel_radius_m,
 params.max_wheel_speed_mps,
 params.encoder_counts_per_revolution,
 params.pwm_limit, params.integral_limit},
 gains) {}

void DifferentialMotorDrive::set_body_dynamics(
 const ChassisBodyDynamics& dynamics) {
 params_.mass_kg = std::max(0.5, dynamics.mass_kg);
 params_.yaw_inertia_kgm2 = std::max(0.05, dynamics.yaw_inertia_kgm2);
}

void DifferentialMotorDrive::set_actuator_feedback(
 const ChassisActuatorFeedback& feedback) {
 feedback_ = feedback;
}

void DifferentialMotorDrive::reset() {
 controller_.reset();
 telemetry_ = {};
 feedback_ = {};
}

DiffState DifferentialMotorDrive::step(const DiffState& state,
 const DiffCmd& command, double dt,
 const GroundContact& ground) {
 DifferentialWheelFeedback feedback;
 const auto* left_feedback = feedback_.find("left");
 const auto* right_feedback = feedback_.find("right");
 feedback.positions_valid = left_feedback && right_feedback;
 if (feedback.positions_valid) {
 feedback.left_position_rad = left_feedback->position_rad;
 feedback.right_position_rad = right_feedback->position_rad;
 feedback.left_velocity_radps = left_feedback->velocity_radps;
 feedback.right_velocity_radps = right_feedback->velocity_radps;
 } else {
 double left_mps = 0.0;
 double right_mps = 0.0;
 DifferentialKinematics::twist_to_wheels(
 state.v, state.w, params_.wheel_base_m, left_mps, right_mps);
 feedback.left_velocity_radps = left_mps / params_.wheel_radius_m;
 feedback.right_velocity_radps = right_mps / params_.wheel_radius_m;
 }
 return apply_control(state, command, controller_.update(command, feedback, dt),
 dt, ground);
}

DiffState DifferentialMotorDrive::step_normalized_duty(
 const DiffState& state, const DiffCmd& command,
 const DifferentialWheelControlOutput& control, double dt,
 const GroundContact& ground) {
 DifferentialWheelControlOutput bounded = control;
 bounded.left.normalized_duty = clamp(bounded.left.normalized_duty,
 -params_.pwm_limit, params_.pwm_limit);
 bounded.right.normalized_duty = clamp(bounded.right.normalized_duty,
 -params_.pwm_limit, params_.pwm_limit);
 return apply_control(state, command, std::move(bounded), dt, ground);
}

DiffState DifferentialMotorDrive::apply_control(
 const DiffState& state, const DiffCmd& command,
 DifferentialWheelControlOutput control, double dt,
 const GroundContact& ground) {
 const auto* left_feedback = feedback_.find("left");
 const auto* right_feedback = feedback_.find("right");
 const bool physical_feedback = left_feedback && right_feedback;
 double actual_left = control.left.actual_mps;
 double actual_right = control.right.actual_mps;
 if (physical_feedback) {
 actual_left = left_feedback->velocity_radps * params_.wheel_radius_m;
 actual_right = right_feedback->velocity_radps * params_.wheel_radius_m;
 } else {
 DifferentialKinematics::twist_to_wheels(
 state.v, state.w, params_.wheel_base_m, actual_left, actual_right);
 }
 control.left.actual_mps = actual_left;
 control.right.actual_mps = actual_right;

 telemetry_ = {};
 telemetry_.differential = true;
 copy_control_telemetry(control.left, telemetry_.left);
 copy_control_telemetry(control.right, telemetry_.right);
 telemetry_.measured_forward_mps = control.measured_forward_mps;
 telemetry_.measured_yaw_radps = control.measured_yaw_radps;
 telemetry_.measured_lateral_mps =
 -params_.drive_axle_x_m * telemetry_.measured_yaw_radps;
 telemetry_.desired_yaw_rate_radps = command.target_w;

 const double weight_per_wheel = params_.mass_kg * kGravity / 2.0;
 const double traction_limit = std::max(0.0, ground.mu_eff) * weight_per_wheel;
 const double resist_coeff = std::max(0.0, ground.rolling_resist);
 auto wheel_force = [&](double duty, double wheel_mps,
 WheelMotorTelemetry& wheel) {
 const double curve = std::max(
 0.0, 1.0 - std::abs(wheel_mps) /
 std::max(0.05, params_.free_speed_mps));
 double force = duty * params_.stall_force_N * curve;
 if (physical_feedback) {
 wheel.slipping = false;
 return force;
 }
 const double resistance =
 resist_coeff * weight_per_wheel *
 std::tanh(wheel_mps /
 std::max(1e-4, params_.coulomb_smoothing_speed_mps)) +
 params_.viscous_drag_N_per_mps * wheel_mps;
 force -= resistance;
 wheel.slipping = std::abs(force) > traction_limit;
 if (wheel.slipping) {
 force = std::copysign(traction_limit, force);
 }
 return force;
 };

 const double force_left = wheel_force(
 telemetry_.left.pwm, actual_left, telemetry_.left);
 const double force_right = wheel_force(
 telemetry_.right.pwm, actual_right, telemetry_.right);
 telemetry_.left.longitudinal_force_N = force_left;
 telemetry_.right.longitudinal_force_N = force_right;
 telemetry_.left.joint_effort_Nm = force_left * params_.wheel_radius_m;
 telemetry_.right.joint_effort_Nm = force_right * params_.wheel_radius_m;
 telemetry_.joint_efforts = {
 {"left", telemetry_.left.joint_effort_Nm},
 {"right", telemetry_.right.joint_effort_Nm},
 };
 telemetry_.traction_saturated =
 telemetry_.left.slipping || telemetry_.right.slipping;

 const double acceleration =
 (force_left + force_right) / std::max(0.5, params_.mass_kg);
 const double angular_acceleration =
 (force_right - force_left) * params_.wheel_base_m / 2.0 /
 std::max(0.05, params_.yaw_inertia_kgm2);
 const double acceleration_limit = std::min(
 params_.max_wheel_accel_mps2,
 std::max(0.5, ground.accel_limit_mps2));
 DiffState next = state;
 next.v = state.v + std::clamp(acceleration * dt,
 -acceleration_limit * dt,
 acceleration_limit * dt);
 next.w = state.w + angular_acceleration * dt;
 if (!physical_feedback) {
 DifferentialKinematics::twist_to_wheels(
 next.v, next.w, params_.wheel_base_m,
 telemetry_.left.actual_mps, telemetry_.right.actual_mps);
 telemetry_.left.actual_mps = clamp(telemetry_.left.actual_mps,
 -params_.max_wheel_speed_mps, params_.max_wheel_speed_mps);
 telemetry_.right.actual_mps = clamp(telemetry_.right.actual_mps,
 -params_.max_wheel_speed_mps, params_.max_wheel_speed_mps);
 DifferentialKinematics::wheels_to_twist(
 telemetry_.left.actual_mps, telemetry_.right.actual_mps,
 params_.wheel_base_m, next.v, next.w);
 }
 next.lateral_v = -params_.drive_axle_x_m * next.w;
 const double heading_mid = state.theta + next.w * dt * 0.5;
 next.x = state.x +
 (next.v * std::cos(heading_mid) -
 next.lateral_v * std::sin(heading_mid)) * dt;
 next.y = state.y +
 (next.v * std::sin(heading_mid) +
 next.lateral_v * std::cos(heading_mid)) * dt;
 next.theta = wrap_angle(state.theta + next.w * dt);
 return next;
}

} // namespace sim

namespace sim {

BuiltChassis create_differential_chassis(const ChassisBuildParams& params,
 const WheelPidGains& gains) {
 BuiltChassis out;
 out.model = std::make_unique<DifferentialMotorDrive>(
 MotorDriveParams{
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
