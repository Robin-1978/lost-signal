#include "robot/control/ackermann_actuator_controller.hpp"

#include "robot/core/math.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sim {

AckermannActuatorController::AckermannActuatorController(
 AckermannActuatorControllerConfig config, WheelPidGains drive_gains)
 : config_(config),
 rear_left_pid_(drive_gains.kp, drive_gains.ki, drive_gains.kd,
 config.drive_duty_limit, config.drive_integral_limit,
 AntiWindupMode::ConditionalIntegration),
 rear_right_pid_(drive_gains.kp, drive_gains.ki, drive_gains.kd,
 config.drive_duty_limit, config.drive_integral_limit,
 AntiWindupMode::ConditionalIntegration) {
 const double values[] = {config_.wheel_base_m, config_.track_width_m,
 config_.wheel_radius_m, config_.max_wheel_speed_mps,
 config_.max_center_steer_rad, config_.max_steer_rate_radps,
 config_.drive_duty_limit, config_.steering_duty_limit,
 config_.steering_position_gain_radps_per_rad,
 config_.steering_velocity_gain_s};
 for (double value : values) {
 if (!std::isfinite(value) || value <= 0.0)
 throw std::invalid_argument("invalid Ackermann actuator controller configuration");
 }
 if (config_.encoder_counts_per_revolution <= 0)
 throw std::invalid_argument("Ackermann encoder resolution must be positive");
 // Both front joints must remain on the same side of 90 degrees and share a
 // rear-axle ICR throughout the authored command range.
 (void)ackermann_steering_angles(config_.max_center_steer_rad,
 config_.wheel_base_m, config_.track_width_m);
}

void AckermannActuatorController::reset() {
 rear_left_pid_.reset();
 rear_right_pid_.reset();
 encoder_initialized_ = false;
 previous_rear_left_count_ = 0.0;
 previous_rear_right_count_ = 0.0;
}

double AckermannActuatorController::encoder_speed(double position_rad,
 double& previous_count, double dt_s) const {
 const double counts_per_rad =
 static_cast<double>(config_.encoder_counts_per_revolution) / (2.0 * kPi);
 const double count = std::round(position_rad * counts_per_rad);
 const double delta = count - previous_count;
 previous_count = count;
 return delta / counts_per_rad * config_.wheel_radius_m / dt_s;
}

AckermannActuatorControlOutput AckermannActuatorController::update(
 const DiffCmd& command, const AckermannActuatorFeedback& feedback,
 double dt_s) {
 if (!std::isfinite(dt_s) || dt_s <= 0.0)
 throw std::invalid_argument("Ackermann controller step must be finite and positive");
 const double center_angle = clamp(command.target_w,
 -config_.max_center_steer_rad, config_.max_center_steer_rad);
 const auto steer = ackermann_steering_angles(center_angle,
 config_.wheel_base_m, config_.track_width_m);
 const auto rear = ackermann_rear_wheel_speeds(
 clamp(command.target_v, -config_.max_wheel_speed_mps,
 config_.max_wheel_speed_mps), center_angle,
 config_.wheel_base_m, config_.track_width_m);

 AckermannActuatorControlOutput output;
 output.rear_left.target_mps = clamp(rear.left_mps,
 -config_.max_wheel_speed_mps, config_.max_wheel_speed_mps);
 output.rear_right.target_mps = clamp(rear.right_mps,
 -config_.max_wheel_speed_mps, config_.max_wheel_speed_mps);
 output.rear_left.actual_mps =
 feedback.rear_left_velocity_radps * config_.wheel_radius_m;
 output.rear_right.actual_mps =
 feedback.rear_right_velocity_radps * config_.wheel_radius_m;
 if (feedback.positions_valid) {
 const double counts_per_rad =
 static_cast<double>(config_.encoder_counts_per_revolution) / (2.0 * kPi);
 if (!encoder_initialized_) {
 previous_rear_left_count_ = std::round(feedback.rear_left_position_rad * counts_per_rad);
 previous_rear_right_count_ = std::round(feedback.rear_right_position_rad * counts_per_rad);
 encoder_initialized_ = true;
 } else {
 output.rear_left.measured_mps = encoder_speed(
 feedback.rear_left_position_rad, previous_rear_left_count_, dt_s);
 output.rear_right.measured_mps = encoder_speed(
 feedback.rear_right_position_rad, previous_rear_right_count_, dt_s);
 }
 } else {
 output.rear_left.measured_mps = output.rear_left.actual_mps;
 output.rear_right.measured_mps = output.rear_right.actual_mps;
 }
 output.rear_left.normalized_duty = rear_left_pid_.update(
 output.rear_left.target_mps - output.rear_left.measured_mps, dt_s);
 output.rear_right.normalized_duty = rear_right_pid_.update(
 output.rear_right.target_mps - output.rear_right.measured_mps, dt_s);
 const auto steering_duty = [&](double target_rad, double position_rad,
 double velocity_radps) {
 const double desired_rate = clamp(
 config_.steering_position_gain_radps_per_rad * (target_rad - position_rad),
 -config_.max_steer_rate_radps, config_.max_steer_rate_radps);
 return clamp(config_.steering_velocity_gain_s * (desired_rate - velocity_radps),
 -config_.steering_duty_limit, config_.steering_duty_limit);
 };
 output.front_left_steer = {steer.left_rad,
 feedback.front_left_steer_position_rad,
 steering_duty(steer.left_rad, feedback.front_left_steer_position_rad,
 feedback.front_left_steer_velocity_radps)};
 output.front_right_steer = {steer.right_rad,
 feedback.front_right_steer_position_rad,
 steering_duty(steer.right_rad, feedback.front_right_steer_position_rad,
 feedback.front_right_steer_velocity_radps)};
 return output;
}

} // namespace sim
