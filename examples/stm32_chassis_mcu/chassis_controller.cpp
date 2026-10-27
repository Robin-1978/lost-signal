#include "chassis_controller.hpp"

#include <cmath>

namespace lost_signal::stm32_reference {

float ChassisController::clamp(float value, float lower, float upper) {
 return value < lower ? lower : (value > upper ? upper : value);
}

void ChassisController::Pid::reset() {
 integral = 0.0f;
 previous_error = 0.0f;
 first = true;
}

float ChassisController::Pid::update(float error, float dt_s) {
 const float derivative = first ? 0.0f : (error - previous_error) / dt_s;
 const float candidate = clamp(integral + error * dt_s,
 -config.integral_limit, config.integral_limit);
 const float candidate_output = config.kp * error +
 config.ki * candidate + config.kd * derivative;
 const bool high = candidate_output > config.output_limit && error > 0.0f;
 const bool low = candidate_output < -config.output_limit && error < 0.0f;
 if (!high && !low) integral = candidate;
 previous_error = error;
 first = false;
 return clamp(config.kp * error + config.ki * integral +
 config.kd * derivative, -config.output_limit, config.output_limit);
}

ChassisController::ChassisController(ChassisConfig config)
 : config_(config), drive_pid_{Pid(config.drive_pid), Pid(config.drive_pid),
 Pid(config.drive_pid), Pid(config.drive_pid)},
 steer_pid_{Pid(config.steer_pid), Pid(config.steer_pid),
 Pid(config.steer_pid), Pid(config.steer_pid)} {}

void ChassisController::reset() {
 have_setpoint_ = false;
 last_sequence_ = 0;
 setpoint_ = {};
 for (auto& pid : drive_pid_) pid.reset();
 for (auto& pid : steer_pid_) pid.reset();
}

bool ChassisController::finite_setpoint(const WireSetpoint& setpoint) const {
 if (setpoint.sequence == 0 || setpoint.stamp_ns < 0 ||
 setpoint.expiry_ns <= setpoint.stamp_ns ||
 setpoint.expiry_ns - setpoint.stamp_ns > config_.max_command_age_ns) {
 return false;
 }
 if (!std::isfinite(setpoint.forward_mps) ||
 !std::isfinite(setpoint.lateral_mps) ||
 !std::isfinite(setpoint.yaw_rate_radps) ||
 !std::isfinite(setpoint.service_brake)) return false;
 for (float value : setpoint.drive_duty) {
 if (!std::isfinite(value)) return false;
 }
 for (float value : setpoint.steer_duty) {
 if (!std::isfinite(value)) return false;
 }
 for (float value : setpoint.drive_radps) {
 if (!std::isfinite(value)) return false;
 }
 for (float value : setpoint.steer_rad) {
 if (!std::isfinite(value)) return false;
 }
 return setpoint.service_brake >= 0.0f && setpoint.service_brake <= 1.0f;
}

Status ChassisController::accept(const WireSetpoint& setpoint,
 std::int64_t now_ns) {
 if (!finite_setpoint(setpoint) || now_ns < setpoint.stamp_ns) {
 return Status::kRejected;
 }
 if (now_ns >= setpoint.expiry_ns) return Status::kWatchdogStop;
 if (setpoint.sequence <= last_sequence_) return Status::kSequence;
 last_sequence_ = setpoint.sequence;
 setpoint_ = setpoint;
 have_setpoint_ = true;
 return Status::kApplied;
}

void ChassisController::stop(MotorOutput& output) const {
 output = {};
 output.service_brake = 1.0f;
 output.watchdog_stop = true;
}

Status ChassisController::tick(std::int64_t now_ns,
 const EncoderFeedback& feedback, MotorOutput& output) {
 stop(output);
 if (!have_setpoint_) return Status::kIdle;
 if (now_ns >= setpoint_.expiry_ns) return Status::kWatchdogStop;
 if (setpoint_.kind == SetpointKind::kStop) return Status::kApplied;
 const float dt_s = static_cast<float>(config_.control_period_ns) * 1e-9f;
 if (!(dt_s > 0.0f) || !feedback.valid) return Status::kRejected;
 output.watchdog_stop = false;
 output.service_brake = setpoint_.service_brake;
 if (setpoint_.kind == SetpointKind::kDuty) {
 for (std::uint8_t i = 0; i < config_.drive_count; ++i) {
 output.drive_duty[i] = clamp(setpoint_.drive_duty[i], -1.0f, 1.0f);
 }
 for (std::uint8_t i = 0; i < config_.steer_count; ++i) {
 output.steer_duty[i] = clamp(setpoint_.steer_duty[i], -1.0f, 1.0f);
 }
 return Status::kApplied;
 }
 float target_left = setpoint_.drive_radps[0];
 float target_right = setpoint_.drive_radps[1];
 if (setpoint_.kind == SetpointKind::kBodyTwist) {
 target_left = (setpoint_.forward_mps -
 setpoint_.yaw_rate_radps * config_.track_width_m * 0.5f) /
 config_.wheel_radius_m;
 target_right = (setpoint_.forward_mps +
 setpoint_.yaw_rate_radps * config_.track_width_m * 0.5f) /
 config_.wheel_radius_m;
 }
 target_left = clamp(target_left, -config_.max_wheel_radps,
 config_.max_wheel_radps);
 target_right = clamp(target_right, -config_.max_wheel_radps,
 config_.max_wheel_radps);
 output.drive_duty[0] = drive_pid_[0].update(
 target_left - feedback.drive_radps[0], dt_s);
 output.drive_duty[1] = drive_pid_[1].update(
 target_right - feedback.drive_radps[1], dt_s);
 for (std::uint8_t i = 0; i < config_.steer_count; ++i) {
 output.steer_duty[i] = steer_pid_[i].update(
 setpoint_.steer_rad[i] - feedback.steer_rad[i], dt_s);
 }
 return Status::kApplied;
}

} // namespace lost_signal::stm32_reference
