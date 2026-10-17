#include "robot/control/holonomic_actuator_controller.hpp"
#include "robot/core/math.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <stdexcept>

namespace sim {
HolonomicActuatorController::HolonomicActuatorController(HolonomicActuatorControllerConfig config,
                                                         WheelPidGains gains)
    : config_(config), drive_pid_{PID(gains.kp, gains.ki, gains.kd, config.duty_limit, config.integral_limit,
                                      AntiWindupMode::ConditionalIntegration),
                                  PID(gains.kp, gains.ki, gains.kd, config.duty_limit, config.integral_limit,
                                      AntiWindupMode::ConditionalIntegration),
                                  PID(gains.kp, gains.ki, gains.kd, config.duty_limit, config.integral_limit,
                                      AntiWindupMode::ConditionalIntegration),
                                  PID(gains.kp, gains.ki, gains.kd, config.duty_limit, config.integral_limit,
                                      AntiWindupMode::ConditionalIntegration)} {
 const double positive_values[]{config_.wheel_base_m, config_.track_width_m, config_.wheel_radius_m,
 config_.max_wheel_speed_mps, config_.duty_limit};
 if (std::any_of(std::begin(positive_values), std::end(positive_values),
 [](double value) { return !std::isfinite(value) || value <= 0.0; }) ||
 !std::isfinite(config_.integral_limit) || config_.integral_limit < 0.0 ||
 !std::isfinite(config_.steer_kp) || config_.steer_kp < 0.0)
  throw std::invalid_argument("invalid holonomic actuator controller configuration");
}
void HolonomicActuatorController::reset() {
 for (auto& pid : drive_pid_)
  pid.reset();
}

HolonomicActuatorControlOutput HolonomicActuatorController::update(const DiffCmd& command,
                                                                   const HolonomicActuatorFeedback& feedback,
                                                                   double dt) {
 if (!(dt > 0.0) || !std::isfinite(dt))
  throw std::invalid_argument("holonomic controller step must be positive");
 const double command_values[]{command.target_v, command.target_lateral_v, command.target_w};
 if (std::any_of(std::begin(command_values), std::end(command_values),
 [](double value) { return !std::isfinite(value); }) ||
 std::any_of(feedback.drive_velocity_radps.begin(), feedback.drive_velocity_radps.end(),
 [](double value) { return !std::isfinite(value); }) ||
 std::any_of(feedback.steer_position_rad.begin(), feedback.steer_position_rad.end(),
 [](double value) { return !std::isfinite(value); }))
 throw std::invalid_argument("holonomic controller input must be finite");
 HolonomicActuatorControlOutput out;
 if (config_.kind == HolonomicControlKind::Swerve) {
  const auto modules =
    swerve_inverse(command.target_v, command.target_lateral_v, command.target_w, config_.wheel_base_m,
                   config_.track_width_m, feedback.steer_position_rad);
  for (std::size_t i = 0; i < 4; ++i) {
   out.target_speed_mps[i] = modules[i].speed_mps;
   out.target_steer_rad[i] = modules[i].steer_rad;
   out.steer_duty[i] =
     clamp(config_.steer_kp * wrap_angle(modules[i].steer_rad - feedback.steer_position_rad[i]), -1.0, 1.0);
  }
 } else {
  out.target_speed_mps = mecanum_inverse(command.target_v, command.target_lateral_v, command.target_w,
                                         config_.wheel_base_m, config_.track_width_m);
 }
 double peak = 0.0;
 for (double v : out.target_speed_mps)
  peak = std::max(peak, std::abs(v));
 const double scale = peak > config_.max_wheel_speed_mps ? config_.max_wheel_speed_mps / peak : 1.0;
 for (std::size_t i = 0; i < 4; ++i) {
  out.target_speed_mps[i] *= scale;
  const double actual = feedback.drive_velocity_radps[i] * config_.wheel_radius_m;
 if (std::abs(out.target_speed_mps[i]) > 1e-4 &&
 out.target_speed_mps[i] * drive_pid_[i].i() < 0.0 &&
 out.target_speed_mps[i] * actual >= 0.0)
 drive_pid_[i].reset();
  out.drive_duty[i] = drive_pid_[i].update(out.target_speed_mps[i] - actual, dt);
 }
 return out;
}
} // namespace sim
