#include "robot/control/holonomic_controller.hpp"

#include <algorithm>
#include <cmath>

#include "robot/platform/holonomic_kinematics.hpp"

namespace lost_signal::robot::control {

HolonomicController::HolonomicController(HolonomicControllerConfig config)
    : config_(config),
      drive_pids_{
          PID(config.kp, config.ki, config.kd, config.duty_limit,
              config.integral_limit, AntiWindupMode::ConditionalIntegration),
          PID(config.kp, config.ki, config.kd, config.duty_limit,
              config.integral_limit, AntiWindupMode::ConditionalIntegration),
          PID(config.kp, config.ki, config.kd, config.duty_limit,
              config.integral_limit, AntiWindupMode::ConditionalIntegration),
          PID(config.kp, config.ki, config.kd, config.duty_limit,
              config.integral_limit, AntiWindupMode::ConditionalIntegration)} {
}

void HolonomicController::reset() {
  for (auto& pid : drive_pids_) pid.reset();
}

HolonomicCommand HolonomicController::update(double target_v_mps,
                                             double target_lateral_mps,
                                             double target_yaw_radps,
                                             const HolonomicFeedback& fb,
                                             double dt_s) {
  HolonomicCommand out;
  if (!(dt_s > 0.0) || !std::isfinite(target_v_mps) ||
      !std::isfinite(target_lateral_mps) || !std::isfinite(target_yaw_radps)) {
    return out;
  }
  if (!fb.valid) return out;  // fail closed: never drive blind
  for (double v : fb.drive_radps) {
    if (!std::isfinite(v)) return out;
  }

  std::array<double, 4> target_speeds = {};
  std::array<double, 4> target_steers = {};
  if (config_.kind == HolonomicKind::Swerve) {
    const auto modules = platform::swerve_inverse(
        target_v_mps, target_lateral_mps, target_yaw_radps,
        config_.wheel_base_m, config_.track_width_m, fb.steer_rad);
    for (std::size_t i = 0; i < 4; ++i) {
      target_speeds[i] = modules[i].speed_mps;
      target_steers[i] = modules[i].steer_rad;
      // A swerve module cannot produce the requested body velocity while
      // its wheel is still pointed elsewhere. Cosine compensation removes
      // the sideways drive impulse during a direction change; the speed
      // returns continuously as the steering error closes.
      const double err = target_steers[i] - fb.steer_rad[i];
      target_speeds[i] *= std::max(0.0, std::cos(err));
      // Position->rate cascade, normalized by max rate.
      const double rate_des =
          std::clamp(config_.steer_kp * err, -config_.max_steer_rate_radps,
                     config_.max_steer_rate_radps);
      out.steer_duty[i] =
          std::clamp(rate_des / config_.max_steer_rate_radps, -1.0, 1.0);
    }
  } else {
    target_speeds = platform::mecanum_inverse(
        target_v_mps, target_lateral_mps, target_yaw_radps,
        config_.wheel_base_m, config_.track_width_m);
  }
  // Common desaturation: preserve direction, bound peak wheel speed.
  double peak = 0.0;
  for (double v : target_speeds) peak = std::max(peak, std::abs(v));
  const double scale =
      peak > config_.max_wheel_speed_mps ? config_.max_wheel_speed_mps / peak : 1.0;
  for (std::size_t i = 0; i < 4; ++i) {
    const double meas_mps = fb.drive_radps[i] * config_.wheel_radius_m;
    const double err = target_speeds[i] * scale - meas_mps;
    if (std::abs(target_speeds[i]) > 1e-4 &&
        target_speeds[i] * drive_pids_[i].i() < 0.0 &&
        target_speeds[i] * meas_mps >= 0.0) {
      drive_pids_[i].reset();  // opposing integral never fights a reversal
    }
    // Model feedforward per independent wheel + PID trim on the residual.
    const double accel_des = (target_speeds[i] * scale - meas_mps) /
                             std::max(1e-3, config_.lead_time_s);
    const auto ff =
        wheel_feedforward(accel_des, meas_mps, config_.ff);
    out.drive_duty[i] =
        std::clamp(ff.pwm_ff + drive_pids_[i].update(err, dt_s),
                   -config_.duty_limit, config_.duty_limit);
  }
  return out;
}

}  // namespace lost_signal::robot::control
