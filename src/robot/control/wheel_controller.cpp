#include "robot/control/wheel_controller.hpp"

#include <algorithm>
#include <cmath>

namespace lost_signal::robot::control {

DifferentialWheelController::DifferentialWheelController(
    WheelControllerConfig config)
    : config_(config),
      left_pid_(config.kp, config.ki, config.kd, config.duty_limit,
                config.integral_limit,
                AntiWindupMode::ConditionalIntegration),
      right_pid_(config.kp, config.ki, config.kd, config.duty_limit,
                 config.integral_limit,
                 AntiWindupMode::ConditionalIntegration) {}

void DifferentialWheelController::reset() {
  left_pid_.reset();
  right_pid_.reset();
}

WheelCommand DifferentialWheelController::update(double target_v_mps,
                                                 double target_w_radps,
                                                 const WheelFeedback& fb,
                                                 double dt_s) {
  WheelCommand out;
  if (!(dt_s > 0.0) || !std::isfinite(target_v_mps) ||
      !std::isfinite(target_w_radps)) {
    return out;
  }
  // twist_to_wheels: v_l = v - w*track/2, v_r = v + w*track/2.
  // Inlined (two lines) so robot_control stays below robot_platform;
  // the canonical kinematics live in platform::DifferentialKinematics.
  double left_mps = target_v_mps - target_w_radps * config_.track_width_m * 0.5;
  double right_mps =
      target_v_mps + target_w_radps * config_.track_width_m * 0.5;
  left_mps = std::clamp(left_mps, -config_.max_wheel_speed_mps,
                        config_.max_wheel_speed_mps);
  right_mps = std::clamp(right_mps, -config_.max_wheel_speed_mps,
                         config_.max_wheel_speed_mps);

  // Feedforward from the model; PID trims the residual. Without valid
  // feedback there is nothing to close the loop on: hold zero (fail closed)
  // rather than driving blind on feedforward alone.
  if (!fb.valid || !std::isfinite(fb.left_radps) ||
      !std::isfinite(fb.right_radps)) {
    return out;
  }
  const double left_meas_mps = fb.left_radps * config_.wheel_radius_m;
  const double right_meas_mps = fb.right_radps * config_.wheel_radius_m;
  const double v_meas = 0.5 * (left_meas_mps + right_meas_mps);
  const double w_meas =
      (right_meas_mps - left_meas_mps) / config_.track_width_m;
  const BodyAccelDes des =
      lead_accel(target_v_mps, v_meas, target_w_radps, w_meas,
                 config_.lead_time_s);
  const TwistFeedforward ff =
      twist_feedforward(des, v_meas, w_meas, config_.track_width_m, config_.ff);

  const double left_err = left_mps - left_meas_mps;
  const double right_err = right_mps - right_meas_mps;
  const double left_fb = left_pid_.update(left_err, dt_s);
  const double right_fb = right_pid_.update(right_err, dt_s);
  out.left_duty = std::clamp(ff.left.pwm_ff + left_fb, -config_.duty_limit,
                             config_.duty_limit);
  out.right_duty = std::clamp(ff.right.pwm_ff + right_fb, -config_.duty_limit,
                              config_.duty_limit);
  return out;
}

}  // namespace lost_signal::robot::control
