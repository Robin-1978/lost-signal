#include "robot/control/ackermann_controller.hpp"

#include <algorithm>
#include <cmath>

#include "robot/platform/ackermann_kinematics.hpp"

namespace lost_signal::robot::control {

AckermannController::AckermannController(AckermannControllerConfig config)
    : config_(config),
      rear_left_pid_(config.kp, config.ki, config.kd, config.duty_limit,
                     config.integral_limit,
                     AntiWindupMode::ConditionalIntegration),
      rear_right_pid_(config.kp, config.ki, config.kd, config.duty_limit,
                      config.integral_limit,
                      AntiWindupMode::ConditionalIntegration) {}

void AckermannController::reset() {
  rear_left_pid_.reset();
  rear_right_pid_.reset();
  previous_target_v_mps_ = 0.0;
  target_initialized_ = false;
}

AckermannCommand AckermannController::update(double target_v_mps,
                                             double target_steer_rad,
                                             const AckermannFeedback& fb,
                                             double dt_s) {
  AckermannCommand out;
  if (!(dt_s > 0.0) || !std::isfinite(target_v_mps) ||
      !std::isfinite(target_steer_rad)) {
    return out;
  }
  // Do not carry a signed velocity-loop integral through a stop or across a
  // direction change.  This is a physical control-segment boundary, not a
  // mission- or scenario-specific exception: a new request must be allowed
  // to use the configured feed-forward and proportional response immediately.
  const bool target_stop = std::abs(target_v_mps) <= 1e-12;
  const bool previous_stop =
      target_initialized_ && std::abs(previous_target_v_mps_) <= 1e-12;
  const bool direction_reversal =
      target_initialized_ && !target_stop && !previous_stop &&
      target_v_mps * previous_target_v_mps_ < 0.0;
  if (target_stop || direction_reversal) {
    rear_left_pid_.reset();
    rear_right_pid_.reset();
  }
  previous_target_v_mps_ = target_v_mps;
  target_initialized_ = true;
  const double steer =
      std::clamp(target_steer_rad, -config_.max_steer_rad, config_.max_steer_rad);
  const auto angles = platform::ackermann_steering_angles(
      steer, config_.wheel_base_m, config_.track_width_m);
  const auto rear = platform::ackermann_rear_wheel_speeds(
      std::clamp(target_v_mps, -config_.max_wheel_speed_mps,
                 config_.max_wheel_speed_mps),
      steer, config_.wheel_base_m, config_.track_width_m);

  // Steering is a geometric position servo: same angle law the plant
  // integrates, so open-loop targets are exact and the P cascade only
  // trims rate. Drives close the full PID+FF loop below.
  if (!fb.valid || !std::isfinite(fb.rear_left_radps) ||
      !std::isfinite(fb.rear_right_radps)) {
    return out;
  }
  const double left_meas_mps = fb.rear_left_radps * config_.wheel_radius_m;
  const double right_meas_mps = fb.rear_right_radps * config_.wheel_radius_m;
  const double v_meas = 0.5 * (left_meas_mps + right_meas_mps);
  const BodyAccelDes des =
      lead_accel(target_v_mps, v_meas, 0.0, 0.0, config_.lead_time_s);
  // Twin rear units share the longitudinal desire; the same linear
  // coefficients as twist_to_wheels distribute it (differentiation is
  // linear, so the feedforward stays consistent with the kinematics).
  const TwistFeedforward ff = twist_feedforward(
      des, v_meas, 0.0, config_.track_width_m, config_.ff);

  const double left_err =
      std::clamp(rear.left_mps, -config_.max_wheel_speed_mps,
                 config_.max_wheel_speed_mps) -
      left_meas_mps;
  const double right_err =
      std::clamp(rear.right_mps, -config_.max_wheel_speed_mps,
                 config_.max_wheel_speed_mps) -
      right_meas_mps;
  out.rear_left_duty = std::clamp(ff.left.pwm_ff + rear_left_pid_.update(left_err, dt_s),
                                  -config_.duty_limit, config_.duty_limit);
  out.rear_right_duty = std::clamp(
      ff.right.pwm_ff + rear_right_pid_.update(right_err, dt_s),
      -config_.duty_limit, config_.duty_limit);

  // Position->rate cascade per front joint, normalized by max rate.
  // P-only on position: the plant's own servo lag provides the damping,
  // and measured steer velocity rides along as telemetry for the sim
  // feedback path (not consumed here, so no double-counted damping).
  const auto steer_duty = [&](double target_rad, double pos_rad) {
    const double rate_des = std::clamp(
        config_.steer_kp * (target_rad - pos_rad), -config_.max_steer_rate_radps,
        config_.max_steer_rate_radps);
    return std::clamp(rate_des / config_.max_steer_rate_radps, -1.0, 1.0);
  };
  out.front_left_steer_duty =
      steer_duty(angles.left_rad, fb.front_left_steer_rad);
  out.front_right_steer_duty =
      steer_duty(angles.right_rad, fb.front_right_steer_rad);
  return out;
}

}  // namespace lost_signal::robot::control
