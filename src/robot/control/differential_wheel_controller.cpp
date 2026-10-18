#include "robot/control/differential_wheel_controller.hpp"

#include "robot/core/math.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sim {

WheelPidGains tune_wheel_pid(double wheel_mass_kg, double stall_force_N,
                             double free_speed_mps, double linearize_at_mps,
                             const WheelControllerDesign& design) {
    const double speed = std::max(0.0, linearize_at_mps);
    const double slope =
        std::max(0.05, 1.0 - speed / std::max(0.05, free_speed_mps));
    const double linear_gain = std::max(1.0, stall_force_N * slope);
    const double bandwidth = std::max(0.01, design.bandwidth_radps);
    const double integral_time = std::max(0.01, design.integral_time_s);
    const double derivative_time = std::max(0.0, design.derivative_time_s);
    const double kp = wheel_mass_kg * bandwidth / linear_gain;
    return {kp, kp / integral_time, kp * derivative_time};
}

DifferentialWheelController::DifferentialWheelController(
    DifferentialWheelControllerConfig config, WheelPidGains gains)
    : config_(config),
      left_pid_(gains.kp, gains.ki, gains.kd, config.duty_limit,
                config.integral_limit, AntiWindupMode::ConditionalIntegration),
      right_pid_(gains.kp, gains.ki, gains.kd, config.duty_limit,
                 config.integral_limit, AntiWindupMode::ConditionalIntegration) {
    if (!std::isfinite(config_.track_width_m) || config_.track_width_m <= 0.0 ||
        !std::isfinite(config_.wheel_radius_m) || config_.wheel_radius_m <= 0.0 ||
        !std::isfinite(config_.max_wheel_speed_mps) ||
        config_.max_wheel_speed_mps <= 0.0 ||
        config_.encoder_counts_per_revolution <= 0 ||
        !std::isfinite(config_.duty_limit) || config_.duty_limit <= 0.0 ||
 !std::isfinite(config_.integral_limit) || config_.integral_limit < 0.0 ||
 !std::isfinite(config_.static_friction_feedforward_duty) ||
 config_.static_friction_feedforward_duty < 0.0 ||
 config_.static_friction_feedforward_duty > config_.duty_limit) {
        throw std::invalid_argument("invalid differential wheel controller configuration");
    }
}

void DifferentialWheelController::reset() {
    left_pid_.reset();
    right_pid_.reset();
    encoder_initialized_ = false;
    previous_left_count_ = 0.0;
    previous_right_count_ = 0.0;
 target_initialized_ = false;
 previous_left_target_mps_ = 0.0;
 previous_right_target_mps_ = 0.0;
}

double DifferentialWheelController::encoder_measure(double position_rad,
                                                     double& previous_count,
                                                     double dt_s) const {
    const double counts_per_rad =
        static_cast<double>(config_.encoder_counts_per_revolution) / (2.0 * kPi);
    const double count = std::round(position_rad * counts_per_rad);
    const double delta = count - previous_count;
    previous_count = count;
    return delta / counts_per_rad * config_.wheel_radius_m / dt_s;
}

DifferentialWheelControlOutput DifferentialWheelController::update(
    const DiffCmd& command, const DifferentialWheelFeedback& feedback,
    double dt_s) {
    if (!std::isfinite(dt_s) || dt_s <= 0.0) {
        throw std::invalid_argument("wheel controller step must be finite and positive");
    }
    const double values[] = {
        command.target_v,
        command.target_w,
        feedback.left_position_rad,
        feedback.right_position_rad,
        feedback.left_velocity_radps,
        feedback.right_velocity_radps,
    };
    for (double value : values) {
        if (!std::isfinite(value)) {
            throw std::invalid_argument("wheel controller input must be finite");
        }
    }

    DifferentialWheelControlOutput output;
    DifferentialKinematics::twist_to_wheels(
        command.target_v, command.target_w, config_.track_width_m,
        output.left.target_mps, output.right.target_mps);
    output.left.target_mps = clamp(output.left.target_mps,
                                   -config_.max_wheel_speed_mps,
                                   config_.max_wheel_speed_mps);
    output.right.target_mps = clamp(output.right.target_mps,
                                    -config_.max_wheel_speed_mps,
                                    config_.max_wheel_speed_mps);
    output.left.actual_mps = feedback.left_velocity_radps * config_.wheel_radius_m;
    output.right.actual_mps = feedback.right_velocity_radps * config_.wheel_radius_m;

    if (feedback.positions_valid) {
        const double counts_per_rad =
            static_cast<double>(config_.encoder_counts_per_revolution) / (2.0 * kPi);
        if (!encoder_initialized_) {
            previous_left_count_ = std::round(feedback.left_position_rad * counts_per_rad);
            previous_right_count_ = std::round(feedback.right_position_rad * counts_per_rad);
            encoder_initialized_ = true;
        } else {
            output.left.measured_mps =
                encoder_measure(feedback.left_position_rad, previous_left_count_, dt_s);
            output.right.measured_mps =
                encoder_measure(feedback.right_position_rad, previous_right_count_, dt_s);
        }
    } else {
        const double distance_per_count =
            2.0 * kPi * config_.wheel_radius_m /
            static_cast<double>(config_.encoder_counts_per_revolution);
        output.left.measured_mps =
            std::round(output.left.actual_mps * dt_s / distance_per_count) *
            distance_per_count / dt_s;
        output.right.measured_mps =
            std::round(output.right.actual_mps * dt_s / distance_per_count) *
            distance_per_count / dt_s;
    }

 const auto prepare_integral = [&](double target_mps, double previous_target_mps,
 double measured_mps, PID& pid) {
 const bool stop_requested = std::abs(target_mps) <= 1e-4;
 const bool direction_changed = target_initialized_ &&
 target_mps * previous_target_mps <= 0.0 &&
 std::abs(target_mps - previous_target_mps) > 1e-4;
 const bool sharply_reduced = target_initialized_ &&
 std::abs(previous_target_mps) > 0.1 &&
 std::abs(target_mps) < 0.5 * std::abs(previous_target_mps);
 const bool opposing = std::abs(target_mps) > 1e-4 &&
 target_mps * pid.i() < 0.0 && target_mps * measured_mps >= 0.0;
 if (stop_requested || direction_changed || sharply_reduced || opposing)
 pid.reset();
 };
 prepare_integral(output.left.target_mps, previous_left_target_mps_,
 output.left.measured_mps, left_pid_);
 prepare_integral(output.right.target_mps, previous_right_target_mps_,
 output.right.measured_mps, right_pid_);
 const auto static_feedforward = [&](double target_mps) {
 return std::abs(target_mps) <= 1e-4
 ? 0.0
 : std::copysign(config_.static_friction_feedforward_duty, target_mps);
 };
 output.left.static_feedforward = static_feedforward(output.left.target_mps);
 output.right.static_feedforward = static_feedforward(output.right.target_mps);
 output.left.normalized_duty = clamp(
 left_pid_.update(output.left.target_mps - output.left.measured_mps, dt_s) +
 output.left.static_feedforward,
 -config_.duty_limit, config_.duty_limit);
 output.right.normalized_duty = clamp(
 right_pid_.update(output.right.target_mps - output.right.measured_mps, dt_s) +
 output.right.static_feedforward,
 -config_.duty_limit, config_.duty_limit);
    output.left.pid_p = left_pid_.p();
    output.left.pid_i = left_pid_.i();
    output.left.pid_d = left_pid_.d();
    output.right.pid_p = right_pid_.p();
    output.right.pid_i = right_pid_.i();
    output.right.pid_d = right_pid_.d();
 previous_left_target_mps_ = output.left.target_mps;
 previous_right_target_mps_ = output.right.target_mps;
 target_initialized_ = true;
    DifferentialKinematics::wheels_to_twist(
        output.left.measured_mps, output.right.measured_mps,
        config_.track_width_m, output.measured_forward_mps,
        output.measured_yaw_radps);
    return output;
}

} // namespace sim
