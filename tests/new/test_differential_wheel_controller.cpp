#include "robot/control/differential_wheel_controller.hpp"

#include <cmath>
#include <iostream>

#define CHECK(condition) \
    do { \
        if (!(condition)) { \
            std::cerr << __FILE__ << ':' << __LINE__ \
                      << ": check failed: " #condition "\n"; \
            return 1; \
        } \
    } while (false)

int main() {
    const sim::DifferentialWheelControllerConfig config{
        0.50, 0.10, 2.0, 1000, 1.0, 2.0};
    sim::DifferentialWheelController controller(config, {0.5, 0.0, 0.0});

    sim::DifferentialWheelFeedback feedback;
    feedback.positions_valid = true;
    const auto first = controller.update({1.0, 1.0}, feedback, 0.02);
    CHECK(std::abs(first.left.target_mps - 0.75) < 1e-12);
    CHECK(std::abs(first.right.target_mps - 1.25) < 1e-12);
    CHECK(first.left.measured_mps == 0.0);
    CHECK(first.right.measured_mps == 0.0);
    CHECK(first.left.normalized_duty > 0.0);
    CHECK(first.right.normalized_duty > first.left.normalized_duty);

 const double radians_per_count = 2.0 * 3.14159265358979323846 / 1.0;
    feedback.left_position_rad = 20.0 * radians_per_count;
    feedback.right_position_rad = 30.0 * radians_per_count;
    feedback.left_velocity_radps = 6.0;
    feedback.right_velocity_radps = 9.0;
    const auto second = controller.update({1.0, 1.0}, feedback, 0.02);
    CHECK(std::abs(second.left.measured_mps - 20.0 * 0.10 * radians_per_count / 0.02) < 1e-12);
    CHECK(std::abs(second.right.measured_mps - 30.0 * 0.10 * radians_per_count / 0.02) < 1e-12);
    CHECK(std::abs(second.measured_forward_mps -
                   0.5 * (second.left.measured_mps + second.right.measured_mps)) < 1e-12);
    CHECK(std::abs(second.measured_yaw_radps -
                   (second.right.measured_mps - second.left.measured_mps) / 0.50) < 1e-12);

    const auto saturated = controller.update({5.0, 0.0}, feedback, 0.02);
    CHECK(saturated.left.target_mps == 2.0);
    CHECK(saturated.right.target_mps == 2.0);
    CHECK(std::abs(saturated.left.normalized_duty) <= 1.0);
    CHECK(std::abs(saturated.right.normalized_duty) <= 1.0);

    controller.reset();
    const auto reset = controller.update({}, feedback, 0.02);
    CHECK(reset.left.measured_mps == 0.0);
    CHECK(reset.right.measured_mps == 0.0);

 // A retained positive integral must never drive a wheel forward after a
 // non-zero target changes sign. This matters when a local planner asks one
 // wheel to reverse for a tight turn under load.
 sim::DifferentialWheelController reversing(config, {0.2, 1.0, 0.0});
 sim::DifferentialWheelFeedback stopped_feedback;
 sim::DifferentialWheelControlOutput forward_windup;
 for (int step = 0; step < 100; ++step)
 forward_windup = reversing.update({1.0, 0.0}, stopped_feedback, 0.02);
 CHECK(forward_windup.left.normalized_duty > 0.5);
 const auto reverse_after_windup = reversing.update({-1.0, 0.0}, stopped_feedback, 0.02);
 CHECK(reverse_after_windup.left.normalized_duty < 0.0);
 CHECK(reverse_after_windup.right.normalized_duty < 0.0);

 // A zero-speed request may brake measured motion through the proportional
 // term, but once the encoder is stopped it must not retain historical PWM.
 const auto zero_after_windup = reversing.update({0.0, 0.0}, stopped_feedback, 0.02);
 CHECK(zero_after_windup.left.normalized_duty == 0.0);
 CHECK(zero_after_windup.right.normalized_duty == 0.0);
 CHECK(zero_after_windup.left.pid_i == 0.0);
 CHECK(zero_after_windup.right.pid_i == 0.0);

 // Zero-speed control under a persistent external load is an active velocity
 // objective. Clear old maneuver windup once, but then retain new integral
 // effort so a finite motor can remove steady-state grade error.
 sim::DifferentialWheelController grade_hold(config, {0.2, 0.8, 0.0});
 sim::DifferentialWheelFeedback downhill_feedback;
 downhill_feedback.left_velocity_radps = -1.0;
 downhill_feedback.right_velocity_radps = -1.0;
 const auto hold_first = grade_hold.update({}, downhill_feedback, 0.02);
 sim::DifferentialWheelControlOutput hold_later;
 for (int step = 0; step < 40; ++step)
 hold_later = grade_hold.update({}, downhill_feedback, 0.02);
 CHECK(hold_first.left.normalized_duty > 0.0);
 CHECK(hold_later.left.pid_i > hold_first.left.pid_i);
 CHECK(hold_later.right.pid_i > hold_first.right.pid_i);
 CHECK(hold_later.left.normalized_duty > hold_first.left.normalized_duty);
 CHECK(hold_later.right.normalized_duty > hold_first.right.normalized_duty);

 // A common SI acceleration request maps through mass sharing and finite
 // motor force. This oracle is independent of the PID: 20 kg * 1 m/s^2 / two
 // units / 50 N per unit = 0.2 duty at zero wheel speed.
 auto compensated_config = config;
 compensated_config.total_mass_kg = 20.0;
 compensated_config.stall_force_per_unit_N = 50.0;
 compensated_config.free_speed_mps = 2.0;
 compensated_config.driven_unit_count = 2.0;
 sim::DifferentialWheelController compensated(
 compensated_config, {0.0, 0.0, 0.0});
 sim::DiffCmd compensation_command;
 compensation_command.target_v = 1.0;
 compensation_command.longitudinal_acceleration_feedforward_mps2 = 1.0;
 const auto compensation = compensated.update(
 compensation_command, stopped_feedback, 0.02);
 CHECK(std::abs(compensation.left.acceleration_feedforward - 0.2) < 1e-12);
 CHECK(std::abs(compensation.right.acceleration_feedforward - 0.2) < 1e-12);
 CHECK(std::abs(compensation.left.normalized_duty - 0.2) < 1e-12);
 CHECK(std::abs(compensation.right.normalized_duty - 0.2) < 1e-12);
 compensation_command.target_v = 0.0;
 const auto grade_hold_compensation = compensated.update(
 compensation_command, stopped_feedback, 0.02);
 CHECK(std::abs(grade_hold_compensation.left.acceleration_feedforward - 0.2) < 1e-12);
 CHECK(std::abs(grade_hold_compensation.right.acceleration_feedforward - 0.2) < 1e-12);

 // Common-mode grade compensation must never reverse the signed duty of an
 // inner wheel that the twist command asks to run backward. Without an
 // angular-acceleration feedforward contract, that wheel remains under its
 // signed static/PID loop while the outer wheel receives the compatible
 // common-mode term.
 auto turning_config = compensated_config;
 turning_config.static_friction_feedforward_duty = 0.08;
 sim::DifferentialWheelController loaded_turn(
 turning_config, {0.5, 0.0, 0.0});
 sim::DiffCmd turning_command;
 turning_command.target_v = 0.20;
 turning_command.target_w = -1.0;
 turning_command.longitudinal_acceleration_feedforward_mps2 = 1.0;
 const auto turning = loaded_turn.update(
 turning_command, stopped_feedback, 0.02);
 CHECK(turning.left.target_mps > 0.0);
 CHECK(turning.right.target_mps < 0.0);
 CHECK(std::abs(turning.left.acceleration_feedforward - 0.2) < 1e-12);
 CHECK(turning.right.acceleration_feedforward == 0.0);
 CHECK(turning.left.normalized_duty > 0.0);
 CHECK(turning.right.normalized_duty < 0.0);

 // Low-speed turns need an identified static-friction term immediately;
 // waiting for unrelated historical integral state made physical behavior
 // depend on the route taken before the turn. The term follows the requested
 // wheel direction and disappears completely on a stop request.
 const sim::DifferentialWheelControllerConfig low_speed_config{
 0.58, 0.10, 2.4, 2048, 1.0, 2.0, 0.08};
 sim::DifferentialWheelController low_speed(low_speed_config, {2.5, 0.5, 0.0});
 const auto spin_start = low_speed.update({0.0, 0.1}, stopped_feedback, 0.02);
 CHECK(std::abs(spin_start.left.target_mps + 0.029) < 1e-12);
 CHECK(std::abs(spin_start.right.target_mps - 0.029) < 1e-12);
 CHECK(spin_start.left.static_feedforward == -0.08);
 CHECK(spin_start.right.static_feedforward == 0.08);
 CHECK(spin_start.left.normalized_duty < -0.08);
 CHECK(spin_start.right.normalized_duty > 0.08);
 const auto spin_stop = low_speed.update({}, stopped_feedback, 0.02);
 CHECK(spin_stop.left.static_feedforward == 0.0);
 CHECK(spin_stop.right.static_feedforward == 0.0);
 CHECK(spin_stop.left.normalized_duty == 0.0);
 CHECK(spin_stop.right.normalized_duty == 0.0);

    bool threw = false;
    try {
        sim::DifferentialWheelController invalid({0.0, 0.1, 1.0, 100, 1.0, 1.0}, {});
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    CHECK(threw);
    return 0;
}
