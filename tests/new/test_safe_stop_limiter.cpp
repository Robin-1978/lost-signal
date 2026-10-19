#include "robot/autonomy/control/safe_stop_limiter.hpp"
#include "robot/control/differential_wheel_controller.hpp"

#include <cmath>
#include <iostream>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

namespace {

sim::ChassisCaps differential_chassis() {
 sim::ChassisCaps caps;
 caps.command_space = sim::ChassisCommandSpace::YawRate;
 caps.geometry.support_min_x_m = -0.31;
 caps.geometry.support_max_x_m = 0.12;
 return caps;
}

sim::SafeStopLimits base_limits() {
 sim::SafeStopLimits limits;
 limits.update_interval_s = 0.1;
 limits.brake_decel_mps2 = 5.0;
 limits.max_speed_mps = 2.0;
 limits.max_turn = 3.0;
 limits.max_turn_rate = 4.0;
 limits.differential_track_m = 0.50;
 limits.max_wheel_speed_mps = 2.0;
 limits.center_of_mass_height_m = 0.40;
 limits.stability_margin_fraction = 1.0;
 return limits;
}

} // namespace

int main() {
 const auto chassis = differential_chassis();
 const sim::SafeStopMotion moving{1.0, 0.0, 0.8};
 const auto level = sim::stage_safe_stop(chassis, moving, base_limits());

 // A stop request from motion must remain a reachable next command. For this
 // asymmetric support, the independent analytic level-ground braking limit is
 // g * front / COM height = 2.943 m/s^2.
 constexpr double expected_decel_mps2 = 9.81 * 0.12 / 0.40;
 constexpr double expected_delta_mps = expected_decel_mps2 * 0.1;
 CHECK(std::abs(level.applied_linear_decel_limit_mps2 - expected_decel_mps2) < 1e-12);
 CHECK(std::abs(level.forward_mps - (1.0 - expected_delta_mps)) < 1e-12);
 CHECK(level.forward_mps > 0.0);

 // Each differential wheel, rather than only body speed, respects the same
 // one-interval bound. This prevents a yaw command from hiding an impossible
 // wheel reversal.
 const double old_left = moving.forward_mps - moving.turn * 0.25;
 const double old_right = moving.forward_mps + moving.turn * 0.25;
 const double new_left = level.forward_mps - level.turn * 0.25;
 const double new_right = level.forward_mps + level.turn * 0.25;
 CHECK(std::abs(new_left - old_left) <= expected_delta_mps + 1e-12);
 CHECK(std::abs(new_right - old_right) <= expected_delta_mps + 1e-12);

 auto downhill_limits = base_limits();
 downhill_limits.longitudinal_grade = -0.11;
 const auto downhill = sim::stage_safe_stop(chassis, moving, downhill_limits);
 CHECK(downhill.applied_linear_decel_limit_mps2 <
 level.applied_linear_decel_limit_mps2);
 CHECK(downhill.forward_mps > level.forward_mps);

 // A taller loaded COM reduces the permitted command step. This oracle uses
 // only declared geometry, grade and motion; simulator contact truth is not an
 // input to the deployable limiter.
 auto loaded_limits = base_limits();
 loaded_limits.center_of_mass_height_m = 0.80;
 const auto loaded = sim::stage_safe_stop(chassis, moving, loaded_limits);
 CHECK(loaded.applied_linear_decel_limit_mps2 <
 level.applied_linear_decel_limit_mps2);
 CHECK(loaded.forward_mps > level.forward_mps);

 // The exact limit used by execution is public to the local planner. A COM
 // projection at the front support has no longitudinal braking margin; the
 // contract reports zero instead of inventing a small stabilizing force.
 auto unsupported_limits = base_limits();
 unsupported_limits.center_of_mass_x_m =
 chassis.geometry.support_max_x_m;
 CHECK(sim::safe_stop_linear_decel_limit(
 chassis, unsupported_limits) == 0.0);

 sim::ChassisCaps ackermann = chassis;
 ackermann.command_space = sim::ChassisCommandSpace::SteeringAngle;
 auto ackermann_limits = base_limits();
 ackermann_limits.max_turn = 0.6;
 ackermann_limits.max_turn_rate = 0.5;
 const auto steering_stop = sim::stage_safe_stop(
 ackermann, {0.8, 0.0, 0.4}, ackermann_limits);
 CHECK(steering_stop.forward_mps > 0.0);
 CHECK(std::abs(steering_stop.turn - 0.35) < 1e-12);

 // A controlled stop cannot inherit integral effort from the preceding
 // maneuver. This independent controller test builds positive integral at a
 // stalled wheel, then asks for a lower speed while the wheel is moving
 // faster than the new target. The braking command must oppose motion.
 sim::DifferentialWheelControllerConfig controller_config;
 controller_config.track_width_m = 0.5;
 controller_config.wheel_radius_m = 0.1;
 sim::DifferentialWheelController ordinary(controller_config, {0.5, 0.5, 0.0});
 sim::DifferentialWheelController braking_controller(
 controller_config, {0.5, 0.5, 0.0});
 const sim::DifferentialWheelFeedback stalled{};
 for (int index = 0; index < 25; ++index) {
 ordinary.update({1.0, 0.0}, stalled, 0.02);
 braking_controller.update({1.0, 0.0}, stalled, 0.02);
 }
 sim::DifferentialWheelFeedback moving_feedback;
 moving_feedback.left_velocity_radps = 8.0;
 moving_feedback.right_velocity_radps = 8.0;
 const auto ordinary_reduction = ordinary.update(
 {0.6, 0.0}, moving_feedback, 0.02);
 sim::DiffCmd controlled_reduction{0.6, 0.0};
 controlled_reduction.controlled_stop = true;
 const auto controlled_output = braking_controller.update(
 controlled_reduction, moving_feedback, 0.02);
 CHECK(ordinary_reduction.left.normalized_duty > 0.0);
 CHECK(controlled_output.left.normalized_duty < 0.0);
 CHECK(controlled_output.right.normalized_duty < 0.0);
 return 0;
}
