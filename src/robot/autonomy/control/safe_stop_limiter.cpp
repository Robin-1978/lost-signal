#include "robot/autonomy/control/safe_stop_limiter.hpp"

#include "robot/autonomy/mapping/terrain.hpp"
#include "robot/autonomy/planning/local/command_window.hpp"

#include <algorithm>

namespace sim {

double safe_stop_linear_decel_limit(const ChassisCaps& chassis,
 const SafeStopLimits& limits) {
 const double front_support_m =
 chassis.geometry.support_max_x_m - limits.center_of_mass_x_m;
 const double rear_support_m =
 limits.center_of_mass_x_m - chassis.geometry.support_min_x_m;
 const double stability_limit_mps2 = longitudinal_stability_acceleration_limit(
 limits.longitudinal_grade, front_support_m, rear_support_m,
 limits.center_of_mass_height_m, limits.stability_margin_fraction);
 return std::min(std::max(0.0, limits.brake_decel_mps2),
 stability_limit_mps2);
}

SafeStopCommand stage_safe_stop(const ChassisCaps& chassis,
 const SafeStopMotion& motion, const SafeStopLimits& limits) {
 const double decel_limit_mps2 =
 safe_stop_linear_decel_limit(chassis, limits);
 const double linear_delta_mps =
 decel_limit_mps2 * std::max(0.0, limits.update_interval_s);

 SafeStopCommand output;
 output.applied_linear_decel_limit_mps2 = decel_limit_mps2;
 if (chassis.command_space == ChassisCommandSpace::YawRate &&
 limits.differential_track_m > 0.0) {
 const double half_track_m = 0.5 * limits.differential_track_m;
 const double left_mps = motion.forward_mps - motion.turn * half_track_m;
 const double right_mps = motion.forward_mps + motion.turn * half_track_m;
 // Subtracting the same absolute speed from both wheels preserves their
 // difference until the slower wheel reaches zero. On a slope that lets a
 // small yaw grow while the chassis is supposedly stopping. Allocate the
 // reachable wheel-speed box to reduce yaw first, then choose the closest
 // feasible longitudinal speed to zero. This never changes either wheel by
 // more than linear_delta_mps and never invents an instantaneous lock.
 const double wheel_limited_yaw_delta =
 2.0 * linear_delta_mps / limits.differential_track_m;
 const double requested_yaw_delta =
 std::max(0.0, limits.max_turn_rate) *
 std::max(0.0, limits.update_interval_s);
 output.turn = next_rate_limited_command(
 motion.turn, 0.0, -limits.max_turn, limits.max_turn,
 std::min(wheel_limited_yaw_delta, requested_yaw_delta));

 double minimum_forward = std::max(
 left_mps - linear_delta_mps + output.turn * half_track_m,
 right_mps - linear_delta_mps - output.turn * half_track_m);
 double maximum_forward = std::min(
 left_mps + linear_delta_mps + output.turn * half_track_m,
 right_mps + linear_delta_mps - output.turn * half_track_m);
 minimum_forward = std::max({
 minimum_forward,
 -limits.max_wheel_speed_mps + output.turn * half_track_m,
 -limits.max_wheel_speed_mps - output.turn * half_track_m,
 });
 maximum_forward = std::min({
 maximum_forward,
 limits.max_wheel_speed_mps + output.turn * half_track_m,
 limits.max_wheel_speed_mps - output.turn * half_track_m,
 });
 if (minimum_forward <= maximum_forward + 1e-12) {
 output.forward_mps = std::clamp(0.0, minimum_forward, maximum_forward);
 } else {
 // Defensive fallback for an already out-of-range measured state. Scaling
 // both wheels preserves curvature and respects the largest per-wheel step.
 const double largest_wheel_speed =
 std::max(std::abs(left_mps), std::abs(right_mps));
 const double scale = largest_wheel_speed > 1e-12
 ? std::max(0.0, 1.0 - linear_delta_mps / largest_wheel_speed)
 : 0.0;
 output.forward_mps = motion.forward_mps * scale;
 output.turn = motion.turn * scale;
 }
 return output;
 }

 output.forward_mps = next_rate_limited_command(
 motion.forward_mps, 0.0, -limits.max_speed_mps,
 limits.max_speed_mps, linear_delta_mps);
 output.lateral_mps = next_rate_limited_command(
 motion.lateral_mps, 0.0, -limits.max_speed_mps,
 limits.max_speed_mps, linear_delta_mps);
 const double turn_delta =
 std::max(0.0, limits.max_turn_rate) * std::max(0.0, limits.update_interval_s);
 output.turn = next_rate_limited_command(
 motion.turn, 0.0, -limits.max_turn, limits.max_turn, turn_delta);
 return output;
}

} // namespace sim
