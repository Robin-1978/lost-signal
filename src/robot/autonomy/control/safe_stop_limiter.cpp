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
 const double next_left_mps = next_rate_limited_command(
 left_mps, 0.0, -limits.max_wheel_speed_mps,
 limits.max_wheel_speed_mps, linear_delta_mps);
 const double next_right_mps = next_rate_limited_command(
 right_mps, 0.0, -limits.max_wheel_speed_mps,
 limits.max_wheel_speed_mps, linear_delta_mps);
 output.forward_mps = 0.5 * (next_left_mps + next_right_mps);
 output.turn = (next_right_mps - next_left_mps) /
 limits.differential_track_m;
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
