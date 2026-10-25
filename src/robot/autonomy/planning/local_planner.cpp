#include "robot/autonomy/planning/local_planner.hpp"

#include "robot/autonomy/planning/dwa_planner.hpp"
#include "robot/autonomy/planning/teb_planner.hpp"

namespace lost_signal::robot::autonomy {

std::unique_ptr<ILocalPlanner> make_local_planner(
 const std::string& planner_kind,
 const platform::NavigationSafetyPolicy& navigation,
 const platform::MotionDynamicsPolicy& motion,
 double terminal_position_tolerance_m,
 double terminal_heading_tolerance_rad,
 double terminal_confidence_probability,
 double terminal_maximum_position_sigma_m,
 double terminal_maximum_heading_sigma_rad,
 int terminal_replan_settle_cycles) {
 if (planner_kind == "dwa") {
 DwaPlannerConfig config;
 config.safety_margin_m = navigation.clearance_margin_m;
 config.soft_inflation_radius_m = navigation.soft_inflation_radius_m;
 config.accel_mps2 = motion.acceleration_mps2;
 config.decel_mps2 = motion.brake_deceleration_mps2;
 config.lateral_accel_mps2 = motion.lateral_acceleration_mps2;
 config.yaw_accel_radps2 = motion.yaw_acceleration_radps2;
 config.response_delay_s = motion.actuator_response_delay_s;
 config.terminal_position_tolerance_m = terminal_position_tolerance_m;
 config.terminal_heading_tolerance_rad = terminal_heading_tolerance_rad;
 config.terminal_confidence_probability = terminal_confidence_probability;
 config.terminal_maximum_position_sigma_m =
 terminal_maximum_position_sigma_m;
 config.terminal_maximum_heading_sigma_rad =
 terminal_maximum_heading_sigma_rad;
 config.terminal_replan_settle_cycles = terminal_replan_settle_cycles;
 return std::make_unique<DwaPlanner>(config);
 }
 if (planner_kind == "teb") {
 TebPlannerConfig config;
 config.extra_clearance_m = navigation.clearance_margin_m;
 config.soft_inflation_radius_m = navigation.soft_inflation_radius_m;
 config.accel_mps2 = motion.acceleration_mps2;
 config.decel_mps2 = motion.brake_deceleration_mps2;
 config.lateral_accel_mps2 = motion.lateral_acceleration_mps2;
 config.yaw_accel_radps2 = motion.yaw_acceleration_radps2;
 config.terminal_position_tolerance_m = terminal_position_tolerance_m;
 config.terminal_confidence_probability = terminal_confidence_probability;
 return std::make_unique<TebPlanner>(config);
 }
 return nullptr;
}

} // namespace lost_signal::robot::autonomy
