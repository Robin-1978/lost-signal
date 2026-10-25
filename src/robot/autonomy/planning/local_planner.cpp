#include "robot/autonomy/planning/local_planner.hpp"

#include "robot/autonomy/planning/dwa_planner.hpp"
#include "robot/autonomy/planning/teb_planner.hpp"

namespace lost_signal::robot::autonomy {

std::unique_ptr<ILocalPlanner> make_local_planner(
 const std::string& planner_kind,
 const platform::NavigationSafetyPolicy& navigation,
 const platform::MotionDynamicsPolicy& motion) {
 if (planner_kind == "dwa") {
 DwaPlannerConfig config;
 config.safety_margin_m = navigation.clearance_margin_m;
 config.soft_inflation_radius_m = navigation.soft_inflation_radius_m;
 config.accel_mps2 = motion.acceleration_mps2;
 config.decel_mps2 = motion.brake_deceleration_mps2;
 config.lateral_accel_mps2 = motion.lateral_acceleration_mps2;
 config.yaw_accel_radps2 = motion.yaw_acceleration_radps2;
 config.response_delay_s = motion.actuator_response_delay_s;
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
 return std::make_unique<TebPlanner>(config);
 }
 return nullptr;
}

} // namespace lost_signal::robot::autonomy
