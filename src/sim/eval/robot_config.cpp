#include "sim/eval/robot_config.hpp"

#include "sim/world/motor.hpp"
#include "sim/world/world.hpp"

namespace lost_signal::sim::eval {

world::MotorParams motor_from_config(
    const robot::platform::RobotConfig& config) {
  world::MotorParams motor =
      world::motor_params_from_profile(config.profile);
  motor.brake_max_torque_Nm = config.brake_max_torque_Nm;
  motor.brake_apply_time_s = config.brake_apply_time_s;
  motor.brake_release_time_s = config.brake_release_time_s;
  return motor;
}

}  // namespace lost_signal::sim::eval
