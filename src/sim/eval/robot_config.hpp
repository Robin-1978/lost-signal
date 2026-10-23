#pragma once

// SIM-side translation of the shared hardware contract: the robot config
// file is owned and parsed ROBOT-side (robot::platform::load_robot_config
// — native-deployable, no sim linkage); this only converts it into SIM
// plant numbers. ROBOT never reads SIM truth; SIM never invents hardware.

#include "robot/platform/robot_config.hpp"
#include "sim/world/motor.hpp"

namespace lost_signal::sim::eval {

world::MotorParams motor_from_config(
    const robot::platform::RobotConfig& config);

}  // namespace lost_signal::sim::eval
