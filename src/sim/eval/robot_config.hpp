#pragma once

// Unified robot configuration (SIM-side file loading only): one JSON
// document configures both sides from the same numbers — the ROBOT
// RobotDescription (chassis kind, limits, sensor/link mounts) and the SIM
// MotorParams (plant numbers + brakes). ROBOT never parses files; the
// suite/session/viewer build their inputs through here.
//
// Top level is a drive-profile superset: flat plant keys + "kind" (or
// "chassis_kind"; both present must agree), plus optional "limits",
// "sensors", "links", "brake", "robot_id", "version" sections. A bare
// drive-profile file loads as a config (factory defaults fill the rest),
// so existing profile JSON keeps working.

#include <cstdint>
#include <string>
#include <vector>

#include "robot/platform/description.hpp"
#include "robot/platform/drive_profile.hpp"

namespace lost_signal::sim::world {
struct MotorParams;
}  // namespace lost_signal::sim::world

namespace lost_signal::sim::eval {

inline constexpr char kRobotConfigFormat[] = "lost-signal-robot/1";

struct RobotConfig {
  robot::platform::DriveProfile profile;  // chassis_kind + plant numbers
  // App bringup defaults (same numbers RobotApplication uses): 2 m/s,
  // 2 rad/s, 0.4 m footprint, 0.55 rad steer, holonomic lateral closed.
  robot::platform::ChassisLimits limits = {2.0, 2.0, 0.4, 0.55, 0.0};
  std::vector<robot::platform::SensorMount> sensors;
  std::vector<robot::platform::LinkMount> links;
  // Service brake (MotorParams defaults: disabled zero torque).
  double brake_max_torque_Nm = 0.0;
  double brake_apply_time_s = 0.12;
  double brake_release_time_s = 0.08;
  std::string robot_id = "ro1";
  std::uint64_t version = 1;
};

// Fail-closed load: untouched output on any mismatch.
bool load_robot_config(const std::string& path, RobotConfig& config,
                       std::string* error);
// Both sides from one config (single source of truth).
robot::platform::RobotDescription description_from_config(
    const RobotConfig& config);
world::MotorParams motor_from_config(const RobotConfig& config);

}  // namespace lost_signal::sim::eval
