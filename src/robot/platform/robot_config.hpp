#pragma once

// Hardware contract configuration (native-deployable: no sim headers,
// no sim libraries, no truth types — the boundary gate enforces this).
// One JSON file ("lost-signal-robot/1") carries the chassis kind, plant
// numbers, limits, sensor/link mounts, brake capability, and identity.
// The same file feeds SIM through sim/eval (MotorParams translation).
// A bare drive-profile JSON (flat keys + "kind") loads as a config with
// factory defaults filling the rest.

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "robot/core/json.hpp"
#include "robot/platform/description.hpp"
#include "robot/platform/drive_profile.hpp"

namespace lost_signal::robot::platform {

inline constexpr char kRobotConfigFormat[] = "lost-signal-robot/1";

struct RobotConfig {
  DriveProfile profile;  // chassis_kind + plant numbers
  // App bringup defaults (same numbers RobotApplication uses): 2 m/s,
  // 2 rad/s, 0.4 m footprint, 0.55 rad steer, holonomic lateral closed.
 ChassisLimits limits = {2.0, 2.0, 0.4, 0.55, 0.0};
 NavigationSafetyPolicy navigation;
 MotionDynamicsPolicy motion;
  std::vector<SensorMount> sensors;
  std::vector<LinkMount> links;
  // Service-brake capability (peak holding torque + lag shaping).
  // SIM consumes these as plant numbers; native maps its own brake
  // authority and may ignore them.
  double brake_max_torque_Nm = 0.0;
  double brake_apply_time_s = 0.12;
  double brake_release_time_s = 0.08;
  std::string robot_id = "ro1";
 std::string local_planner_kind = "dwa";
  std::uint64_t version = 1;
};

// Fail-closed file load: untouched output on any mismatch. Never reads
// SIM truth — only the hardware contract above.
bool load_robot_config(const std::string& path, RobotConfig& config,
                       std::string* error);
RobotDescription description_from_config(const RobotConfig& config);
// Shared merge primitives: ONE contract for the machine file ("drive" /
// "chassis" objects) and legacy flat drive profiles (flat keys + "kind").
// merge_drive_profile_keys takes flat drive keys + encoder + scrub;
// merge_chassis_config takes the "chassis" object or null (absent block =
// validate the defaulted geometry only). Fail-closed throughout.
bool merge_drive_profile_keys(const core::Json& obj, DriveProfile& merged,
                              std::string& err);
bool merge_chassis_config(const core::Json* chassis_obj, DriveProfile& merged,
                          std::string& err);
// Tool/bring-up default chain: explicit path wins; otherwise
// ./configs/robot/differential.json when it exists; otherwise nullopt and
// the caller falls back to code defaults. Never fails hard.
std::optional<RobotConfig> load_default_robot_config(
    const std::string& explicit_path, std::string* used_path,
    std::string* error);

}  // namespace lost_signal::robot::platform
