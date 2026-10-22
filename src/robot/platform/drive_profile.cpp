#include "robot/platform/drive_profile.hpp"

namespace lost_signal::robot::platform {

bool DriveProfile::for_chassis(const std::string& kind, DriveProfile& out) {
  DriveProfile base;
  base.chassis_kind = kind;
  if (kind == "differential") {
    base.wheel_radius_m = 0.1;
    base.track_width_m = 0.5;
    base.wheel_base_m = 0.56;  // unused by diff kinematics, kept for parity
  } else if (kind == "ackermann") {
    base.wheel_radius_m = 0.1;
    base.wheel_base_m = 0.62;
    base.track_width_m = 0.58;
  } else if (kind == "swerve" || kind == "mecanum") {
    base.wheel_radius_m = 0.1;
    base.wheel_base_m = 0.56;
    base.track_width_m = 0.56;
  } else {
    return false;
  }
  out = base;
  return true;
}

}  // namespace lost_signal::robot::platform
