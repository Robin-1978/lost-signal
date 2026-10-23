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
  } else if (kind == "tracked") {
    // Skid-steer tracks: wide stance, long contact, yaw scrub from the
    // track-ground shear (Coulomb + viscous, main's TrackedDrive numbers
    // scaled to this mass). Same (2,0) duty layout as differential —
    // one side per duty channel — so the plant, odometry, and chassis all
    // reuse the skid-steer path with track numbers.
    base.wheel_radius_m = 0.1;
    base.wheel_base_m = 0.7;
    base.track_width_m = 0.6;
    base.yaw_scrub_coulomb_Nm = 8.0;
    base.yaw_scrub_viscous_Nm_per_radps = 6.0;
  } else {
    return false;
  }
  out = base;
  return true;
}

}  // namespace lost_signal::robot::platform
