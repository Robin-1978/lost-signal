#pragma once

#include "robot/core/time.hpp"

namespace lost_signal::robot::iface {

// Body command spaces. New spaces arrive as new enum values with their own
// named fields; existing fields are never reinterpreted across spaces.
enum class CommandSpace {
  kStop = 0,
  kYawRate = 1,    // forward_mps + yaw_rate_radps (differential default)
  kSteerAngle = 2,  // forward_mps + steer_angle_rad (ackermann native)
  kHolonomic = 3,   // forward_mps + lateral_mps + yaw_rate_radps
};

struct MotionCommand {
  CommandSpace space = CommandSpace::kStop;
  core::MonotonicNs stamp_ns = core::kInvalidTime;
  core::MonotonicNs expiry_ns = core::kInvalidTime;
  double forward_mps = 0.0;
  double yaw_rate_radps = 0.0;  // kYawRate + kHolonomic
  double steer_angle_rad = 0.0;  // kSteerAngle: equivalent front-wheel angle
  double lateral_mps = 0.0;      // kHolonomic: body-left velocity

  static MotionCommand SafeStop(core::MonotonicNs now_ns) {
    MotionCommand c;
    c.space = CommandSpace::kStop;
    c.stamp_ns = now_ns;
    c.expiry_ns = now_ns + 200'000'000;  // 200 ms watchdog
    return c;
  }

  bool is_safe_stop() const {
    return space == CommandSpace::kStop && forward_mps == 0.0 &&
           yaw_rate_radps == 0.0 && steer_angle_rad == 0.0 &&
           lateral_mps == 0.0;
  }
};

bool valid_motion_command(const MotionCommand& cmd, core::MonotonicNs now_ns);

}  // namespace lost_signal::robot::iface
