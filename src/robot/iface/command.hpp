#pragma once

#include "robot/core/time.hpp"

namespace lost_signal::robot::iface {

// Single canonical body command space for v1. More spaces are added as new
// enum values, never by reinterpreting these fields.
enum class CommandSpace {
  kStop = 0,
  kYawRate = 1,  // forward_mps + yaw_rate_radps
};

struct MotionCommand {
  CommandSpace space = CommandSpace::kStop;
  core::MonotonicNs stamp_ns = core::kInvalidTime;
  core::MonotonicNs expiry_ns = core::kInvalidTime;
  double forward_mps = 0.0;
  double yaw_rate_radps = 0.0;

  static MotionCommand SafeStop(core::MonotonicNs now_ns) {
    MotionCommand c;
    c.space = CommandSpace::kStop;
    c.stamp_ns = now_ns;
    c.expiry_ns = now_ns + 200'000'000;  // 200 ms watchdog
    return c;
  }

  bool is_safe_stop() const {
    return space == CommandSpace::kStop && forward_mps == 0.0 &&
           yaw_rate_radps == 0.0;
  }
};

bool valid_motion_command(const MotionCommand& cmd, core::MonotonicNs now_ns);

}  // namespace lost_signal::robot::iface
