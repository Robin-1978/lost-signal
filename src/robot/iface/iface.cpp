#include "robot/iface/command.hpp"
#include "robot/iface/motion.hpp"
#include "robot/iface/observation.hpp"

#include <cmath>

namespace lost_signal::robot::iface {

bool valid_observation_frame(const ObservationFrame& frame) {
  if (frame.source.empty()) return false;
  if (frame.stamp_ns == core::kInvalidTime) return false;
  if (frame.sequence == 0) return false;
  if (frame.empty()) return false;
  return true;
}

bool valid_motion_command(const MotionCommand& cmd, core::MonotonicNs now_ns) {
  if (cmd.stamp_ns == core::kInvalidTime) return false;
  if (cmd.expiry_ns <= cmd.stamp_ns) return false;
  if (now_ns != core::kInvalidTime && now_ns > cmd.expiry_ns) return false;
  if (cmd.space == CommandSpace::kStop) {
    return cmd.is_safe_stop();
  }
  return true;
}

bool valid_actuator_command(const ActuatorCommand& cmd,
                            core::MonotonicNs now_ns) {
  if (cmd.mode != ActuatorMode::kNormalizedDuty) return false;
  if (cmd.stamp_ns == core::kInvalidTime) return false;
  if (cmd.expiry_ns <= cmd.stamp_ns) return false;
  if (now_ns != core::kInvalidTime && now_ns > cmd.expiry_ns) return false;
  if (!std::isfinite(cmd.left) || !std::isfinite(cmd.right)) return false;
  if (cmd.left < -1.0 || cmd.left > 1.0 || cmd.right < -1.0 ||
      cmd.right > 1.0) {
    return false;
  }
  return true;
}

}  // namespace lost_signal::robot::iface
