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
 if (frame.odometry_valid &&
 (!std::isfinite(frame.forward_mps) ||
 !std::isfinite(frame.lateral_mps) ||
 !std::isfinite(frame.yaw_rate_radps))) {
 return false;
 }
 const bool explicit_sections = !frame.scan_header.empty() ||
 !frame.motion_header.empty() || !frame.imu_header.empty();
 if (frame.scan_valid &&
 !(explicit_sections ? frame.scan_header.valid()
 : scan_measurement_header(frame).valid())) {
 return false;
 }
 if ((frame.odometry_valid || frame.has_joints()) &&
 !(explicit_sections ? frame.motion_header.valid()
 : motion_measurement_header(frame).valid())) {
 return false;
 }
 if (frame.imu_valid &&
 !(explicit_sections ? frame.imu_header.valid()
 : imu_measurement_header(frame).valid())) {
 return false;
 }
  return true;
}

bool valid_motion_command(const MotionCommand& cmd, core::MonotonicNs now_ns) {
  if (cmd.stamp_ns == core::kInvalidTime) return false;
  if (cmd.expiry_ns <= cmd.stamp_ns) return false;
  if (now_ns != core::kInvalidTime && now_ns > cmd.expiry_ns) return false;
  if (!std::isfinite(cmd.forward_mps) || !std::isfinite(cmd.yaw_rate_radps) ||
      !std::isfinite(cmd.steer_angle_rad) || !std::isfinite(cmd.lateral_mps)) {
    return false;
  }
  switch (cmd.space) {
    case CommandSpace::kStop:
      return cmd.is_safe_stop();
    case CommandSpace::kYawRate:
      return cmd.steer_angle_rad == 0.0 && cmd.lateral_mps == 0.0;
    case CommandSpace::kSteerAngle:
      return cmd.yaw_rate_radps == 0.0 && cmd.lateral_mps == 0.0;
    case CommandSpace::kHolonomic:
      return cmd.steer_angle_rad == 0.0;
  }
  return false;
}

bool valid_actuator_command(const ActuatorCommand& cmd,
                            core::MonotonicNs now_ns) {
  if (cmd.stamp_ns == core::kInvalidTime) return false;
  if (cmd.expiry_ns <= cmd.stamp_ns) return false;
  if (now_ns != core::kInvalidTime && now_ns > cmd.expiry_ns) return false;
 if (!std::isfinite(cmd.service_brake) || cmd.service_brake < 0.0 ||
 cmd.service_brake > 1.0) {
 return false;
 }
  const std::uint8_t n_drive = actuator_drive_count(cmd.mode);
  const std::uint8_t n_steer = actuator_steer_count(cmd.mode);
  if (n_drive == 0) return false;
  for (std::uint8_t i = 0; i < n_drive; ++i) {
    if (!std::isfinite(cmd.drive[i])) return false;
    if (cmd.drive[i] < -1.0 || cmd.drive[i] > 1.0) return false;
  }
  for (std::uint8_t i = n_drive; i < 4; ++i) {
    if (cmd.drive[i] != 0.0) return false;  // unused channels stay zero
  }
  for (std::uint8_t i = 0; i < n_steer; ++i) {
    if (!std::isfinite(cmd.steer[i])) return false;
    if (cmd.steer[i] < -1.0 || cmd.steer[i] > 1.0) return false;
  }
  for (std::uint8_t i = n_steer; i < 4; ++i) {
    if (cmd.steer[i] != 0.0) return false;
  }
  return true;
}

}  // namespace lost_signal::robot::iface
