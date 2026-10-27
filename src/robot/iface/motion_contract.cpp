#include "robot/iface/motion.hpp"

#include <cmath>
#include <limits>

namespace lost_signal::robot::iface {
namespace {

bool zero_channels(const std::array<double, 4>& values) {
 for (double value : values) {
 if (value != 0.0) return false;
 }
 return true;
}

bool finite_channels(const std::array<double, 4>& values, std::uint8_t count,
 double lower, double upper) {
 if (count > values.size()) return false;
 for (std::uint8_t i = 0; i < count; ++i) {
 if (!std::isfinite(values[i]) || values[i] < lower || values[i] > upper) {
 return false;
 }
 }
 for (std::uint8_t i = count; i < values.size(); ++i) {
 if (values[i] != 0.0) return false;
 }
 return true;
}

} // namespace

bool valid_motion_capabilities(const MotionCapabilities& capabilities) {
 if (capabilities.profile_id.empty() || capabilities.profile_revision == 0 ||
 capabilities.control_period_ns <= 0 ||
 capabilities.feedback_period_ns <= 0 ||
 capabilities.maximum_command_age_ns <= 0) {
 return false;
 }
 if (capabilities.drive_count != actuator_drive_count(capabilities.chassis_mode) ||
 capabilities.steer_count != actuator_steer_count(capabilities.chassis_mode)) {
 return false;
 }
 switch (capabilities.control_owner) {
 case MotionControlOwner::kHostMotionLoop:
 case MotionControlOwner::kMcuJointLoop:
 case MotionControlOwner::kMcuBodyLoop:
 return true;
 }
 return false;
}

bool valid_motion_setpoint(const MotionSetpoint& setpoint,
 const MotionCapabilities& capabilities, core::MonotonicNs now_ns) {
 if (!valid_motion_capabilities(capabilities) || setpoint.sequence == 0 ||
 setpoint.stamp_ns < 0 || setpoint.expiry_ns <= setpoint.stamp_ns ||
 (now_ns != core::kInvalidTime && now_ns >= setpoint.expiry_ns) ||
 setpoint.expiry_ns - setpoint.stamp_ns > capabilities.maximum_command_age_ns ||
 setpoint.chassis_mode != capabilities.chassis_mode ||
 setpoint.frame != core::Frame::kBaseLink ||
 !std::isfinite(setpoint.service_brake) ||
 setpoint.service_brake < 0.0 || setpoint.service_brake > 1.0) {
 return false;
 }
 const bool zero_body = setpoint.forward_mps == 0.0 &&
 setpoint.lateral_mps == 0.0 && setpoint.yaw_rate_radps == 0.0;
 const bool zero_duty = zero_channels(setpoint.drive_duty) &&
 zero_channels(setpoint.steer_duty);
 const bool zero_joints = zero_channels(setpoint.drive_radps) &&
 zero_channels(setpoint.steer_rad);
 switch (setpoint.kind) {
 case MotionSetpointKind::kStop:
 return zero_body && zero_duty && zero_joints;
 case MotionSetpointKind::kActuatorDuty:
 return capabilities.control_owner == MotionControlOwner::kHostMotionLoop &&
 finite_channels(setpoint.drive_duty, capabilities.drive_count, -1.0, 1.0) &&
 finite_channels(setpoint.steer_duty, capabilities.steer_count, -1.0, 1.0) &&
 zero_body && zero_joints;
 case MotionSetpointKind::kJointVelocity:
 return capabilities.control_owner == MotionControlOwner::kMcuJointLoop &&
 finite_channels(setpoint.drive_radps, capabilities.drive_count,
 -std::numeric_limits<double>::infinity(),
 std::numeric_limits<double>::infinity()) &&
 finite_channels(setpoint.steer_rad, capabilities.steer_count,
 -std::numeric_limits<double>::infinity(),
 std::numeric_limits<double>::infinity()) &&
 zero_body && zero_duty;
 case MotionSetpointKind::kBodyTwist:
 return capabilities.control_owner == MotionControlOwner::kMcuBodyLoop &&
 std::isfinite(setpoint.forward_mps) &&
 std::isfinite(setpoint.lateral_mps) &&
 std::isfinite(setpoint.yaw_rate_radps) && zero_duty && zero_joints;
 }
 return false;
}

} // namespace lost_signal::robot::iface
