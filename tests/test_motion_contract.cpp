#include <cassert>
#include <cstdio>

#include "robot/iface/motion.hpp"

using lost_signal::robot::core::Frame;
using lost_signal::robot::iface::ActuatorMode;
using lost_signal::robot::iface::MotionCapabilities;
using lost_signal::robot::iface::MotionControlOwner;
using lost_signal::robot::iface::MotionSetpoint;
using lost_signal::robot::iface::MotionSetpointKind;

namespace {

MotionCapabilities diff(MotionControlOwner owner) {
 MotionCapabilities out;
 out.control_owner = owner;
 out.chassis_mode = ActuatorMode::kDiffDuty;
 out.drive_count = 2;
 out.steer_count = 0;
 out.control_period_ns = 1'000'000;
 out.feedback_period_ns = 2'000'000;
 out.maximum_command_age_ns = 20'000'000;
 out.profile_id = "test.diff";
 out.profile_revision = 1;
 return out;
}

MotionSetpoint base(MotionSetpointKind kind) {
 MotionSetpoint out;
 out.kind = kind;
 out.chassis_mode = ActuatorMode::kDiffDuty;
 out.sequence = 1;
 out.stamp_ns = 100;
 out.expiry_ns = 10'100;
 out.frame = Frame::kBaseLink;
 return out;
}

} // namespace

int main() {
 const auto host = diff(MotionControlOwner::kHostMotionLoop);
 assert(valid_motion_capabilities(host));
 auto duty = base(MotionSetpointKind::kActuatorDuty);
 duty.drive_duty[0] = 0.25;
 duty.drive_duty[1] = -0.25;
 assert(valid_motion_setpoint(duty, host, 100));
 duty.drive_radps[0] = 1.0; // A duty setpoint cannot smuggle joint units.
 assert(!valid_motion_setpoint(duty, host, 100));

 const auto joint = diff(MotionControlOwner::kMcuJointLoop);
 auto velocity = base(MotionSetpointKind::kJointVelocity);
 velocity.drive_radps[0] = 3.0;
 velocity.drive_radps[1] = 3.1;
 assert(valid_motion_setpoint(velocity, joint, 100));
 assert(!valid_motion_setpoint(velocity, host, 100));

 const auto body = diff(MotionControlOwner::kMcuBodyLoop);
 auto twist = base(MotionSetpointKind::kBodyTwist);
 twist.forward_mps = 0.5;
 twist.yaw_rate_radps = 0.2;
 assert(valid_motion_setpoint(twist, body, 100));
 twist.frame = Frame::kMap;
 assert(!valid_motion_setpoint(twist, body, 100));

 auto stop = base(MotionSetpointKind::kStop);
 stop.service_brake = 1.0;
 assert(valid_motion_setpoint(stop, body, 100));
 stop.expiry_ns += 20'000'001;
 assert(!valid_motion_setpoint(stop, body, 100));

 auto malformed = host;
 malformed.drive_count = 1;
 assert(!valid_motion_capabilities(malformed));
 std::puts("motion_contract ok");
 return 0;
}
