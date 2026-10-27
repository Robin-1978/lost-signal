#include <cassert>
#include <cstdio>

#include "chassis_controller.hpp"

using namespace lost_signal::stm32_reference;

int main() {
 ChassisController controller;
 WireSetpoint setpoint;
 setpoint.kind = SetpointKind::kBodyTwist;
 setpoint.sequence = 1;
 setpoint.stamp_ns = 1'000'000;
 setpoint.expiry_ns = 100'000'000;
 setpoint.forward_mps = 0.5f;
 assert(controller.accept(setpoint, setpoint.stamp_ns) == Status::kApplied);
 WireSetpoint wrong_owner = setpoint;
 wrong_owner.sequence = 2;
 wrong_owner.kind = SetpointKind::kDuty;
 assert(controller.accept(wrong_owner, setpoint.stamp_ns) == Status::kRejected);
 EncoderFeedback feedback;
 feedback.valid = true;
 MotorOutput output;
 assert(controller.tick(2'000'000, feedback, output) == Status::kApplied);
 assert(output.drive_duty[0] > 0.0f && output.drive_duty[1] > 0.0f);
 assert(!output.watchdog_stop);
 assert(controller.tick(100'000'000, feedback, output) ==
 Status::kWatchdogStop);
 assert(output.watchdog_stop && output.service_brake == 1.0f);
 std::puts("stm32_chassis_controller ok");
 return 0;
}
