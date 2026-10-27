#include <cassert>
#include <cstdio>

#include "chassis_controller.hpp"
#include "serial_codec.hpp"

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

 // UART framing is independent of C++ layout and survives split reads.
 WireSetpoint wire;
 wire.kind = SetpointKind::kBodyTwist;
 wire.sequence = 9;
 wire.stamp_ns = 2'000'000;
 wire.expiry_ns = 100'000'000;
 wire.forward_mps = 0.7f;
 wire.yaw_rate_radps = -0.2f;
 std::uint8_t encoded[kSerialMaxFrame]{};
 const std::size_t encoded_size = encode_setpoint(
 wire, encoded, sizeof(encoded));
 assert(encoded_size > 0);
 SerialFrameParser parser;
 SerialMessageKind kind = SerialMessageKind::kMotorOutput;
 WireSetpoint decoded;
 MotorOutput decoded_output;
 assert(parser.feed(encoded, 7));
 assert(parser.next(kind, decoded, decoded_output) == SerialStatus::kNoFrame);
 assert(parser.feed(encoded + 7, encoded_size - 7));
 assert(parser.next(kind, decoded, decoded_output) == SerialStatus::kAccepted);
 assert(kind == SerialMessageKind::kSetpoint);
 assert(decoded.sequence == wire.sequence);
 assert(decoded.forward_mps == wire.forward_mps);

 // A CRC failure is rejected without applying a partially decoded command.
 encoded[encoded_size - 1] ^= 0x80;
 parser.reset();
 assert(parser.feed(encoded, encoded_size));
 assert(parser.next(kind, decoded, decoded_output) == SerialStatus::kBadFrame);

 MotorOutput motor;
 motor.drive_duty[0] = 0.25f;
 motor.service_brake = 0.0f;
 const std::size_t motor_size = encode_motor_output(
 motor, encoded, sizeof(encoded));
 parser.reset();
 assert(parser.feed(encoded, motor_size));
 assert(parser.next(kind, decoded, decoded_output) == SerialStatus::kAccepted);
 assert(kind == SerialMessageKind::kMotorOutput);
 assert(decoded_output.drive_duty[0] == 0.25f);
 std::puts("stm32_chassis_controller ok");
 return 0;
}
