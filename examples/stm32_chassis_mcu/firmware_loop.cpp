#include "firmware_loop.hpp"

namespace lost_signal::stm32_reference {

FirmwareLoop::FirmwareLoop(ChassisConfig config) : controller_(config) {}

bool FirmwareLoop::receive_uart(const std::uint8_t* bytes, std::size_t count) {
 return parser_.feed(bytes, count);
}

FirmwareTickStatus FirmwareLoop::tick(std::int64_t now_ns,
 const EncoderFeedback& feedback, std::uint8_t* tx_bytes,
 std::size_t tx_capacity, std::size_t& tx_size) {
 tx_size = 0;
 SerialMessageKind kind = SerialMessageKind::kMotorOutput;
 WireSetpoint setpoint;
 MotorOutput ignored_output;
 SerialStatus receive_status = SerialStatus::kNoFrame;
 while ((receive_status = parser_.next(kind, setpoint, ignored_output)) ==
 SerialStatus::kAccepted) {
 if (kind == SerialMessageKind::kSetpoint) controller_.accept(setpoint, now_ns);
 }
 if (receive_status == SerialStatus::kBadFrame ||
 receive_status == SerialStatus::kBadPayload ||
 receive_status == SerialStatus::kUnsupportedVersion) {
 return FirmwareTickStatus::kRejected;
 }
 MotorOutput output;
 const Status status = controller_.tick(now_ns, feedback, output);
 if (tx_bytes == nullptr || tx_capacity < kSerialMaxFrame) {
 return FirmwareTickStatus::kTxBufferTooSmall;
 }
 tx_size = encode_motor_output(output, tx_bytes, tx_capacity);
 if (tx_size == 0) return FirmwareTickStatus::kTxBufferTooSmall;
 if (status == Status::kWatchdogStop) return FirmwareTickStatus::kWatchdogStop;
 if (status == Status::kRejected) return FirmwareTickStatus::kRejected;
 if (status == Status::kIdle) return FirmwareTickStatus::kIdle;
 return FirmwareTickStatus::kApplied;
}

void FirmwareLoop::reset() {
 controller_.reset();
 parser_.reset();
}

} // namespace lost_signal::stm32_reference
