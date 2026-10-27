#pragma once

#include <cstddef>
#include <cstdint>

#include "chassis_controller.hpp"
#include "serial_codec.hpp"

namespace lost_signal::stm32_reference {

enum class FirmwareTickStatus : std::uint8_t {
 kApplied = 0,
 kIdle = 1,
 kWatchdogStop = 2,
 kRejected = 3,
 kTxBufferTooSmall = 4,
};

// HAL-free board seam. CubeMX code calls receive_uart() from its UART/DMA
// receive task and tick() from the hardware-timer ISR or a fixed-priority
// RTOS task. The class never reads wall clock or sleeps.
class FirmwareLoop {
 public:
 explicit FirmwareLoop(ChassisConfig config = {});
 bool receive_uart(const std::uint8_t* bytes, std::size_t count);
 FirmwareTickStatus tick(std::int64_t now_ns,
 const EncoderFeedback& feedback,
 std::uint8_t* tx_bytes, std::size_t tx_capacity,
 std::size_t& tx_size);
 void reset();

 private:
 ChassisController controller_;
 SerialFrameParser parser_;
};

} // namespace lost_signal::stm32_reference
