#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "chassis_controller.hpp"

namespace lost_signal::stm32_reference {

inline constexpr std::uint16_t kSerialWireVersion = 1;
inline constexpr std::size_t kSerialMaxPayload = 160;
inline constexpr std::size_t kSerialMaxFrame = 3 + 2 + 1 + 2 +
 kSerialMaxPayload + 4;

enum class SerialMessageKind : std::uint8_t {
 kSetpoint = 1,
 kMotorOutput = 2,
};

enum class SerialStatus : std::uint8_t {
 kNoFrame = 0,
 kAccepted = 1,
 kOverflow = 2,
 kBadFrame = 3,
 kUnsupportedVersion = 4,
 kBadPayload = 5,
};

// Framing is deliberately independent from C++ object layout:
// magic[16 43 48], version:u16, kind:u8, payload_len:u16, payload, crc32:u32.
// CRC covers version through payload. The board UART/DMA layer feeds bytes to
// SerialFrameParser; it never casts a receive buffer to WireSetpoint.
std::size_t encode_setpoint(const WireSetpoint& setpoint,
 std::uint8_t* destination, std::size_t capacity);
std::size_t encode_motor_output(const MotorOutput& output,
 std::uint8_t* destination, std::size_t capacity);

class SerialFrameParser {
 public:
 SerialFrameParser() = default;

 bool feed(const std::uint8_t* bytes, std::size_t count);
 SerialStatus next(SerialMessageKind& kind, WireSetpoint& setpoint,
 MotorOutput& output);
 void reset();
 std::size_t buffered_bytes() const { return size_; }

 private:
 std::array<std::uint8_t, kSerialMaxFrame * 2> buffer_{};
 std::size_t size_ = 0;
};

} // namespace lost_signal::stm32_reference
