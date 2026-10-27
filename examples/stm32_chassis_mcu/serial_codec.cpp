#include "serial_codec.hpp"

#include <algorithm>
#include <cstring>

namespace lost_signal::stm32_reference {
namespace {

constexpr std::uint8_t kMagic[3] = {0x16, 0x43, 0x48};
constexpr std::size_t kHeaderBytes = 3 + 2 + 1 + 2;
constexpr std::size_t kCrcBytes = 4;

void put_u16(std::uint8_t* p, std::uint16_t value) {
 p[0] = static_cast<std::uint8_t>(value);
 p[1] = static_cast<std::uint8_t>(value >> 8);
}

void put_u32(std::uint8_t* p, std::uint32_t value) {
 for (int i = 0; i < 4; ++i) p[i] = static_cast<std::uint8_t>(value >> (i * 8));
}

void put_u64(std::uint8_t* p, std::uint64_t value) {
 for (int i = 0; i < 8; ++i) p[i] = static_cast<std::uint8_t>(value >> (i * 8));
}

void put_i64(std::uint8_t* p, std::int64_t value) {
 put_u64(p, static_cast<std::uint64_t>(value));
}

void put_f32(std::uint8_t* p, float value) {
 std::uint32_t bits = 0;
 static_assert(sizeof(bits) == sizeof(value), "float must be IEEE-754 binary32");
 std::memcpy(&bits, &value, sizeof(bits));
 put_u32(p, bits);
}

std::uint16_t get_u16(const std::uint8_t* p) {
 return static_cast<std::uint16_t>(p[0]) |
 static_cast<std::uint16_t>(p[1]) << 8;
}

std::uint32_t get_u32(const std::uint8_t* p) {
 return static_cast<std::uint32_t>(p[0]) |
 static_cast<std::uint32_t>(p[1]) << 8 |
 static_cast<std::uint32_t>(p[2]) << 16 |
 static_cast<std::uint32_t>(p[3]) << 24;
}

std::uint64_t get_u64(const std::uint8_t* p) {
 std::uint64_t out = 0;
 for (int i = 7; i >= 0; --i) out = (out << 8) | p[i];
 return out;
}

std::int64_t get_i64(const std::uint8_t* p) {
 return static_cast<std::int64_t>(get_u64(p));
}

float get_f32(const std::uint8_t* p) {
 const std::uint32_t bits = get_u32(p);
 float value = 0.0f;
 std::memcpy(&value, &bits, sizeof(value));
 return value;
}

std::uint32_t crc32(const std::uint8_t* data, std::size_t count) {
 std::uint32_t crc = 0xffffffffu;
 for (std::size_t i = 0; i < count; ++i) {
 crc ^= data[i];
 for (int bit = 0; bit < 8; ++bit) {
 crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
 }
 }
 return ~crc;
}

std::size_t payload_size(SerialMessageKind kind) {
 // The fixed sizes are part of the protocol revision, not C++ sizeof().
 return kind == SerialMessageKind::kSetpoint ? 105 : 37;
}

void put_header(std::uint8_t* destination, SerialMessageKind kind,
 std::size_t payload) {
 destination[0] = kMagic[0];
 destination[1] = kMagic[1];
 destination[2] = kMagic[2];
 put_u16(destination + 3, kSerialWireVersion);
 destination[5] = static_cast<std::uint8_t>(kind);
 put_u16(destination + 6, static_cast<std::uint16_t>(payload));
}

} // namespace

std::size_t encode_setpoint(const WireSetpoint& setpoint,
 std::uint8_t* destination, std::size_t capacity) {
 const std::size_t total = kHeaderBytes + payload_size(SerialMessageKind::kSetpoint) +
 kCrcBytes;
 if (destination == nullptr || capacity < total) return 0;
 put_header(destination, SerialMessageKind::kSetpoint, payload_size(
 SerialMessageKind::kSetpoint));
 std::uint8_t* p = destination + kHeaderBytes;
 p[0] = static_cast<std::uint8_t>(setpoint.kind);
 put_u64(p + 1, setpoint.sequence);
 put_i64(p + 9, setpoint.stamp_ns);
 put_i64(p + 17, setpoint.expiry_ns);
 std::size_t offset = 25;
 for (float value : setpoint.drive_duty) { put_f32(p + offset, value); offset += 4; }
 for (float value : setpoint.steer_duty) { put_f32(p + offset, value); offset += 4; }
 for (float value : setpoint.drive_radps) { put_f32(p + offset, value); offset += 4; }
 for (float value : setpoint.steer_rad) { put_f32(p + offset, value); offset += 4; }
 put_f32(p + offset, setpoint.forward_mps); offset += 4;
 put_f32(p + offset, setpoint.lateral_mps); offset += 4;
 put_f32(p + offset, setpoint.yaw_rate_radps); offset += 4;
 put_f32(p + offset, setpoint.service_brake);
 put_u32(destination + total - kCrcBytes,
 crc32(destination + 3, total - kCrcBytes - 3));
 return total;
}

std::size_t encode_motor_output(const MotorOutput& output,
 std::uint8_t* destination, std::size_t capacity) {
 const std::size_t total = kHeaderBytes + payload_size(
 SerialMessageKind::kMotorOutput) + kCrcBytes;
 if (destination == nullptr || capacity < total) return 0;
 put_header(destination, SerialMessageKind::kMotorOutput,
 payload_size(SerialMessageKind::kMotorOutput));
 std::uint8_t* p = destination + kHeaderBytes;
 std::size_t offset = 0;
 for (float value : output.drive_duty) { put_f32(p + offset, value); offset += 4; }
 for (float value : output.steer_duty) { put_f32(p + offset, value); offset += 4; }
 put_f32(p + offset, output.service_brake); offset += 4;
 p[offset] = output.watchdog_stop ? 1 : 0;
 put_u32(destination + total - kCrcBytes,
 crc32(destination + 3, total - kCrcBytes - 3));
 return total;
}

bool SerialFrameParser::feed(const std::uint8_t* bytes, std::size_t count) {
 if (bytes == nullptr || count == 0) return true;
 if (count > buffer_.size() - size_) return false;
 std::copy(bytes, bytes + count, buffer_.begin() + size_);
 size_ += count;
 return true;
}

void SerialFrameParser::reset() { size_ = 0; }

SerialStatus SerialFrameParser::next(SerialMessageKind& kind,
 WireSetpoint& setpoint, MotorOutput& output) {
 std::size_t magic = size_;
 for (std::size_t i = 0; i + 3 <= size_; ++i) {
 if (buffer_[i] == kMagic[0] && buffer_[i + 1] == kMagic[1] &&
 buffer_[i + 2] == kMagic[2]) { magic = i; break; }
 }
 if (magic == size_) {
 if (size_ > 2) {
 buffer_[0] = buffer_[size_ - 2];
 buffer_[1] = buffer_[size_ - 1];
 size_ = 2;
 }
 return SerialStatus::kNoFrame;
 }
 if (magic > 0) {
 std::move(buffer_.begin() + magic, buffer_.begin() + size_, buffer_.begin());
 size_ -= magic;
 }
 if (size_ < kHeaderBytes) return SerialStatus::kNoFrame;
 const std::uint16_t version = get_u16(buffer_.data() + 3);
 if (version != kSerialWireVersion) {
 std::move(buffer_.begin() + 3, buffer_.begin() + size_, buffer_.begin());
 size_ -= 3;
 return SerialStatus::kUnsupportedVersion;
 }
 const auto message = static_cast<SerialMessageKind>(buffer_[5]);
 const std::uint16_t payload = get_u16(buffer_.data() + 6);
 if ((message != SerialMessageKind::kSetpoint &&
 message != SerialMessageKind::kMotorOutput) ||
 payload != payload_size(message)) {
 std::move(buffer_.begin() + 3, buffer_.begin() + size_, buffer_.begin());
 size_ -= 3;
 return SerialStatus::kBadPayload;
 }
 const std::size_t total = kHeaderBytes + payload + kCrcBytes;
 if (size_ < total) return SerialStatus::kNoFrame;
 const std::uint32_t expected = get_u32(buffer_.data() + total - kCrcBytes);
 const std::uint32_t actual = crc32(buffer_.data() + 3, total - kCrcBytes - 3);
 if (expected != actual) {
 std::move(buffer_.begin() + 3, buffer_.begin() + size_, buffer_.begin());
 size_ -= 3;
 return SerialStatus::kBadFrame;
 }
 const std::uint8_t* p = buffer_.data() + kHeaderBytes;
 if (message == SerialMessageKind::kSetpoint) {
 setpoint = {};
 setpoint.kind = static_cast<SetpointKind>(p[0]);
 setpoint.sequence = get_u64(p + 1);
 setpoint.stamp_ns = get_i64(p + 9);
 setpoint.expiry_ns = get_i64(p + 17);
 std::size_t offset = 25;
 for (float& value : setpoint.drive_duty) { value = get_f32(p + offset); offset += 4; }
 for (float& value : setpoint.steer_duty) { value = get_f32(p + offset); offset += 4; }
 for (float& value : setpoint.drive_radps) { value = get_f32(p + offset); offset += 4; }
 for (float& value : setpoint.steer_rad) { value = get_f32(p + offset); offset += 4; }
 setpoint.forward_mps = get_f32(p + offset); offset += 4;
 setpoint.lateral_mps = get_f32(p + offset); offset += 4;
 setpoint.yaw_rate_radps = get_f32(p + offset); offset += 4;
 setpoint.service_brake = get_f32(p + offset);
 } else {
 output = {};
 std::size_t offset = 0;
 for (float& value : output.drive_duty) { value = get_f32(p + offset); offset += 4; }
 for (float& value : output.steer_duty) { value = get_f32(p + offset); offset += 4; }
 output.service_brake = get_f32(p + offset); offset += 4;
 output.watchdog_stop = p[offset] != 0;
 }
 kind = message;
 std::move(buffer_.begin() + total, buffer_.begin() + size_, buffer_.begin());
 size_ -= total;
 return SerialStatus::kAccepted;
}

} // namespace lost_signal::stm32_reference
