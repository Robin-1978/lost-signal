#include "robot/transport/serial_sensor_protocol.hpp"

#include <cmath>
#include <cstring>

namespace lost_signal::robot::transport::serial_sensor {
namespace {

constexpr std::uint8_t kMagic[3] = {0x16, 0x4e, 0x54};
constexpr std::size_t kHeaderBytes = 3 + 2 + 1 + 1 + 4 + 8 + 8 + 2;
constexpr std::size_t kCrcBytes = 4;

bool finite(float value) { return std::isfinite(value); }

std::uint16_t read_u16(const std::uint8_t* p) {
 return static_cast<std::uint16_t>(p[0]) |
 static_cast<std::uint16_t>(p[1]) << 8;
}

std::uint32_t read_u32(const std::uint8_t* p) {
 return static_cast<std::uint32_t>(p[0]) |
 static_cast<std::uint32_t>(p[1]) << 8 |
 static_cast<std::uint32_t>(p[2]) << 16 |
 static_cast<std::uint32_t>(p[3]) << 24;
}

std::uint64_t read_u64(const std::uint8_t* p) {
 std::uint64_t value = 0;
 for (int i = 7; i >= 0; --i) value = (value << 8) | p[i];
 return value;
}

std::int64_t read_i64(const std::uint8_t* p) {
 return static_cast<std::int64_t>(read_u64(p));
}

float read_f32(const std::uint8_t* p) {
 const std::uint32_t bits = read_u32(p);
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

bool valid_kind(std::uint8_t kind) {
 return kind == static_cast<std::uint8_t>(SensorKind::kLidar2d) ||
 kind == static_cast<std::uint8_t>(SensorKind::kImu) ||
 kind == static_cast<std::uint8_t>(SensorKind::kJoints);
}

} // namespace

SerialFrameDecoder::SerialFrameDecoder(std::size_t capacity)
 : rx_(capacity < 2 ? 2 : capacity) {}

bool SerialFrameDecoder::feed(const std::uint8_t* bytes, std::size_t count) {
 if (bytes == nullptr || count == 0) return true;
 return rx_.push(bytes, count) == count;
}

void SerialFrameDecoder::reset() {
 rx_.clear();
 for (std::size_t i = 0; i < 3; ++i) {
 boot_ids_[i] = 0;
 last_sequences_[i] = 0;
 }
}

DecodeStatus SerialFrameDecoder::next(SerialPacket& out) {
 const std::size_t available = rx_.size();
 if (available < 3) return DecodeStatus::kNoFrame;
 std::size_t magic_at = available;
 for (std::size_t i = 0; i + 3 <= available; ++i) {
 std::uint8_t value[3]{};
 if (rx_.peek(value, 3, i) != 3) return DecodeStatus::kNoFrame;
 if (value[0] == kMagic[0] && value[1] == kMagic[1] &&
 value[2] == kMagic[2]) {
 magic_at = i;
 break;
 }
 }
 if (magic_at == available) {
 rx_.discard(available > 2 ? available - 2 : 0);
 return DecodeStatus::kNoFrame;
 }
 if (available < magic_at + kHeaderBytes) return DecodeStatus::kNoFrame;
 if (rx_.peek(frame_.data(), kHeaderBytes, magic_at) != kHeaderBytes) {
 return DecodeStatus::kNoFrame;
 }
 const std::uint16_t version = read_u16(frame_.data() + 3);
 const std::uint16_t payload_len = read_u16(frame_.data() + 27);
 if (version != kWireVersion) {
 rx_.discard(magic_at + 3);
 return DecodeStatus::kUnsupportedVersion;
 }
 if (payload_len > kMaxPayloadBytes) {
 rx_.discard(magic_at + 3);
 return DecodeStatus::kBadPayload;
 }
 const std::size_t total = kHeaderBytes + payload_len + kCrcBytes;
 if (total > frame_.size() || available < magic_at + total) {
 return DecodeStatus::kNoFrame;
 }
 if (rx_.peek(frame_.data(), total, magic_at) != total) {
 return DecodeStatus::kNoFrame;
 }
 const std::uint32_t expected_crc = read_u32(frame_.data() + total - kCrcBytes);
 const std::uint32_t actual_crc = crc32(
 frame_.data() + 3, kHeaderBytes - 3 + payload_len);
 if (expected_crc != actual_crc) {
 rx_.discard(magic_at + 3);
 return DecodeStatus::kBadFrame;
 }
 out.kind = frame_[5];
 out.boot_id = read_u32(frame_.data() + 7);
 out.sequence = read_u64(frame_.data() + 11);
 out.stamp_ns = read_i64(frame_.data() + 19);
 if (!valid_kind(out.kind) || out.boot_id == 0 || out.sequence == 0 ||
 out.stamp_ns < 0) {
 rx_.discard(magic_at + 3);
 return DecodeStatus::kBadFrame;
 }
 const std::size_t kind_index = static_cast<std::size_t>(out.kind - 1);
 if (boot_ids_[kind_index] != 0 && boot_ids_[kind_index] != out.boot_id) {
 rx_.discard(magic_at + total);
 return DecodeStatus::kSequence;
 }
 if (boot_ids_[kind_index] == 0) boot_ids_[kind_index] = out.boot_id;
 if (out.sequence <= last_sequences_[kind_index]) {
 rx_.discard(magic_at + total);
 return DecodeStatus::kSequence;
 }
 last_sequences_[kind_index] = out.sequence;
 out.payload.assign(frame_.data() + kHeaderBytes,
 frame_.data() + kHeaderBytes + payload_len);
 rx_.discard(magic_at + total);
 return DecodeStatus::kAccepted;
}

DecodeStatus decode_lidar(const SerialPacket& packet,
 const SerialSensorConfig& config,
 LidarMeasurement& out) {
 if (packet.kind != static_cast<std::uint8_t>(SensorKind::kLidar2d)) {
 return DecodeStatus::kBadPayload;
 }
 if (packet.payload.size() < 10) return DecodeStatus::kBadPayload;
 const std::uint16_t count = read_u16(packet.payload.data());
 if (count == 0 || count > config.max_lidar_beams || count > kMaxLidarBeams ||
 packet.payload.size() != 10 + static_cast<std::size_t>(count) * 2) {
 return DecodeStatus::kBadPayload;
 }
 out = {};
 out.sequence = packet.sequence;
 out.stamp_ns = packet.stamp_ns;
 out.beam_count = count;
 out.angle_min_rad = static_cast<float>(static_cast<std::int32_t>(
 read_u32(packet.payload.data() + 2))) * 1e-6f;
 out.angle_increment_rad = static_cast<float>(static_cast<std::int32_t>(
 read_u32(packet.payload.data() + 6))) * 1e-6f;
 if (!(out.angle_increment_rad != 0.0f) ||
 !(std::abs(out.angle_increment_rad) <= 6.3f)) {
 return DecodeStatus::kBadPayload;
 }
 for (std::uint16_t i = 0; i < count; ++i) {
 const float range = static_cast<float>(read_u16(
 packet.payload.data() + 10 + static_cast<std::size_t>(i) * 2)) *
 1e-3f;
 if (!finite(range) || range <= 0.0f ||
 range > static_cast<float>(config.max_range_m)) {
 return DecodeStatus::kBadPayload;
 }
 out.ranges_m[i] = range;
 }
 return DecodeStatus::kAccepted;
}

DecodeStatus decode_imu(const SerialPacket& packet, ImuMeasurement& out) {
 if (packet.kind != static_cast<std::uint8_t>(SensorKind::kImu) ||
 packet.payload.size() != 24) {
 return DecodeStatus::kBadPayload;
 }
 ImuMeasurement candidate;
 candidate.sequence = packet.sequence;
 candidate.stamp_ns = packet.stamp_ns;
 candidate.gyro_x_radps = read_f32(packet.payload.data());
 candidate.gyro_y_radps = read_f32(packet.payload.data() + 4);
 candidate.gyro_z_radps = read_f32(packet.payload.data() + 8);
 candidate.accel_x_mps2 = read_f32(packet.payload.data() + 12);
 candidate.accel_y_mps2 = read_f32(packet.payload.data() + 16);
 candidate.accel_z_mps2 = read_f32(packet.payload.data() + 20);
 if (!finite(candidate.gyro_x_radps) || !finite(candidate.gyro_y_radps) ||
 !finite(candidate.gyro_z_radps) || !finite(candidate.accel_x_mps2) ||
 !finite(candidate.accel_y_mps2) || !finite(candidate.accel_z_mps2)) {
 return DecodeStatus::kBadPayload;
 }
 out = candidate;
 return DecodeStatus::kAccepted;
}

DecodeStatus decode_joints(const SerialPacket& packet,
 const SerialSensorConfig& config,
 JointsMeasurement& out) {
 if (packet.kind != static_cast<std::uint8_t>(SensorKind::kJoints) ||
 packet.payload.size() < 4 ||
 packet.payload[0] != config.drive_count ||
 packet.payload[1] != config.steer_count) {
 return DecodeStatus::kBadPayload;
 }
 const std::size_t expected = 4 + static_cast<std::size_t>(config.drive_count) * 4 +
 static_cast<std::size_t>(config.steer_count) * 8;
 if (packet.payload.size() != expected) return DecodeStatus::kBadPayload;
 JointsMeasurement candidate;
 candidate.sequence = packet.sequence;
 candidate.stamp_ns = packet.stamp_ns;
 candidate.drive_count = config.drive_count;
 candidate.steer_count = config.steer_count;
 std::size_t offset = 4;
 for (std::uint8_t i = 0; i < config.drive_count; ++i, offset += 4) {
 candidate.drive_radps[i] = read_f32(packet.payload.data() + offset);
 if (!finite(candidate.drive_radps[i])) return DecodeStatus::kBadPayload;
 }
 for (std::uint8_t i = 0; i < config.steer_count; ++i, offset += 8) {
 candidate.steer_rad[i] = read_f32(packet.payload.data() + offset);
 candidate.steer_vel_radps[i] = read_f32(packet.payload.data() + offset + 4);
 if (!finite(candidate.steer_rad[i]) ||
 !finite(candidate.steer_vel_radps[i])) return DecodeStatus::kBadPayload;
 }
 out = candidate;
 return DecodeStatus::kAccepted;
}

} // namespace lost_signal::robot::transport::serial_sensor
