#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "robot/iface/ring_buffer.hpp"

namespace lost_signal::examples::serial_sensor {

inline constexpr std::uint16_t kWireVersion = 1;
inline constexpr std::size_t kMaxPayloadBytes = 4096;
inline constexpr std::size_t kMaxLidarBeams = 720;
inline constexpr std::size_t kMaxFrameBytes = kMaxPayloadBytes + 64;

enum class SensorKind : std::uint8_t { kLidar2d = 1, kImu = 2, kJoints = 3 };

enum class DecodeStatus : std::uint8_t {
 kNoFrame = 0,
 kAccepted = 1,
 kBadFrame = 2,
 kUnsupportedVersion = 3,
 kBadPayload = 4,
 kSequence = 5,
 kBufferOverflow = 6,
};

struct SerialPacket {
 std::uint8_t kind = 0;
 std::uint32_t boot_id = 0;
 std::uint64_t sequence = 0;
 std::int64_t stamp_ns = -1;
 std::vector<std::uint8_t> payload;
};

struct LidarMeasurement {
 std::uint64_t sequence = 0;
 std::int64_t stamp_ns = -1;
 float angle_min_rad = 0.0f;
 float angle_increment_rad = 0.0f;
 std::array<float, kMaxLidarBeams> ranges_m{};
 std::uint16_t beam_count = 0;
};

struct ImuMeasurement {
 std::uint64_t sequence = 0;
 std::int64_t stamp_ns = -1;
 float gyro_x_radps = 0.0f;
 float gyro_y_radps = 0.0f;
 float gyro_z_radps = 0.0f;
 float accel_x_mps2 = 0.0f;
 float accel_y_mps2 = 0.0f;
 float accel_z_mps2 = 0.0f;
};

struct JointsMeasurement {
 std::uint64_t sequence = 0;
 std::int64_t stamp_ns = -1;
 std::uint8_t drive_count = 0;
 std::uint8_t steer_count = 0;
 std::array<float, 4> drive_radps{};
 std::array<float, 4> steer_rad{};
 std::array<float, 4> steer_vel_radps{};
};

struct SerialSensorConfig {
 std::uint8_t drive_count = 2;
 std::uint8_t steer_count = 0;
 std::uint16_t max_lidar_beams = 720;
 double max_range_m = 30.0;
};

// Wire format (little endian):
// magic[16 4e 54] version:u16 kind:u8 flags:u8 boot_id:u32
// sequence:u64 stamp_ns:i64 payload_len:u16 payload[] crc32:u32.
// CRC covers every byte after magic through the payload. The parser is a
// stream decoder: Feed() may split, merge, or corrupt frames arbitrarily.
class SerialFrameDecoder {
 public:
 SerialFrameDecoder(std::size_t capacity = 8192);

 // Copies received bytes into the byte ring. false means the ring filled up;
 // the caller should reset framing and discard the oldest unsynchronised data.
 bool feed(const std::uint8_t* bytes, std::size_t count);
 DecodeStatus next(SerialPacket& out);
 void reset();

 std::size_t buffered_bytes() const { return rx_.size(); }

 private:
 lost_signal::robot::iface::RingBuffer rx_;
 std::array<std::uint8_t, kMaxFrameBytes> frame_{};
 std::uint32_t boot_ids_[3] = {};
 std::uint64_t last_sequences_[3] = {};
};

DecodeStatus decode_lidar(const SerialPacket& packet,
 const SerialSensorConfig& config,
 LidarMeasurement& out);
DecodeStatus decode_imu(const SerialPacket& packet, ImuMeasurement& out);
DecodeStatus decode_joints(const SerialPacket& packet,
 const SerialSensorConfig& config,
 JointsMeasurement& out);

} // namespace lost_signal::examples::serial_sensor
