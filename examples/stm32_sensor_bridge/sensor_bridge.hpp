#pragma once

// HAL-free STM32 sensor wire/validation reference.  The board side owns
// acquisition and timestamping; this layer only validates a packet and
// exposes fixed-width values to a host adapter.  No simulator or ROBOT state
// is visible here.

#include <cstddef>
#include <cstdint>

namespace lost_signal::stm32_reference {

inline constexpr std::uint16_t kWireVersion = 1;
inline constexpr std::size_t kMaxLidarBeams = 720;

enum class SensorKind : std::uint8_t {
 kLidar2d = 1,
 kImu = 2,
 kJoints = 3,
};

enum class DecodeStatus : std::uint8_t {
 kAccepted = 0,
 kInvalidHeader = 1,
 kWrongKind = 2,
 kSequence = 3,
 kInvalidPayload = 4,
};

// This header is serialized field-by-field by CAN/UART/Ethernet code.  It is
// deliberately not packed and must never be sent with sizeof(Header).
struct PacketHeader {
 std::uint16_t version = kWireVersion;
 SensorKind kind = SensorKind::kLidar2d;
 std::uint8_t reserved = 0;
 std::uint32_t boot_id = 0;
 std::uint64_t sequence = 0;
 std::int64_t stamp_ns = -1; // acquisition time in the robot monotonic domain
};

struct Lidar2dPacket {
 PacketHeader header{};
 std::uint16_t beam_count = 0;
 std::int32_t angle_min_urad = 0;
 std::int32_t angle_increment_urad = 0;
 // Millimetres are used on the wire so a board does not need a float parser.
 std::uint16_t range_mm[kMaxLidarBeams] = {};
};

struct ImuPacket {
 PacketHeader header{};
 float gyro_x_radps = 0.0f;
 float gyro_y_radps = 0.0f;
 float gyro_z_radps = 0.0f;
 float accel_x_mps2 = 0.0f;
 float accel_y_mps2 = 0.0f;
 float accel_z_mps2 = 0.0f;
};

struct JointPacket {
 PacketHeader header{};
 std::uint8_t drive_count = 0;
 std::uint8_t steer_count = 0;
 std::uint8_t reserved[2] = {};
 float drive_radps[4] = {};
 float steer_rad[4] = {};
 float steer_vel_radps[4] = {};
};

struct LidarMeasurement {
 PacketHeader header{};
 std::uint16_t beam_count = 0;
 float angle_min_rad = 0.0f;
 float angle_increment_rad = 0.0f;
 float ranges_m[kMaxLidarBeams] = {};
};

struct ImuMeasurement {
 PacketHeader header{};
 float gyro_x_radps = 0.0f;
 float gyro_y_radps = 0.0f;
 float gyro_z_radps = 0.0f;
 float accel_x_mps2 = 0.0f;
 float accel_y_mps2 = 0.0f;
 float accel_z_mps2 = 0.0f;
};

struct JointMeasurement {
 PacketHeader header{};
 std::uint8_t drive_count = 0;
 std::uint8_t steer_count = 0;
 float drive_radps[4] = {};
 float steer_rad[4] = {};
 float steer_vel_radps[4] = {};
};

struct SensorBridgeConfig {
 std::uint8_t drive_count = 2;
 std::uint8_t steer_count = 0;
 std::uint16_t max_lidar_beams = 360;
 std::uint16_t max_range_mm = 10'000;
};

class SensorBridge {
 public:
 explicit SensorBridge(SensorBridgeConfig config = {});

 DecodeStatus decode(const Lidar2dPacket& packet, LidarMeasurement& out);
 DecodeStatus decode(const ImuPacket& packet, ImuMeasurement& out);
 DecodeStatus decode(const JointPacket& packet, JointMeasurement& out);

 // A board reboot starts a new sequence domain.  The host adapter calls this
 // only after it has observed a new boot_id in the transport handshake.
 void reset();

 private:
 DecodeStatus validate_header(const PacketHeader& header,
 SensorKind expected) const;
 bool accept_sequence(const PacketHeader& header);

 SensorBridgeConfig config_;
 std::uint32_t boot_id_ = 0;
 std::uint64_t lidar_sequence_ = 0;
 std::uint64_t imu_sequence_ = 0;
 std::uint64_t joint_sequence_ = 0;
};

} // namespace lost_signal::stm32_reference
