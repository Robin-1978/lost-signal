#include "sensor_bridge.hpp"

#include <cmath>

namespace lost_signal::stm32_reference {
namespace {

bool finite(float value) { return std::isfinite(value); }

bool valid_kind(SensorKind kind) {
 return kind == SensorKind::kLidar2d || kind == SensorKind::kImu ||
 kind == SensorKind::kJoints;
}

} // namespace

SensorBridge::SensorBridge(SensorBridgeConfig config) : config_(config) {
 if (config_.drive_count > 4) config_.drive_count = 0;
 if (config_.steer_count > 4) config_.steer_count = 0;
 if (config_.max_lidar_beams == 0 ||
 config_.max_lidar_beams > kMaxLidarBeams) {
 config_.max_lidar_beams = static_cast<std::uint16_t>(kMaxLidarBeams);
 }
}

void SensorBridge::reset() {
 boot_id_ = 0;
 lidar_sequence_ = 0;
 imu_sequence_ = 0;
 joint_sequence_ = 0;
}

DecodeStatus SensorBridge::validate_header(const PacketHeader& header,
 SensorKind expected) const {
 if (header.version != kWireVersion || !valid_kind(header.kind) ||
 header.kind != expected || header.boot_id == 0 || header.sequence == 0 ||
 header.stamp_ns < 0) {
 return DecodeStatus::kInvalidHeader;
 }
 return DecodeStatus::kAccepted;
}

bool SensorBridge::accept_sequence(const PacketHeader& header) {
 if (boot_id_ != 0 && header.boot_id != boot_id_) {
 // A new boot is not implicitly accepted: the host must call reset after
 // completing the transport handshake, so stale packets cannot slip through.
 return false;
 }
 if (boot_id_ == 0) boot_id_ = header.boot_id;
 std::uint64_t* last = nullptr;
 switch (header.kind) {
 case SensorKind::kLidar2d: last = &lidar_sequence_; break;
 case SensorKind::kImu: last = &imu_sequence_; break;
 case SensorKind::kJoints: last = &joint_sequence_; break;
 }
 if (last == nullptr || header.sequence <= *last) return false;
 *last = header.sequence;
 return true;
}

DecodeStatus SensorBridge::decode(const Lidar2dPacket& packet,
 LidarMeasurement& out) {
 const DecodeStatus header = validate_header(packet.header,
 SensorKind::kLidar2d);
 if (header != DecodeStatus::kAccepted) return header;
 if (packet.beam_count == 0 || packet.beam_count > config_.max_lidar_beams ||
 packet.angle_increment_urad == 0 || packet.angle_increment_urad < -6'300'000 ||
 packet.angle_increment_urad > 6'300'000) {
 return DecodeStatus::kInvalidPayload;
 }
 for (std::uint16_t i = 0; i < packet.beam_count; ++i) {
 if (packet.range_mm[i] == 0 || packet.range_mm[i] > config_.max_range_mm) {
 return DecodeStatus::kInvalidPayload;
 }
 }
 if (!accept_sequence(packet.header)) return DecodeStatus::kSequence;
 out = {};
 out.header = packet.header;
 out.beam_count = packet.beam_count;
 out.angle_min_rad = static_cast<float>(packet.angle_min_urad) * 1e-6f;
 out.angle_increment_rad =
 static_cast<float>(packet.angle_increment_urad) * 1e-6f;
 for (std::uint16_t i = 0; i < packet.beam_count; ++i) {
 out.ranges_m[i] = static_cast<float>(packet.range_mm[i]) * 1e-3f;
 }
 return DecodeStatus::kAccepted;
}

DecodeStatus SensorBridge::decode(const ImuPacket& packet,
 ImuMeasurement& out) {
 const DecodeStatus header = validate_header(packet.header, SensorKind::kImu);
 if (header != DecodeStatus::kAccepted) return header;
 if (!finite(packet.gyro_x_radps) || !finite(packet.gyro_y_radps) ||
 !finite(packet.gyro_z_radps) || !finite(packet.accel_x_mps2) ||
 !finite(packet.accel_y_mps2) || !finite(packet.accel_z_mps2)) {
 return DecodeStatus::kInvalidPayload;
 }
 if (!accept_sequence(packet.header)) return DecodeStatus::kSequence;
 out.header = packet.header;
 out.gyro_x_radps = packet.gyro_x_radps;
 out.gyro_y_radps = packet.gyro_y_radps;
 out.gyro_z_radps = packet.gyro_z_radps;
 out.accel_x_mps2 = packet.accel_x_mps2;
 out.accel_y_mps2 = packet.accel_y_mps2;
 out.accel_z_mps2 = packet.accel_z_mps2;
 return DecodeStatus::kAccepted;
}

DecodeStatus SensorBridge::decode(const JointPacket& packet,
 JointMeasurement& out) {
 const DecodeStatus header = validate_header(packet.header, SensorKind::kJoints);
 if (header != DecodeStatus::kAccepted) return header;
 if (packet.drive_count != config_.drive_count ||
 packet.steer_count != config_.steer_count) {
 return DecodeStatus::kInvalidPayload;
 }
 for (std::uint8_t i = 0; i < packet.drive_count; ++i) {
 if (!finite(packet.drive_radps[i])) return DecodeStatus::kInvalidPayload;
 }
 for (std::uint8_t i = 0; i < packet.steer_count; ++i) {
 if (!finite(packet.steer_rad[i]) || !finite(packet.steer_vel_radps[i])) {
 return DecodeStatus::kInvalidPayload;
 }
 }
 if (!accept_sequence(packet.header)) return DecodeStatus::kSequence;
 out = {};
 out.header = packet.header;
 out.drive_count = packet.drive_count;
 out.steer_count = packet.steer_count;
 for (std::uint8_t i = 0; i < packet.drive_count; ++i) {
 out.drive_radps[i] = packet.drive_radps[i];
 }
 for (std::uint8_t i = 0; i < packet.steer_count; ++i) {
 out.steer_rad[i] = packet.steer_rad[i];
 out.steer_vel_radps[i] = packet.steer_vel_radps[i];
 }
 return DecodeStatus::kAccepted;
}

} // namespace lost_signal::stm32_reference
