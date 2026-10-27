#include "observation_adapter.hpp"

namespace lost_signal::examples::serial_sensor {

bool ObservationAdapter::make(const SerialPacket& packet,
 const SerialSensorConfig& sensor_config,
 robot::iface::ObservationFrame& out) const {
 out = {};
 switch (static_cast<SensorKind>(packet.kind)) {
 case SensorKind::kLidar2d: {
 LidarMeasurement measurement;
 if (decode_lidar(packet, sensor_config, measurement) !=
 DecodeStatus::kAccepted) return false;
 out.sequence = measurement.sequence;
 out.stamp_ns = measurement.stamp_ns;
 out.source = config_.lidar_source;
 out.frame = config_.lidar_frame;
 out.scan_header = {measurement.sequence, measurement.stamp_ns,
 config_.lidar_source, config_.lidar_frame};
 out.scan_valid = true;
 out.scan_angle_min_rad = measurement.angle_min_rad;
 out.scan_angle_increment_rad = measurement.angle_increment_rad;
 out.ranges_m.assign(measurement.ranges_m.begin(),
 measurement.ranges_m.begin() + measurement.beam_count);
 return robot::iface::valid_observation_frame(out);
 }
 case SensorKind::kImu: {
 ImuMeasurement measurement;
 if (decode_imu(packet, measurement) != DecodeStatus::kAccepted) return false;
 out.sequence = measurement.sequence;
 out.stamp_ns = measurement.stamp_ns;
 out.source = config_.imu_source;
 out.frame = config_.imu_frame;
 out.imu_header = {measurement.sequence, measurement.stamp_ns,
 config_.imu_source, config_.imu_frame};
 out.imu_valid = true;
 out.imu_gyro_x_radps = measurement.gyro_x_radps;
 out.imu_gyro_y_radps = measurement.gyro_y_radps;
 out.imu_gyro_z_radps = measurement.gyro_z_radps;
 out.imu_ax_mps2 = measurement.accel_x_mps2;
 out.imu_ay_mps2 = measurement.accel_y_mps2;
 out.imu_az_mps2 = measurement.accel_z_mps2;
 return robot::iface::valid_observation_frame(out);
 }
 case SensorKind::kJoints: {
 JointsMeasurement measurement;
 if (decode_joints(packet, sensor_config, measurement) !=
 DecodeStatus::kAccepted) return false;
 out.sequence = measurement.sequence;
 out.stamp_ns = measurement.stamp_ns;
 out.source = config_.joints_source;
 out.frame = config_.joints_frame;
 out.motion_header = {measurement.sequence, measurement.stamp_ns,
 config_.joints_source, config_.joints_frame};
 out.drive_count = measurement.drive_count;
 out.steer_count = measurement.steer_count;
 for (std::uint8_t i = 0; i < measurement.drive_count; ++i) {
 out.drive_radps[i] = measurement.drive_radps[i];
 }
 for (std::uint8_t i = 0; i < measurement.steer_count; ++i) {
 out.steer_rad[i] = measurement.steer_rad[i];
 out.steer_vel_radps[i] = measurement.steer_vel_radps[i];
 }
 return robot::iface::valid_observation_frame(out);
 }
 }
 return false;
}

} // namespace lost_signal::examples::serial_sensor
