#pragma once

// Host-side glue only. This file is intentionally not part of the STM32
// firmware target: std::string/vector are used at the native boundary to
// construct the repository's provider-neutral ObservationFrame.

#include <string>
#include <utility>

#include "robot/iface/observation.hpp"
#include "sensor_bridge.hpp"

namespace lost_signal::stm32_reference {

struct ObservationAdapterConfig {
 std::string lidar_source = "ro1.lidar";
 std::string imu_source = "ro1.imu";
 std::string joints_source = "ro1.encoder";
 robot::core::Frame lidar_frame = robot::core::Frame::kLidarLink;
 robot::core::Frame imu_frame = robot::core::Frame::kImuLink;
 robot::core::Frame joints_frame = robot::core::Frame::kBaseLink;
};

class ObservationAdapter {
 public:
 explicit ObservationAdapter(ObservationAdapterConfig config = {})
 : config_(std::move(config)) {}

 robot::iface::ObservationFrame lidar(const LidarMeasurement& measurement) const {
 robot::iface::ObservationFrame out;
 out.sequence = measurement.header.sequence;
 out.stamp_ns = measurement.header.stamp_ns;
 out.source = config_.lidar_source;
 out.frame = config_.lidar_frame;
 out.scan_header = {measurement.header.sequence, measurement.header.stamp_ns,
 config_.lidar_source, config_.lidar_frame};
 out.scan_valid = true;
 out.scan_angle_min_rad = measurement.angle_min_rad;
 out.scan_angle_increment_rad = measurement.angle_increment_rad;
 out.ranges_m.assign(measurement.ranges_m,
 measurement.ranges_m + measurement.beam_count);
 return out;
 }

 robot::iface::ObservationFrame imu(const ImuMeasurement& measurement) const {
 robot::iface::ObservationFrame out;
 out.sequence = measurement.header.sequence;
 out.stamp_ns = measurement.header.stamp_ns;
 out.source = config_.imu_source;
 out.frame = config_.imu_frame;
 out.imu_header = {measurement.header.sequence, measurement.header.stamp_ns,
 config_.imu_source, config_.imu_frame};
 out.imu_valid = true;
 out.imu_gyro_x_radps = measurement.gyro_x_radps;
 out.imu_gyro_y_radps = measurement.gyro_y_radps;
 out.imu_gyro_z_radps = measurement.gyro_z_radps;
 out.imu_ax_mps2 = measurement.accel_x_mps2;
 out.imu_ay_mps2 = measurement.accel_y_mps2;
 out.imu_az_mps2 = measurement.accel_z_mps2;
 return out;
 }

 robot::iface::ObservationFrame joints(
 const JointMeasurement& measurement) const {
 robot::iface::ObservationFrame out;
 out.sequence = measurement.header.sequence;
 out.stamp_ns = measurement.header.stamp_ns;
 out.source = config_.joints_source;
 out.frame = config_.joints_frame;
 out.motion_header = {measurement.header.sequence, measurement.header.stamp_ns,
 config_.joints_source, config_.joints_frame};
 out.drive_count = measurement.drive_count;
 out.steer_count = measurement.steer_count;
 out.drive_radps[0] = measurement.drive_radps[0];
 out.drive_radps[1] = measurement.drive_radps[1];
 out.drive_radps[2] = measurement.drive_radps[2];
 out.drive_radps[3] = measurement.drive_radps[3];
 out.steer_rad[0] = measurement.steer_rad[0];
 out.steer_rad[1] = measurement.steer_rad[1];
 out.steer_rad[2] = measurement.steer_rad[2];
 out.steer_rad[3] = measurement.steer_rad[3];
 out.steer_vel_radps[0] = measurement.steer_vel_radps[0];
 out.steer_vel_radps[1] = measurement.steer_vel_radps[1];
 out.steer_vel_radps[2] = measurement.steer_vel_radps[2];
 out.steer_vel_radps[3] = measurement.steer_vel_radps[3];
 return out;
 }

 private:
 ObservationAdapterConfig config_;
};

} // namespace lost_signal::stm32_reference
