#pragma once

#include <string>
#include <utility>

#include "robot/iface/observation.hpp"
#include "serial_protocol.hpp"

namespace lost_signal::examples::serial_sensor {

struct ObservationAdapterConfig {
 std::string lidar_source = "ro1.lidar.serial";
 std::string imu_source = "ro1.imu.serial";
 std::string joints_source = "ro1.encoder.serial";
 robot::core::Frame lidar_frame = robot::core::Frame::kLidarLink;
 robot::core::Frame imu_frame = robot::core::Frame::kImuLink;
 robot::core::Frame joints_frame = robot::core::Frame::kBaseLink;
};

class ObservationAdapter {
 public:
 explicit ObservationAdapter(ObservationAdapterConfig config = {})
 : config_(std::move(config)) {}

 bool make(const SerialPacket& packet,
 const SerialSensorConfig& sensor_config,
 robot::iface::ObservationFrame& out) const;

 private:
 ObservationAdapterConfig config_;
};

} // namespace lost_signal::examples::serial_sensor
