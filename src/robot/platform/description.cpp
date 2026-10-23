#include "robot/platform/description.hpp"

namespace lost_signal::robot::platform {

bool valid_robot_description(const RobotDescription& desc) {
  if (desc.robot_id.empty()) return false;
  if (desc.version == 0) return false;
  if (desc.links.empty()) return false;
  if (desc.limits.max_forward_mps <= 0.0) return false;
  if (desc.limits.max_yaw_rate_radps <= 0.0) return false;
  if (desc.limits.footprint_radius_m <= 0.0) return false;
  return true;
}

std::uint64_t hash_robot_description(const RobotDescription& desc) {
  // FNV-1a over robot_id + version + limits + mount ids.
  std::uint64_t h = 1469598103934665603ULL;
  auto mix = [&](const std::string& s) {
    for (unsigned char c : s) {
      h ^= c;
      h *= 1099511628211ULL;
    }
  };
  mix(desc.robot_id);
  mix(std::to_string(desc.version));
  for (const auto& l : desc.links) {
    mix(l.link);
    mix(l.parent);
  }
  for (const auto& s : desc.sensors) {
    mix(s.sensor_id);
    mix(s.link);
  }
  return h;
}

std::vector<LinkMount> default_link_mounts() {
  LinkMount base, lidar, imu;
  base.link = "base_link";
  base.parent = "world";
  lidar.link = "lidar_link";
  lidar.parent = "base_link";
  lidar.xyz_m[0] = 0.25;
  lidar.xyz_m[2] = 0.35;
  imu.link = "imu_link";
  imu.parent = "base_link";
  imu.xyz_m[2] = 0.15;
  return {base, lidar, imu};
}

std::vector<SensorMount> default_sensor_mounts() {
  SensorMount lidar, imu;
  lidar.sensor_id = "ro1.lidar";
  lidar.link = "lidar_link";
  lidar.xyz_m[0] = 0.25;
  lidar.xyz_m[2] = 0.35;
  imu.sensor_id = "ro1.imu";
  imu.link = "imu_link";
  imu.xyz_m[2] = 0.15;
  return {lidar, imu};
}

RobotDescription default_robot_description() {
  RobotDescription desc;
  desc.robot_id = "ro1";
  desc.version = 2;
  desc.links = default_link_mounts();
  desc.sensors = default_sensor_mounts();
  desc.limits.max_forward_mps = 2.0;
  desc.limits.max_yaw_rate_radps = 2.0;
  desc.limits.footprint_radius_m = 0.4;
  desc.limits.max_steer_rad = 0.55;
  desc.limits.max_lateral_mps = 0.0;
  desc.chassis_kind = "differential";
  return desc;
}

}  // namespace lost_signal::robot::platform
