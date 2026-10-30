#include "robot/platform/description.hpp"

#include <algorithm>
#include <cmath>

namespace lost_signal::robot::platform {

bool valid_robot_description(const RobotDescription& desc) {
 if (desc.robot_id.empty()) return false;
 if (desc.local_planner_kind != "dwa" &&
 desc.local_planner_kind != "teb") return false;
  if (desc.version == 0) return false;
  if (desc.links.empty()) return false;
 auto finite_vec3 = [](const double values[3]) {
  return std::all_of(values, values + 3,
                     [](double value) { return std::isfinite(value); });
 };
 for (const auto& link : desc.links) {
  if (link.link.empty() || link.parent.empty() ||
      !finite_vec3(link.xyz_m) || !finite_vec3(link.rpy_rad)) {
   return false;
  }
 }
 for (const auto& sensor : desc.sensors) {
  if (sensor.sensor_id.empty() || sensor.link.empty() ||
      !finite_vec3(sensor.xyz_m) || !finite_vec3(sensor.rpy_rad)) {
   return false;
  }
 }
  if (desc.limits.max_forward_mps <= 0.0) return false;
  if (desc.limits.max_yaw_rate_radps <= 0.0) return false;
  if (desc.limits.footprint_radius_m <= 0.0) return false;
 if (!std::isfinite(desc.limits.max_forward_mps) ||
     !std::isfinite(desc.limits.max_yaw_rate_radps) ||
     !std::isfinite(desc.limits.footprint_radius_m) ||
     !std::isfinite(desc.limits.max_steer_rad) ||
     !std::isfinite(desc.limits.max_lateral_mps) ||
     !std::isfinite(desc.limits.max_curvature_inv_m) ||
     !std::isfinite(desc.limits.differential_track_m) ||
     !std::isfinite(desc.limits.max_wheel_speed_mps) ||
     desc.limits.max_steer_rad < 0.0 ||
     desc.limits.max_lateral_mps < 0.0 ||
     desc.limits.max_curvature_inv_m < 0.0 ||
     desc.limits.differential_track_m < 0.0 ||
     desc.limits.max_wheel_speed_mps < 0.0) {
  return false;
 }
 if (!std::isfinite(desc.navigation.clearance_margin_m) ||
 desc.navigation.clearance_margin_m < 0.0) return false;
 if (!std::isfinite(desc.navigation.soft_inflation_radius_m) ||
 desc.navigation.soft_inflation_radius_m < 0.0) return false;
 if (!std::isfinite(desc.arrival.position_tolerance_m) ||
 desc.arrival.position_tolerance_m <= 0.0) return false;
 if (!std::isfinite(desc.arrival.heading_tolerance_rad) ||
 desc.arrival.heading_tolerance_rad <= 0.0) return false;
 if (!std::isfinite(desc.arrival.confidence_probability) ||
 desc.arrival.confidence_probability <= 0.0 ||
 desc.arrival.confidence_probability >= 1.0) return false;
 if (!std::isfinite(desc.arrival.maximum_position_sigma_m) ||
 desc.arrival.maximum_position_sigma_m <= 0.0) return false;
 if (!std::isfinite(desc.arrival.maximum_heading_sigma_rad) ||
 desc.arrival.maximum_heading_sigma_rad <= 0.0) return false;
 if (!std::isfinite(desc.arrival.maximum_forward_speed_mps) ||
 desc.arrival.maximum_forward_speed_mps < 0.0) return false;
 if (!std::isfinite(desc.arrival.maximum_yaw_rate_radps) ||
 desc.arrival.maximum_yaw_rate_radps < 0.0) return false;
 if (desc.arrival.settle_cycles <= 0) return false;
 if (!std::isfinite(desc.motion.acceleration_mps2) ||
 desc.motion.acceleration_mps2 <= 0.0) return false;
 if (!std::isfinite(desc.motion.brake_deceleration_mps2) ||
 desc.motion.brake_deceleration_mps2 <= 0.0) return false;
 if (!std::isfinite(desc.motion.lateral_acceleration_mps2) ||
 desc.motion.lateral_acceleration_mps2 <= 0.0) return false;
 if (!std::isfinite(desc.motion.yaw_acceleration_radps2) ||
 desc.motion.yaw_acceleration_radps2 <= 0.0) return false;
 if (!std::isfinite(desc.motion.actuator_response_delay_s) ||
 desc.motion.actuator_response_delay_s < 0.0) return false;
 if (!std::isfinite(desc.motion.safety_reaction_time_s) ||
 desc.motion.safety_reaction_time_s < 0.0) return false;
 if (!std::isfinite(desc.motion.execution_fault_confirmation_s) ||
 desc.motion.execution_fault_confirmation_s < 0.0) return false;
  if (desc.plant.track_width_m <= 0.0) return false;
  if (desc.plant.wheel_radius_m <= 0.0) return false;
  if (desc.plant.wheel_base_m <= 0.0) return false;
 if (!std::isfinite(desc.plant.track_width_m) ||
     !std::isfinite(desc.plant.wheel_radius_m) ||
     !std::isfinite(desc.plant.wheel_base_m)) {
  return false;
 }
 if (desc.plant.body_half_length_m <= 0.0 ||
  desc.plant.body_half_width_m <= 0.0 ||
  desc.plant.body_half_height_m <= 0.0) return false;
 if (!std::isfinite(desc.plant.body_half_length_m) ||
     !std::isfinite(desc.plant.body_half_width_m) ||
     !std::isfinite(desc.plant.body_half_height_m)) {
  return false;
 }
 if (desc.limits.footprint_radius_m + 1e-12 <
  std::hypot(desc.plant.body_half_length_m,
  desc.plant.body_half_width_m)) return false;
  if (desc.plant.com_height_m <= 0.0) return false;
  if (desc.plant.total_mass_kg <= 0.0) return false;
 if (!std::isfinite(desc.plant.com_height_m) ||
     !std::isfinite(desc.plant.total_mass_kg)) {
  return false;
 }
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
 mix(desc.local_planner_kind);
  mix(std::to_string(desc.version));
  for (const auto& l : desc.links) {
    mix(l.link);
    mix(l.parent);
  }
  for (const auto& s : desc.sensors) {
    mix(s.sensor_id);
    mix(s.link);
  }
  // Plant loadout pins the calibration too: a COM/mass change without a
  // version bump still hashes differently (never a stale envelope).
  mix(std::to_string(desc.plant.track_width_m));
  mix(std::to_string(desc.plant.wheel_radius_m));
 mix(std::to_string(desc.plant.body_half_length_m));
 mix(std::to_string(desc.plant.body_half_width_m));
 mix(std::to_string(desc.plant.body_half_height_m));
  mix(std::to_string(desc.plant.com_height_m));
  mix(std::to_string(desc.plant.total_mass_kg));
 mix(std::to_string(desc.navigation.clearance_margin_m));
 mix(std::to_string(desc.navigation.soft_inflation_radius_m));
 mix(std::to_string(desc.arrival.position_tolerance_m));
 mix(std::to_string(desc.arrival.heading_tolerance_rad));
 mix(std::to_string(desc.arrival.confidence_probability));
 mix(std::to_string(desc.arrival.maximum_position_sigma_m));
 mix(std::to_string(desc.arrival.maximum_heading_sigma_rad));
 mix(std::to_string(desc.arrival.maximum_forward_speed_mps));
 mix(std::to_string(desc.arrival.maximum_yaw_rate_radps));
 mix(std::to_string(desc.arrival.settle_cycles));
 mix(std::to_string(desc.motion.acceleration_mps2));
 mix(std::to_string(desc.motion.brake_deceleration_mps2));
 mix(std::to_string(desc.motion.lateral_acceleration_mps2));
 mix(std::to_string(desc.motion.yaw_acceleration_radps2));
 mix(std::to_string(desc.motion.actuator_response_delay_s));
 mix(std::to_string(desc.motion.safety_reaction_time_s));
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
 desc.local_planner_kind = "dwa";
  return desc;
}

}  // namespace lost_signal::robot::platform
