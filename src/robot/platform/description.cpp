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

}  // namespace lost_signal::robot::platform
