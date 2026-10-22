#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace lost_signal::robot::platform {

// Minimal URDF-like description, ROBOT-owned and versioned.
// This is configuration, not physics: no MuJoCo bodies, contacts, motors,
// rendering meshes, or scenario presentation assets here. SIM physics and
// scenario presentation each keep their own adapter keyed by robot_id.
struct LinkMount {
  std::string link;         // e.g. "base_link", "lidar_link"
  std::string parent;       // parent link; "world" only for base
  double xyz_m[3] = {};     // parent -> link translation (m)
  double rpy_rad[3] = {};   // parent -> link rotation (rad)
};

struct SensorMount {
  std::string sensor_id;  // matches EndpointDescriptor.endpoint_id
  std::string link;       // mounting link
  double xyz_m[3] = {};
  double rpy_rad[3] = {};
};

struct ChassisLimits {
  double max_forward_mps = 0.0;
  double max_yaw_rate_radps = 0.0;
  double footprint_radius_m = 0.0;
  // Ackermann steering envelope (clamp + twist conversion bound).
  double max_steer_rad = 0.55;
  // Holonomic lateral envelope.
  double max_lateral_mps = 0.0;
};

struct RobotDescription {
  std::string robot_id;      // e.g. "ro1"
  std::string chassis_kind = "differential";  // selects the chassis factory
  std::uint64_t version = 1;  // bump on any mount/limit change
  std::vector<LinkMount> links;
  std::vector<SensorMount> sensors;
  ChassisLimits limits;
};

bool valid_robot_description(const RobotDescription& desc);
// FNV-1a hash of the canonical fields, for calibration/version pinning.
std::uint64_t hash_robot_description(const RobotDescription& desc);

}  // namespace lost_signal::robot::platform
