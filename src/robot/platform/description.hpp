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
 // Optional Ackermann curvature envelope (1/m). Zero leaves the
 // differential/holonomic yaw-rate window unchanged.
 double max_curvature_inv_m = 0.0;
};

// Declared plant loadout for autonomy envelopes (rollover support/COM,
// ride height, mass) — description from the machine file, never measured
// truth. COM height folds the declared payload master-style (weighted
// average); the SIM plant folds the identical numbers engine-side, so
// the two cannot disagree about what the loadout is.
struct PlantGeometry {
  double wheel_radius_m = 0.1;  // ride height (body origin above ground)
  double track_width_m = 0.5;   // rollover support width
  double wheel_base_m = 0.56;
  double com_height_m = 0.1;    // effective (payload-folded) COM height
  double total_mass_kg = 24.0;  // chassis + declared payload
};

struct RobotDescription {
  std::string robot_id;      // e.g. "ro1"
  std::string chassis_kind = "differential";  // selects the chassis factory
 // ROBOT autonomy selection. Both planners consume the same estimates,
 // local map and dynamic prediction contract.
 std::string local_planner_kind = "dwa";
  std::uint64_t version = 1;  // bump on any mount/limit change
  std::vector<LinkMount> links;
  std::vector<SensorMount> sensors;
  ChassisLimits limits;
  PlantGeometry plant;  // declared loadout for autonomy envelopes (COM,
                        // ride, support, mass) — description, never truth
};

bool valid_robot_description(const RobotDescription& desc);
// FNV-1a hash of the canonical fields, for calibration/version pinning.
std::uint64_t hash_robot_description(const RobotDescription& desc);

// Canonical mast (single source for code defaults): the lidar rides a
// 0.35 m mast 0.25 m forward, the IMU low on the deck. Seed files carry
// the same numbers explicitly; the loader fills absent sections from here
// so a bare `{"kind": ...}` never silently parks the lidar at the deck.
std::vector<LinkMount> default_link_mounts();
std::vector<SensorMount> default_sensor_mounts();
// Full canonical description (ro1 differential + limits + mast).
RobotDescription default_robot_description();

}  // namespace lost_signal::robot::platform
