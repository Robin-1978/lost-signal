#pragma once

// Draw model for the viewer. Pure SIM-side data (truth + provider outputs
// already recorded in the episode step), no raylib dependency, so unit
// tests build frames without a window. The frontend projects world-space
// primitives through ViewTransform and draws them; 2D and 3D frontends
// share this model.

#include <cstddef>
#include <string>
#include <vector>

namespace lost_signal::sim::viewer {

struct FrameLinkPose {
 bool valid = false;
 double x_m = 0.0, y_m = 0.0, z_m = 0.0;
 double qw = 1.0, qx = 0.0, qy = 0.0, qz = 0.0;
};

struct FrameWheel {
  double x_m = 0.0, y_m = 0.0;  // module position in the base frame
  double steer_rad = 0.0;       // module yaw (fixed wheels read 0)
  double spin_rad = 0.0;        // live joint angle (turns visibly)
  double radius_m = 0.1;
  bool slipping = false;        // engine traction flag: drawn red
  // Mecanum rollers are NOT simulated state (analytic wrench, rollers not
  // meshed): static 45-degree tread stripes, honest decoration only.
 double spin_rate_radps = 0.0;
 double steer_rate_radps = 0.0;
 double duty = 0.0;
 bool driven = false;
 bool mecanum_rollers = false;
 // Exact MuJoCo wheel-body pose in the base frame. The joint angles above
 // remain useful diagnostics; 3D rendering uses this pose when valid.
 FrameLinkPose pose;
};

struct FrameCaster {
 double pivot_x_m = 0.0, pivot_y_m = 0.0;
 double x_m = 0.0, y_m = 0.0;
 double radius_m = 0.03;
 double yaw_rad = 0.0; // MuJoCo swivel joint angle
 double spin_rad = 0.0; // MuJoCo passive wheel roll angle
 FrameLinkPose fork_pose;
 FrameLinkPose wheel_pose;
};

struct FrameBody {
 double forward_mps = 0.0;
 double lateral_mps = 0.0;
 double yaw_rate_radps = 0.0;
  double x_m = 0.0, y_m = 0.0, yaw_rad = 0.0;
  double half_length_m = 0.3;  // body-forward extent (wheel_base/2 floor)
  double half_width_m = 0.25;  // body-left extent (track/2 floor)
  double z_m = 0.0;            // engine height (3D frontend only)
  double roll_rad = 0.0;       // live body attitude (ramps tilt the viz)
  double pitch_rad = 0.0;
  // True sensor origin in world (body pose composed with the mount — the
  // ray fan starts here, never at the body center).
  double sens_x_m = 0.0, sens_y_m = 0.0, sens_z_m = 0.0;
  // Wheels in drive order (empty for skid-steer slabs, see tracked below);
  // module positions read the backend layout table, angles read truth.
  std::vector<FrameWheel> wheels;
 // Unpowered physical caster supports. They are SIM-only presentation state,
 // never drive channels, traction evidence, or ROBOT observations.
 std::vector<FrameCaster> casters;
  // Skid-steer tracks (no rolling wheels to draw): slabs sized from the
  // chassis params, slip-reddening per side from the traction flags.
  bool tracked = false;
  // ROBOT estimate at the same tick (offline truth-vs-estimate overlay).
  // estimate_valid=false hides the ghost.
  bool estimate_valid = false;
  double est_x_m = 0.0, est_y_m = 0.0, est_yaw_rad = 0.0;
};

struct FrameBox {
  double x0_m = 0.0, y0_m = 0.0, x1_m = 0.0, y1_m = 0.0;
  double height_m = 0.0;
  bool selected = false;
  int selected_corner = -1;  // 0..3 (min/min, max/min, max/max, min/max)
};

struct FrameActor {
  double x_m = 0.0, y_m = 0.0, radius_m = 0.4;
  double vx_mps = 0.0, vy_mps = 0.0;
 double heading_rad = 0.0;
  double height_m = 1.5;  // sensor cylinder height (vehicle 1.5, pedestrian 1.8)
  bool pedestrian = false;  // explicit SIM visual kind, never inferred by size
};

// Ground-material zone overlay (SIM truth paint, never ROBOT input): the
// frontend tints the ground per zone so slick/sand reads visually.
struct FrameZone {
  double x0_m = 0.0, y0_m = 0.0, x1_m = 0.0, y1_m = 0.0;
  std::string material_id;
};

// Terrain patch footprint (frontend samples heights itself for the mesh).
struct FramePatch {
  double x0_m = 0.0, y0_m = 0.0, x1_m = 0.0, y1_m = 0.0;
};

struct FramePlace {
  std::string place_id;
  double x_m = 0.0, y_m = 0.0, yaw_rad = 0.0;
  bool reached = false;
  bool selected = false;
};

struct Frame {
  FrameBody body;
  std::vector<std::pair<double, double>> trail;  // past truth (x, y), oldest first
  std::vector<FrameBox> boxes;
  std::vector<FrameActor> actors;
  std::vector<FramePlace> places;
  std::vector<FrameZone> zones;
  std::vector<FramePatch> patches;
  std::vector<std::pair<double, double>> lidar_hits;  // world-space endpoints
  bool lidar_valid = false;
  bool static_collision = false;
  bool dynamic_collision = false;
  double static_clearance_m = 999.0;
  std::uint64_t step = 0;
};

}  // namespace lost_signal::sim::viewer
