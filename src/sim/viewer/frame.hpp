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

struct FrameBody {
  double x_m = 0.0, y_m = 0.0, yaw_rad = 0.0;
  double half_length_m = 0.3;  // body-forward extent (wheel_base/2 floor)
  double half_width_m = 0.25;  // body-left extent (track/2 floor)
  double z_m = 0.0;            // engine height (3D frontend only)
  double roll_rad = 0.0;       // live body attitude (ramps tilt the viz)
  double pitch_rad = 0.0;
  // True sensor origin in world (body pose composed with the mount — the
  // ray fan starts here, never at the body center).
  double sens_x_m = 0.0, sens_y_m = 0.0, sens_z_m = 0.0;
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
  std::vector<std::pair<double, double>> lidar_hits;  // world-space endpoints
  bool lidar_valid = false;
  bool static_collision = false;
  bool dynamic_collision = false;
  double static_clearance_m = 999.0;
  std::uint64_t step = 0;
};

}  // namespace lost_signal::sim::viewer
