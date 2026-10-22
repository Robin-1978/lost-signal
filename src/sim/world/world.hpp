#pragma once

#include <cstdint>
#include <vector>

namespace lost_signal::sim::world {

// SIM-owned ground truth. Must never be included from src/robot/.
// Physics, actors, contacts, and scoring inputs live here.
struct WorldTruth {
  double body_x_m = 0.0;
  double body_y_m = 0.0;
  double body_yaw_rad = 0.0;
  double forward_mps = 0.0;
  std::uint64_t step = 0;
};

// Authored scenario geometry (static) and moving actors (dynamic).
// Raycast sources, never ROBOT inputs: the suite turns them into measured
// scans (full raycast) and a static reference (static geometry only).
struct StaticBox {
  double x0 = 0.0, y0 = 0.0, x1 = 0.0, y1 = 0.0;
};

struct Actor {
  double x = 0.0, y = 0.0, vx_mps = 0.0, vy_mps = 0.0, radius_m = 0.4;
};

class World {
 public:
  void step(double dt_s);
  // Kinematic plant v1: the body tracks the last delivered command exactly.
  // A dynamic motor/contact model replaces this body, not its callers.
  void apply_command(double forward_mps) { truth_.forward_mps = forward_mps; }
  const WorldTruth& truth() const { return truth_; }

  void set_static_boxes(std::vector<StaticBox> boxes) {
    static_boxes_ = std::move(boxes);
  }
  const std::vector<StaticBox>& static_boxes() const { return static_boxes_; }
  void add_actor(const Actor& actor) { actors_.push_back(actor); }
  const std::vector<Actor>& actors() const { return actors_; }

 private:
  WorldTruth truth_{};
  std::vector<StaticBox> static_boxes_;
  std::vector<Actor> actors_;
};

}  // namespace lost_signal::sim::world
