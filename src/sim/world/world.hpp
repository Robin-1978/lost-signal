#pragma once

#include <cstdint>

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

class World {
 public:
  void step(double dt_s);
  const WorldTruth& truth() const { return truth_; }

 private:
  WorldTruth truth_{};
};

}  // namespace lost_signal::sim::world
