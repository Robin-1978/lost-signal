#include "sim/world/world.hpp"

namespace lost_signal::sim::world {

void World::step(double dt_s) {
  truth_.step += 1;
  truth_.body_x_m += truth_.forward_mps * dt_s;
}

}  // namespace lost_signal::sim::world
