#include "sim/world/world.hpp"

#include <algorithm>
#include <cmath>

namespace lost_signal::sim::world {

void World::step(double dt_s) {
  truth_.step += 1;
  // First-order motor lag toward the duty-implied wheel speeds.
  const double free_radps =
      motor_.free_speed_mps / std::max(1e-6, motor_.wheel_radius_m);
  const double alpha =
      dt_s <= 0.0 ? 1.0
                  : 1.0 - std::exp(-dt_s / std::max(1e-6, motor_.time_constant_s));
  truth_.left_radps +=
      (truth_.duty_left * free_radps - truth_.left_radps) * alpha;
  truth_.right_radps +=
      (truth_.duty_right * free_radps - truth_.right_radps) * alpha;
  // Rigid differential body follows the wheels.
  const double vl = truth_.left_radps * motor_.wheel_radius_m;
  const double vr = truth_.right_radps * motor_.wheel_radius_m;
  truth_.forward_mps = 0.5 * (vl + vr);
  truth_.yaw_rate_radps =
      (vr - vl) / std::max(1e-6, motor_.track_width_m);
  truth_.body_yaw_rad += truth_.yaw_rate_radps * dt_s;
  truth_.body_x_m += truth_.forward_mps * std::cos(truth_.body_yaw_rad) * dt_s;
  truth_.body_y_m += truth_.forward_mps * std::sin(truth_.body_yaw_rad) * dt_s;
  for (auto& a : actors_) {
    a.x += a.vx_mps * dt_s;
    a.y += a.vy_mps * dt_s;
  }
}

}  // namespace lost_signal::sim::world
