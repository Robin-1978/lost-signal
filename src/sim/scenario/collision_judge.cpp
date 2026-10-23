#include "sim/scenario/collision_judge.hpp"

#include <algorithm>
#include <cmath>

namespace lost_signal::sim::scenario {

eval::CollisionOutcome assess_collision(
    double body_x_m, double body_y_m, double robot_radius_m,
    const std::vector<world::StaticBox>& boxes,
    const std::vector<world::Actor>& actors) {
  eval::CollisionOutcome out;
  const double radius = std::max(0.0, robot_radius_m);
  for (const auto& b : boxes) {
    if (!(b.height_m > 0.0)) continue;  // flat paint: no collision volume
    // Closest point on the (axis-aligned) box to the robot center.
    const double cx =
        std::clamp(body_x_m, std::min(b.x0, b.x1), std::max(b.x0, b.x1));
    const double cy =
        std::clamp(body_y_m, std::min(b.y0, b.y1), std::max(b.y0, b.y1));
    const double clearance = std::hypot(body_x_m - cx, body_y_m - cy) - radius;
    out.static_clearance_m = std::min(out.static_clearance_m, clearance);
    if (clearance < 0.0) out.static_collision = true;
  }
  for (const auto& a : actors) {
    const double clearance = std::hypot(body_x_m - a.x, body_y_m - a.y) -
                             radius - std::max(0.0, a.radius_m);
    out.dynamic_clearance_m = std::min(out.dynamic_clearance_m, clearance);
    if (clearance < 0.0) out.dynamic_collision = true;
  }
  return out;
}

}  // namespace lost_signal::sim::scenario
