#include "sim/scenario/collision_judge.hpp"

#include <algorithm>
#include <cmath>

#include "sim/world/terrain.hpp"

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

eval::CollisionOutcome assess_terrain_contact(
    double body_x_m, double body_y_m, double body_z_m,
    double footprint_radius_m,
    const std::vector<world::TerrainPatch>& patches,
    const std::vector<world::SupportSlab>& slabs,
    double ground_clearance_m) {
  eval::CollisionOutcome out;
  const double r = std::max(0.0, footprint_radius_m) / std::sqrt(2.0);
  double surface_m = -999.0;
  const double taps[5][2] = {
      {body_x_m, body_y_m},
      {body_x_m + r, body_y_m},
      {body_x_m - r, body_y_m},
      {body_x_m, body_y_m + r},
      {body_x_m, body_y_m - r},
  };
  for (const auto& tap : taps) {
    const world::TerrainSample s =
        world::sample_terrain(patches, slabs, tap[0], tap[1]);
    surface_m = std::max(surface_m, s.height_m);
  }
  const double clearance =
      body_z_m - surface_m - std::max(0.0, ground_clearance_m);
  out.terrain_clearance_m = clearance;
  out.terrain_collision = clearance < 0.0;
  return out;
}

}  // namespace lost_signal::sim::scenario
