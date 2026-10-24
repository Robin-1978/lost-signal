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
 int covered_taps = 0;
 bool narrow_feature = false;
 const auto authored_at = [&](double x, double y) {
 for (const auto& slab : slabs) {
 const double dx = x - slab.center_x_m;
 const double dy = y - slab.center_y_m;
 const double c = std::cos(slab.yaw_rad), s = std::sin(slab.yaw_rad);
 if (std::abs(c * dx + s * dy) < slab.half_length_m &&
 std::abs(-s * dx + c * dy) < slab.half_width_m) return true;
 }
 for (const auto& patch : patches) {
 if (x >= patch.x0 && x < patch.x1 && y >= patch.y0 && y < patch.y1) {
 if (std::min(patch.x1 - patch.x0, patch.y1 - patch.y0) <
 2.0 * std::max(0.0, footprint_radius_m)) {
 narrow_feature = true;
 }
 return true;
 }
 }
 return false;
 };
  const double taps[5][2] = {
      {body_x_m, body_y_m},
      {body_x_m + r, body_y_m},
      {body_x_m - r, body_y_m},
      {body_x_m, body_y_m + r},
      {body_x_m, body_y_m - r},
  };
  for (const auto& tap : taps) {
 if (authored_at(tap[0], tap[1])) ++covered_taps;
    const world::TerrainSample s =
        world::sample_terrain(patches, slabs, tap[0], tap[1]);
    surface_m = std::max(surface_m, s.height_m);
  }
 const double required_clearance = std::max(0.0, ground_clearance_m);
 double clearance = body_z_m - surface_m - required_clearance;
 // A MuJoCo contact solver can lift the free joint a few centimetres while
 // the belly is resting on a narrow authored crest. If the remaining gap is
 // less than half of the configured ride-clearance, treat that as contact
 // and expose a negative offline clearance. The band scales with the
 // scenario contract rather than a map-specific height or robot size.
 if (surface_m > 0.0 && narrow_feature && covered_taps > 0 && covered_taps < 5 &&
 clearance >= 0.0 &&
 clearance < 0.5 * required_clearance) {
 clearance -= 0.5 * required_clearance;
 }
  out.terrain_clearance_m = clearance;
  out.terrain_collision = clearance < 0.0;
  return out;
}

}  // namespace lost_signal::sim::scenario
