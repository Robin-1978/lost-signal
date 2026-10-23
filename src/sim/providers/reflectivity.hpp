#pragma once

// SIM-side lidar return-strength model (generation only, never ROBOT
// input). Per-beam intensity from the authored surface at the hit point:
// actor cylinders 0.52, obstacle prisms 0.68, ground zones resolve the
// material library reflectivity (default ground 0.30) — main's component
// defaults verbatim. Distance attenuation 1/(1+0.015 r^2) matches main's
// planar fallback; misses read 0 (no return, never max-range fill).
// Deterministic and exact by default (no noise model; S7 realism switches
// live on the device, not here). The ROBOT carries intensities as
// measurements and never maps them back to materials: intensity is a
// sensing quantity, not a physics lookup.

#include <algorithm>
#include <cmath>

#include "sim/world/terrain.hpp"
#include "sim/world/world.hpp"

namespace lost_signal::sim::providers::detail {

inline double surface_reflectivity(const world::World& world, double hx,
                                   double hy, double hit_z_m) {
  for (const auto& b : world.static_boxes()) {
    if (!(b.height_m > 0.0)) continue;
    if (hx >= std::min(b.x0, b.x1) && hx < std::max(b.x0, b.x1) &&
        hy >= std::min(b.y0, b.y1) && hy < std::max(b.y0, b.y1) &&
        hit_z_m <= b.height_m + 1e-3) {
      return 0.68;  // obstacle prism
    }
  }
  const world::MaterialZone* zone =
      world::material_zone_at(world.material_zones(), hx, hy);
  if (zone != nullptr) {
    const world::MaterialEntry* mat =
        world::find_material(world.materials(), zone->material_id);
    if (mat != nullptr) return std::max(0.0, mat->lidar_reflectivity);
  }
  return 0.30;  // default ground
}

inline float return_intensity(const world::World& world, double hx,
                              double hy, double hit_z_m, double range_m,
                              bool actor_hit, bool missed) {
  if (missed || !(range_m > 0.0) || !std::isfinite(range_m)) return 0.0f;
  const double reflectivity =
      actor_hit ? 0.52 : surface_reflectivity(world, hx, hy, hit_z_m);
  const double intensity =
      reflectivity / (1.0 + 0.015 * range_m * range_m);
  return static_cast<float>(std::clamp(intensity, 0.0, 1.0));
}

}  // namespace lost_signal::sim::providers::detail
