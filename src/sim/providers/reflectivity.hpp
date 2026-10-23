#pragma once

// SIM-side lidar return-strength model (generation only, never ROBOT
// input). Per-beam intensity from the authored surface at the hit point:
// actor cylinders 0.52, obstacle prisms 0.68, ground zones resolve the
// material library reflectivity (default ground 0.30) — main's component
// defaults verbatim. Incidence (master's angular-response doctrine):
// head-on returns read full strength, grazing returns dim to the
// incidence floor (0.15); normals resolve analytically per hit class
// (actor cylinder radial/top, axis-aligned box faces, canonical-terrain
// gradient — the same function the engine builds geometry from, so
// synthesis and contact cannot disagree). Distance attenuation
// 1/(1+0.015 r^2) matches main's planar fallback; misses read 0 (no
// return, never max-range fill). Deterministic and exact by default (no
// noise model; S7 realism switches live on the device, not here). The
// ROBOT carries intensities as measurements and never maps them back to
// materials: intensity is a sensing quantity, not a physics lookup.

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
                              bool actor_hit, bool missed, double dirx,
                              double diry, double dirz, double actor_cx_m,
                              double actor_cy_m, double actor_h_m) {
  if (missed || !(range_m > 0.0) || !std::isfinite(range_m)) return 0.0f;
  // Surface normal per hit class (unit, pointing out of the surface).
  double nx = 0.0, ny = 0.0, nz = 1.0;
  if (actor_hit) {
    // Upright cylinder: top cap reads +z, the wall reads radial.
    const double rx = hx - actor_cx_m, ry = hy - actor_cy_m;
    const double rr = std::hypot(rx, ry);
    if (hit_z_m >= actor_h_m - 1e-3 || !(rr > 1e-9)) {
      nx = 0.0;
      ny = 0.0;
      nz = 1.0;
    } else {
      nx = rx / rr;
      ny = ry / rr;
      nz = 0.0;
    }
  } else {
    bool on_box = false;
    for (const auto& b : world.static_boxes()) {
      if (!(b.height_m > 0.0)) continue;
      const double xlo = std::min(b.x0, b.x1), xhi = std::max(b.x0, b.x1);
      const double ylo = std::min(b.y0, b.y1), yhi = std::max(b.y0, b.y1);
      if (hx >= xlo && hx < xhi && hy >= ylo && hy < yhi &&
          hit_z_m <= b.height_m + 1e-3) {
        // Axis-aligned prism: the hit face is the nearest boundary plane.
        const double px = std::min(hx - xlo, xhi - hx);
        const double py = std::min(hy - ylo, yhi - hy);
        const double pz = b.height_m - hit_z_m;
        if (pz <= px && pz <= py) {
          nx = 0.0;
          ny = 0.0;
          nz = 1.0;
        } else if (px <= py) {
          nx = (hx - xlo < xhi - hx) ? -1.0 : 1.0;
          ny = 0.0;
          nz = 0.0;
        } else {
          nx = 0.0;
          ny = (hy - ylo < yhi - hy) ? -1.0 : 1.0;
          nz = 0.0;
        }
        on_box = true;
        break;
      }
    }
    if (!on_box) {
      // Authored ground (patches/slabs/relief): gradient of the canonical
      // height the engine builds from (finite differences, eps far below
      // the ~1 m relief wavelength). Flat floor reads +z exactly.
      constexpr double kGradEpsM = 0.05;
      const double h0 = world::sample_terrain(world.terrain_patches(),
                                              world.support_slabs(), hx, hy)
                            .height_m;
      const double hx1 = world::sample_terrain(world.terrain_patches(),
                                               world.support_slabs(),
                                               hx + kGradEpsM, hy)
                             .height_m;
      const double hy1 = world::sample_terrain(world.terrain_patches(),
                                               world.support_slabs(), hx,
                                               hy + kGradEpsM)
                             .height_m;
      double gx = 0.0, gy = 0.0;
      if (std::isfinite(h0) && std::isfinite(hx1)) gx = (hx1 - h0) / kGradEpsM;
      if (std::isfinite(h0) && std::isfinite(hy1)) gy = (hy1 - h0) / kGradEpsM;
      const double inv = 1.0 / std::hypot(gx, gy, 1.0);
      nx = -gx * inv;
      ny = -gy * inv;
      nz = inv;
    }
  }
  // Master's angular response: cosine between the incoming ray and the
  // surface normal, floored so grazing returns dim but never vanish.
  const double incidence =
      std::clamp(-(dirx * nx + diry * ny + dirz * nz), 0.0, 1.0);
  constexpr double kIncidenceFloor = 0.15;
  const double angular =
      kIncidenceFloor + (1.0 - kIncidenceFloor) * incidence;
  const double reflectivity =
      actor_hit ? 0.52 : surface_reflectivity(world, hx, hy, hit_z_m);
  const double intensity =
      reflectivity * angular / (1.0 + 0.015 * range_m * range_m);
  return static_cast<float>(std::clamp(intensity, 0.0, 1.0));
}

}  // namespace lost_signal::sim::providers::detail
