#pragma once

// SIM-owned authored-terrain sampling (World internals, never ROBOT input).
// Single canonical height function: the engine builds contact geometry from
// it and the survey rasterizes elevation from it, so the two cannot drift
// apart. Plane math ported verbatim from main's TerrainModel::sample (minus
// the roughness waveform, which needs mesh support — T2):
//   height = base + gx*dx + gy*dy + gxy*dx*dy (bilinear cross-slope included).
// Later revisions win: support slabs override patches, and within one list
// the later entry wins. Outside everything the height is 0 (flat floor).

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#include "sim/world/world.hpp"

namespace lost_signal::sim::world {

struct TerrainSample {
  double height_m = 0.0;
  double grade_x = 0.0;
  double grade_y = 0.0;
};

inline bool sample_support_slab(const SupportSlab& slab, double x, double y,
                                TerrainSample& out) {
  const double dx = x - slab.center_x_m;
  const double dy = y - slab.center_y_m;
  const double cosine = std::cos(slab.yaw_rad);
  const double sine = std::sin(slab.yaw_rad);
  const double local_x = cosine * dx + sine * dy;
  const double local_y = -sine * dx + cosine * dy;
  if (std::abs(local_x) >= slab.half_length_m ||
      std::abs(local_y) >= slab.half_width_m) {
    return false;
  }
  out.height_m =
      slab.center_height_m + slab.grade_along_local_x * local_x;
  out.grade_x = slab.grade_along_local_x * cosine;
  out.grade_y = slab.grade_along_local_x * sine;
  return true;
}

inline TerrainSample sample_terrain(
    const std::vector<TerrainPatch>& patches,
    const std::vector<SupportSlab>& slabs, double x, double y) {
  for (auto it = slabs.rbegin(); it != slabs.rend(); ++it) {
    TerrainSample sample;
    if (sample_support_slab(*it, x, y, sample)) return sample;
  }
  for (auto it = patches.rbegin(); it != patches.rend(); ++it) {
    const TerrainPatch& patch = *it;
    if (!(x >= patch.x0 && x < patch.x1 && y >= patch.y0 &&
          y < patch.y1)) {
      continue;
    }
    const double dx = x - patch.x0;
    const double dy = y - patch.y0;
    // T2: + rough_relief(patch.roughness, w, d, dx, dy) height + gradient.
    // Until mesh support lands the engine refuses roughness > 0 fail-closed,
    // so reaching here with nonzero roughness is an authoring error, never
    // a silently smoothed surface.
    return {patch.base_height_m + patch.grade_x * dx + patch.grade_y * dy +
                patch.grade_xy_per_m * dx * dy,
            patch.grade_x + patch.grade_xy_per_m * dy,
            patch.grade_y + patch.grade_xy_per_m * dx};
  }
  return {};
}

// Material zone at a point, later entries win. Null when uncovered: the
// caller falls back to the default ground material.
inline const MaterialZone* material_zone_at(
    const std::vector<MaterialZone>& zones, double x, double y) {
  for (auto it = zones.rbegin(); it != zones.rend(); ++it) {
    if (x >= it->x0 && x < it->x1 && y >= it->y0 && y < it->y1) {
      return &(*it);
    }
  }
  return nullptr;
}

// Material entry by id, first wins. Null when unknown: the caller treats
// the zone as invalid (world files fail closed on this at load).
inline const MaterialEntry* find_material(
    const std::vector<MaterialEntry>& materials, const std::string& id) {
  for (const auto& m : materials) {
    if (m.material_id == id) return &m;
  }
  return nullptr;
}

// Time-varying grip multiplier for one material (main's environment
// keyframes: values interpolate on simulation time, missing stays 1.0,
// ends clamp). Order-independent scan, so files need no sorting.
inline double environment_grip_at(
    const std::vector<EnvironmentKeyframe>& environment,
    const std::string& material_id, double time_s) {
  bool has_lo = false, has_hi = false;
  double lo_t = 0.0, lo_v = 1.0, hi_t = 0.0, hi_v = 1.0;
  bool seen = false;
  for (const auto& kf : environment) {
    double mult = 1.0;
    bool has = false;
    for (const auto& [id, v] : kf.material_grip) {
      if (id == material_id) {
        mult = v;
        has = true;
        break;
      }
    }
    if (!has) continue;
    seen = true;
    if (kf.time_s <= time_s && (!has_lo || kf.time_s >= lo_t)) {
      has_lo = true;
      lo_t = kf.time_s;
      lo_v = mult;
    }
    if (kf.time_s >= time_s && (!has_hi || kf.time_s <= hi_t)) {
      has_hi = true;
      hi_t = kf.time_s;
      hi_v = mult;
    }
  }
  if (!seen) return 1.0;
  if (has_lo && has_hi && hi_t > lo_t) {
    const double u = (time_s - lo_t) / (hi_t - lo_t);
    return lo_v + u * (hi_v - lo_v);
  }
  return has_lo ? lo_v : hi_v;
}

}  // namespace lost_signal::sim::world
