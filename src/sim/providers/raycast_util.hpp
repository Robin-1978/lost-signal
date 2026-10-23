#pragma once

// Shared SIM-side raycast helper (generation only, never ROBOT input).
// Upright finite cylinder for kinematic actors (ground-based z in
// [0, height]): curved side + top cap. Ported from main's
// raycast_upright_finite_cylinder (center-based there, ground-based here —
// actors stand on the floor). Returns nearest positive distance or +inf.

#include <algorithm>
#include <cmath>
#include <limits>

namespace lost_signal::sim::providers::detail {

inline double ray_actor_cylinder(double ox, double oy, double oz, double dx,
                                 double dy, double dz, double ax, double ay,
                                 double radius_m, double height_m,
                                 double max_m) {
  if (!(radius_m > 0.0) || !(height_m > 0.0)) {
    return std::numeric_limits<double>::infinity();
  }
  const double len = std::sqrt(dx * dx + dy * dy + dz * dz);
  if (!(len > 1e-12) || !std::isfinite(len)) {
    return std::numeric_limits<double>::infinity();
  }
  dx /= len;
  dy /= len;
  dz /= len;
  double best = std::numeric_limits<double>::infinity();
  // Curved side (both roots: the ray may start inside the projection).
  const double a = dx * dx + dy * dy;
  if (a > 1e-12) {
    const double ocx = ox - ax, ocy = oy - ay;
    const double b = 2.0 * (ocx * dx + ocy * dy);
    const double c = ocx * ocx + ocy * ocy - radius_m * radius_m;
    const double disc = b * b - 4.0 * a * c;
    if (disc >= 0.0) {
      const double root = std::sqrt(std::max(0.0, disc));
      for (double t : {(-b - root) / (2.0 * a), (-b + root) / (2.0 * a)}) {
        if (t < 0.0 || t > max_m || t >= best) continue;
        const double z = oz + dz * t;
        if (z >= -1e-9 && z <= height_m + 1e-9) best = t;
      }
    }
  }
  // Top cap.
  if (std::abs(dz) > 1e-12) {
    const double t = (height_m - oz) / dz;
    if (t >= 0.0 && t <= max_m && t < best) {
      const double hx = ox + dx * t - ax, hy = oy + dy * t - ay;
      if (hx * hx + hy * hy <= radius_m * radius_m) best = t;
    }
  }
  return best;
}

}  // namespace lost_signal::sim::providers::detail
