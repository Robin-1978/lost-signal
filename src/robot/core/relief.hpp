#pragma once

// Deterministic ground-relief primitive (pure math, no truth, no policy).
// Shared by the SIM terrain author (engine contact + survey rasterize the
// same function, so they cannot disagree) — the shape both sides evaluate
// from authored numbers. The ROBOT never evaluates this: it estimates its
// own roughness belief from IMU motion evidence (T3), never from the
// author's amplitude. Wavelengths 1.35/0.95 m with a sine edge fade, so
// relief joins the base plane continuously at patch borders.

#include <algorithm>
#include <cmath>

#include "robot/core/geometry.hpp"

namespace lost_signal::robot::core {

struct ReliefSample {
  double height_m = 0.0;
  double gradient_x = 0.0;
  double gradient_y = 0.0;
};

inline ReliefSample rough_relief(double roughness_m, double width_m,
                                 double depth_m, double local_x,
                                 double local_y) {
  ReliefSample out;
  const double width = std::max(0.1, width_m);
  const double depth = std::max(0.1, depth_m);
  if (!(roughness_m > 0.0) || local_x < 0.0 || local_x > width ||
      local_y < 0.0 || local_y > depth) {
    return out;
  }
  constexpr double kWavelengthXM = 1.35;
  constexpr double kWavelengthYM = 0.95;
  const double u = std::clamp(local_x / width, 0.0, 1.0);
  const double v = std::clamp(local_y / depth, 0.0, 1.0);
  const double fade = std::sin(kPi * u) * std::sin(kPi * v);
  const double kx = 2.0 * kPi / kWavelengthXM;
  const double ky = 2.0 * kPi / kWavelengthYM;
  const double wave_x = std::sin(kx * local_x);
  const double wave_y = std::sin(ky * local_y);
  out.height_m = roughness_m * fade * wave_x * wave_y;
  out.gradient_x =
      roughness_m *
      (kPi * std::cos(kPi * u) / width * std::sin(kPi * v) * wave_x * wave_y +
       fade * kx * std::cos(kx * local_x) * wave_y);
  out.gradient_y =
      roughness_m *
      (kPi * std::sin(kPi * u) * std::cos(kPi * v) / depth * wave_x * wave_y +
       fade * ky * std::cos(ky * local_y) * wave_x);
  return out;
}

}  // namespace lost_signal::robot::core
