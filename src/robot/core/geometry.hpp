#pragma once

#include <cmath>

namespace lost_signal::robot::core {

// Canonical 2D geometry primitive. All autonomy math (rollouts, scoring,
// scan projection, map queries) shares this type; there is exactly one.
inline constexpr double kPi = 3.14159265358979323846;

struct Vec2 {
  double x{0.0};
  double y{0.0};
};

inline double clamp(double v, double lo, double hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

inline double wrap_angle(double a) {
  while (a > kPi) a -= 2.0 * kPi;
  while (a < -kPi) a += 2.0 * kPi;
  return a;
}

}  // namespace lost_signal::robot::core
