#pragma once

// ROBOT-owned scan deskew (pure functions over measurements, never SIM
// truth). A spinning/translating scanner acquires each beam at a different
// pose; the provider stamps every beam with its acquisition offset
// (<= 0, scan-end frame). Consumers re-express each endpoint as if taken
// at the stamp pose using the MEASURED body twist (estimate forward +
// yaw rate) — first-order rigid correction, master's rolling-shutter form
// without its framework:
//
//   p_end = R(-w*dt) * (p_acq - [v*dt, 0]),  dt = -toff >= 0.
//
// Empty/absent offsets read every beam at the stamp pose (frozen legacy,
// today's behavior exactly). Lateral body motion has no estimate channel,
// so only forward + yaw correct (documented, not silent).

#include <cmath>
#include <cstddef>
#include <vector>

#include "robot/core/geometry.hpp"  // core::Vec2, core::kPi

namespace lost_signal::robot::core {

struct DeskewTwist {
  double forward_mps = 0.0;
  double yaw_rate_radps = 0.0;
};

// One beam endpoint re-expressed in the scan-end sensor frame.
inline Vec2 deskew_beam(double range_m, double beam_angle_rad,
                              double time_offset_s, DeskewTwist twist) {
  const double px = range_m * std::cos(beam_angle_rad);
  const double py = range_m * std::sin(beam_angle_rad);
  const double dt_s = -time_offset_s;
  if (!(dt_s > 0.0) || !(range_m > 0.0) || !std::isfinite(dt_s) ||
      !std::isfinite(twist.forward_mps) ||
      !std::isfinite(twist.yaw_rate_radps)) {
    return Vec2{px, py};
  }
  // Acquisition frame rotated by -w*dt relative to the end frame; the
  // translation completes in the END frame (provider convention: the
  // displacement rotates by the stamp yaw). Exact inverse of generation.
  const double a = -twist.yaw_rate_radps * dt_s;
  const double c = std::cos(a), s = std::sin(a);
  return Vec2{c * px - s * py - twist.forward_mps * dt_s,
              s * px + c * py};
}

// Beam angle for the canonical full-rotation layout (angle_min -pi, even
// spacing, forward at index n/2 — the provider contract).
inline double beam_angle(std::size_t i, std::size_t n) {
  return -kPi + 2.0 * kPi * static_cast<double>(i) /
                           static_cast<double>(n);
}

}  // namespace lost_signal::robot::core
