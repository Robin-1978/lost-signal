#pragma once

// Shared SIM-side raycast helper (generation only, never ROBOT input).
// Upright finite cylinder for kinematic actors (ground-based z in
// [0, height]): curved side + top cap. Ported from main's
// raycast_upright_finite_cylinder (center-based there, ground-based here —
// actors stand on the floor). Returns nearest positive distance or +inf.

#include <algorithm>
#include <cmath>
#include <limits>

#include "robot/core/transform.hpp"

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

// Rolling-shutter acquisition pose (SIM-side generation, shared by the
// 2D/3D/composite producers so the three cannot disagree): body
// translation advances by the truth twist over the beam offset, yaw
// advances by the yaw rate; height/roll/pitch ride frozen at the stamp
// (master's stated limits, one doctrine). Sensor pose composes the live
// body position + attitude with the mount extrinsics (REP-103 RPY).
struct ShutterBody {
  double x_m = 0.0, y_m = 0.0, z_m = 0.0;
  double roll_rad = 0.0, pitch_rad = 0.0, yaw_rad = 0.0;
  double yaw_rate_radps = 0.0;
  double forward_mps = 0.0, lateral_mps = 0.0;
};

struct AcquisitionPose {
  double ox = 0.0, oy = 0.0, oz = 0.0;
  robot::core::Quaternion sensor_q{0.0, 0.0, 0.0, 1.0};
};

inline AcquisitionPose acquisition_pose(const ShutterBody& body,
                                        const double mount_xyz_m[3],
                                        const double mount_rpy_rad[3],
                                        double toff_s) {
  const robot::core::Quaternion body_q =
      robot::core::quaternion_from_rpy(body.roll_rad, body.pitch_rad,
                                       body.yaw_rad);
  const robot::core::Vec3 m{mount_xyz_m[0], mount_xyz_m[1], mount_xyz_m[2]};
  const robot::core::Vec3 off = robot::core::rotate_vector(body_q, m);
  const double yaw_i = body.yaw_rad + body.yaw_rate_radps * toff_s;
  const double cy = std::cos(body.yaw_rad), sy = std::sin(body.yaw_rad);
  AcquisitionPose out;
  out.ox = body.x_m + off.x +
           (body.forward_mps * cy - body.lateral_mps * sy) * toff_s;
  out.oy = body.y_m + off.y +
           (body.forward_mps * sy + body.lateral_mps * cy) * toff_s;
  out.oz = body.z_m + off.z;
  const robot::core::Quaternion body_qi =
      robot::core::quaternion_from_rpy(body.roll_rad, body.pitch_rad, yaw_i);
  out.sensor_q = robot::core::quaternion_multiply(
      body_qi, robot::core::quaternion_from_rpy(
                   mount_rpy_rad[0], mount_rpy_rad[1], mount_rpy_rad[2]));
  return out;
}

// Nearest kinematic-actor hit over one beam (actors propagate over the
// beam offset, same shutter discipline as the body). Strict less-than
// against the running distance, first wins ties — callers compare the
// returned distance against the engine distance with the same strict
// rule, so inlining or sharing never changes the winner. The winning
// (propagated) center + height feed the cylinder normal for incidence.
template <typename ActorRange>
inline bool actor_beam_hit(const ActorRange& actors, double ox, double oy,
                           double oz, double dx, double dy, double dz,
                           double toff_s, double max_m, double& dist_m,
                           double& cx_m, double& cy_m, double& h_m) {
  bool hit = false;
  dist_m = max_m;
  cx_m = 0.0;
  cy_m = 0.0;
  h_m = 0.0;
  for (const auto& actor : actors) {
    const double ad = ray_actor_cylinder(
        ox, oy, oz, dx, dy, dz, actor.x + actor.vx_mps * toff_s,
        actor.y + actor.vy_mps * toff_s, actor.radius_m, actor.height_m,
        max_m);
    if (ad < dist_m) {
      dist_m = ad;
      hit = true;
      cx_m = actor.x + actor.vx_mps * toff_s;
      cy_m = actor.y + actor.vy_mps * toff_s;
      h_m = actor.height_m;
    }
  }
  return hit;
}

}  // namespace lost_signal::sim::providers::detail
