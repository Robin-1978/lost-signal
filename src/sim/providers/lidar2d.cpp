#include "sim/providers/lidar2d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "sim/world/world.hpp"

namespace lost_signal::sim::providers {

namespace {

// Upright finite cylinder for actors (ground-based z in [0, height]):
// curved side + top cap. Ported from main's raycast_upright_finite_cylinder
// (center-based there, ground-based here — actors stand on the floor).
// Returns nearest positive distance or +inf.
double ray_actor_cylinder(double ox, double oy, double oz, double dx,
                          double dy, double dz, double ax, double ay,
                          double radius_m, double height_m, double max_m) {
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

}  // namespace

Sim2DLidarProvider::Sim2DLidarProvider(world::World* world,
                                       const char* endpoint_id, int beam_count,
                                       float max_range_m)
    : world_(world),
      beam_count_(beam_count > 0 ? beam_count : 72),
      max_range_m_(max_range_m > 0.0f ? max_range_m : 10.0f) {
  desc_.endpoint_id = endpoint_id != nullptr ? endpoint_id : "sim.lidar2d";
  desc_.role = robot::iface::EndpointRole::kLidar;
  desc_.backend = robot::iface::ProviderBackend::kSim;
  desc_.frame_id = "lidar_link";
}

const robot::iface::EndpointDescriptor& Sim2DLidarProvider::descriptor()
    const {
  return desc_;
}

void Sim2DLidarProvider::set_mount(const double xyz_m[3],
                                   const double rpy_rad[3]) {
  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(xyz_m[i]) || !std::isfinite(rpy_rad[i])) {
      return;  // fail closed: keep previous mount
    }
  }
  for (int i = 0; i < 3; ++i) {
    mount_xyz_m_[i] = xyz_m[i];
    mount_rpy_rad_[i] = rpy_rad[i];
  }
}

bool Sim2DLidarProvider::poll(robot::iface::Lidar2DScan& out,
                              bool include_actors) {
  if (world_ == nullptr || beam_count_ <= 0) return false;
  const world::WorldTruth& t = world_->truth();
  seq_ += 1;
  out.seq = seq_;
  out.stamp = static_cast<robot::core::MonotonicNs>(t.step * 20'000'000);
  out.src = desc_.endpoint_id;
  out.frm = robot::core::Frame::kLidarLink;
  out.ranges_m.assign(static_cast<std::size_t>(beam_count_), max_range_m_);
  out.angle_min_rad = -static_cast<float>(M_PI);
  out.angle_increment_rad =
      static_cast<float>(2.0 * M_PI / beam_count_);
  // True 3D sensor pose: body position + attitude (engine roll/pitch/yaw)
  // composed with the mount extrinsics. RPY follows REP-103
  // (R = Rz(yaw) * Ry(pitch) * Rx(roll)).
  const robot::core::Quaternion body_q = robot::core::quaternion_from_rpy(
      t.body_roll_rad, t.body_pitch_rad, t.body_yaw_rad);
  const robot::core::Vec3 m{mount_xyz_m_[0], mount_xyz_m_[1],
                            mount_xyz_m_[2]};
  const robot::core::Vec3 off = robot::core::rotate_vector(body_q, m);
  const double ox = t.body_x_m + off.x;
  const double oy = t.body_y_m + off.y;
  const double oz = t.body_z_m + off.z;
  const robot::core::Quaternion sensor_q = robot::core::quaternion_multiply(
      body_q, robot::core::quaternion_from_rpy(
                  mount_rpy_rad_[0], mount_rpy_rad_[1], mount_rpy_rad_[2]));
  for (int i = 0; i < beam_count_; ++i) {
    const double a = -M_PI + 2.0 * M_PI * i / beam_count_;
    const robot::core::Vec3 d =
        robot::core::rotate_vector(sensor_q, {std::cos(a), std::sin(a), 0.0});
    double dist = max_range_m_;
    double hit = 0.0;
    if (world_->raycast(ox, oy, oz, d.x, d.y, d.z, max_range_m_, hit)) {
      dist = hit;
    }
    if (include_actors) {
      for (const auto& actor : world_->actors()) {
        dist = std::min(dist,
                        ray_actor_cylinder(ox, oy, oz, d.x, d.y, d.z, actor.x,
                                           actor.y, actor.radius_m,
                                           actor.height_m, max_range_m_));
      }
    }
    out.ranges_m[static_cast<std::size_t>(i)] = static_cast<float>(dist);
  }
  return out.valid();
}

}  // namespace lost_signal::sim::providers
