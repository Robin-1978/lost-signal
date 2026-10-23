#include "sim/providers/lidar2d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "sim/providers/raycast_util.hpp"
#include "sim/providers/reflectivity.hpp"
#include "sim/world/world.hpp"

namespace lost_signal::sim::providers {

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

void Sim2DLidarProvider::set_beams(int beam_count) {
  if (beam_count > 0) beam_count_ = beam_count;
}

void Sim2DLidarProvider::set_max_range(float max_range_m) {
  if (std::isfinite(max_range_m) && max_range_m > 0.0f) {
    max_range_m_ = max_range_m;
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
  out.intensities.assign(static_cast<std::size_t>(beam_count_), 0.0f);
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
    bool engine_hit = false;
    double hit = 0.0;
    if (world_->raycast(ox, oy, oz, d.x, d.y, d.z, max_range_m_, hit)) {
      dist = hit;
      engine_hit = true;
    }
    bool actor_hit = false;
    if (include_actors) {
      for (const auto& actor : world_->actors()) {
        const double ad = detail::ray_actor_cylinder(
            ox, oy, oz, d.x, d.y, d.z, actor.x, actor.y, actor.radius_m,
            actor.height_m, max_range_m_);
        if (ad < dist) {
          dist = ad;
          actor_hit = true;
        }
      }
    }
    out.ranges_m[static_cast<std::size_t>(i)] = static_cast<float>(dist);
    const bool missed = !engine_hit && !actor_hit;
    const double hx = ox + d.x * dist;
    const double hy = oy + d.y * dist;
    const double hz = oz + d.z * dist;
    out.intensities[static_cast<std::size_t>(i)] =
        detail::return_intensity(*world_, hx, hy, hz, dist, actor_hit,
                                 missed);
  }
  return out.valid();
}

}  // namespace lost_signal::sim::providers
