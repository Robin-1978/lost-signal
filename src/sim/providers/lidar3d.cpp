#include "sim/providers/lidar3d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "sim/providers/raycast_util.hpp"
#include "sim/providers/reflectivity.hpp"
#include "sim/world/world.hpp"

namespace lost_signal::sim::providers {

Sim3DLidarProvider::Sim3DLidarProvider(world::World* world,
                                       const char* endpoint_id, int ring_count,
                                       int azimuth_beams, float max_range_m)
    : world_(world),
      ring_count_(ring_count > 0 ? ring_count : 16),
      azimuth_beams_(azimuth_beams > 0 ? azimuth_beams : 72),
      max_range_m_(max_range_m > 0.0f ? max_range_m : 10.0f) {
  desc_.endpoint_id = endpoint_id != nullptr ? endpoint_id : "sim.lidar3d";
  desc_.role = robot::iface::EndpointRole::kLidar;
  desc_.backend = robot::iface::ProviderBackend::kSim;
  desc_.frame_id = "lidar_link";
}

const robot::iface::EndpointDescriptor& Sim3DLidarProvider::descriptor()
    const {
  return desc_;
}

void Sim3DLidarProvider::set_mount(const double xyz_m[3],
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

void Sim3DLidarProvider::set_rings(int ring_count) {
  if (ring_count > 0) ring_count_ = ring_count;
}

void Sim3DLidarProvider::set_azimuths(int azimuth_beams) {
  if (azimuth_beams > 0) azimuth_beams_ = azimuth_beams;
}

void Sim3DLidarProvider::set_vertical_fov(double min_elev_rad,
                                          double max_elev_rad) {
  if (!std::isfinite(min_elev_rad) || !std::isfinite(max_elev_rad)) return;
  if (!(min_elev_rad < max_elev_rad)) return;
  if (min_elev_rad < -1.5707963 || max_elev_rad > 1.5707963) return;
  min_elev_rad_ = min_elev_rad;
  max_elev_rad_ = max_elev_rad;
}

void Sim3DLidarProvider::set_max_range(float max_range_m) {
  if (std::isfinite(max_range_m) && max_range_m > 0.0f) {
    max_range_m_ = max_range_m;
  }
}

void Sim3DLidarProvider::set_scan_period(float scan_period_s) {
  if (std::isfinite(scan_period_s) && scan_period_s > 0.0f) {
    scan_period_s_ = scan_period_s;
  }
}

bool Sim3DLidarProvider::poll(robot::iface::Lidar3DCloud& out,
                              bool include_actors) {
  if (world_ == nullptr || ring_count_ <= 0 || azimuth_beams_ <= 0) {
    return false;
  }
  const world::WorldTruth& t = world_->truth();
  seq_ += 1;
  out.seq = seq_;
  out.stamp = static_cast<robot::core::MonotonicNs>(t.step * 20'000'000);
  out.src = desc_.endpoint_id;
  out.frm = robot::core::Frame::kLidarLink;
  out.ring_count = static_cast<std::uint16_t>(ring_count_);
  out.scan_period_s = scan_period_s_;
  out.points.clear();
  out.points.reserve(static_cast<std::size_t>(ring_count_) *
                     static_cast<std::size_t>(azimuth_beams_));
  // True 3D sensor pose: body position + attitude composed with the mount
  // (same math as the 2D producer; REP-103 RPY).
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
  // Azimuth-major: all rings per azimuth step. Ring elevations are uniform
  // over the FOV (a single ring sits at 0 elevation).
  for (int i = 0; i < azimuth_beams_; ++i) {
    const double az =
        -M_PI + 2.0 * M_PI * static_cast<double>(i) / azimuth_beams_;
    const float time_offset_s = static_cast<float>(
        -scan_period_s_ *
        (1.0 - static_cast<double>(i) / azimuth_beams_));
    for (int j = 0; j < ring_count_; ++j) {
      const double elev =
          (ring_count_ == 1)
              ? 0.0
              : min_elev_rad_ +
                    (max_elev_rad_ - min_elev_rad_) * j / (ring_count_ - 1);
      const double ce = std::cos(elev);
      const robot::core::Vec3 d = robot::core::rotate_vector(
          sensor_q, {ce * std::cos(az), ce * std::sin(az), std::sin(elev)});
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
      if (!(dist < max_range_m_)) continue;  // miss: dropped, never filled
      const double ce_az = std::cos(elev);
      robot::iface::Lidar3DPoint p;
      p.x_m = static_cast<float>(dist * ce_az * std::cos(az));
      p.y_m = static_cast<float>(dist * ce_az * std::sin(az));
      p.z_m = static_cast<float>(dist * std::sin(elev));
      p.intensity = detail::return_intensity(
          *world_, ox + d.x * dist, oy + d.y * dist, oz + d.z * dist, dist,
          actor_hit, !engine_hit && !actor_hit);
      p.ring = static_cast<std::uint16_t>(j);
      p.time_offset_s = time_offset_s;
      out.points.push_back(p);
    }
  }
  return out.valid();
}

bool Sim3DLidarProvider::poll_composite(std::vector<float>& ranges_m,
                                        bool include_actors) {
  std::vector<float> ignored;
  return poll_composite(ranges_m, ignored, include_actors);
}

bool Sim3DLidarProvider::poll_composite(std::vector<float>& ranges_m,
                                        std::vector<float>& intensities_m,
                                        bool include_actors) {
  if (world_ == nullptr || ring_count_ <= 0 || azimuth_beams_ <= 0) {
    return false;
  }
  const world::WorldTruth& t = world_->truth();
  ranges_m.assign(static_cast<std::size_t>(azimuth_beams_), max_range_m_);
  intensities_m.assign(static_cast<std::size_t>(azimuth_beams_), 0.0f);
  // Same sensor pose as poll(): body + attitude composed with the mount.
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
  for (int i = 0; i < azimuth_beams_; ++i) {
    const double az =
        -M_PI + 2.0 * M_PI * static_cast<double>(i) / azimuth_beams_;
    float best = max_range_m_;
    float best_intensity = 0.0f;
    for (int j = 0; j < ring_count_; ++j) {
      const double elev =
          (ring_count_ == 1)
              ? 0.0
              : min_elev_rad_ +
                    (max_elev_rad_ - min_elev_rad_) * j / (ring_count_ - 1);
      const double ce = std::cos(elev);
      const robot::core::Vec3 d = robot::core::rotate_vector(
          sensor_q, {ce * std::cos(az), ce * std::sin(az), std::sin(elev)});
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
      if (!(dist < max_range_m_)) continue;
      // Ground filter on the world hit height (d is unit): floor excluded.
      const double hit_z = oz + d.z * dist;
      if (hit_z < ground_cut_m_) continue;
      if (static_cast<float>(dist) < best) {
        best = static_cast<float>(dist);
        best_intensity = detail::return_intensity(
            *world_, ox + d.x * dist, oy + d.y * dist, hit_z, dist, actor_hit,
            !engine_hit && !actor_hit);
      }
    }
    ranges_m[static_cast<std::size_t>(i)] = best;
    intensities_m[static_cast<std::size_t>(i)] = best_intensity;
  }
  return true;
}

}  // namespace lost_signal::sim::providers
