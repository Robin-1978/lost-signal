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
  out.time_offsets_s.assign(static_cast<std::size_t>(beam_count_), 0.0f);
  out.angle_min_rad = -static_cast<float>(M_PI);
  out.angle_increment_rad =
      static_cast<float>(2.0 * M_PI / beam_count_);
  // True 3D sensor pose per beam: shared shutter discipline (body +
  // mount); RPY follows REP-103 (R = Rz(yaw) * Ry(pitch) * Rx(roll)).
  const detail::ShutterBody body{t.body_x_m,
                                t.body_y_m,
                                t.body_z_m,
                                t.body_roll_rad,
                                t.body_pitch_rad,
                                t.body_yaw_rad,
                                t.yaw_rate_radps,
                                t.forward_mps,
                                t.lateral_mps};
  for (int i = 0; i < beam_count_; ++i) {
    const double a = -M_PI + 2.0 * M_PI * i / beam_count_;
    // Rolling shutter: beam 0 oldest, stamp = scan end.
    const double toff =
        -scan_period_s_ + scan_period_s_ * i / beam_count_;
    out.time_offsets_s[static_cast<std::size_t>(i)] =
        static_cast<float>(toff);
    const detail::AcquisitionPose pose =
        detail::acquisition_pose(body, mount_xyz_m_, mount_rpy_rad_, toff);
    const double ox = pose.ox, oy = pose.oy, oz = pose.oz;
    const robot::core::Vec3 d =
        robot::core::rotate_vector(pose.sensor_q, {std::cos(a), std::sin(a),
                                                   0.0});
    double dist = max_range_m_;
    bool engine_hit = false;
    double hit = 0.0;
    if (world_->raycast(ox, oy, oz, d.x, d.y, d.z, max_range_m_, hit)) {
      dist = hit;
      engine_hit = true;
    }
    bool actor_hit = false;
    double win_ax = 0.0, win_ay = 0.0, win_ah = 0.0;  // winning cylinder
    if (include_actors) {
      double adist = max_range_m_;
      if (detail::actor_beam_hit(world_->actors(), ox, oy, oz, d.x, d.y, d.z,
                                 toff, max_range_m_, adist, win_ax, win_ay,
                                 win_ah) &&
          adist < dist) {
        dist = adist;
        actor_hit = true;
      }
    }
    // Device observation model (SIM-side synthesis): an ideal hit passes
    // through bias/white-noise/quantization/dropout/near-field; a driven
    // no-return publishes miss/max-range, never a clamped value. Misses
    // stay absent. The exact default model leaves dist untouched.
    // Intensity synthesizes at the ideal surface (device range noise never
    // alters return strength — same layering as master's sensor model).
    const bool missed = !engine_hit && !actor_hit;
    const double hx = ox + d.x * dist;
    const double hy = oy + d.y * dist;
    const double hz = oz + d.z * dist;
    const float intensity = detail::return_intensity(
        *world_, hx, hy, hz, dist, actor_hit, missed, d.x, d.y, d.z, win_ax,
        win_ay, win_ah);
    if (!missed) {
      double measured = dist;
      if (device_range(dist, max_range_m_, device_, rng_, measured)) {
        dist = measured;
      } else {
        out.ranges_m[static_cast<std::size_t>(i)] = max_range_m_;
        out.intensities[static_cast<std::size_t>(i)] = 0.0f;
        continue;
      }
    }
    out.ranges_m[static_cast<std::size_t>(i)] = static_cast<float>(dist);
    out.intensities[static_cast<std::size_t>(i)] = intensity;
  }
  return out.valid();
}

}  // namespace lost_signal::sim::providers
