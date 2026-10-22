#include "sim/providers/lidar2d.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "sim/world/world.hpp"

namespace lost_signal::sim::providers {

namespace {

// Slab ray vs axis-aligned box. Returns distance or +inf on miss.
double ray_box(double ox, double oy, double dx, double dy,
               const world::StaticBox& b) {
  double tmin = 0.0, tmax = std::numeric_limits<double>::infinity();
  const double bounds[4] = {b.x0, b.x1, b.y0, b.y1};
  const double o[2] = {ox, oy}, d[2] = {dx, dy};
  for (int axis = 0; axis < 2; ++axis) {
    if (std::abs(d[axis]) < 1e-12) {
      if (o[axis] < bounds[axis * 2] || o[axis] > bounds[axis * 2 + 1]) {
        return std::numeric_limits<double>::infinity();
      }
    } else {
      double t0 = (bounds[axis * 2] - o[axis]) / d[axis];
      double t1 = (bounds[axis * 2 + 1] - o[axis]) / d[axis];
      if (t0 > t1) std::swap(t0, t1);
      tmin = std::max(tmin, t0);
      tmax = std::min(tmax, t1);
      if (tmin > tmax) return std::numeric_limits<double>::infinity();
    }
  }
  return tmin;
}

// Ray vs circle. Returns nearest positive distance or +inf.
double ray_circle(double ox, double oy, double dx, double dy, double cx,
                  double cy, double r) {
  const double ocx = ox - cx, ocy = oy - cy;
  const double b = ocx * dx + ocy * dy;
  const double c = ocx * ocx + ocy * ocy - r * r;
  const double disc = b * b - c;
  if (disc < 0.0) return std::numeric_limits<double>::infinity();
  const double t = -b - std::sqrt(disc);
  return t > 0.0 ? t : std::numeric_limits<double>::infinity();
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

bool Sim2DLidarProvider::poll(robot::iface::Lidar2DScan& out,
                              bool include_actors) {
  if (world_ == nullptr || beam_count_ <= 0) return false;
  seq_ += 1;
  out.seq = seq_;
  out.stamp = static_cast<robot::core::MonotonicNs>(world_->truth().step *
                                                    20'000'000);
  out.src = desc_.endpoint_id;
  out.frm = robot::core::Frame::kLidarLink;
  out.ranges_m.assign(static_cast<std::size_t>(beam_count_), max_range_m_);
  out.angle_min_rad = -static_cast<float>(M_PI);
  out.angle_increment_rad =
      static_cast<float>(2.0 * M_PI / beam_count_);
  const double ox = world_->truth().body_x_m;
  const double oy = world_->truth().body_y_m;
  const double base_yaw = world_->truth().body_yaw_rad;
  for (int i = 0; i < beam_count_; ++i) {
    const double angle =
        -M_PI + 2.0 * M_PI * i / beam_count_ + base_yaw;
    const double dx = std::cos(angle), dy = std::sin(angle);
    double t = max_range_m_;
    for (const auto& b : world_->static_boxes()) {
      t = std::min(t, ray_box(ox, oy, dx, dy, b));
    }
    if (include_actors) {
      for (const auto& a : world_->actors()) {
        t = std::min(t, ray_circle(ox, oy, dx, dy, a.x, a.y, a.radius_m));
      }
    }
    out.ranges_m[static_cast<std::size_t>(i)] = static_cast<float>(t);
  }
  return out.valid();
}

}  // namespace lost_signal::sim::providers
