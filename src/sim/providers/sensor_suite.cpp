#include "sim/providers/sensor_suite.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

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

SimSensorSuite::SimSensorSuite(world::World* world)
    : world_(world), motion_(world), lidar_("sim.lidar2d", 72, 10.0f) {}

void SimSensorSuite::register_all(robot::app::IRobot& app) {
  app.register_provider(motion_.descriptor());
  app.register_provider(lidar_.descriptor());
  app.register_provider(imu_.descriptor());
}

bool SimSensorSuite::poll_envelope(robot::iface::ObservationFrame& out) {
  if (world_ == nullptr) return false;
  robot::core::MonotonicNs stamp = static_cast<robot::core::MonotonicNs>(
      world_->truth().step * 20'000'000);

  // Dual raycast from the same geometry: measured sees static + actors,
  // static reference sees static geometry only. Their difference is the
  // foreground the perception layer classifies; nothing else crosses.
  static constexpr int kBeams = 72;
  static constexpr float kMaxRange = 10.0f;
  const double ox = world_->truth().body_x_m;
  const double oy = world_->truth().body_y_m;
  std::vector<float> ranges(kBeams), reference(kBeams);
  for (int i = 0; i < kBeams; ++i) {
    const double angle = -M_PI + 2.0 * M_PI * i / kBeams;
    const double dx = std::cos(angle), dy = std::sin(angle);
    double t_static = kMaxRange, t_all = kMaxRange;
    for (const auto& b : world_->static_boxes()) {
      const double t = ray_box(ox, oy, dx, dy, b);
      t_static = std::min(t_static, t);
      t_all = std::min(t_all, t);
    }
    for (const auto& a : world_->actors()) {
      t_all = std::min(t_all, ray_circle(ox, oy, dx, dy, a.x, a.y, a.radius_m));
    }
    ranges[i] = static_cast<float>(t_all);
    reference[i] = static_cast<float>(t_static);
  }

  robot::iface::MotionFeedback fb;
  if (!motion_.poll_feedback(fb)) return false;
  fb.stamp_ns = stamp;
  robot::iface::ImuMessage imu;
  imu.seq = fb.sequence;
  imu.stamp = stamp;
  imu.src = imu_.descriptor().endpoint_id;
  imu.frm = robot::core::Frame::kImuLink;
  imu.gyro_x_radps = 0.0;
  imu.gyro_y_radps = 0.0;
  imu.gyro_z_radps = 0.0;
  imu.accel_x_mps2 = 0.0;
  imu.accel_y_mps2 = 0.0;
  imu.accel_z_mps2 = 9.81;  // level: specific force opposes gravity

  out = robot::iface::make_envelope(fb);
  out.ranges_m = std::move(ranges);
  out.scan_valid = true;
  out.static_ranges_m = std::move(reference);
  out.scan_static_valid = true;
  if (!robot::iface::set_imu(out, imu)) return false;
  return robot::iface::valid_observation_frame(out);
}

void SimSensorSuite::consume(const robot::iface::MotionCommand& cmd) {
  motion_.consume_command(cmd);
}

}  // namespace lost_signal::sim::providers
