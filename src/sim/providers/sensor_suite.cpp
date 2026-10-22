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

// Deterministic streams (splitmix64): explicit per-suite state, no global
// RNG, no platform variance. Same seed + call sequence = identical.
std::uint64_t splitmix64(std::uint64_t& state) {
  std::uint64_t z = (state += 0x9E3779B97F4A7C15ULL);
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
  return z ^ (z >> 31);
}

double rand01(std::uint64_t& state) {
  return static_cast<double>(splitmix64(state) >> 11) *
         (1.0 / 9007199254740992.0);
}

// Exact standard normal, fixed 2-draw consumption (Box-Muller, spare
// discarded). Deterministic per stream position by construction.
double randn(std::uint64_t& state) {
  const double u1 = std::max(rand01(state), 1e-300);
  const double u2 = rand01(state);
  return std::sqrt(-2.0 * std::log(u1)) *
         std::cos(2.0 * 3.14159265358979323846 * u2);
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
  // Bearings are ROBOT-frame (angle_min + i*increment, forward = beam 36):
  // the cast adds the true body yaw so a turning base scans the world,
  // not a frozen world-frame fan. Downstream projects with +est.yaw, which
  // inverts exactly this rotation; yaw ~ 0 missions never noticed.
  static constexpr int kBeams = 72;
  static constexpr float kMaxRange = 10.0f;
  const double ox = world_->truth().body_x_m;
  const double oy = world_->truth().body_y_m;
  const double base_yaw = world_->truth().body_yaw_rad;
  std::vector<float> ranges(kBeams), reference(kBeams);
  for (int i = 0; i < kBeams; ++i) {
    const double angle =
        -M_PI + 2.0 * M_PI * i / kBeams + base_yaw;
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
  // Yaw rate is truth here (S7 adds bounded noise/bias, seeded). It must be
  // truthful, not zero: the estimator's slip gate trusts the gyro over
  // wheel odometry on disagreement, so a hard zero would freeze the
  // heading estimate through every turn. Planar motion: no roll/pitch rate.
  imu.gyro_z_radps = world_->truth().yaw_rate_radps +
                     imu_realism_.gyro_bias_radps +
                     imu_realism_.gyro_noise_radps * randn(imu_rng_);
  imu.gyro_x_radps = imu_realism_.gyro_noise_radps * randn(imu_rng_);
  imu.gyro_y_radps = imu_realism_.gyro_noise_radps * randn(imu_rng_);
  imu.accel_x_mps2 = imu_realism_.accel_noise_mps2 * randn(imu_rng_);
  imu.accel_y_mps2 = imu_realism_.accel_noise_mps2 * randn(imu_rng_);
  imu.accel_z_mps2 =
      9.81 + imu_realism_.accel_noise_mps2 * randn(imu_rng_);
  // Level: specific force opposes gravity; noise rides on top when enabled.

  out = robot::iface::make_envelope(fb);
  out.ranges_m = std::move(ranges);
  out.scan_valid = true;
  // Layout matches the raycast loop below: angle_min + i * increment.
  out.scan_angle_min_rad = -M_PI;
  out.scan_angle_increment_rad = 2.0 * M_PI / kBeams;
  out.static_ranges_m = std::move(reference);
  out.scan_static_valid = true;
  if (!robot::iface::set_imu(out, imu)) return false;
  if (!robot::iface::valid_observation_frame(out)) return false;
  // S7 delivery shaping (default off = passthrough, byte-identical path).
  if (delivery_.latency_steps <= 0 && delivery_.drop_probability <= 0.0) {
    return true;
  }
  if (rand01(delivery_rng_) < delivery_.drop_probability) {
    return false;  // dropped: never delivers, existing no-data path
  }
  delivery_queue_.push_back(out);
  if (static_cast<int>(delivery_queue_.size()) <= delivery_.latency_steps) {
    return false;  // filling the hold-back: nothing due yet
  }
  out = delivery_queue_.front();
  delivery_queue_.pop_front();
  return true;
}

void SimSensorSuite::set_delivery(DeliveryConfig config) {
  if (config.latency_steps < 0 || !(config.drop_probability >= 0.0) ||
      !(config.drop_probability <= 1.0)) {
    return;  // fail closed: keep previous settings
  }
  delivery_ = config;
  delivery_queue_.clear();
  delivery_rng_ = config.seed ^ 0x9E3779B97F4A7C15ULL;
}

void SimSensorSuite::set_imu_realism(ImuRealismConfig config) {
  if (!(config.gyro_noise_radps >= 0.0) ||
      !std::isfinite(config.gyro_bias_radps) ||
      !(config.accel_noise_mps2 >= 0.0)) {
    return;  // fail closed: keep previous settings
  }
  imu_realism_ = config;
  imu_rng_ = config.seed ^ 0x9E3779B97F4A7C15ULL;
}

void SimSensorSuite::consume(const robot::iface::ActuatorCommand& cmd) {
  motion_.consume_command(cmd);
}

}  // namespace lost_signal::sim::providers
