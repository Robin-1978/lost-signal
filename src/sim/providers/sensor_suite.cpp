#include "sim/providers/sensor_suite.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

namespace lost_signal::sim::providers {

namespace {

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

}  // namespace

SimSensorSuite::SimSensorSuite(world::World* world)
    : world_(world),
      motion_(world),
      lidar_(world, "sim.lidar2d", 72, 10.0f),
      lidar3d_(world, "sim.lidar3d", 16, 72, 10.0f),
      imu_(world) {}

void SimSensorSuite::register_all(robot::app::IRobot& app) {
  app.register_provider(motion_.descriptor());
  app.register_provider(lidar_.descriptor());
  app.register_provider(lidar3d_.descriptor());
  app.register_provider(imu_.descriptor());
}

bool SimSensorSuite::poll_envelope(robot::iface::ObservationFrame& out) {
  if (world_ == nullptr) return false;
  // Pure composition: typed messages from owned providers, packed through
  // the canonical robot-owned helpers. The suite adds no sensing of its
  // own; the static reference is a second raycast pass without actors.
  robot::iface::MotionFeedback fb;
  if (!motion_.poll_feedback(fb)) return false;
  robot::iface::ImuMessage imu;
  if (!imu_.poll(imu)) return false;
  robot::iface::Lidar2DScan scan;
  robot::iface::Lidar2DScan reference;
  if (composite_scan_) {
    // Composite front end: same message, same layout, multi-line source.
    std::vector<float> ranges, ref, intensities, ref_intensities;
    if (!lidar3d_.poll_composite(ranges, intensities, true)) return false;
    if (!lidar3d_.poll_composite(ref, ref_intensities, false)) return false;
    scan.src = lidar3d_.descriptor().endpoint_id;
    scan.frm = robot::core::Frame::kLidarLink;
    scan.seq = ++composite_seq_;
    if (scan.seq == 0) scan.seq = ++composite_seq_;  // never zero
    scan.stamp = static_cast<robot::core::MonotonicNs>(world_->truth().step *
                                                       20'000'000);
    scan.ranges_m = std::move(ranges);
    scan.intensities = std::move(intensities);
    scan.angle_min_rad = -static_cast<float>(M_PI);
    scan.angle_increment_rad =
        static_cast<float>(2.0 * M_PI / scan.ranges_m.size());
    reference = scan;
    reference.ranges_m = std::move(ref);
    reference.src = lidar3d_.descriptor().endpoint_id;
  } else {
    if (!lidar_.poll(scan, true)) return false;
    if (!lidar_.poll(reference, false)) return false;
  }

  out = robot::iface::make_envelope(fb);
  if (!robot::iface::set_scan(out, scan)) return false;
  out.static_ranges_m = reference.ranges_m;
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
  imu_.set_realism(config);  // validated device-side, fail closed there
}

void SimSensorSuite::set_mounts(
    const std::vector<robot::platform::SensorMount>& sensors) {
  for (const auto& s : sensors) {
    if (s.link == lidar_.descriptor().frame_id) {
      lidar_.set_mount(s.xyz_m, s.rpy_rad);
      lidar3d_.set_mount(s.xyz_m, s.rpy_rad);
    } else if (s.link == imu_.descriptor().frame_id) {
      imu_.set_mount(s.xyz_m, s.rpy_rad);
    }
    // Motion needs no mount (joint-space + base twist, documented on the
    // endpoint); anything else has no provider yet and is ignored.
  }
}

void SimSensorSuite::consume(const robot::iface::ActuatorCommand& cmd) {
  motion_.consume_command(cmd);
}

}  // namespace lost_signal::sim::providers
