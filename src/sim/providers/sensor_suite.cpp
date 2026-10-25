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
 return poll_envelope(true, true, true, out);
}

bool SimSensorSuite::poll_envelope(bool sample_scan, bool sample_motion,
 bool sample_imu, robot::iface::ObservationFrame& out) {
  if (world_ == nullptr) return false;
 bool fresh_motion = false;
 bool fresh_imu = false;
 bool fresh_scan = false;
 if (sample_motion) {
 fresh_motion = motion_.poll_feedback(motion_cache_);
 if (fresh_motion) have_motion_ = true;
 }
 if (sample_imu) {
 fresh_imu = imu_.poll(imu_cache_);
 if (fresh_imu) have_imu_ = true;
 }
 if (sample_scan) {
 robot::iface::Lidar2DScan scan;
 if (composite_scan_) {
    // Composite front end: same message, same layout, multi-line source.
 std::vector<float> ranges, intensities;
 std::vector<float> offsets;
    if (!lidar3d_.poll_composite(ranges, intensities, true, &offsets)) {
      return false;
    }
    scan.src = lidar3d_.descriptor().endpoint_id;
    scan.frm = robot::core::Frame::kLidarLink;
    scan.seq = ++composite_seq_;
    if (scan.seq == 0) scan.seq = ++composite_seq_;  // never zero
 scan.stamp = world_->sim_time_ns();
    scan.ranges_m = std::move(ranges);
    scan.intensities = std::move(intensities);
    scan.time_offsets_s = std::move(offsets);
    scan.angle_min_rad = -static_cast<float>(M_PI);
    scan.angle_increment_rad =
        static_cast<float>(2.0 * M_PI / scan.ranges_m.size());
  } else {
    if (!lidar_.poll(scan, true)) return false;
  }
 scan_cache_ = std::move(scan);
 fresh_scan = true;
 have_scan_ = true;
 }
 if (!fresh_motion && !fresh_imu && !fresh_scan) return false;

 // One envelope remains the transport unit, but each populated section is a
 // newly acquired typed measurement. ROBOT owns the last-value caches used by
 // decisions and actuator feedback; the provider never republishes a stale
 // section under a fresh transport sequence.
 robot::iface::MotionFeedback fb = motion_cache_;
 if (!have_motion_) {
 fb.source = motion_.descriptor().endpoint_id;
 fb.frame = robot::core::Frame::kBaseLink;
 fb.stamp_ns = world_->sim_time_ns();
 }
 out = robot::iface::make_envelope(fb);
 out.sequence = ++envelope_seq_;
 if (out.sequence == 0) out.sequence = ++envelope_seq_;
 out.stamp_ns = robot::core::kInvalidTime;
 if (fresh_motion && have_motion_) out.stamp_ns = motion_cache_.stamp_ns;
 if (fresh_imu && have_imu_)
 out.stamp_ns = std::max(out.stamp_ns, imu_cache_.stamp);
 if (fresh_scan && have_scan_)
 out.stamp_ns = std::max(out.stamp_ns, scan_cache_.stamp);
 out.odometry_valid = fresh_motion && motion_cache_.odometry_valid;
 if (!fresh_motion) {
 out.motion_header = {};
 out.drive_count = 0;
 out.steer_count = 0;
 out.drive_radps = {};
 out.steer_rad = {};
 out.steer_vel_radps = {};
 }
 if (fresh_scan && !robot::iface::set_scan(out, scan_cache_)) return false;
 if (!fresh_scan) {
 out.ranges_m.clear();
 out.intensities.clear();
 out.time_offsets_s.clear();
 out.scan_valid = false;
 out.intensity_valid = false;
 out.time_offset_valid = false;
 }
 // A real lidar cannot provide an actor-free rendering of the same frame.
 // Dynamic/static classification belongs to ROBOT perception and temporal
 // tracking; SIM must not publish a truth-side reference scan.
 out.static_ranges_m.clear();
 out.scan_static_valid = false;
 if (fresh_imu && have_imu_) {
 if (!robot::iface::set_imu(out, imu_cache_)) return false;
 } else {
 out.imu_valid = false;
 }
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
