#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

#include "robot/iface/conformance.hpp"
#include "robot/iface/envelope.hpp"
#include "robot/providers/real_lidar2d.hpp"
#include "sim/providers/sensor_suite.hpp"
#include "sim/world/world.hpp"

namespace {
lost_signal::robot::iface::ObservationFrame native_frame(
    lost_signal::robot::providers::Real2DLidarProvider& lidar,
    lost_signal::robot::core::MonotonicNs stamp_ns) {
  using namespace lost_signal::robot;
  lidar.feed(stamp_ns);
  iface::Lidar2DScan scan;
  assert(lidar.poll(scan));
  iface::ObservationFrame o;
  o.sequence = scan.seq;
  o.stamp_ns = scan.stamp;
  o.source = scan.src;
  o.frame = core::Frame::kLidarLink;
  assert(iface::set_scan(o, scan));
  return o;
}
}  // namespace

int main() {
  using namespace lost_signal::robot;

  // 1. Native provider stream passes: ordered, fresh, on-rate, live.
  {
    providers::Real2DLidarProvider lidar("ro1.lidar", std::vector<float>(72, 5.0f),
                                         0.087f);
    iface::ProviderConformanceMonitor monitor;
    monitor.expect(lidar.descriptor(), 50.0);  // 50 Hz nominal
    for (int i = 1; i <= 20; ++i) {
      core::MonotonicNs t = i * 20'000'000LL;
      monitor.observe(native_frame(lidar, t), t);
    }
    monitor.check_liveness(20 * 20'000'000LL);
    iface::ProviderConformanceReport rep = monitor.report(20 * 20'000'000LL);
    assert(rep.passed);
    assert(rep.frames == 20);
    assert(std::abs(rep.observed_rate_hz.at("ro1.lidar") - 50.0) < 5.0);
  }

  // 2. Violations are caught with rule names, never thrown.
  {
    iface::ProviderConformanceMonitor monitor;
    monitor.expect({"bad.lidar", iface::EndpointRole::kLidar,
                    iface::ProviderBackend::kNative, "lidar_link"},
                   50.0);
    iface::ObservationFrame o;
    o.sequence = 1;
    o.stamp_ns = 20'000'000;
    o.source = "bad.lidar";
    o.frame = core::Frame::kLidarLink;
    o.scan_valid = true;
    o.ranges_m.assign(8, 5.0f);
    monitor.observe(o, 20'000'000);
    // Stale sequence.
    monitor.observe(o, 40'000'000);
    // Ancient sample (freshness).
    iface::ObservationFrame old = o;
    old.sequence = 2;
    old.stamp_ns = 1;
    monitor.observe(old, 1'000'000'000);
    // Map-frame pose without opt-in (localization contract crossing).
    iface::ObservationFrame abs = o;
    abs.sequence = 3;
    abs.stamp_ns = 1'020'000'000;
    abs.frame = core::Frame::kMap;
    monitor.observe(abs, 1'020'000'000);
    iface::ProviderConformanceReport rep = monitor.report(1'020'000'000);
    assert(!rep.passed);
    bool saw_order = false, saw_fresh = false, saw_pose = false;
    for (const auto& f : rep.failures) {
      if (f.rule == "ordering") saw_order = true;
      if (f.rule == "freshness") saw_fresh = true;
      if (f.rule == "absolute_pose") saw_pose = true;
    }
    assert(saw_order && saw_fresh && saw_pose);
    // Silence: no frames for 2 s.
    monitor.check_liveness(4'000'000'000LL);
    iface::ProviderConformanceReport rep2 = monitor.report(4'000'000'000LL);
    bool saw_silence = false;
    for (const auto& f : rep2.failures) {
      if (f.rule == "silence") saw_silence = true;
    }
    assert(saw_silence);
  }

  // A future-dated acquisition is a freshness fault even when its sequence
 // and frame identity are otherwise valid.
 {
 iface::ProviderConformanceMonitor monitor;
 monitor.expect({"future.imu", iface::EndpointRole::kImu,
 iface::ProviderBackend::kNative, "imu_link"});
 iface::ObservationFrame observation;
 observation.sequence = 1;
 observation.stamp_ns = 100'000'000;
 observation.source = "future.imu";
 observation.frame = core::Frame::kImuLink;
 observation.imu_valid = true;
 monitor.observe(observation, 1'000'000);
 const iface::ProviderConformanceReport report = monitor.report(1'000'000);
 assert(!report.passed);
 bool saw_future = false;
 for (const auto& failure : report.failures) {
 if (failure.rule == "freshness" &&
 failure.detail.find("future") != std::string::npos) {
 saw_future = true;
 }
 }
 assert(saw_future);
 }

 // 3. Sim suite stream passes the same gate (one monitor for all
  // backends: sim, replay, native).
  {
    ::lost_signal::sim::world::World world;
    ::lost_signal::sim::providers::SimSensorSuite suite(&world);
    iface::ProviderConformanceMonitor monitor;
    monitor.expect({"sim.motion", iface::EndpointRole::kEncoder,
                    iface::ProviderBackend::kSim, "base_link"},
                   50.0);
 monitor.expect({"sim.lidar2d", iface::EndpointRole::kLidar,
 iface::ProviderBackend::kSim, "lidar_link"},
 50.0);
 monitor.expect({"sim.imu", iface::EndpointRole::kImu,
 iface::ProviderBackend::kSim, "imu_link"},
 50.0);
    for (int i = 0; i < 12; ++i) {
      world.step(0.02);
      iface::ObservationFrame obs;
      assert(suite.poll_envelope(obs));
      // Suite time starts at step 1 (20 ms); observe at sample time.
      monitor.observe(obs, obs.stamp_ns);
    }
    monitor.check_liveness(12 * 20'000'000LL);
    iface::ProviderConformanceReport rep = monitor.report(12 * 20'000'000LL);
    assert(rep.passed);
 assert(rep.frames == 12 && rep.measurements == 36);
 assert(rep.observed_rate_hz.count("sim.motion") == 1);
 assert(rep.observed_rate_hz.count("sim.lidar2d") == 1);
 assert(rep.observed_rate_hz.count("sim.imu") == 1);
  }

  std::printf("conformance ok\n");
  return 0;
}
