#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

#include "robot/autonomy/coordinator.hpp"
#include "robot/autonomy/safety/safety.hpp"

namespace {
lost_signal::robot::iface::ObservationFrame fresh_obs(
    std::uint64_t seq, lost_signal::robot::core::MonotonicNs stamp,
    float range) {
  lost_signal::robot::iface::ObservationFrame o;
  o.sequence = seq;
  o.stamp_ns = stamp;
  o.source = "t.lidar";
  o.frame = lost_signal::robot::core::Frame::kLidarLink;
  o.odometry_valid = true;
  o.scan_valid = true;
  o.ranges_m = {range, range};
  return o;
}
}  // namespace

int main() {
  using namespace lost_signal::robot;
  using namespace lost_signal::robot::autonomy;

  // Nominal: fresh evidence, clear scan -> motion allowed through the gate.
  {
 SafetyMonitor mon;
    mon.reset();
    PerceptionResult perc;
    perc.scan_ok = true;
    iface::ChannelHealth ch{};
    SafetyVerdict v = mon.evaluate(EstimatedState{}, perc,
                                   fresh_obs(1, 1'000'000, 5.0f), ch,
                                   1'000'000);
    assert(v.state == SafetyState::kNominal && v.allow_motion);
    iface::MotionCommand cmd;
    cmd.space = iface::CommandSpace::kYawRate;
    cmd.stamp_ns = 1'000'000;
    cmd.expiry_ns = 2'000'000;
    cmd.forward_mps = 1.0;
    assert(mon.gate(cmd, v, 1'000'000).forward_mps == 1.0);
  }

  // Stale observation -> latched STOPPED; fresh evidence afterwards does NOT
  // auto-clear; only deliberate recovery does.
  {
    SafetyMonitor mon;
    mon.reset();
    PerceptionResult perc;
    perc.scan_ok = true;
    iface::ChannelHealth ch{};
    SafetyVerdict v = mon.evaluate(EstimatedState{}, perc,
                                   fresh_obs(1, 1'000'000, 5.0f), ch,
                                   1'000'000 + 600'000'000);
    assert(v.state == SafetyState::kStopped && !v.allow_motion);
    SafetyVerdict still = mon.evaluate(EstimatedState{}, perc,
                                       fresh_obs(2, 700'000'000, 5.0f), ch,
                                       700'000'000);
    assert(still.state == SafetyState::kStopped);
    iface::MotionCommand cmd = iface::MotionCommand::SafeStop(0);
    assert(mon.gate(cmd, still, 700'000'000).is_safe_stop());
    mon.clear_safety();
    SafetyVerdict ok = mon.evaluate(EstimatedState{}, perc,
                                    fresh_obs(3, 800'000'000, 5.0f), ch,
                                    800'000'000);
    assert(ok.state == SafetyState::kNominal);
  }

 // A fresh motion envelope cannot make an older cached LiDAR scan fresh.
 // Safety uses the scan section's acquisition time, not the aggregate time.
 {
 SafetyMonitor mon;
 mon.reset();
 PerceptionResult perc;
 perc.scan_ok = true;
 iface::ChannelHealth ch{};
 iface::ObservationFrame mixed = fresh_obs(9, 700'000'000, 5.0f);
 mixed.scan_header = {
 1, 1'000'000, "t.lidar", core::Frame::kLidarLink};
 mixed.motion_header = {
 9, 700'000'000, "t.odom", core::Frame::kBaseLink};
 const SafetyVerdict stale = mon.evaluate(
 EstimatedState{}, perc, mixed, ch, 700'000'000);
 assert(stale.state == SafetyState::kStopped);
 assert(stale.reason == "obs-stale");
 }

 // Near-field contact -> latched STOPPED even with fresh stamps.
  {
    SafetyMonitor mon;
    mon.reset();
    PerceptionResult perc;
    perc.scan_ok = true;
    iface::ChannelHealth ch{};
    SafetyVerdict v = mon.evaluate(EstimatedState{}, perc,
                                   fresh_obs(1, 1'000'000, 0.1f), ch,
                                   1'000'000);
    assert(v.state == SafetyState::kStopped);
  }

 // Contact is measured against the configured body footprint in base_link,
 // not against raw range from an offset lidar. A rear return 0.6 m from a
 // lidar mounted 0.25 m forward is only 0.35 m from the body center and must
 // latch for the default 0.4 m footprint.
 {
 SafetyMonitor mon;
 mon.reset();
 mon.set_lidar_mount({{0.25, 0.0, 0.0},
 core::quaternion_from_rpy(0.0, 0.0, 0.0)});
 iface::ObservationFrame o = fresh_obs(1, 1'000'000, 0.6f);
 o.ranges_m = {0.6f};
 o.scan_angle_min_rad = core::kPi;
 o.scan_angle_increment_rad = 1.0;
 PerceptionResult perc;
 perc.scan_ok = true;
 iface::ChannelHealth ch{};
 const SafetyVerdict v = mon.evaluate(
 EstimatedState{}, perc, o, ch, 1'000'000);
 assert(v.state == SafetyState::kStopped);
 assert(v.reason == "contact");
 }

 // Even when the current pose is clear, the independent final gate checks
 // the complete reaction + braking sweep against raw base-frame endpoints.
 {
 SafetyMonitor mon;
 mon.reset();
 iface::ObservationFrame o = fresh_obs(1, 1'000'000, 0.9f);
 o.ranges_m = {0.9f};
 o.scan_angle_min_rad = 0.0;
 o.scan_angle_increment_rad = 1.0;
 PerceptionResult perc;
 perc.scan_ok = true;
 iface::ChannelHealth ch{};
 const SafetyVerdict v = mon.evaluate(
 EstimatedState{}, perc, o, ch, 1'000'000);
 assert(v.state == SafetyState::kNominal);
 iface::MotionCommand cmd;
 cmd.space = iface::CommandSpace::kYawRate;
 cmd.stamp_ns = 1'000'000;
 cmd.expiry_ns = 201'000'000;
 cmd.forward_mps = 1.0;
 assert(mon.gate(cmd, v, 1'000'000).is_safe_stop());
 assert(mon.last_verdict().reason == "static-swept-proximity");
 }

 // Proximity is transient: a bounded command that increases raw footprint
 // clearance is admitted without consulting a map pose or simulator truth.
 {
 SafetyMonitor mon;
 mon.reset();
 iface::ObservationFrame o = fresh_obs(1, 1'000'000, 0.45f);
 o.ranges_m = {0.45f};
 o.scan_angle_min_rad = 0.8;
 o.scan_angle_increment_rad = 1.0;
 PerceptionResult perc;
 perc.scan_ok = true;
 iface::ChannelHealth ch{};
 const SafetyVerdict v = mon.evaluate(
 EstimatedState{}, perc, o, ch, 1'000'000);
 assert(v.state == SafetyState::kDegraded);
 assert(v.reason == "static-proximity");
 iface::MotionCommand escape;
 escape.space = iface::CommandSpace::kYawRate;
 escape.stamp_ns = 1'000'000;
 escape.expiry_ns = 201'000'000;
 escape.forward_mps = -0.25;
 assert(mon.gate(escape, v, 1'000'000).forward_mps < 0.0);
 }

  // Channel drops -> transient DEGRADED (blocks motion), auto-recovers.
  {
    SafetyMonitor mon;
    mon.reset();
    PerceptionResult perc;
    perc.scan_ok = true;
    iface::ChannelHealth ch{};
    ch.obs_dropped = 1;
    SafetyVerdict v = mon.evaluate(EstimatedState{}, perc,
                                   fresh_obs(1, 1'000'000, 5.0f), ch,
                                   1'000'000);
    assert(v.state == SafetyState::kDegraded && !v.allow_motion);
    SafetyVerdict ok = mon.evaluate(EstimatedState{}, perc,
                                    fresh_obs(2, 2'000'000, 5.0f), ch,
                                    2'000'000);
    assert(ok.state == SafetyState::kNominal);
  }

 // Forward-sector classification follows the complete base<-lidar
 // rotation. With a 90-degree pitch, a beam near sensor-forward projects
 // vertically/sideways and must not be mistaken for a base-forward obstacle.
 {
 SafetyConfig config;
 config.footprint_radius_m = 0.05;
 config.static_margin_m = 0.0;
 SafetyMonitor mon(config);
 mon.reset();
 mon.set_lidar_mount({{0.0, 0.0, 0.0},
 core::quaternion_from_rpy(0.0, M_PI * 0.5, 0.0)});
 iface::ObservationFrame o = fresh_obs(1, 1'000'000, 0.2f);
 o.ranges_m = {0.2f}; // angle = pi/8, within old yaw-only sector
 o.scan_angle_min_rad = M_PI / 8.0;
 o.scan_angle_increment_rad = M_PI / 8.0;
 PerceptionResult perc;
 perc.scan_ok = true;
 iface::ChannelHealth ch{};
 SafetyVerdict v = mon.evaluate(EstimatedState{}, perc, o, ch, 1'000'000);
 assert(v.state == SafetyState::kNominal && v.allow_motion);
 }

  // End to end: coordinator pipeline passes the safety gate on good evidence.
  {
    AutonomyCoordinator coord;
    assert(coord.configure().ok && coord.activate().ok);
    coord.ingest_observation(fresh_obs(1, 1'000'000, 5.0f));
    iface::ChannelHealth ch{};
    iface::MotionCommand cmd =
        coord.update(iface::MapSnapshot{}, ch, 1'000'000);
    assert(cmd.is_safe_stop());  // v1 holds, and safety agrees
    assert(coord.last_safety().state == SafetyState::kNominal);
  }

 // A newer motion-only envelope updates odometry/joints without erasing the
 // latest scan used by planning and the independent safety monitor.
 {
 AutonomyCoordinator coord;
 assert(coord.configure().ok && coord.activate().ok);
 iface::ObservationFrame scan = fresh_obs(1, 20'000'000, 5.0f);
 coord.ingest_observation(scan);
 iface::ObservationFrame motion;
 motion.sequence = 2;
 motion.stamp_ns = 40'000'000;
 motion.source = "t.odom";
 motion.frame = core::Frame::kBaseLink;
 motion.odometry_valid = true;
 motion.forward_mps = 0.1;
 coord.ingest_observation(motion);
 const iface::ObservationFrame& cached = coord.last_observation();
 assert(cached.scan_valid && cached.ranges_m == scan.ranges_m);
 assert(iface::scan_stamp_ns(cached) == 20'000'000);
 assert(iface::motion_stamp_ns(cached) == 40'000'000);
 iface::ChannelHealth ch{};
 (void)coord.update(iface::MapSnapshot{}, ch, 40'000'000);
 assert(coord.last_safety().state == SafetyState::kNominal);
 }

 std::printf("safety ok\n");
  return 0;
}
