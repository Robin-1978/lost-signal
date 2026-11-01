#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

#include "robot/autonomy/coordinator.hpp"
#include "robot/autonomy/safety/safety.hpp"
#include "robot/autonomy/validation/validator.hpp"

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
 const auto swept = mon.last_verdict();
 assert(swept.reason == "static-swept-proximity");
 assert(std::abs(swept.static_command_envelope_radius_m - 0.5) < 1e-12);
 assert(swept.static_command_sweep.size() > 2);
 assert(swept.static_command_sweep.front().x == 0.0);
 assert(swept.static_command_sweep.front().y == 0.0);
 assert(swept.static_command_sweep.back().x > 0.0);
 assert(std::abs(swept.static_command_sweep.back().y) < 1e-12);
 }

 // The final sweep starts from measured motion, not only the newly requested
 // target. A low command while the chassis is still moving must retain the
 // current reaction/braking distance or the independent gate can under-check
 // a raw obstacle that the validator already rejects.
 {
 SafetyConfig config;
 config.footprint_radius_m = 0.1;
 config.static_margin_m = 0.1;
 config.brake_decel_mps2 = 4.0;
 config.command_reaction_time_s = 0.2;
 SafetyMonitor mon(config);
 mon.reset();
 iface::ObservationFrame o = fresh_obs(1, 1'000'000, 0.65f);
 o.ranges_m = {0.65f};
 o.scan_angle_min_rad = 0.0;
 o.scan_angle_increment_rad = 1.0;
 PerceptionResult perc;
 perc.scan_ok = true;
 iface::ChannelHealth ch{};
 EstimatedState moving;
 moving.valid = true;
 moving.stamp_ns = 1'000'000;
 moving.forward_mps = 1.5;
 const SafetyVerdict v = mon.evaluate(moving, perc, o, ch, 1'000'000);
 assert(v.state == SafetyState::kNominal);
 iface::MotionCommand brake;
 brake.space = iface::CommandSpace::kYawRate;
 brake.stamp_ns = 1'000'000;
 brake.expiry_ns = 201'000'000;
 brake.forward_mps = 0.1;
 assert(mon.gate(brake, v, 1'000'000).is_safe_stop());
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

 // Safety and Validator must reject the same braking-tail hazards even
 // though Safety only sees raw scan evidence. Both start from the measured
 // twist: a low requested speed cannot hide a chassis still moving at 1.5
 // m/s, and a crossing track cannot be accepted by one gate while rejected
 // by the other.
 {
   SafetyConfig safety_config;
   safety_config.footprint_radius_m = 0.1;
   safety_config.static_margin_m = 0.1;
   safety_config.brake_decel_mps2 = 4.0;
   safety_config.command_reaction_time_s = 0.2;
   SafetyMonitor safety(safety_config);
   ClearanceValidatorConfig validator_config;
   validator_config.footprint_radius_m = 0.1;
   validator_config.extra_margin_m = 0.1;
   validator_config.brake_decel_mps2 = 4.0;
   validator_config.actuator_response_delay_s = 0.2;
   ClearanceValidator validator(validator_config);
   EstimatedState moving;
   moving.valid = true;
   moving.stamp_ns = 1'000'000;
   moving.forward_mps = 1.5;
   PerceptionResult open;
   open.scan_ok = true;
   iface::ChannelHealth ch{};
   iface::ObservationFrame obstacle = fresh_obs(10, 1'000'000, 5.0f);
   obstacle.ranges_m = {0.65f};
   obstacle.scan_angle_min_rad = 0.0;
   obstacle.scan_angle_increment_rad = 1.0;
   iface::MotionCommand braking;
   braking.space = iface::CommandSpace::kYawRate;
   braking.stamp_ns = 1'000'000;
   braking.expiry_ns = 201'000'000;
   braking.forward_mps = 0.1;
   const SafetyVerdict clear = safety.evaluate(
       moving, open, obstacle, ch, 1'000'000);
   assert(clear.state == SafetyState::kNominal);
   assert(safety.gate(braking, clear, 1'000'000).is_safe_stop());
   LocalCandidate braking_candidate;
   braking_candidate.feasible = true;
   braking_candidate.forward_mps = braking.forward_mps;
   const ValidationResult braking_result = validator.validate(
       braking_candidate, moving, open, obstacle);
   assert(!braking_result.accepted);

   iface::ObservationFrame crossing = fresh_obs(11, 2'000'000, 5.0f);
   crossing.ranges_m = {5.0f};
   crossing.scan_angle_min_rad = 0.0;
   crossing.scan_angle_increment_rad = 1.0;
   PerceptionResult traffic = open;
   DynamicTrack track;
   track.id = 77;
   track.prediction_ready = true;
   track.x_m = 1.0;
   track.vx_mps = -1.5;
   traffic.tracks.push_back(track);
   EstimatedState stopped = moving;
   stopped.stamp_ns = 2'000'000;
   stopped.forward_mps = 0.0;
   const SafetyVerdict dynamic = safety.evaluate(
       stopped, traffic, crossing, ch, 2'000'000);
   assert(dynamic.state == SafetyState::kDegraded);
   assert(dynamic.reason == "dynamic-proximity");
   iface::MotionCommand forward = braking;
   forward.stamp_ns = 2'000'000;
   forward.expiry_ns = 202'000'000;
   forward.forward_mps = 0.5;
   assert(safety.gate(forward, dynamic, 2'000'000).is_safe_stop());
   LocalCandidate forward_candidate;
   forward_candidate.feasible = true;
   forward_candidate.forward_mps = forward.forward_mps;
   const ValidationResult dynamic_result = validator.validate(
       forward_candidate, stopped, traffic, crossing);
   assert(!dynamic_result.accepted);
   assert(dynamic_result.verdict == TrajectoryVerdict::kDynamicCollision);
 }

 std::printf("safety ok\n");
  return 0;
}
