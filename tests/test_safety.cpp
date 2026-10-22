#include <cassert>
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

  std::printf("safety ok\n");
  return 0;
}
