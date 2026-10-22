#include <cassert>
#include <cmath>
#include <cstdio>

#include "robot/autonomy/coordinator.hpp"
#include "robot/autonomy/estimation/estimator.hpp"
#include "robot/autonomy/planning/global_planner.hpp"

namespace {
lost_signal::robot::iface::ObservationFrame make_odom(
    std::uint64_t seq, lost_signal::robot::core::MonotonicNs stamp,
    double forward) {
  lost_signal::robot::iface::ObservationFrame o;
  o.sequence = seq;
  o.stamp_ns = stamp;
  o.source = "t.odom";
  o.frame = lost_signal::robot::core::Frame::kBaseLink;
  o.odometry_valid = true;
  o.forward_mps = forward;
  o.yaw_rate_radps = 0.0;
  o.scan_valid = true;
  o.ranges_m = {5.0f, 5.0f};
  return o;
}
}  // namespace

int main() {
  using namespace lost_signal::robot;
  using namespace lost_signal::robot::autonomy;

  // Estimator: integrates measured twist, no truth involved.
  {
    DeadReckoningEstimator est;
    est.reset();
    est.predict(make_odom(1, 0, 1.0));
    est.predict(make_odom(2, 20'000'000, 1.0));  // 20 ms at 1 m/s
    assert(est.estimate().valid);
    assert(std::abs(est.estimate().x_m - 0.02) < 1e-9);
  }

  // Coordinator pipeline: configure -> activate -> ingest -> update.
  // v1 holds position through perception/map/plan/validate/control.
  {
    AutonomyCoordinator coord;
    assert(coord.configure().ok);
    assert(coord.activate().ok);
    coord.ingest_observation(make_odom(1, 1'000'000, 0.0));
    iface::ChannelHealth ch{};
    iface::MotionCommand cmd =
        coord.update(iface::MapSnapshot{}, ch, 1'000'000);
    assert(iface::valid_motion_command(cmd, 1'000'000));
    assert(cmd.is_safe_stop());
    assert(coord.perception().scan_ok);
    assert(coord.local_map().valid);
  }

  // Pre-activation: ingest ignored, update holds.
  {
    AutonomyCoordinator coord;
    coord.ingest_observation(make_odom(1, 1'000'000, 1.0));
    assert(!coord.has_observation());
    iface::ChannelHealth ch{};
    iface::MotionCommand cmd =
        coord.update(iface::MapSnapshot{}, ch, 1'000'000);
    assert(cmd.is_safe_stop());
  }

  // Validator: hold passes without scan; motion needs clearance.
  {
    ClearanceValidator v;
    PerceptionResult no_scan;
    iface::ObservationFrame empty;
    LocalCandidate hold;
    hold.feasible = true;
    assert(v.validate(hold, EstimatedState{}, no_scan, empty).accepted);

    LocalCandidate go;
    go.feasible = true;
    go.forward_mps = 1.0;
    assert(!v.validate(go, EstimatedState{}, no_scan, empty).accepted);

    PerceptionResult ok;
    ok.scan_ok = true;
    iface::ObservationFrame near = make_odom(1, 1, 0.0);
    near.ranges_m = {0.3f};
    assert(!v.validate(go, EstimatedState{}, ok, near).accepted);
    iface::ObservationFrame far = make_odom(1, 1, 0.0);
    far.ranges_m = {5.0f};
    assert(v.validate(go, EstimatedState{}, ok, far).accepted);
  }

  // Global planner: straight line from estimate to goal.
  {
    StraightLinePlanner planner;
    EstimatedState s;
    s.valid = true;
    RoutePoint goal{2.0, 0.0, 0.0};
    Route r = planner.plan(s, goal, iface::MapSnapshot{});
    assert(r.valid && r.points.size() >= 2);
    assert(std::abs(r.points.front().x_m) < 1e-9);
    assert(std::abs(r.points.back().x_m - 2.0) < 1e-9);
    EstimatedState bad;
    assert(!planner.plan(bad, goal, iface::MapSnapshot{}).valid);
  }

  std::printf("autonomy ok\n");
  return 0;
}
