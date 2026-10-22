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

  // Estimator: integrates measured twist, fuses gyro yaw, grows covariance.
  {
    FusedOdometryEstimator est;
    est.reset();
    est.predict(make_odom(1, 0, 1.0));
    est.predict(make_odom(2, 20'000'000, 1.0));  // 20 ms at 1 m/s
    assert(est.estimate().valid);
    assert(std::abs(est.estimate().x_m - 0.02) < 1e-9);
    assert(est.estimate().position_sigma_m > 0.05);  // grew with motion
    assert(est.estimate().confidence < 1.0);
  }

  // Slip gate: wheels and gyro disagree -> trust gyro, inflate yaw sigma.
  {
    FusedOdometryEstimator est;
    est.reset();
    auto o1 = make_odom(1, 0, 1.0);
    o1.imu_valid = true;
    o1.imu_gyro_z_radps = 0.0;
    o1.imu_az_mps2 = 9.81;
    est.predict(o1);
    auto o2 = make_odom(2, 20'000'000, 1.0);
    o2.imu_valid = true;
    o2.imu_gyro_z_radps = 2.0;  // spinning wheels vs turning body
    o2.imu_az_mps2 = 9.81;
    const double sigma_before = est.estimate().heading_sigma_rad;
    est.predict(o2);
    // Gyro-led yaw: estimate follows the IMU, not the slipping wheels.
    assert(std::abs(est.estimate().yaw_rate_radps - 2.0) < 1e-9);
    assert(est.estimate().heading_sigma_rad >
           sigma_before + 0.19);  // slip inflation applied
  }

  // Attitude: static tilt converges roll/pitch toward the gravity vector.
  {
    FusedOdometryEstimator est;
    est.reset();
    // 10-degree nose-up tilt, no rotation, 1 s of 50 Hz IMU.
    const double ax = -std::sin(10.0 * M_PI / 180.0) * 9.81;
    const double az = std::cos(10.0 * M_PI / 180.0) * 9.81;
    for (int k = 1; k <= 50; ++k) {
      iface::ObservationFrame o;
      o.sequence = static_cast<std::uint64_t>(k);
      o.stamp_ns = k * 20'000'000;
      o.source = "t.imu";
      o.frame = core::Frame::kImuLink;
      o.odometry_valid = true;
      o.forward_mps = 0.0;
      o.imu_valid = true;
      o.imu_gyro_x_radps = 0.0;
      o.imu_gyro_y_radps = 0.0;
      o.imu_gyro_z_radps = 0.0;
      o.imu_ax_mps2 = ax;
      o.imu_ay_mps2 = 0.0;
      o.imu_az_mps2 = az;
      est.predict(o);
    }
    assert(est.estimate().attitude_valid);
    assert(std::abs(est.estimate().pitch_rad - 0.1745) < 0.05);
    assert(std::abs(est.estimate().roll_rad) < 0.02);
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

  // Route wiring: goal -> global plan -> local planner receives route.
  // The coordinator holds position without a goal; with a goal on an open
  // map the DWA drives toward it (validator + safety still gate raw scan).
  {
    AutonomyCoordinator coord;
    assert(coord.configure().ok && coord.activate().ok);
    assert(!coord.active_route().valid);
    coord.ingest_observation(make_odom(1, 1'000'000, 0.0));
    iface::ChannelHealth ch{};
    iface::MotionCommand hold =
        coord.update(iface::MapSnapshot{}, ch, 1'000'000);
    assert(hold.is_safe_stop());
    assert(!coord.active_route().valid);
    coord.set_goal({2.0, 0.0, 0.0});
    iface::MotionCommand drive =
        coord.update(iface::MapSnapshot{}, ch, 1'100'000);
    assert(coord.active_route().valid);
    assert(coord.active_route().points.size() >= 2);
    assert(drive.forward_mps > 0.0);  // DWA moves on the open map
    assert(iface::valid_motion_command(drive, 1'100'000));
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

  // Cadence: decision branch runs at the decision period; safety gates
  // every tick; replan request and map revision force an early decision.
  {
    platform::ChassisLimits lim{.max_forward_mps = 2.0,
                                .max_yaw_rate_radps = 1.5,
                                .footprint_radius_m = 0.4};
    AutonomyCoordinator coord(lim, 100'000'000);
    assert(coord.configure().ok && coord.activate().ok);
    coord.ingest_observation(make_odom(1, 0, 0.0));
    iface::ChannelHealth ch{};
    iface::MapSnapshot map;
    coord.update(map, ch, 0);
    assert(coord.last_decision_ns() == 0);
    coord.update(map, ch, 10'000'000);  // 10 ms later: hold last, no replan
    assert(coord.last_decision_ns() == 0);
    coord.request_replan();  // event wakeup
    coord.update(map, ch, 10'000'000);
    assert(coord.last_decision_ns() == 10'000'000);
    map.revision = 1;  // new map revision forces a decision
    coord.update(map, ch, 20'000'000);
    assert(coord.last_decision_ns() == 20'000'000);
    coord.update(map, ch, 120'000'000);  // period elapsed: due again
    assert(coord.last_decision_ns() == 120'000'000);
  }

  // Scan-rate gating: odometry-only frames advance estimation but never
  // perception; without scan evidence the planner has no plan.
  {
    AutonomyCoordinator coord;
    assert(coord.configure().ok && coord.activate().ok);
    auto odom_only = make_odom(1, 1'000'000, 0.5);
    odom_only.scan_valid = false;
    odom_only.ranges_m.clear();
    coord.ingest_observation(odom_only);
    assert(coord.has_observation());
    assert(!coord.perception().scan_ok);
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
