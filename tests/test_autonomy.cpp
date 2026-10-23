#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

#include "robot/autonomy/control/grip_adapter.hpp"
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

  // Slip observer: wheel-vs-body residual with honesty states.
  {
    // Invalid body: refusal, never silent substitution.
    SlipObservation bad = observe_slip({1.0, 0.0}, SlipBodyView{});
    assert(bad.quality == SlipQuality::kInvalid && bad.confidence == 0.0);
    // Below floor: unobservable, however large the disagreement.
    SlipBodyView slow{0.0, 2.0, true, false, 0.1};
    SlipObservation creep = observe_slip({0.0, 0.0}, slow);
    assert(creep.quality == SlipQuality::kUnobservableLowSpeed);
    // Observable: signed ratio + residual + confidence from trace.
    SlipBodyView body{1.0, 0.1, true, false, 0.5};
    SlipObservation spin = observe_slip({1.5, 0.6}, body);
    assert(spin.quality == SlipQuality::kObservable);
    assert(std::abs(spin.longitudinal_ratio - 0.5 / 1.5) < 1e-9);
    assert(std::abs(spin.yaw_residual_rads - 0.5) < 1e-9);
    assert(std::abs(spin.confidence - 1.0 / 1.5) < 1e-9);
    // Stale body: still computed, confidence halved, never hidden.
    SlipBodyView old = body;
    old.stale = true;
    SlipObservation stale = observe_slip({1.5, 0.6}, old);
    assert(stale.quality == SlipQuality::kStaleEstimator);
    assert(std::abs(stale.confidence - 0.5 / 1.5) < 1e-9);
    assert(std::string(slip_quality_name(SlipQuality::kObservable)) ==
           "Observable");
  }

  // Estimator exposes the latest slip observation for diagnostics.
  {
    FusedOdometryEstimator est;
    est.reset();
    assert(est.last_slip().quality == SlipQuality::kInvalid);
    auto o1 = make_odom(1, 0, 1.0);
    o1.imu_valid = true;
    o1.imu_gyro_z_radps = 0.0;
    o1.imu_az_mps2 = 9.81;
    est.predict(o1);
    // Sustained disagreement: gyro says turning, wheels say straight.
    // The inline sensor gate reacts first; after several cycles the
    // estimate follows the gyro and the observer residual (wheel yaw 0
    // vs estimator body yaw ~2) carries the same slip at estimate level.
    for (int k = 2; k <= 12; ++k) {
      auto o = make_odom(k, k * 20'000'000LL, 1.0);
      o.imu_valid = true;
      o.imu_gyro_z_radps = 2.0;
      o.imu_az_mps2 = 9.81;
      est.predict(o);
    }
    assert(est.last_slip().quality == SlipQuality::kObservable);
    assert(est.last_slip().yaw_residual_rads < -0.5);  // wheels lag the body
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
    // The controlled-stop latch withholds motion until the tread settles:
    // the pre-goal hold counts as carrying a stop, so the first post-goal
    // decisions stay held and release once settled (3 cycles at 0 tread).
    iface::MotionCommand drive = coord.update(iface::MapSnapshot{}, ch, 1'100'000);
    assert(drive.is_safe_stop());
    drive = coord.update(iface::MapSnapshot{}, ch, 1'200'000);
    assert(drive.is_safe_stop());
    // Third carrying tick reaches the settle count and releases.
    drive = coord.update(iface::MapSnapshot{}, ch, 1'300'000);
    assert(coord.active_route().valid);
    assert(coord.active_route().points.size() >= 2);
    assert(drive.forward_mps > 0.0);  // DWA moves on the open map
    assert(iface::valid_motion_command(drive, 1'300'000));
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
    // Eight-beam scans: beam 4 points straight ahead (angle 0).
    iface::ObservationFrame near = make_odom(1, 1, 0.0);
    near.ranges_m = {5.0f, 5.0f, 5.0f, 5.0f, 0.3f, 5.0f, 5.0f, 5.0f};
    assert(!v.validate(go, EstimatedState{}, ok, near).accepted);
    iface::ObservationFrame far = make_odom(1, 1, 0.0);
    far.ranges_m.assign(8, 5.0f);
    assert(v.validate(go, EstimatedState{}, ok, far).accepted);
    // Side-only obstacle never vetoes forward motion: the rollout's own
    // collision check owns the swept path, the validator owns the drive
    // direction.
    iface::ObservationFrame side = make_odom(1, 1, 0.0);
    side.ranges_m = {0.2f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f, 5.0f};
    assert(v.validate(go, EstimatedState{}, ok, side).accepted);
  }

  // Trajectory verdicts (valid estimate => full rollout validation).
  {
    ClearanceValidator v;
    PerceptionResult ok;
    ok.scan_ok = true;
    EstimatedState est;
    est.valid = true;  // x=y=yaw=0
    LocalCandidate go;
    go.feasible = true;
    go.forward_mps = 1.0;

    // Static collision: wall 1 m dead ahead, rollout drives through it.
    iface::ObservationFrame wall = make_odom(1, 1, 0.0);
    wall.ranges_m.assign(8, 10.0f);
    wall.ranges_m[4] = 1.0f;  // beam 4: straight ahead
    ValidationResult hit = v.validate(go, est, ok, wall);
    assert(!hit.accepted &&
           hit.verdict == TrajectoryVerdict::kStaticCollision);

    // Dynamic collision: track crossing the rollout centerline.
    iface::ObservationFrame open = make_odom(2, 2, 0.0);
    open.ranges_m.assign(8, 10.0f);
    PerceptionResult traffic = ok;
    DynamicTrack crosser;
    crosser.x_m = 2.0;
    crosser.vx_mps = -1.0;  // closing at 2 m/s vs the 1 m/s candidate
    traffic.tracks.push_back(crosser);
    ValidationResult dyn = v.validate(go, est, traffic, open);
    assert(!dyn.accepted &&
           dyn.verdict == TrajectoryVerdict::kDynamicCollision);

    // Minimum-risk escape: obstacle inside the margin behind the base
    // (beam 0 points straight back), slow creep forward gains clearance
    // where holding would strand the base inside the margin.
    iface::ObservationFrame behind = make_odom(3, 3, 0.0);
    behind.ranges_m.assign(8, 10.0f);
    behind.ranges_m[0] = 0.2f;  // 0.2 m dead astern: inside every margin
    LocalCandidate creep;
    creep.feasible = true;
    creep.forward_mps = 0.2;
    ValidationResult mre = v.validate(creep, est, ok, behind);
    assert(mre.accepted &&
           mre.verdict == TrajectoryVerdict::kMinimumRiskEscape);
    assert(mre.escape_clearance_gain_m > 0.15);
    // Same scene at speed is no escape: fast inside the margin rejects.
    LocalCandidate fast = creep;
    fast.forward_mps = 1.0;
    ValidationResult noesc = v.validate(fast, est, ok, behind);
    assert(!noesc.accepted);
  }

  // Terrain-aware validation (surveyed elevation prior, never SIM truth):
  // ramp faces read as ground, walls still read as walls, cliffs refuse.
  {
    // Surveyed 0.1 ramp over x in [0.5, 6.5] (top 0.6), 80x8 at 0.25 m.
    iface::MapSnapshot ramp;
    ramp.map_id = "test-ramp";
    ramp.revision = 3;
    ramp.width = 80;
    ramp.height = 8;
    ramp.resolution_m = 0.25;
    ramp.origin_x_m = 0.0;
    ramp.origin_y_m = -1.0;
    ramp.frame_id = "map";
    ramp.occupancy.assign(80 * 8, -1);
    ramp.elevation_m.assign(80 * 8, 0.0f);
    ramp.elevation_sigma_m.assign(80 * 8, 0.0f);
    for (std::uint32_t gy = 0; gy < 8; ++gy) {
      for (std::uint32_t gx = 0; gx < 80; ++gx) {
        const double cx = (gx + 0.5) * 0.25;
        ramp.elevation_m[gy * 80 + gx] =
            static_cast<float>(std::clamp((cx - 0.5) * 0.1, 0.0, 0.6));
      }
    }
    assert(ramp.has_elevation());
    EstimatedState est;
    est.valid = true;  // origin, yaw 0, z 0 (spawn plane)
    PerceptionResult ok;
    ok.scan_ok = true;
    SurveyContext ctx;
    ctx.map = &ramp;
    ctx.sensor_height_m = 0.35;  // TF mount: sensor plane at 0.45 absolute
    // Ramp face 4 m dead ahead (survey reads 0.35 there — the sensor
    // crossing contour, |0.35 - 0.45| within tol): fast rollout reaches
    // it, planar reads wall, survey reads ground.
    iface::ObservationFrame face = make_odom(10, 10, 0.0);
    face.ranges_m.assign(8, 4.0f);
    LocalCandidate drive;
    drive.feasible = true;
    drive.forward_mps = 2.5;
    ClearanceValidator v;
    ValidationResult planar = v.validate(drive, est, ok, face);
    assert(!planar.accepted &&
           planar.verdict == TrajectoryVerdict::kStaticCollision);
    ValidationResult survey = v.validate(drive, est, ok, face, ctx);
    assert(survey.accepted);
    // Same returns over a FLAT survey are still a wall (no blind pass).
    iface::MapSnapshot flat = ramp;
    std::fill(flat.elevation_m.begin(), flat.elevation_m.end(), 0.0f);
    SurveyContext flat_ctx = ctx;
    flat_ctx.map = &flat;
    ValidationResult wall = v.validate(drive, est, ok, face, flat_ctx);
    assert(!wall.accepted &&
           wall.verdict == TrajectoryVerdict::kStaticCollision);
    // Cliff across the rollout refuses as terrain (open scan, grade gate).
    iface::MapSnapshot cliff = ramp;
    for (std::uint32_t gy = 0; gy < 8; ++gy) {
      for (std::uint32_t gx = 0; gx < 80; ++gx) {
        const double cx = (gx + 0.5) * 0.25;
        cliff.elevation_m[gy * 80 + gx] = cx < 1.0 ? 0.0f : 1.0f;
      }
    }
    iface::ObservationFrame open = make_odom(11, 11, 0.0);
    open.ranges_m.assign(8, 10.0f);
    LocalCandidate slow;
    slow.feasible = true;
    slow.forward_mps = 1.0;
    SurveyContext cliff_ctx = ctx;
    cliff_ctx.map = &cliff;
    ValidationResult grade = v.validate(slow, est, ok, open, cliff_ctx);
    assert(!grade.accepted &&
           grade.verdict == TrajectoryVerdict::kTerrainInfeasible);
    // Live bridge: survey lost elevation at the face (stale revision with
    // a NaN crater reads wall), but the mapper's live layer measured the
    // surface there — the validator prefers live where confident.
    iface::MapSnapshot crater = ramp;
    for (std::uint32_t gx = 10; gx < 24; ++gx) {
      crater.elevation_m[4 * 80 + gx] =
          std::numeric_limits<float>::quiet_NaN();
    }
    SurveyContext crater_ctx = ctx;
    crater_ctx.map = &crater;
    ValidationResult stale = v.validate(drive, est, ok, face, crater_ctx);
    assert(!stale.accepted);  // no live: crater reads wall, correctly
    // Live raster in the mapper window geometry (0.1 m, origin -5, -5):
    // surface measured at the sensor-crossing height along the face.
    std::vector<float> live_h(100 * 100,
                              std::numeric_limits<float>::quiet_NaN());
    for (int gx = 85; gx < 95; ++gx) {
      live_h[50 * 100 + gx] = 0.42f;  // |(0.42-0.45)| within tol
    }
    crater_ctx.live.height_m = live_h.data();
    crater_ctx.live.width = 100;
    crater_ctx.live.height = 100;
    crater_ctx.live.resolution_m = 0.1;
    crater_ctx.live.origin_x_m = -5.0;
    crater_ctx.live.origin_y_m = -5.0;
    ValidationResult bridged = v.validate(drive, est, ok, face, crater_ctx);
    assert(bridged.accepted);  // live bridges the survey gap
  }

  // Dynamic rollover envelope (surface tilt + cornering load over the
  // support envelope): a 0.35 side-slope within the static cap passes slow
  // and straight, but carrying speed into a turn on it trips the risk cap.
  {
    // Side-slope survey: elevation = 0.35 * y over the 80x8 window.
    iface::MapSnapshot slope;
    slope.map_id = "test-slope";
    slope.revision = 1;
    slope.width = 80;
    slope.height = 8;
    slope.resolution_m = 0.25;
    slope.origin_x_m = 0.0;
    slope.origin_y_m = -1.0;
    slope.occupancy.assign(80 * 8, -1);
    slope.elevation_m.assign(80 * 8, 0.0f);
    for (std::uint32_t gy = 0; gy < 8; ++gy) {
      for (std::uint32_t gx = 0; gx < 80; ++gx) {
        const double cy = (gy + 0.5) * 0.25 - 1.0;
        slope.elevation_m[gy * 80 + gx] = static_cast<float>(0.35 * cy);
      }
    }
    EstimatedState est;
    est.valid = true;  // origin, yaw 0, z 0
    PerceptionResult ok;
    ok.scan_ok = true;
    SurveyContext ctx;
    ctx.map = &slope;
    ctx.sensor_height_m = 0.35;
    iface::ObservationFrame open = make_odom(12, 12, 0.0);
    open.ranges_m.assign(8, 10.0f);  // open sector, far wall at 10 m
    ClearanceValidator v;
    // Slow and straight: static risk 0.54 < 0.8 cap, accepted.
    LocalCandidate cruise;
    cruise.feasible = true;
    cruise.forward_mps = 1.0;
    ValidationResult steady = v.validate(cruise, est, ok, open, ctx);
    assert(steady.accepted);
    // Fast into a turn (v*w = 3.0 cornering load): risk 0.95 > 0.8 cap.
    LocalCandidate hot;
    hot.feasible = true;
    hot.forward_mps = 2.5;
    hot.yaw_rate_radps = 1.2;
    ValidationResult tippy = v.validate(hot, est, ok, open, ctx);
    assert(!tippy.accepted &&
           tippy.verdict == TrajectoryVerdict::kTerrainInfeasible);
    assert(tippy.reason == "dynamic rollover risk");
    // No survey: planar legacy, envelope sleeps (same candidate passes).
    ValidationResult planar = v.validate(hot, est, ok, open);
    assert(planar.accepted);
  }

  // Survey-aware mapper: background ramp returns clear but never paint;
  // wall returns still paint. Same classifier as the validator.
  {
    iface::MapSnapshot ramp;
    ramp.map_id = "test-ramp";
    ramp.revision = 3;
    ramp.width = 80;
    ramp.height = 8;
    ramp.resolution_m = 0.25;
    ramp.origin_x_m = 0.0;
    ramp.origin_y_m = -1.0;
    ramp.occupancy.assign(80 * 8, -1);
    ramp.elevation_m.assign(80 * 8, 0.0f);
    for (std::uint32_t gy = 0; gy < 8; ++gy) {
      for (std::uint32_t gx = 0; gx < 80; ++gx) {
        const double cx = (gx + 0.5) * 0.25;
        ramp.elevation_m[gy * 80 + gx] =
            static_cast<float>(std::clamp((cx - 0.5) * 0.1, 0.0, 0.6));
      }
    }
    EstimatedState est;
    est.valid = true;
    iface::ObservationFrame face = make_odom(10, 10, 0.0);
    face.ranges_m.assign(8, 4.0f);
    face.static_ranges_m = face.ranges_m;  // background: matches reference
    face.scan_static_valid = true;
    core::RigidTransform mount{{0.0, 0.0, 0.35}, {0.0, 0.0, 0.0, 1.0}};
    // Endpoint cell of the forward beam (4, 0) in the 0.1 rolling window.
    const int fx = static_cast<int>(std::floor((4.0 + 5.0) / 0.1));
    const int fy = static_cast<int>(std::floor((0.0 + 5.0) / 0.1));
    const std::size_t fi = static_cast<std::size_t>(fy) * 100 + fx;
    {
      RayClearingMapper mapper;
      mapper.set_lidar_mount(mount);
      mapper.update(face, est);
      LocalMap m = mapper.local_map();
      assert(m.valid && m.occupancy[fi] == 100);  // no survey: painted
    }
    {
      RayClearingMapper mapper;
      mapper.set_lidar_mount(mount);
      mapper.set_survey(ramp);
      mapper.update(face, est);
      LocalMap m = mapper.local_map();
      assert(m.valid && m.occupancy[fi] != 100);  // ground: never painted
      assert(!m.is_blocked(4.0, 0.0));  // unknown reads traversable (DWA)
      assert(m.segment_is_traversable(0.0, 0.0, 4.0, 0.0));
      // Live elevation: the surveyed-ground return measured the surface
      // (TF sensor height 0.35, level beam) into the dormant layer.
      assert(m.has_elevation());
      assert(std::abs(m.elevation_m[fi] - 0.35f) < 1e-6);
      // Actor returns never enter the terrain layer: foreground-only scans
      // (no static match) fuse nothing.
      iface::ObservationFrame fg = face;
      fg.static_ranges_m.assign(8, 10.0f);  // all foreground vs reference
      mapper.update(fg, est);
      LocalMap m2 = mapper.local_map();
      assert(m2.occupancy[fi] != 100);
      // Foreground clears occupancy rays but never touches terrain heights.
      assert(std::abs(m2.elevation_m[fi] - 0.35f) < 1e-6);
    }
    {
      // No survey: no ground definition, live layer stays unknown (NaN).
      RayClearingMapper mapper;
      mapper.set_lidar_mount(mount);
      mapper.update(face, est);
      LocalMap m = mapper.local_map();
      assert(m.valid && m.occupancy[fi] == 100);  // no survey: painted
      assert(!std::isfinite(m.elevation_at(4.0, 0.0)));
    }
  }

  // Deskewed mapper: a yawing scan of a static box paints the TRUE cell
  // (acquisition pose), not the stamp-pose projection. Uniform -0.1 s
  // offsets at yaw rate 1.0 isolate the yaw term.
  {
    EstimatedState est;
    est.valid = true;
    est.yaw_rate_radps = 1.0;
    iface::ObservationFrame spin = make_odom(20, 20, 0.0);
    spin.ranges_m.assign(8, 10.0f);
    spin.ranges_m[4] = 4.0f;  // forward beam saw the box face
    spin.static_ranges_m = spin.ranges_m;
    spin.scan_static_valid = true;
    spin.time_offsets_s.assign(8, -0.1f);
    spin.time_offset_valid = true;
    RayClearingMapper mapper;
    mapper.update(spin, est);
    LocalMap m = mapper.local_map();
    assert(m.valid);
    // True cell: R(-0.1) * (4, 0) = (3.98, -0.40), computed exactly
    // (decimal literals would floor the boundary cell wrong).
    const double tex = 4.0 * std::cos(-0.1), tey = 4.0 * std::sin(-0.1);
    const int tx = static_cast<int>(std::floor((tex + 5.0) / 0.1));
    const int ty = static_cast<int>(std::floor((tey + 5.0) / 0.1));
    assert(m.occupancy[static_cast<std::size_t>(ty) * 100 + tx] == 100);
    // Frozen projection (4, 0) stays unpainted: no smear.
    const int fx = static_cast<int>(std::floor((4.0 + 5.0) / 0.1));
    const int fy = static_cast<int>(std::floor((0.0 + 5.0) / 0.1));
    assert(fx != tx || fy != ty);
    assert(m.occupancy[static_cast<std::size_t>(fy) * 100 + fx] != 100);
  }

  // Scan deskew (shared pure helper): frozen reads identity, spin and
  // translation invert the provider's acquisition shift exactly.
  {
    using core::deskew_beam;
    using core::DeskewTwist;
    // Identity without offsets.
    core::Vec2 id = deskew_beam(5.0, 0.5, 0.0, {1.0, 2.0});
    assert(std::abs(id.x - 5.0 * std::cos(0.5)) < 1e-12);
    assert(std::abs(id.y - 5.0 * std::sin(0.5)) < 1e-12);
    // Pure spin: beam ahead, w = 1, dt = 0.1.
    core::Vec2 spin = deskew_beam(5.0, 0.0, -0.1, {0.0, 1.0});
    assert(std::abs(spin.x - 5.0 * std::cos(0.1)) < 1e-9);
    assert(std::abs(spin.y + 5.0 * std::sin(0.1)) < 1e-9);
    // Pure translation: v = 2, dt = 0.1, beam ahead.
    core::Vec2 lin = deskew_beam(5.0, 0.0, -0.1, {2.0, 0.0});
    assert(std::abs(lin.x - 4.8) < 1e-9 && std::abs(lin.y) < 1e-12);
    // Round-trip with the provider convention (same-contract proof, pure
    // math both sides): world endpoint from the acquisition pose equals
    // the deskewed endpoint from the end pose.
    {
      const double yaw_end = 0.5, v = 1.0, w = 0.5, dt = 0.1;
      const double beam = 1.0, range = 3.0;
      const double sx = 2.0, sy = -1.0;  // end pose
      // Provider convention: displacement rotates by the stamp yaw.
      const double acq_x =
          sx + (v * std::cos(yaw_end)) * -dt;
      const double acq_y =
          sy + (v * std::sin(yaw_end)) * -dt;
      const double yaw_acq = yaw_end + w * -dt;
      const double wx = acq_x + range * std::cos(yaw_acq + beam);
      const double wy = acq_y + range * std::sin(yaw_acq + beam);
      core::Vec2 local = deskew_beam(range, beam, -dt, {v, w});
      const double dx = sx + local.x * std::cos(yaw_end) -
                        local.y * std::sin(yaw_end);
      const double dy = sy + local.x * std::sin(yaw_end) +
                        local.y * std::cos(yaw_end);
      assert(std::hypot(dx - wx, dy - wy) < 1e-9);
    }
  }

  // Log-odds fusion (pure belief update over observation evidence):
  // first-sight walls paint promptly, confirmed walls ride out noise.
  {
    OccupancyLogOdds fusion;
    assert(fusion.cell_count() == 0);
    fusion.reset(4);
    assert(fusion.value(0) == -1);  // unseen
    fusion.observe_free(0);
    assert(fusion.value(0) == 0);  // one passage frees an unseen cell
    fusion.observe_occupied(1);
    assert(fusion.value(1) == 100);  // one endpoint confirms (+1.6->p .83)
    // Hysteresis: a confirmed wall survives stray clears (1.6 - 0.4k
    // stays above the p = 0.35 floor until the 6th free vote).
    for (int k = 0; k < 3; ++k) fusion.observe_free(1);
    assert(fusion.value(1) == 100);
    for (int k = 0; k < 3; ++k) fusion.observe_free(1);
    assert(fusion.value(1) == 0);  // sustained evidence clears it
    fusion.observe_occupied(1);
    assert(fusion.value(1) == 100);  // prompt re-confirmation
    fusion.observe_free(99);          // out of range: ignored, never traps
    assert(fusion.log_odds(99) == 0.0f);
    LogOddsConfig bad{-1.0f, -1.0f, 1.0f, -1.0f, 0.9f, 0.1f};
    OccupancyLogOdds broken(bad);
    broken.reset(4);
    broken.observe_occupied(2);
    assert(broken.value(2) == -1);  // invalid config: fail closed
  }

  // Mapper hysteresis end-to-end (measurement evidence only, no truth):
  // a painted wall survives 3 noisy clears, sustained clears erase it.
  {
    EstimatedState est;
    est.valid = true;
    auto wall_scan = [&](float beam_range) {
      iface::ObservationFrame o = make_odom(20, 20, 0.0);
      o.ranges_m.assign(8, 10.0f);
      o.ranges_m[4] = beam_range;  // forward beam
      o.static_ranges_m = o.ranges_m;
      o.scan_static_valid = true;
      return o;
    };
    RayClearingMapper mapper;
    mapper.update(wall_scan(4.0f), est);  // wall face at (4, 0)
    LocalMap m0 = mapper.local_map();
    assert(m0.valid);
    const auto idx = [&](double wx, double wy) {
      const int gx = static_cast<int>(std::floor((wx - m0.origin_x_m) / 0.1));
      const int gy = static_cast<int>(std::floor((wy - m0.origin_y_m) / 0.1));
      return static_cast<std::size_t>(gy) * 100 + gx;
    };
    assert(m0.occupancy[idx(4.0, 0.0)] == 100);  // first sight paints
    for (int k = 0; k < 3; ++k) mapper.update(wall_scan(10.0f), est);
    assert(mapper.local_map().occupancy[idx(4.0, 0.0)] == 100);
    for (int k = 0; k < 3; ++k) mapper.update(wall_scan(10.0f), est);
    assert(mapper.local_map().occupancy[idx(4.0, 0.0)] == 0);
  }

  // Live elevation fusion (pure belief update over measurement heights):
  // first return publishes, sustained disagreement invalidates, the
  // estimate follows real changes (forgetting) instead of freezing.
  {
    ElevationFusion fusion;
    assert(fusion.cell_count() == 0);
    fusion.reset(4);
    assert(!fusion.estimate(0).valid);  // unseen
    ElevationEstimate e1 = fusion.fuse(0, 0.35);
    assert(e1.valid && std::abs(e1.height_m - 0.35) < 1e-12);
    assert(e1.observations == 1);
    // Agreeing views tighten toward the mean.
    ElevationEstimate e2 = fusion.fuse(0, 0.37);
    assert(e2.valid && e2.height_m > 0.35 && e2.height_m < 0.37);
    // A wildly disagreeing view blows the sigma cap: reported unreliable,
    // evidence kept (later agreement can still recover the cell).
    ElevationEstimate bad = fusion.fuse(0, 5.0);
    assert(!bad.valid);
    assert(fusion.estimate(9).valid == false);  // out of range: rejected
    // NaN rejected with stored evidence untouched (still 3 observations).
    assert(!fusion.fuse(0, std::numeric_limits<double>::quiet_NaN()).valid);
    assert(fusion.estimate(0).observations == 3);
    // Forgetting: sustained new evidence moves a settled cell.
    ElevationFusion follow;
    follow.reset(1);
    for (int k = 0; k < 100; ++k) follow.fuse(0, 0.0);
    assert(follow.estimate(0).valid);
    for (int k = 0; k < 40; ++k) follow.fuse(0, 1.0);
    ElevationEstimate moved = follow.estimate(0);
    assert(moved.valid && moved.height_m > 0.5);
    // Invalid config fails closed (never fuses, never publishes).
    ElevationFusionConfig broken_cfg;
    broken_cfg.return_sigma_m = -1.0;
    ElevationFusion broken(broken_cfg);
    broken.reset(2);
    assert(!broken.fuse(0, 1.0).valid);
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

  // GripAdapter (control layer): slip-tightened speed envelope.
  {
    // Tighten on measured sensor disagreement while moving.
    GripAdapter g;
    iface::ObservationFrame o;
    o.odometry_valid = true;
    o.imu_valid = true;
    o.yaw_rate_radps = 0.0;
    o.imu_gyro_z_radps = 2.0;  // > 0.5 slip threshold
    EstimatedState est;
    est.valid = true;
    est.forward_mps = 1.0;
    SlipObservation no_slip;  // Invalid: sensor pair is the evidence
    g.update(0, o, est, no_slip, 0.0);  // stamps the clock
    g.update(500'000'000, o, est, no_slip, 0.0);  // 0.5 s at 0.4/s
    assert(std::abs(g.factor() - 0.8) < 1e-9);
    // Hysteresis band: residual between release (0.2) and slip (0.5)
    // freezes the factor.
    o.imu_gyro_z_radps = 0.3;
    g.update(1'000'000'000, o, est, no_slip, 0.0);
    assert(std::abs(g.factor() - 0.8) < 1e-9);
    // Clean rolling relaxes back toward 1.0 slowly (0.08/s).
    o.imu_gyro_z_radps = 0.0;
    g.update(1'500'000'000, o, est, no_slip, 0.0);
    assert(std::abs(g.factor() - 0.84) < 1e-9);
    // Floor: sustained slip pins at min_grip, never below.
    o.imu_gyro_z_radps = 2.0;
    for (int k = 2; k <= 12; ++k) {
      g.update(k * 500'000'000LL, o, est, no_slip, 0.0);
    }
    assert(std::abs(g.factor() - 0.4) < 1e-9);
    g.update(7'000'000'000LL, o, est, no_slip, 0.0);
    assert(std::abs(g.factor() - 0.4) < 1e-9);
  }

  // GripAdapter: observer evidence tightens even at low estimated speed;
  // standstill sensor disagreement is not evidence.
  {
    GripAdapter g;
    iface::ObservationFrame o;
    o.odometry_valid = true;
    o.imu_valid = true;
    o.yaw_rate_radps = 0.0;
    o.imu_gyro_z_radps = 2.0;
    EstimatedState still;
    still.valid = true;
    still.forward_mps = 0.0;  // standstill: sensor pair gated off
    SlipObservation none;
    g.update(0, o, still, none, 0.0);
    g.update(500'000'000, o, still, none, 0.0);
    assert(std::abs(g.factor() - 1.0) < 1e-9);
    // Observer fired (its own 0.5 m/s floor already proved motion).
    SlipObservation slip;
    slip.quality = SlipQuality::kObservable;
    slip.yaw_residual_rads = 1.0;
    g.update(1'000'000'000, o, still, slip, 0.0);
    assert(g.factor() < 1.0);
    // Invalid estimate freezes regardless of evidence.
    GripAdapter h;
    EstimatedState bad;  // valid == false
    bad.forward_mps = 1.0;
    h.update(0, o, bad, slip, 0.0);
    h.update(500'000'000, o, bad, slip, 0.0);
    assert(std::abs(h.factor() - 1.0) < 1e-9);
  }

  // GripAdapter::effective_limits: scales every speed cap, applies the
  // behavior cap on top, never widens.
  {
    GripAdapter g;
    platform::ChassisLimits base{.max_forward_mps = 2.0,
                                 .max_yaw_rate_radps = 1.5,
                                 .footprint_radius_m = 0.4,
                                 .max_steer_rad = 0.55,
                                 .max_lateral_mps = 1.0};
    // Untightened: behavior cap is the only constraint.
    platform::ChassisLimits full = g.effective_limits(base, 1.0);
    assert(std::abs(full.max_forward_mps - 1.0) < 1e-9);
    assert(std::abs(full.max_yaw_rate_radps - 1.5) < 1e-9);
    assert(std::abs(full.max_lateral_mps - 1.0) < 1e-9);
    // Tighten to ~0.8 via one slip tick, then check proportional shrink.
    iface::ObservationFrame o;
    o.odometry_valid = true;
    o.imu_valid = true;
    o.yaw_rate_radps = 0.0;
    o.imu_gyro_z_radps = 2.0;
    EstimatedState est;
    est.valid = true;
    est.forward_mps = 1.0;
    g.update(0, o, est, SlipObservation{}, 0.0);
    g.update(500'000'000, o, est, SlipObservation{}, 0.0);
    platform::ChassisLimits tight = g.effective_limits(base, 99.0);
    assert(std::abs(tight.max_forward_mps - 1.6) < 1e-9);
    assert(std::abs(tight.max_yaw_rate_radps - 1.2) < 1e-9);
    assert(std::abs(tight.max_lateral_mps - 0.8) < 1e-9);
    // reset() restores nominal.
    g.reset();
    assert(std::abs(g.factor() - 1.0) < 1e-9);
  }

  // Longitudinal channel (adapter-owned): wheel accel vs IMU body accel
  // with survey-grade gravity compensation. Straight-line wheelspin the
  // yaw gate cannot see.
  {
    auto spin_obs = [](double forward, double ax) {
      iface::ObservationFrame o;
      o.odometry_valid = true;
      o.forward_mps = forward;
      o.yaw_rate_radps = 0.0;
      o.imu_valid = true;
      o.imu_gyro_z_radps = 0.0;
      o.imu_ax_mps2 = ax;
      o.imu_az_mps2 = 9.81;
      return o;
    };
    auto run_ramp = [&](double wheel_accel, double body_ax, double survey,
                        int n) {
      GripAdapter g;
      EstimatedState est;
      est.valid = true;
      est.forward_mps = 1.0;
      g.update(0, spin_obs(0.0, body_ax), est, SlipObservation{}, survey);
      for (int k = 1; k <= n; ++k) {
        const double v = std::min(8.0, k * 0.02 * wheel_accel);
        g.update(k * 20'000'000LL, spin_obs(v, body_ax), est,
                 SlipObservation{}, survey);
      }
      return g.factor();
    };
    // Gripped launch (wheels and body accelerate together): clean, nominal.
    assert(std::abs(run_ramp(4.0, 4.0, 0.0, 60) - 1.0) < 1e-9);
    // Ice launch (wheels spin up, body lags): confirms, tightens hard.
    assert(run_ramp(4.0, 0.5, 0.0, 80) < 0.7);
    // Steady surveyed grade at constant speed: tilt cancels, no tighten.
    {
      GripAdapter g;
      EstimatedState est;
      est.valid = true;
      est.forward_mps = 1.0;
      const double grade = 0.1;
      const double ax = 9.81 * std::sin(grade);
      g.update(0, spin_obs(1.0, ax), est, SlipObservation{}, grade);
      for (int k = 1; k <= 50; ++k) {
        g.update(k * 20'000'000LL, spin_obs(1.0, ax), est, SlipObservation{},
                 grade);
      }
      assert(std::abs(g.factor() - 1.0) < 1e-9);
    }
    // Unsurveyed grade (NaN): flat assumption errs toward caution.
    {
      GripAdapter g;
      EstimatedState est;
      est.valid = true;
      est.forward_mps = 1.0;
      const double ax = 9.81 * std::sin(0.3);
      const double nan = std::numeric_limits<double>::quiet_NaN();
      g.update(0, spin_obs(1.0, ax), est, SlipObservation{}, nan);
      for (int k = 1; k <= 50; ++k) {
        g.update(k * 20'000'000LL, spin_obs(1.0, ax), est, SlipObservation{},
                 nan);
      }
      assert(g.factor() < 1.0);
    }
    // No IMU: longitudinal frozen, yaw-only behavior preserved.
    {
      GripAdapter g;
      EstimatedState est;
      est.valid = true;
      est.forward_mps = 1.0;
      iface::ObservationFrame o;
      o.odometry_valid = true;
      o.forward_mps = 1.0;
      o.imu_valid = false;
      g.update(0, o, est, SlipObservation{}, 0.0);
      g.update(500'000'000, o, est, SlipObservation{}, 0.0);
      assert(std::abs(g.factor() - 1.0) < 1e-9);
    }
  }

  // GripAdapter relax needs every observed channel clean (no dither),
  // and the longitudinal channel confirms over consecutive ticks.
  {
    GripAdapter g;
    EstimatedState est;
    est.valid = true;
    est.forward_mps = 1.0;
    iface::ObservationFrame o;
    o.odometry_valid = true;
    o.forward_mps = 1.0;  // wheels steady
    o.yaw_rate_radps = 0.0;
    o.imu_valid = true;
    o.imu_gyro_z_radps = 0.0;
    o.imu_ax_mps2 = 5.0;  // ...body still: spin residual, sustained
    g.update(0, o, est, SlipObservation{}, 0.0);
    // 14 over-threshold ticks: confirmation pending, envelope untouched.
    for (int k = 1; k <= 14; ++k) {
      g.update(k * 100'000'000LL, o, est, SlipObservation{}, 0.0);
    }
    assert(std::abs(g.factor() - 1.0) < 1e-9);
    // 15th consecutive tick confirms: tightens from the next one on.
    g.update(1'500'000'000LL, o, est, SlipObservation{}, 0.0);
    assert(std::abs(g.factor() - 1.0) < 1e-9);  // counter just completed
    g.update(1'600'000'000LL, o, est, SlipObservation{}, 0.0);
    const double tight = g.factor();
    assert(tight < 1.0);
    g.update(1'700'000'000LL, o, est, SlipObservation{}, 0.0);
    assert(g.factor() < tight);  // still firing, never relaxing
    // Body catches up (ax back to ~0): residual decays, envelope recovers.
    // Slow relax (0.08/s) needs a long runway back.
    o.imu_ax_mps2 = 0.0;
    for (int k = 18; k <= 150; ++k) {
      g.update(k * 100'000'000LL, o, est, SlipObservation{}, 0.0);
    }
    assert(g.factor() > tight);
  }

  // Coordinator probe: grip starts nominal, resets on configure.
  {
    AutonomyCoordinator coord;
    assert(coord.configure().ok && coord.activate().ok);
    assert(std::abs(coord.grip_factor() - 1.0) < 1e-9);
  }

  std::printf("autonomy ok\n");
  return 0;
}
