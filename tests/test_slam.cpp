#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

#include "robot/autonomy/mapping/loop_closure.hpp"
#include "robot/autonomy/mapping/pose_graph.hpp"
#include "robot/autonomy/mapping/slam.hpp"

namespace {
// Minimal slab raycast oracle (independent of the sim suite).
double cast_ray(double ox, double oy, double dx, double dy,
                double x0, double y0, double x1, double y1, double max_range) {
  double tmin = 0.0, tmax = max_range;
  if (std::abs(dx) < 1e-12) {
    if (ox < x0 || ox > x1) return max_range;
  } else {
    double t0 = (x0 - ox) / dx, t1 = (x1 - ox) / dx;
    if (t0 > t1) std::swap(t0, t1);
    tmin = std::max(tmin, t0);
    tmax = std::min(tmax, t1);
    if (tmin > tmax) return max_range;
  }
  if (std::abs(dy) < 1e-12) {
    if (oy < y0 || oy > y1) return max_range;
  } else {
    double t0 = (y0 - oy) / dy, t1 = (y1 - oy) / dy;
    if (t0 > t1) std::swap(t0, t1);
    tmin = std::max(tmin, t0);
    tmax = std::min(tmax, t1);
    if (tmin > tmax) return max_range;
  }
  return tmin > 0.0 ? tmin : max_range;
}

struct Box {
  double x0, y0, x1, y1;
};

std::vector<float> scan_from(double x, double y, double yaw,
                             const std::vector<Box>& boxes) {
  const int n = 72;
  std::vector<float> ranges;
  for (int i = 0; i < n; ++i) {
    double a = -M_PI + 2.0 * M_PI * i / n + yaw;
    double best = 10.0;
    for (const auto& b : boxes) {
      best = std::min(best,
                      cast_ray(x, y, std::cos(a), std::sin(a), b.x0, b.y0,
                               b.x1, b.y1, 10.0));
    }
    ranges.push_back(static_cast<float>(best));
  }
  return ranges;
}
}  // namespace

int main() {
  using namespace lost_signal::robot;
  using namespace lost_signal::robot::autonomy;

  // 1. Pose2 algebra round-trips.
  {
    Pose2 a{core::Vec2{1.0, 2.0}, 0.5};
    Pose2 b{core::Vec2{-0.5, 1.0}, -0.2};
    Pose2 rel = relative_pose(a, b);
    Pose2 back = compose_pose(a, rel);
    assert(std::abs(back.position.x - b.position.x) < 1e-9);
    assert(std::abs(back.position.y - b.position.y) < 1e-9);
    assert(std::abs(core::wrap_angle(back.heading_rad - b.heading_rad)) < 1e-9);
    Pose2 inv = inverse_pose(a);
    Pose2 ident = compose_pose(a, inv);
    assert(std::hypot(ident.position.x, ident.position.y) < 1e-9);
    assert(std::abs(core::wrap_angle(ident.heading_rad)) < 1e-9);
  }

  // 2. Backend: drifted chain + loop constraint pulls the tail back.
  {
    PoseGraphBackend graph;
    std::string reason;
    for (int i = 0; i < 4; ++i) {
      PoseGraphKeyframe kf;
      kf.id = static_cast<std::uint64_t>(i + 1);
      kf.stamp_s = i * 1.0;
      // True chain along +x; node 4 carries +0.6 m drift.
      kf.initial_pose = Pose2{core::Vec2{i * 1.0 + (i == 3 ? 0.6 : 0.0), 0.0}, 0.0};
      kf.fixed = (i == 0);
      assert(graph.add_keyframe(kf, &reason));
    }
    for (int i = 0; i < 3; ++i) {
      PoseGraphConstraint c;
      c.id = 100 + i;
      c.from = i + 1;
      c.to = i + 2;
      c.measured_relative_pose =
          relative_pose(graph.find_keyframe(i + 1)->initial_pose,
                        graph.find_keyframe(i + 2)->initial_pose);
      c.information = pose_graph_information(0.05, 0.02);
      c.kind = PoseGraphConstraintKind::Odometry;
      assert(graph.add_constraint(c, &reason));
    }
    // Loop: node 4 is actually back at node 1 (square closed).
    PoseGraphConstraint loop;
    loop.id = 200;
    loop.from = 1;
    loop.to = 4;
    loop.measured_relative_pose =
        Pose2{core::Vec2{3.0, 0.0}, 0.0};  // true relative displacement
    loop.information = pose_graph_information(0.05, 0.02);
    loop.kind = PoseGraphConstraintKind::LoopClosure;
    assert(graph.add_constraint(loop, &reason));
    auto r = graph.optimize();
    assert(r.success);
    assert(r.final_cost < r.initial_cost);
    const PoseGraphKeyframe* tail = graph.find_keyframe(4);
    // Fixed node 1 unmoved; drifted tail pulled toward truth (3.0, not 3.6).
    assert(std::abs(graph.find_keyframe(1)->initial_pose.position.x) < 1e-9);
    assert(std::abs(tail->initial_pose.position.x - 3.0) < 0.3);
  }

  // 3. Gate: temporal separation + two-way support required.
  {
    LoopClosureGate gate;
    PoseGraphKeyframe from, to;
    from.id = 1;
    from.stamp_s = 0.0;
    to.id = 9;
    to.stamp_s = 3.0;  // only 3 s apart: too fresh
    LoopClosureEvidence ev;
    ev.match_score = 0.9;
    ev.overlap = 0.8;
    ev.score_margin = 0.2;
    assert(!gate.validate(from, to, ev).accepted);
    to.stamp_s = 10.0;
    assert(gate.validate(from, to, ev).accepted);
    ev.overlap = 0.4;  // one-sided: not enough support
    assert(!gate.validate(from, to, ev).accepted);
  }

  // 4. Session keyframing: motion thresholds gate, ring drops the oldest.
  {
    SlamConfig cfg;
    cfg.keyframe_translation_m = 0.5;
    cfg.max_keyframes = 4;
    SlamSession session(cfg);
    EstimatedState est;
    est.valid = true;
    iface::ObservationFrame obs;
    obs.scan_valid = true;
    obs.ranges_m.assign(72, 5.0f);
    obs.scan_angle_min_rad = -M_PI;
    obs.scan_angle_increment_rad = 2.0 * M_PI / 72;
    for (int k = 0; k < 6; ++k) {
      est.x_m = k * 0.5;
      obs.stamp_ns = (k + 1) * 1'000'000'000LL;
      session.ingest(est, obs);
    }
    assert(session.keyframe_count() == 4);  // ring bound holds
    assert(session.keyframes().front().id == 3);  // oldest dropped
  }

    // 5. Ambiguous single-view loop: a symmetric walled room and opposite
    // return heading can produce a locally plausible association. One scan
    // must not move the graph; the full mapless mission test below this unit
    // covers successful multi-view closures in asymmetric geometry.
  {
    std::vector<Box> boxes{{3.0, -1.0, 4.0, 5.0},
                           {-1.0, 4.0, 3.0, 5.0},
                           {-2.0, -5.0, 4.0, -4.0},
                           {-2.0, -5.0, -1.0, 5.0}};
    SlamConfig cfg;
    cfg.keyframe_translation_m = 0.5;
    cfg.keyframe_heading_rad = 99.0;  // translation-only keyframes here
    cfg.gate.minimum_temporal_separation_s = 1.0;
    SlamSession session(cfg);
    // Square: out along +x, back along -x at y=2 (poses face travel dir).
    const std::vector<std::array<double, 3>> truth = {
        {0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {2.0, 0.0, 0.0},
        {2.0, 1.0, 0.0}, {2.0, 2.0, 0.0}, {1.0, 2.0, M_PI},
        {0.0, 2.0, M_PI}, {0.0, 1.0, M_PI}, {0.0, 0.0, M_PI},
    };
    double t = 0.0;
    for (const auto& tp : truth) {
      t += 2.0;
      EstimatedState est;
      est.valid = true;
      est.stamp_ns = static_cast<core::MonotonicNs>(t * 1e9);
      // Drift: +0.1 m/s in x after t=4 (1.0 m total by the return).
      const double drift = t > 4.0 ? 0.1 * (t - 4.0) : 0.0;
      est.x_m = tp[0] + drift;
      est.y_m = tp[1];
      est.yaw_rad = tp[2];
      iface::ObservationFrame obs;
      obs.sequence = static_cast<std::uint64_t>(t);
      obs.stamp_ns = est.stamp_ns;
      obs.source = "t.lidar";
      obs.scan_valid = true;
      obs.ranges_m = scan_from(tp[0], tp[1], tp[2], boxes);
      obs.scan_angle_min_rad = -M_PI;
      obs.scan_angle_increment_rad = 2.0 * M_PI / 72;
      session.ingest(est, obs);
    }
    assert(session.keyframe_count() >= 4);
    // Return leg revisits the start: attempt closure with the drifted pose.
    EstimatedState ret;
    ret.valid = true;
    ret.stamp_ns = static_cast<core::MonotonicNs>(20e9);
    ret.x_m = 0.0 + 1.0;  // 1 m drift
    ret.y_m = 0.0;
    ret.yaw_rad = M_PI;
    iface::ObservationFrame obs;
    obs.sequence = 99;
    obs.stamp_ns = ret.stamp_ns;
    obs.source = "t.lidar";
    obs.scan_valid = true;
    obs.ranges_m = scan_from(0.0, 0.0, M_PI, boxes);
    obs.scan_angle_min_rad = -M_PI;
    obs.scan_angle_increment_rad = 2.0 * M_PI / 72;
    SlamLoopResult loop = session.try_close_loops(ret, obs);
 assert(!loop.closed);
    assert(session.loop_closure_count() == 0);
  }

  // 6. Incremental tracking: live scan against the newest keyframe submap
  // in a small window. Bounds incremental drift without establishing the
  // frame (no loop event needed); empty scans reject instead of guessing.
  {
    std::vector<Box> boxes{{-2.0, -2.0, 3.0, -1.8},
                           {-2.0, -2.0, -1.8, 3.0},
                           {2.6, -1.0, 3.0, 2.0},
                           {0.5, 0.5, 0.9, 0.9}};
    SlamConfig cfg;
    SlamSession session(cfg);
    EstimatedState kf;
    kf.valid = true;
    kf.stamp_ns = 1'000'000'000LL;
    iface::ObservationFrame kobs;
    kobs.scan_valid = true;
    kobs.ranges_m = scan_from(0.0, 0.0, 0.0, boxes);
    kobs.scan_angle_min_rad = -M_PI;
    kobs.scan_angle_increment_rad = 2.0 * M_PI / 72;
    kobs.stamp_ns = kf.stamp_ns;
    session.ingest(kf, kobs);
    assert(session.keyframe_count() == 1);
    // Robot truly moved to B; the prior carries drift inside the window.
    const double bx = 0.25, by = 0.05, byaw = 0.02;
    EstimatedState prior;
    prior.valid = true;
    prior.x_m = bx + 0.10;
    prior.y_m = by - 0.06;
    prior.yaw_rad = byaw + 0.03;
    prior.stamp_ns = 1'200'000'000LL;
    iface::ObservationFrame obs;
    obs.scan_valid = true;
    obs.ranges_m = scan_from(bx, by, byaw, boxes);
    obs.scan_angle_min_rad = -M_PI;
    obs.scan_angle_increment_rad = 2.0 * M_PI / 72;
    obs.stamp_ns = prior.stamp_ns;
    SlamTrackResult tr = session.track(prior, obs);
    assert(tr.attempted);
 if (!tr.accepted) {
 std::fprintf(stderr,
 "track failed: %s score=%.3f overlap=%.3f margin=%.3f "
 "correction=%.3f/%.3f\n",
 tr.diagnostic.c_str(), tr.score, tr.overlap, tr.score_margin,
 tr.correction_m, tr.correction_rad);
 }
    assert(tr.accepted);
    assert(std::hypot(tr.x_m - bx, tr.y_m - by) < 0.1);
    assert(std::abs(core::wrap_angle(tr.yaw_rad - byaw)) < 0.1);
    assert(session.track_corrections() == 1);
    // Empty scan: no geometric returns, must reject, never snap.
    iface::ObservationFrame blank = obs;
    blank.ranges_m.assign(72, 10.0f);
    blank.stamp_ns = 1'500'000'000LL;
    EstimatedState still = prior;
    still.stamp_ns = blank.stamp_ns;
    still.x_m += 0.3;
    SlamTrackResult tr2 = session.track(still, blank);
    assert(tr2.attempted);
    assert(!tr2.accepted);
  }

 // An opposing scan correction is the expected signature when wheel
 // odometry overstates travel during longitudinal slip. One such scan stays
 // pending; a geometrically verified correction with the same sign from an
 // independently moved view is accepted. This uses encoder and LiDAR only.
 {
 std::vector<Box> boxes{{-2.0, -2.0, 3.0, -1.8},
 {-2.0, -2.0, -1.8, 3.0},
 {2.6, -1.0, 3.0, 2.0},
 {0.5, 0.5, 0.9, 0.9}};
 SlamSession session;
 EstimatedState anchor;
 anchor.valid = true;
 anchor.stamp_ns = 1'000'000'000LL;
 iface::ObservationFrame anchor_obs;
 anchor_obs.scan_valid = true;
 anchor_obs.ranges_m = scan_from(0.0, 0.0, 0.0, boxes);
 anchor_obs.scan_angle_min_rad = -M_PI;
 anchor_obs.scan_angle_increment_rad = 2.0 * M_PI / 72;
 anchor_obs.stamp_ns = anchor.stamp_ns;
 session.ingest(anchor, anchor_obs);

 EstimatedState first_prior;
 first_prior.valid = true;
 first_prior.x_m = 0.35; // true x=0.25: wheel odometry is 0.10 m ahead
 first_prior.y_m = -0.01; // true y=0.05
 first_prior.yaw_rad = 0.05; // true yaw=0.02
 first_prior.stamp_ns = 1'200'000'000LL;
 iface::ObservationFrame first;
 first.scan_valid = true;
 first.odometry_valid = true;
 first.forward_mps = 1.0;
 first.ranges_m = scan_from(0.25, 0.05, 0.02, boxes);
 first.scan_angle_min_rad = -M_PI;
 first.scan_angle_increment_rad = 2.0 * M_PI / 72;
 first.stamp_ns = first_prior.stamp_ns;
 const SlamTrackResult surveyed = session.track(first_prior, first);
 assert(surveyed.attempted);
 assert(!surveyed.accepted);
 assert(surveyed.diagnostic.find("opposes odometry") != std::string::npos);

 // Mapless mode has no scan-to-map authority, so restart the same evidence
 // and enable the explicit multi-view fallback.
 session.reset();
 session.ingest(anchor, anchor_obs);
 const SlamTrackResult pending = session.track(
 first_prior, first, true);
 assert(pending.attempted);
 assert(!pending.accepted);
 if (pending.diagnostic.find("opposing odometry") == std::string::npos) {
 std::fprintf(stderr,
 "opposing track first view: %s correction=%.3f/%.3f score=%.3f "
 "overlap=%.3f margin=%.3f\n",
 pending.diagnostic.c_str(), pending.correction_m,
 pending.correction_rad, pending.score, pending.overlap,
 pending.score_margin);
 }
 assert(pending.diagnostic.find("opposing odometry") != std::string::npos);
 assert(session.track_corrections() == 0);

 EstimatedState second_prior = first_prior;
 second_prior.x_m = 0.47; // true x=0.35: slip error persists
 second_prior.stamp_ns = 1'500'000'000LL;
 iface::ObservationFrame second = first;
 second.ranges_m = scan_from(0.35, 0.05, 0.02, boxes);
 second.stamp_ns = second_prior.stamp_ns;
 const SlamTrackResult confirmed = session.track(
 second_prior, second, true);
 assert(confirmed.attempted);
 assert(confirmed.accepted);
 assert(confirmed.x_m < second_prior.x_m);
 assert(session.track_corrections() == 1);
 }

  // Dynamic actor returns must not enter the static submap. A static wall
 // behind the robot stays occupied; an actor 1 m ahead does not.
 {
 SlamSession slam;
 EstimatedState est;
 est.valid = true;
 est.x_m = 0.0;
 est.y_m = 0.0;
 est.yaw_rad = 0.0;
 est.z_m = 0.0;
 est.stamp_ns = 1'000'000'000LL;
 iface::ObservationFrame obs;
 obs.scan_valid = true;
 obs.frame = core::Frame::kLidarLink;
 obs.stamp_ns = est.stamp_ns;
 obs.ranges_m.assign(72, 10.0f);
 obs.static_ranges_m.assign(72, 10.0f);
 obs.scan_static_valid = true;
 obs.ranges_m[0] = 2.0f; // static wall at -x
 obs.static_ranges_m[0] = 2.0f;
 obs.ranges_m[36] = 1.0f; // dynamic actor at +x
 slam.ingest(est, obs);
 assert(slam.keyframe_count() == 1);
 const iface::MapSnapshot sub = slam.submap(1);
 assert(sub.width > 0 && sub.occupancy.size() ==
 static_cast<std::size_t>(sub.width) * sub.height);
 auto at = [&](double x, double y) {
 const int gx = static_cast<int>(
 std::floor((x - sub.origin_x_m) / sub.resolution_m));
 const int gy = static_cast<int>(
 std::floor((y - sub.origin_y_m) / sub.resolution_m));
 assert(gx >= 0 && gy >= 0 && gx < static_cast<int>(sub.width) &&
 gy < static_cast<int>(sub.height));
 return sub.occupancy[gy * sub.width + gx];
 };
 assert(at(-2.0, 0.0) == 100);
 assert(at(1.0, 0.0) != 100);
 }

 std::printf("slam ok\n");
  return 0;
}
