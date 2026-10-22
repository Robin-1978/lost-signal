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
    ev.overlap = 0.5;  // one-sided: not enough support
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

  // 5. Full loop closure with drift: square loop inside a walled room,
  // estimate drifts +1 m in x by the return leg; the closure must pull it
  // back. The room gives dense returns so the strict two-way gate passes.
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
    assert(loop.closed);
    // Correction points back toward truth: ~1 m, and the corrected pose
    // is closer to the true start than the drifted estimate was.
    assert(loop.correction_m > 0.3);
    assert(std::hypot(loop.corrected_pose.position.x - 0.0,
                      loop.corrected_pose.position.y - 0.0) < 0.6);
    assert(session.loop_closure_count() == 1);
  }

  std::printf("slam ok\n");
  return 0;
}
