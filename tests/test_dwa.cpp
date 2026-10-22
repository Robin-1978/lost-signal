#include <cassert>
#include <cmath>
#include <cstdio>

#include "robot/autonomy/planning/dwa_planner.hpp"
#include "robot/autonomy/planning/global_planner.hpp"
#include "robot/autonomy/planning/prediction.hpp"

namespace {
lost_signal::robot::autonomy::LocalMap make_map(int w, int h, double res,
                                                double ox, double oy,
                                                int8_t fill) {
  lost_signal::robot::autonomy::LocalMap m;
  m.valid = true;
  m.resolution_m = res;
  m.width = w;
  m.height = h;
  m.origin_x_m = ox;
  m.origin_y_m = oy;
  m.occupancy.assign(w * h, fill);
  return m;
}

void fill_box(lost_signal::robot::autonomy::LocalMap& m, double x0, double y0,
              double x1, double y1) {
  for (int gy = 0; gy < static_cast<int>(m.height); ++gy)
    for (int gx = 0; gx < static_cast<int>(m.width); ++gx) {
      double cx = m.origin_x_m + (gx + 0.5) * m.resolution_m;
      double cy = m.origin_y_m + (gy + 0.5) * m.resolution_m;
      if (cx >= x0 && cx < x1 && cy >= y0 && cy < y1)
        m.occupancy[gy * m.width + gx] = 100;
    }
}
}  // namespace

int main() {
  using namespace lost_signal::robot;
  using namespace lost_signal::robot::autonomy;

  platform::ChassisLimits limits{.max_forward_mps = 2.0,
                                 .max_yaw_rate_radps = 1.5,
                                 .footprint_radius_m = 0.4};

  // LocalMap queries: observed-free vs blocked vs unknown.
  {
    LocalMap m = make_map(100, 100, 0.1, -5.0, -5.0, -1);
    assert(!m.is_free(0.0, 0.0));      // unknown is not free (conservative)
    assert(!m.is_blocked(0.0, 0.0));   // unknown is not blocked (optimistic)
    assert(!m.segment_is_free(0.0, 0.0, 3.0, 0.0));  // conservative
    assert(m.segment_is_traversable(0.0, 0.0, 3.0, 0.0));  // optimistic
    fill_box(m, 4.0, -5.0, 5.0, 5.0);
    assert(!m.is_free(4.5, 0.0));
    assert(m.is_blocked(4.5, 0.0));
    assert(!m.segment_is_traversable(0.0, 0.0, 8.0, 0.0));
    assert(m.clearance(0.0, 0.0) > 3.0);
  }

  // DWA on an open map drives toward the route goal, deterministically.
  {
    LocalMap m = make_map(100, 100, 0.1, -5.0, -5.0, 0);
    EstimatedState est;
    est.valid = true;
    est.stamp_ns = 1'000'000;
    est.x_m = 1.0;
    est.y_m = 1.0;
    PerceptionResult perc;
    perc.scan_ok = true;
    StraightLinePlanner global;
    Route route = global.plan(est, {8.0, 1.0, 0.0}, iface::MapSnapshot{});
    assert(route.valid);
    DwaPlanner planner;
    LocalPlan a = planner.plan(est, perc, m, route, limits);
    LocalPlan b = planner.plan(est, perc, m, route, limits);
    assert(a.has_plan && a.best.feasible);
    assert(a.best.forward_mps > 0.0);  // moves, not a hold
    assert(a.best.forward_mps == b.best.forward_mps &&
           a.best.yaw_rate_radps == b.best.yaw_rate_radps);
  }

  // DWA threads a gap: wall with a 2 m opening, goal straight through.
  {
    LocalMap m = make_map(100, 100, 0.1, -5.0, -5.0, 0);
    fill_box(m, 4.0, -5.0, 5.0, 4.0);
    fill_box(m, 4.0, 6.0, 5.0, 5.0);
    EstimatedState est;
    est.valid = true;
    est.stamp_ns = 1'000'000;
    est.x_m = 1.0;
    est.y_m = 5.0;
    PerceptionResult perc;
    perc.scan_ok = true;
    StraightLinePlanner global;
    Route route = global.plan(est, {8.0, 5.0, 0.0}, iface::MapSnapshot{});
    DwaPlanner planner;
    LocalPlan p = planner.plan(est, perc, m, route, limits);
    assert(p.has_plan && p.best.feasible);
    assert(p.best.forward_mps > 0.0);
  }

  // DWA with a track sitting on the ego pose: every rollout collides.
  {
    LocalMap m = make_map(100, 100, 0.1, -5.0, -5.0, 0);
    EstimatedState est;
    est.valid = true;
    est.stamp_ns = 1'000'000;
    est.x_m = 1.0;
    est.y_m = 1.0;
    PerceptionResult perc;
    perc.scan_ok = true;
    DynamicTrack t;
    t.id = 1;
    t.x_m = 1.5;
    t.y_m = 1.0;
    perc.tracks.push_back(t);
    StraightLinePlanner global;
    Route route = global.plan(est, {8.0, 1.0, 0.0}, iface::MapSnapshot{});
    DwaPlanner planner;
    LocalPlan p = planner.plan(est, perc, m, route, limits);
    assert(!p.has_plan);  // no collision-free rollout: hold upstream
  }

  // Predictor on new types: CV track extrapolates along velocity.
  {
    DynamicTrack t;
    t.id = 3;
    t.x_m = 0.0;
    t.vx_mps = 1.0;
    auto frames =
        predict_frames({predict_input_from_track(t, 0.0)}, 0.0, 1.0, 0.5, 5.0);
    assert(frames.size() == 3);
    assert(std::abs(frames[0][0].x_m - 0.0) < 1e-9);
    assert(std::abs(frames[1][0].x_m - 0.5) < 1e-9);
    assert(std::abs(frames[2][0].x_m - 1.0) < 1e-9);
  }

  std::printf("dwa ok\n");
  return 0;
}
