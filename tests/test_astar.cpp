#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

#include "robot/autonomy/planning/astar_planner.hpp"
#include "robot/autonomy/planning/route_pipeline.hpp"

namespace {
lost_signal::robot::iface::MapSnapshot make_map(int w, int h, double res,
                                                 double ox, double oy) {
  lost_signal::robot::iface::MapSnapshot m;
  m.map_id = "test";
  m.revision = 1;
  m.width = w;
  m.height = h;
  m.resolution_m = res;
  m.origin_x_m = ox;
  m.origin_y_m = oy;
  m.occupancy.assign(w * h, -1);
  m.elevation_m.assign(
      w * h, std::numeric_limits<float>::quiet_NaN());  // unknown height
  m.frame_id = "map";
  return m;
}

void fill_box(lost_signal::robot::iface::MapSnapshot& m, double x0, double y0,
              double x1, double y1) {
  int w = m.width, h = m.height;
  for (int gy = 0; gy < h; ++gy)
    for (int gx = 0; gx < w; ++gx) {
      double cx = m.origin_x_m + (gx + 0.5) * m.resolution_m;
      double cy = m.origin_y_m + (gy + 0.5) * m.resolution_m;
      if (cx >= x0 && cx < x1 && cy >= y0 && cy < y1)
        m.occupancy[gy * w + gx] = 100;
    }
}

lost_signal::robot::autonomy::EstimatedState at(double x, double y) {
  lost_signal::robot::autonomy::EstimatedState e;
  e.valid = true;
  e.x_m = x;
  e.y_m = y;
  return e;
}
}  // namespace

class CountingPlanner : public lost_signal::robot::autonomy::IGlobalPlanner {
 public:
 lost_signal::robot::autonomy::Route plan(
 const lost_signal::robot::autonomy::EstimatedState& start,
 const lost_signal::robot::autonomy::RoutePoint& goal,
 const lost_signal::robot::iface::MapSnapshot&) override {
 ++calls;
 lost_signal::robot::autonomy::Route out;
 out.valid = start.valid;
 out.points = {{start.x_m, start.y_m, start.yaw_rad}, goal};
 return out;
 }
 int calls = 0;
};

int main() {
  using namespace lost_signal::robot;
  using namespace lost_signal::robot::autonomy;

  AStarPlanner planner;

  // No raster: degraded straight-line route.
  {
    Route r = planner.plan(at(0.0, 0.0), {2.0, 0.0, 0.0}, iface::MapSnapshot{});
    assert(r.valid && r.points.size() >= 2);
  }

 // Route refresh policy is module-owned: an explicit replan calls the
 // planner even when an existing route is still valid.
 {
 CountingPlanner counting;
 Route active;
 const auto map = iface::MapSnapshot{};
 assert(RoutePlanningPipeline::refresh(counting, at(0.0, 0.0),
 {2.0, 0.0, 0.0}, true, map, false, active));
 assert(counting.calls == 1 && active.valid);
 assert(!RoutePlanningPipeline::refresh(counting, at(0.0, 0.0),
 {2.0, 0.0, 0.0}, true, map, false, active));
 assert(counting.calls == 1);
 assert(RoutePlanningPipeline::refresh(counting, at(0.0, 0.0),
 {2.0, 0.0, 0.0}, true, map, true, active));
 assert(counting.calls == 2);
 }

  // Wall with a gap: route threads the gap, never through occupied cells.
  {
    auto map = make_map(40, 40, 0.5, -10.0, -10.0);
    fill_box(map, 4.0, -10.0, 5.0, -1.0);
    fill_box(map, 4.0, 1.0, 5.0, 10.0);  // 2 m gap at y in [-1, 1]
    Route r = planner.plan(at(1.0, 0.0), {8.0, 0.0, 0.0}, map);
    assert(r.valid);
    assert(r.points.size() >= 2);
    for (const auto& p : r.points) assert(!map.blocked(p.x_m, p.y_m));
    bool in_gap = false;
    for (const auto& p : r.points) {
      if (p.x_m > 3.5 && p.x_m < 5.5) {
        assert(p.y_m > -1.5 && p.y_m < 1.5);
        in_gap = true;
      }
    }
    assert(in_gap);
  }

  // Full wall: no path, invalid route (fail closed upstream).
  {
    auto map = make_map(40, 40, 0.5, -10.0, -10.0);
    fill_box(map, 4.0, -10.0, 5.0, 10.0);
    Route r = planner.plan(at(1.0, 0.0), {8.0, 0.0, 0.0}, map);
    assert(!r.valid);
  }

  // Grade: a hill between start and goal is skirted, not climbed.
  {
    auto map = make_map(30, 20, 1.0, 0.0, 0.0);
    for (int gy = 0; gy < 20; ++gy)
      for (int gx = 0; gx < 30; ++gx) {
        map.occupancy[gy * 30 + gx] = 0;  // all free: isolate grade effect
        const double cx = gx + 0.5, cy = gy + 0.5;
        const double dx = cx - 15.0, dy = cy - 10.0;
        map.elevation_m[gy * 30 + gx] =
            static_cast<float>(5.0 * std::exp(-(dx * dx + dy * dy) / 18.0));
      }
    Route r = planner.plan(at(1.0, 10.0), {28.0, 10.0, 0.0}, map);
    assert(r.valid);
    // Went around the hill, not over it.
    for (const auto& p : r.points) {
      const double d =
          std::hypot(p.x_m - 15.0, p.y_m - 10.0);
      assert(d > 2.5);
    }
    // Detour is longer than the straight line.
    double length = 0.0;
    for (std::size_t i = 1; i < r.points.size(); ++i) {
      length += std::hypot(r.points[i].x_m - r.points[i - 1].x_m,
                           r.points[i].y_m - r.points[i - 1].y_m);
    }
    assert(length > 27.0 + 2.0);
  }

  // Cliff: a full-height elevation step is impassable, route invalid.
  {
    auto map = make_map(30, 20, 1.0, 0.0, 0.0);
    for (int gy = 0; gy < 20; ++gy)
      for (int gx = 0; gx < 30; ++gx) {
        map.occupancy[gy * 30 + gx] = 0;
        map.elevation_m[gy * 30 + gx] = gx < 10 ? 0.0f : 5.0f;
      }
    Route r = planner.plan(at(1.0, 10.0), {28.0, 10.0, 0.0}, map);
    assert(!r.valid);
  }

  // Believed roughness (traversal model hook): rough bands with a smooth
  // gap route through the gap; without the model the same map goes
  // straight. Roughness is the ROBOT's own IMU belief, never the author's.
  {
    auto map = make_map(40, 40, 0.5, -10.0, -10.0);
    for (auto& c : map.occupancy) c = 0;  // all free: isolate roughness
    TraversalModel model;
    model.bind(map);
    // Observe the bands rough (bump hits at 0.5 m/s); the gap stays prior.
    for (int gx = 28; gx <= 29; ++gx) {
      for (int gy = 0; gy < 40; ++gy) {
        const double cy = -10.0 + (gy + 0.5) * 0.5;
        if (cy >= -1.0 && cy < 1.0) continue;  // smooth gap
        EstimatedState est;
        est.valid = true;
        est.attitude_valid = true;
        est.x_m = -10.0 + (gx + 0.5) * 0.5;
        est.y_m = cy;
        est.forward_mps = 0.5;
        for (int k = 1; k <= 50; ++k) {
          model.observe_motion(est, 9.81 + 4.5, k * 0.02, 0.02);
        }
      }
    }
    double conf = 0.0;
    assert(model.roughness_at(4.25, 5.0, &conf) > 0.05);
    assert(model.roughness_at(4.25, 0.0, &conf) == 0.02 && conf == 0.0);
    // No model: straight through (shortest).
    {
      Route r = planner.plan(at(1.0, 0.0), {8.0, 0.0, 0.0}, map);
      assert(r.valid);
    }
    // With the belief: threads the smooth gap.
    {
      AStarPlanner rough_planner;
      AStarPlanner::Config cfg;
      cfg.roughness_cost_per_m = 30.0;  // test knob: price the belief hard
      rough_planner = AStarPlanner(cfg);
      rough_planner.set_traversal(&model);
      Route r = rough_planner.plan(at(1.0, 0.0), {8.0, 0.0, 0.0}, map);
      assert(r.valid);
      bool in_gap = false;
      for (const auto& p : r.points) {
        if (p.x_m > 3.5 && p.x_m < 5.5) {
          assert(p.y_m > -1.5 && p.y_m < 1.5);
          in_gap = true;
        }
      }
      assert(in_gap);
    }
  }


  // Traction memory (beyond main): slipped-on bands with a clean gap
  // route through the gap; without the memory the same map goes straight.
  // Slip is measured fire evidence, never the author's friction.
  {
    auto map = make_map(40, 40, 0.5, -10.0, -10.0);
    for (auto& c : map.occupancy) c = 0;  // all free: isolate traction
    TraversalModel model;
    model.bind(map);
    // Fire latched along the bands (severity 0.5); the gap stays prior.
    for (int gx = 28; gx <= 29; ++gx) {
      for (int gy = 0; gy < 40; ++gy) {
        const double cx = -10.0 + (gx + 0.5) * 0.5;
        const double cy = -10.0 + (gy + 0.5) * 0.5;
        if (cy >= -1.0 && cy < 1.0) continue;  // clean gap
        model.observe_slip(cx, cy, 0.5, 1.0 + gx * 0.01 + gy * 0.001);
      }
    }
    double conf = 0.0;
    assert(model.slip_at(4.25, 5.0, &conf) == 0.5 && conf == 1.0);
    assert(model.slip_at(4.25, 0.0, &conf) == 0.0 && conf == 0.0);
    // No memory: straight through (shortest).
    {
      Route r = planner.plan(at(1.0, 0.0), {8.0, 0.0, 0.0}, map);
      assert(r.valid);
    }
    // With the memory: threads the clean gap.
    {
      AStarPlanner slip_planner;
      slip_planner.set_traversal(&model);
      Route r = slip_planner.plan(at(1.0, 0.0), {8.0, 0.0, 0.0}, map);
      assert(r.valid);
      bool in_gap = false;
      for (const auto& p : r.points) {
        if (p.x_m > 3.5 && p.x_m < 5.5) {
          assert(p.y_m > -1.5 && p.y_m < 1.5);
          in_gap = true;
        }
      }
      assert(in_gap);
    }
  }

  std::printf("astar ok\n");
  return 0;
}
