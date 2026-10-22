#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <vector>

#include "robot/autonomy/planning/astar_planner.hpp"

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

int main() {
  using namespace lost_signal::robot;
  using namespace lost_signal::robot::autonomy;

  AStarPlanner planner;

  // No raster: degraded straight-line route.
  {
    Route r = planner.plan(at(0.0, 0.0), {2.0, 0.0, 0.0}, iface::MapSnapshot{});
    assert(r.valid && r.points.size() >= 2);
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

  std::printf("astar ok\n");
  return 0;
}
