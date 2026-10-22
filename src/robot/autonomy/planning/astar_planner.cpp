#include "robot/autonomy/planning/astar_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

namespace lost_signal::robot::autonomy {

AStarPlanner::AStarPlanner(Config config) : config_(config) {}

AStarPlanner::AStarPlanner() : config_() {}

AStarPlanner::AStarPlanner(double unknown_cost_per_m)
    : config_() {
  config_.unknown_cost_per_m = unknown_cost_per_m;
}

Route AStarPlanner::plan(const EstimatedState& start, const RoutePoint& goal,
                         const iface::MapSnapshot& map) {
  if (!start.valid) return Route{};
  // Degraded mode: no raster to search.
  if (!map.has_raster()) return fallback_.plan(start, goal, map);

  const int w = static_cast<int>(map.width);
  const int h = static_cast<int>(map.height);
  const double res = map.resolution_m;
  auto to_grid = [&](double wx, double wy, int& gx, int& gy) {
    gx = static_cast<int>(std::floor((wx - map.origin_x_m) / res));
    gy = static_cast<int>(std::floor((wy - map.origin_y_m) / res));
  };
  auto cell_value = [&](int gx, int gy) -> int {
    if (gx < 0 || gy < 0 || gx >= w || gy >= h) return 100;  // outside: wall
    return map.occupancy[gy * w + gx];
  };
  auto cell_elevation = [&](int x, int y) -> float {
    if (!map.has_elevation()) return std::numeric_limits<float>::quiet_NaN();
    return map.elevation_m[y * w + x];
  };
  int sx, sy, gx, gy;
  to_grid(start.x_m, start.y_m, sx, sy);
  to_grid(goal.x_m, goal.y_m, gx, gy);
  auto inside = [&](int x, int y) {
    return x >= 0 && y >= 0 && x < w && y < h;
  };
  if (!inside(sx, sy) || !inside(gx, gy)) return Route{};
  if (cell_value(sx, sy) >= 50 || cell_value(gx, gy) >= 50) return Route{};

  struct Node {
    double f = std::numeric_limits<double>::infinity();
    double g = std::numeric_limits<double>::infinity();
    int parent = -1;
    bool closed = false;
  };
  std::vector<Node> nodes(w * h);
  auto idx = [&](int x, int y) { return y * w + x; };
  auto heuristic = [&](int x, int y) {
    const double dx = (x - gx) * res, dy = (y - gy) * res;
    return std::hypot(dx, dy);
  };
  using Entry = std::pair<double, int>;  // (f, index)
  std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
  nodes[idx(sx, sy)].g = 0.0;
  nodes[idx(sx, sy)].f = heuristic(sx, sy);
  open.emplace(nodes[idx(sx, sy)].f, idx(sx, sy));

  // 8-connected, no corner cutting through occupied cells.
  const int kDx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
  const int kDy[8] = {0, 1, 1, 1, 0, -1, -1, -1};
  int expansions = 0;
  const int kMaxExpansions = w * h;
  int found = -1;
  while (!open.empty() && expansions < kMaxExpansions) {
    auto [f, cur] = open.top();
    open.pop();
    if (nodes[cur].closed) continue;
    nodes[cur].closed = true;
    ++expansions;
    const int cx = cur % w, cy = cur / w;
    if (cx == gx && cy == gy) {
      found = cur;
      break;
    }
    for (int dir = 0; dir < 8; ++dir) {
      const int nx = cx + kDx[dir], ny = cy + kDy[dir];
      if (!inside(nx, ny)) continue;
      const int v = cell_value(nx, ny);
      if (v >= 50) continue;
      if (dir % 2 == 1) {  // diagonal: both orthogonal neighbors must pass
        if (cell_value(cx + kDx[dir], cy) >= 50 ||
            cell_value(cx, cy + kDy[dir]) >= 50)
          continue;
      }
      const double step = ((dir % 2 == 0) ? 1.0 : 1.4142135623730951) * res;
      double cost = step;
      if (v < 0) cost += config_.unknown_cost_per_m * step;  // unknown
      // Grade: penalize |dh|/step, reject cliffs beyond max_grade. Unknown
      // elevation pays no grade cost (surveyed prior may be sparse).
      const float h0 = cell_elevation(cx, cy);
      const float h1 = cell_elevation(nx, ny);
      if (!std::isnan(h0) && !std::isnan(h1) && step > 1e-9) {
        const double grade = std::abs(h1 - h0) / step;
        if (grade > config_.max_grade) continue;  // cliff: impassable
        cost += config_.grade_cost_per_m * std::abs(h1 - h0);
      }
      const double ng = nodes[cur].g + cost;
      if (ng < nodes[idx(nx, ny)].g) {
        nodes[idx(nx, ny)].g = ng;
        nodes[idx(nx, ny)].f = ng + heuristic(nx, ny);
        nodes[idx(nx, ny)].parent = cur;
        open.emplace(nodes[idx(nx, ny)].f, idx(nx, ny));
      }
    }
  }
  if (found < 0) return Route{};

  Route route;
  for (int cur = found; cur >= 0;) {
    const int cx = cur % w, cyy = cur / w;
    route.points.push_back(
        {map.origin_x_m + (cx + 0.5) * res,
         map.origin_y_m + (cyy + 0.5) * res, goal.yaw_rad});
    if (cur == idx(sx, sy)) break;
    cur = nodes[cur].parent;
  }
  std::reverse(route.points.begin(), route.points.end());
  // Headings follow the path; the terminal inherits the goal heading.
  for (std::size_t i = 0; i + 1 < route.points.size(); ++i) {
    route.points[i].yaw_rad = std::atan2(
        route.points[i + 1].y_m - route.points[i].y_m,
        route.points[i + 1].x_m - route.points[i].x_m);
  }
  route.valid = route.points.size() >= 2;
  return route;
}

}  // namespace lost_signal::robot::autonomy
