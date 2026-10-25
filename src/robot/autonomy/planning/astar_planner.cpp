#include "robot/autonomy/planning/astar_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

#include "robot/autonomy/mapping/mapper.hpp"

namespace lost_signal::robot::autonomy {

AStarPlanner::AStarPlanner(Config config) : config_(config) {}

AStarPlanner::AStarPlanner() : config_() {}

AStarPlanner::AStarPlanner(double unknown_cost_per_m)
    : config_() {
  config_.unknown_cost_per_m = unknown_cost_per_m;
}

Route AStarPlanner::plan(const EstimatedState& start, const RoutePoint& goal,
                         const iface::MapSnapshot& map) {
 if (!start.valid || !std::isfinite(start.x_m) ||
 !std::isfinite(start.y_m) || !std::isfinite(goal.x_m) ||
 !std::isfinite(goal.y_m) || !std::isfinite(goal.yaw_rad)) {
 return Route{};
 }
 if (!(config_.unknown_cost_per_m >= 0.0) ||
 !(config_.grade_cost_per_m >= 0.0) || !(config_.max_grade >= 0.0) ||
 !(config_.clearance_cost_per_m >= 0.0) ||
 !(config_.footprint_radius_m > 0.0) ||
 !(config_.hard_clearance_margin_m >= 0.0) ||
 !(config_.soft_inflation_radius_m >= 0.0) ||
 !(config_.roughness_cost_per_m >= 0.0) ||
 !(config_.slip_cost_per_m >= 0.0) ||
 !std::isfinite(config_.unknown_cost_per_m) ||
 !std::isfinite(config_.grade_cost_per_m) ||
 !std::isfinite(config_.max_grade) ||
 !std::isfinite(config_.clearance_cost_per_m) ||
 !std::isfinite(config_.footprint_radius_m) ||
 !std::isfinite(config_.hard_clearance_margin_m) ||
 !std::isfinite(config_.soft_inflation_radius_m) ||
 !std::isfinite(config_.roughness_cost_per_m) ||
 !std::isfinite(config_.slip_cost_per_m)) {
 return Route{};
 }
  // Degraded mode: no raster to search.
  if (!map.has_raster()) return fallback_.plan(start, goal, map);

 if (map.width > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
 map.height > static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
 return Route{};
 }

  const int w = static_cast<int>(map.width);
  const int h = static_cast<int>(map.height);
  const double res = map.resolution_m;
 const double hard_radius =
 std::max(0.0, config_.footprint_radius_m) +
 std::max(0.0, config_.hard_clearance_margin_m);
 const double soft_radius =
 std::max(0.0, config_.soft_inflation_radius_m);
 // Unknown space is intentionally traversable at a configured penalty. It
 // never seeds footprint inflation; newly observed occupied cells will
 // trigger a new map revision and route refresh.
 if (!costmap_cached_ || cached_map_id_ != map.map_id ||
 cached_map_revision_ != map.revision) {
 cached_costmap_ = make_global_costmap(
 map, hard_radius, soft_radius, true);
 cached_map_id_ = map.map_id;
 cached_map_revision_ = map.revision;
 costmap_cached_ = true;
 }
 const LocalCostmap& planning_map = cached_costmap_;
 if (!planning_map.valid()) return Route{};
  auto to_grid = [&](double wx, double wy, int& gx, int& gy) {
    gx = static_cast<int>(std::floor((wx - map.origin_x_m) / res));
    gy = static_cast<int>(std::floor((wy - map.origin_y_m) / res));
  };
  auto cell_value = [&](int gx, int gy) -> int {
    if (gx < 0 || gy < 0 || gx >= w || gy >= h) return 100;  // outside: wall
 return planning_map.lethal.occupancy[gy * w + gx];
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
      if (map.occupancy[ny * w + nx] < 0) {
        cost += config_.unknown_cost_per_m * step;
      }
      const double wx = map.origin_x_m + (nx + 0.5) * res;
      const double wy = map.origin_y_m + (ny + 0.5) * res;
      cost += config_.clearance_cost_per_m * step *
              planning_map.cost_at(wx, wy);
      // Grade: penalize |dh|/step, reject cliffs beyond max_grade. Unknown
      // elevation pays no grade cost (surveyed prior may be sparse).
      const float h0 = cell_elevation(cx, cy);
      const float h1 = cell_elevation(nx, ny);
      if (!std::isnan(h0) && !std::isnan(h1) && step > 1e-9) {
        const double grade = std::abs(h1 - h0) / step;
        if (grade > config_.max_grade) continue;  // cliff: impassable
        cost += config_.grade_cost_per_m * std::abs(h1 - h0);
      }
      // Believed roughness (the ROBOT's own IMU observer, null when no
      // model is bound): rough cells cost more, so equal-grade routes
      // prefer smooth ground. Surveyed elevation still owns the grade
      // term above; this only prices what the survey cannot see.
      // Traction memory prices slipped-on cells the same way (measured
      // slip, never the author's friction): routes prefer gripped ground.
      if (traversal_ != nullptr && step > 1e-9) {
        cost += config_.roughness_cost_per_m * traversal_->roughness_at(wx,
                                                                        wy) *
                step;
        cost += config_.slip_cost_per_m * traversal_->slip_at(wx, wy) * step;
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
 // The goal lies inside the accepted free raster cell, whose interior is
 // convex. Finish at the authored pose instead of its cell center so local
 // control does not inherit up to half a cell of avoidable terminal error.
 const RoutePoint cell_goal = route.points.back();
 if (std::hypot(cell_goal.x_m - goal.x_m,
 cell_goal.y_m - goal.y_m) > 1e-9) {
 route.points.push_back(goal);
 } else {
 route.points.back().yaw_rad = goal.yaw_rad;
 }
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
