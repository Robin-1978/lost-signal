#pragma once
#include "autonomy/mapping/grid_map.hpp"
#include "autonomy/planning/path.hpp"

#include <functional>
#include <vector>
namespace sim {
struct AStarResult {
 std::vector<RoutePoint> path;
 int expanded{0};
 bool ok{false};
};
// 8-dir A*, octile heuristic * h_weight. h_weight=1 optimal, >1 faster but suboptimal:
// This is the first planning tradeoff the player encounters.
using TraversalCost = std::function<double(Vec2 from, Vec2 to)>;
AStarResult astar_plan(const GridMap& costmap, Vec2 start, Vec2 goal, double h_weight = 1.0,
 const TraversalCost& traversal_cost = {}, double goal_tolerance_m = 0.0);
} // namespace sim
