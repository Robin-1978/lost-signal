#pragma once
#include "grid_map.hpp"
#include <vector>
namespace sim {
struct AStarResult {
 std::vector<Vec2> path;
 int expanded{0};
 bool ok{false};
};
// 8-dir A*, octile heuristic * h_weight. h_weight=1 optimal, >1 faster but suboptimal:
// 这就是要给玩家体会的第一个取舍。
AStarResult astar_plan(const GridMap& costmap, Vec2 start, Vec2 goal, double h_weight = 1.0);
} // namespace sim
