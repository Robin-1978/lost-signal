#pragma once

#include "robot/autonomy/planning/global/astar.hpp"

namespace sim {

struct HybridAStarConfig {
 double wheel_base_m{0.6};
 double max_steer_rad{0.55};
 double primitive_length_m{0.4};
 int heading_bins{72};
 bool allow_reverse{true};
 double reverse_cost{1.35};
 double direction_change_cost_m{0.8};
 double steer_cost{0.04};
 double goal_position_tolerance_m{0.45};
 double goal_heading_tolerance_rad{0.8};
 int max_expansions{250000};
};

// Curvature-constrained search for car-like chassis. Each edge is a bicycle
// model arc and is collision/terrain checked along the full primitive.
AStarResult hybrid_astar_plan(const GridMap& map, Vec2 start, double start_heading_rad, Vec2 goal,
 double goal_heading_rad, const HybridAStarConfig& config,
 const TraversalCost& traversal_cost = {});

} // namespace sim
