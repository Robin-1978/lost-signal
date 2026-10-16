#include "planning_system.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace game {

sim::AStarResult AStarPathPlanner::plan_segment(const sim::GridMap& map, const PlanningWaypoint& from,
 const PlanningWaypoint& to, const sim::TraversalCost& traversal_cost) const {
 return sim::astar_plan(map, from.position, to.position, 1.0, traversal_cost);
}

sim::AStarResult HybridAStarPathPlanner::plan_segment(const sim::GridMap& map,
 const PlanningWaypoint& from, const PlanningWaypoint& to,
 const sim::TraversalCost& traversal_cost) const {
 return sim::hybrid_astar_plan(map, from.position, from.heading_rad, to.position, to.heading_rad,
 config_, traversal_cost);
}

std::shared_ptr<const IPathPlanner> default_path_planner() {
 static const auto planner = std::make_shared<AStarPathPlanner>();
 return planner;
}

std::shared_ptr<const IPathPlanner> ackermann_path_planner(double wheel_base_m, double max_steer_rad,
 double map_resolution_m) {
 sim::HybridAStarConfig config;
 config.wheel_base_m = wheel_base_m;
 config.max_steer_rad = max_steer_rad;
 config.primitive_length_m = std::max(map_resolution_m * 1.5, 0.35);
 config.goal_position_tolerance_m = std::max(map_resolution_m * 1.8, 0.45);
 return std::make_shared<HybridAStarPathPlanner>(config);
}

PlanningSystem::PlanningSystem(std::shared_ptr<const IPathPlanner> planner) : planner_(std::move(planner)) {
 if (!planner_) {
  throw std::invalid_argument("planning system requires a path planner");
 }
}

sim::AStarResult PlanningSystem::plan(const sim::GridMap& map,
 const std::vector<PlanningWaypoint>& waypoints,
 const sim::TraversalCost& traversal_cost) const {
 sim::AStarResult result;
 if (waypoints.size() < 2) {
  return result;
 }
 result.ok = true;
 for (std::size_t index = 1; index < waypoints.size(); ++index) {
 auto segment = planner_->plan_segment(map, waypoints[index - 1], waypoints[index], traversal_cost);
  result.expanded += segment.expanded;
  if (!segment.ok) {
   result.ok = false;
   result.path.clear();
   return result;
  }
  if (!result.path.empty() && !segment.path.empty()) {
   segment.path.erase(segment.path.begin());
  }
  result.path.insert(result.path.end(), segment.path.begin(), segment.path.end());
 }
 return result;
}

} // namespace game
