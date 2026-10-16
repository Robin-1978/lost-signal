#pragma once

#include "algo/astar.hpp"
#include "algo/hybrid_astar.hpp"
#include "algo/terrain.hpp"

#include <memory>
#include <vector>

namespace game {

struct PlanningWaypoint {
 sim::Vec2 position;
 double heading_rad{0.0};
};

class IPathPlanner {
 public:
 virtual ~IPathPlanner() = default;
 virtual sim::AStarResult plan_segment(const sim::GridMap& map, const PlanningWaypoint& from,
 const PlanningWaypoint& to, const sim::TraversalCost& traversal_cost) const = 0;
};

class AStarPathPlanner final : public IPathPlanner {
 public:
 sim::AStarResult plan_segment(const sim::GridMap& map, const PlanningWaypoint& from,
 const PlanningWaypoint& to, const sim::TraversalCost& traversal_cost) const override;
};

class HybridAStarPathPlanner final : public IPathPlanner {
 public:
 explicit HybridAStarPathPlanner(sim::HybridAStarConfig config) : config_(config) {}
 sim::AStarResult plan_segment(const sim::GridMap& map, const PlanningWaypoint& from,
 const PlanningWaypoint& to, const sim::TraversalCost& traversal_cost) const override;

 private:
 sim::HybridAStarConfig config_;
};

std::shared_ptr<const IPathPlanner> default_path_planner();
std::shared_ptr<const IPathPlanner> ackermann_path_planner(double wheel_base_m, double max_steer_rad,
 double map_resolution_m);

class PlanningSystem {
 public:
 explicit PlanningSystem(std::shared_ptr<const IPathPlanner> planner);

 sim::AStarResult plan(const sim::GridMap& map, const std::vector<PlanningWaypoint>& waypoints,
 const sim::TraversalCost& traversal_cost) const;

 private:
 std::shared_ptr<const IPathPlanner> planner_;
};

} // namespace game
