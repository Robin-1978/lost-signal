#include "autonomy/planning/global/global_planner.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace autonomy {
namespace {

GlobalPlan adapt_plan(sim::AStarResult result) {
 return {std::move(result.path), result.expanded, result.ok, result.ok ? "" : "no-path"};
}

} // namespace

GlobalPlan AStarGlobalPlanner::plan(const GlobalPlannerRequest& request) const {
 return adapt_plan(sim::astar_plan(request.map, request.start.position, request.goal.position, 1.0,
 request.traversal_cost));
}

GlobalPlan HybridAStarGlobalPlanner::plan(const GlobalPlannerRequest& request) const {
 return adapt_plan(sim::hybrid_astar_plan(
 request.map, request.start.position, request.start.heading_rad, request.goal.position,
 request.goal.heading_rad, config_, request.traversal_cost));
}

void GlobalPlannerRegistry::register_factory(std::string id, GlobalPlannerFactory factory) {
 if (id.empty()) {
 throw std::invalid_argument("global planner id must not be empty");
 }
 if (!factory) {
 throw std::invalid_argument("global planner factory must not be empty");
 }
 if (!factories_.emplace(std::move(id), std::move(factory)).second) {
 throw std::invalid_argument("global planner id is already registered");
 }
}

bool GlobalPlannerRegistry::contains(std::string_view id) const {
 return factories_.find(id) != factories_.end();
}

std::shared_ptr<const IGlobalPlanner> GlobalPlannerRegistry::create(std::string_view id) const {
 const auto found = factories_.find(id);
 if (found == factories_.end()) {
 throw std::invalid_argument("unknown global planner: " + std::string(id));
 }
 auto planner = found->second();
 if (!planner) {
 throw std::runtime_error("global planner factory returned null: " + std::string(id));
 }
 if (planner->id() != id) {
 throw std::runtime_error("global planner factory returned mismatched id: " + std::string(id));
 }
 return planner;
}

std::vector<std::string> GlobalPlannerRegistry::ids() const {
 std::vector<std::string> result;
 result.reserve(factories_.size());
 for (const auto& entry : factories_) {
 result.push_back(entry.first);
 }
 return result;
}

GlobalPlannerRegistry GlobalPlannerRegistry::with_builtins() {
 GlobalPlannerRegistry registry;
 registry.register_factory("astar-grid", [] { return std::make_shared<AStarGlobalPlanner>(); });
 return registry;
}

std::shared_ptr<const IGlobalPlanner> default_global_planner() {
 static const auto planner = std::make_shared<AStarGlobalPlanner>();
 return planner;
}

std::shared_ptr<const IGlobalPlanner> ackermann_global_planner(
 double wheel_base_m, double max_steer_rad, double map_resolution_m) {
 sim::HybridAStarConfig config;
 config.wheel_base_m = wheel_base_m;
 config.max_steer_rad = max_steer_rad;
 config.primitive_length_m = std::max(map_resolution_m * 1.5, 0.35);
 config.goal_position_tolerance_m = std::max(map_resolution_m * 1.8, 0.45);
 return std::make_shared<HybridAStarGlobalPlanner>(config);
}

GlobalPlanningSystem::GlobalPlanningSystem(std::shared_ptr<const IGlobalPlanner> planner)
 : planner_(std::move(planner)) {
 if (!planner_) {
 throw std::invalid_argument("global planning system requires a planner");
 }
}

GlobalPlan GlobalPlanningSystem::plan(const sim::GridMap& map,
 const std::vector<PlanningPose>& waypoints, const sim::TraversalCost& traversal_cost) const {
 GlobalPlan result;
 if (waypoints.size() < 2) {
 result.diagnostic = "at-least-two-waypoints-required";
 return result;
 }
 result.ok = true;
 for (std::size_t index = 1; index < waypoints.size(); ++index) {
 auto segment = planner_->plan({map, waypoints[index - 1], waypoints[index], traversal_cost});
 result.expanded += segment.expanded;
 if (!segment.ok) {
 result.ok = false;
 result.path.clear();
 result.diagnostic = segment.diagnostic.empty() ? "no-path" : std::move(segment.diagnostic);
 return result;
 }
 if (!result.path.empty() && !segment.path.empty() &&
 std::hypot(result.path.back().x - segment.path.front().x,
 result.path.back().y - segment.path.front().y) < 1e-9) {
 segment.path.erase(segment.path.begin());
 }
 result.path.insert(result.path.end(), segment.path.begin(), segment.path.end());
 }
 return result;
}

} // namespace autonomy
