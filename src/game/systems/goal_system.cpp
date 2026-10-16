#include "goal_system.hpp"

#include <cmath>

namespace game {

std::optional<GoalArrival> GoalSystem::update(const ObjectiveDefinition& objective, const MapDefinition& map,
 sim::Vec2 robot_position, double robot_speed_mps, bool completion_allowed, double dt_s) {
 if (next_goal_ >= objective.goals.size()) {
  return std::nullopt;
 }
 const GoalStep& goal = objective.goals[next_goal_];
 const auto marker = map.markers.at(goal.marker);
 if (std::hypot(robot_position.x - marker.x, robot_position.y - marker.y) > goal.radius_m) {
 dwell_s_ = 0.0;
  return std::nullopt;
 }
 if (goal.action == "complete" && !completion_allowed) {
  return std::nullopt;
 }
 if (goal.action == "complete" && std::abs(robot_speed_mps) > objective.max_arrival_speed_mps) {
 return std::nullopt;
 }
 if (goal.action == "inspect") {
 if (std::abs(robot_speed_mps) > 0.18) {
 dwell_s_ = 0.0;
 return std::nullopt;
 }
 dwell_s_ += dt_s;
 if (dwell_s_ + 1e-9 < goal.dwell_s)
 return std::nullopt;
 }
 GoalArrival arrival;
 arrival.index = next_goal_;
 arrival.goal = goal;
 arrival.mission_complete = goal.action == "complete";
 ++next_goal_;
 dwell_s_ = 0.0;
 return arrival;
}

} // namespace game
