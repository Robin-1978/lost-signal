#include "goal_system.hpp"

#include <algorithm>
#include <cmath>

namespace mission {

void GoalSystem::reset(const ObjectiveDefinition& objective) {
 completed_.assign(objective.goals.size(), false);
 abandoned_.assign(objective.goals.size(), false);
 completion_order_.clear();
 abandonment_order_.clear();
 completed_count_ = 0;
 dwell_s_ = 0.0;
 active_goal_ = objective.goal_order == "autonomous" || objective.goals.empty()
 ? std::optional<std::size_t>{}
 : std::optional<std::size_t>{0};
}

std::vector<std::size_t> GoalSystem::eligible_goals(const ObjectiveDefinition& objective) const {
 std::vector<std::size_t> candidates;
 bool unfinished_nonterminal = false;
 for (std::size_t index = 0; index < objective.goals.size(); ++index) {
 if (!completed(index) && !abandoned(index) && objective.goals[index].action != "complete") {
 unfinished_nonterminal = true;
 break;
 }
 }
 for (std::size_t index = 0; index < objective.goals.size(); ++index) {
 if (completed(index) || abandoned(index))
 continue;
 const bool terminal = objective.goals[index].action == "complete";
 if (terminal != unfinished_nonterminal)
 candidates.push_back(index);
 }
 return candidates;
}

bool GoalSystem::activate(const ObjectiveDefinition& objective, std::size_t index) {
 if (active_goal_ || index >= objective.goals.size())
 return false;
 const auto eligible = eligible_goals(objective);
 if (std::find(eligible.begin(), eligible.end(), index) == eligible.end())
 return false;
 active_goal_ = index;
 dwell_s_ = 0.0;
 return true;
}

bool GoalSystem::abandon(const ObjectiveDefinition& objective, std::size_t index) {
 if (active_goal_ || index >= objective.goals.size() || completed(index) || abandoned(index) ||
 objective.goals[index].action == "complete") {
 return false;
 }
 abandoned_[index] = true;
 abandonment_order_.push_back(index);
 return true;
}

std::vector<std::string> GoalSystem::completed_goal_ids(const ObjectiveDefinition& objective) const {
 std::vector<std::string> ids;
 for (const std::size_t index : completion_order_) {
 if (index < objective.goals.size()) {
 const auto& goal = objective.goals[index];
 ids.push_back(goal.id.empty() ? goal.marker : goal.id);
 }
 }
 return ids;
}

std::vector<std::string> GoalSystem::abandoned_goal_ids(const ObjectiveDefinition& objective) const {
 std::vector<std::string> ids;
 for (const std::size_t index : abandonment_order_) {
 if (index < objective.goals.size()) {
 const auto& goal = objective.goals[index];
 ids.push_back(goal.id.empty() ? goal.marker : goal.id);
 }
 }
 return ids;
}

std::optional<GoalArrival> GoalSystem::update(const ObjectiveDefinition& objective, const MapDefinition& map,
 sim::Vec2 robot_position, double robot_speed_mps, bool completion_allowed, double dt_s) {
 if (!active_goal_ || *active_goal_ >= objective.goals.size()) {
  return std::nullopt;
 }
 const std::size_t index = *active_goal_;
 const GoalStep& goal = objective.goals[index];
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
 arrival.index = index;
 arrival.goal = goal;
 arrival.mission_complete = goal.action == "complete";
 completed_[index] = true;
 completion_order_.push_back(index);
 ++completed_count_;
 active_goal_.reset();
 if (objective.goal_order != "autonomous") {
 for (std::size_t next = index + 1; next < objective.goals.size(); ++next) {
 if (!completed(next) && !abandoned(next)) {
 active_goal_ = next;
 break;
 }
 }
 }
 dwell_s_ = 0.0;
 return arrival;
}

} // namespace mission
