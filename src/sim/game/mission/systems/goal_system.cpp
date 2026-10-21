#include "goal_system.hpp"

#include <limits>
#include <stdexcept>
#include <utility>

namespace mission {

void GoalSystem::reset(const ObjectiveDefinition& objective, const sim::runtime::IMapProvider& maps) {
 executive::TaskPlan plan;
 plan.request_id = "scenario-objective";
 plan.ordering = objective.goal_order == "autonomous" || objective.goal_order == "player"
 ? executive::TaskOrdering::Autonomous
 : executive::TaskOrdering::Authored;
 plan.goals.reserve(objective.goals.size());
 for (const auto& goal : objective.goals) {
 const auto marker = maps.resolve_place(goal.marker);
 if (!marker)
 throw std::invalid_argument("task goal references an unresolved semantic place: " + goal.marker);
 const bool inspect = goal.action == "inspect";
 plan.goals.push_back({
 goal.id.empty() ? goal.marker : goal.id,
 marker->position_m,
 goal.radius_m,
 goal.action == "complete",
 inspect,
 inspect ? goal.dwell_s : 0.0,
 goal.action == "complete"
 ? objective.max_arrival_speed_mps
 : (inspect ? 0.18 : std::numeric_limits<double>::max()),
 });
 }
 if (!executive_.accept(std::move(plan)))
 throw std::invalid_argument("objective cannot be translated to a task plan");
}

std::vector<std::size_t> GoalSystem::eligible_goals(const ObjectiveDefinition& objective) const {
 (void)objective;
 return executive_.eligible_goals();
}

bool GoalSystem::activate(const ObjectiveDefinition& objective, std::size_t index) {
 (void)objective;
 return executive_.activate(index);
}

bool GoalSystem::abandon(const ObjectiveDefinition& objective, std::size_t index) {
 (void)objective;
 return executive_.abandon(index);
}

std::vector<std::string> GoalSystem::completed_goal_ids(const ObjectiveDefinition& objective) const {
 (void)objective;
 return executive_.completed_goal_ids();
}

std::vector<std::string> GoalSystem::abandoned_goal_ids(const ObjectiveDefinition& objective) const {
 (void)objective;
 return executive_.abandoned_goal_ids();
}

std::optional<GoalArrival> GoalSystem::update(const ObjectiveDefinition& objective,
 sim::Vec2 robot_position, double robot_speed_mps, bool completion_allowed, double dt_s) {
 const auto active = executive_.active_goal_index();
 if (!active || *active >= objective.goals.size())
 return std::nullopt;
 const GoalStep goal = objective.goals[*active];
 const auto arrival = executive_.update({
 robot_position, robot_speed_mps, completion_allowed, dt_s,
 });
 if (!arrival)
 return std::nullopt;
 return GoalArrival{arrival->index, goal, arrival->task_complete};
}

executive::LocalizationDirective GoalSystem::localization_directive(
 sim::Vec2 robot_position, double robot_speed_mps,
 bool localization_valid, double localization_confidence,
 double localization_age_s) const {
 return executive_.localization_directive({
 robot_position, robot_speed_mps, true, 0.0,
 localization_valid, localization_confidence, localization_age_s});
}

} // namespace mission
