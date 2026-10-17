#include "robot/executive/task_executive.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <unordered_set>

namespace executive {

bool TaskExecutive::accept(TaskPlan plan) {
 if (plan.request_id.empty() || plan.goals.empty())
 return false;
 std::unordered_set<std::string> ids;
 for (const auto& goal : plan.goals) {
 if (goal.id.empty() || !ids.insert(goal.id).second ||
 !std::isfinite(goal.position.x) || !std::isfinite(goal.position.y) ||
 !std::isfinite(goal.radius_m) || goal.radius_m <= 0.0 ||
 !std::isfinite(goal.dwell_s) || goal.dwell_s < 0.0 ||
 !std::isfinite(goal.max_arrival_speed_mps) || goal.max_arrival_speed_mps < 0.0) {
 return false;
 }
 }
 plan_ = std::move(plan);
 completed_.assign(plan_.goals.size(), false);
 abandoned_.assign(plan_.goals.size(), false);
 completion_order_.clear();
 abandonment_order_.clear();
 completed_count_ = 0;
 dwell_s_ = 0.0;
 active_goal_ = plan_.ordering == TaskOrdering::Authored
 ? std::optional<std::size_t>{0}
 : std::optional<std::size_t>{};
 state_ = TaskState::Active;
 return true;
}

void TaskExecutive::clear() {
 plan_ = {};
 completed_.clear();
 abandoned_.clear();
 completion_order_.clear();
 abandonment_order_.clear();
 active_goal_.reset();
 completed_count_ = 0;
 dwell_s_ = 0.0;
 state_ = TaskState::Idle;
}

bool TaskExecutive::pause() {
 if (state_ != TaskState::Active)
 return false;
 state_ = TaskState::Paused;
 return true;
}

bool TaskExecutive::resume() {
 if (state_ != TaskState::Paused)
 return false;
 state_ = TaskState::Active;
 return true;
}

bool TaskExecutive::cancel() {
 if (state_ != TaskState::Active && state_ != TaskState::Paused)
 return false;
 state_ = TaskState::Cancelled;
 active_goal_.reset();
 dwell_s_ = 0.0;
 return true;
}

bool TaskExecutive::fail() {
 if (state_ != TaskState::Active && state_ != TaskState::Paused)
 return false;
 state_ = TaskState::Failed;
 active_goal_.reset();
 dwell_s_ = 0.0;
 return true;
}

bool TaskExecutive::completed(std::size_t index) const {
 return index < completed_.size() && completed_[index];
}

bool TaskExecutive::abandoned(std::size_t index) const {
 return index < abandoned_.size() && abandoned_[index];
}

std::vector<std::size_t> TaskExecutive::eligible_goals() const {
 std::vector<std::size_t> candidates;
 bool unfinished_nonterminal = false;
 for (std::size_t index = 0; index < plan_.goals.size(); ++index) {
 if (!completed(index) && !abandoned(index) && !plan_.goals[index].terminal) {
 unfinished_nonterminal = true;
 break;
 }
 }
 for (std::size_t index = 0; index < plan_.goals.size(); ++index) {
 if (completed(index) || abandoned(index))
 continue;
 if (plan_.goals[index].terminal != unfinished_nonterminal)
 candidates.push_back(index);
 }
 return candidates;
}

bool TaskExecutive::activate(std::size_t index) {
 if (state_ != TaskState::Active || active_goal_ || index >= plan_.goals.size())
 return false;
 const auto eligible = eligible_goals();
 if (std::find(eligible.begin(), eligible.end(), index) == eligible.end())
 return false;
 active_goal_ = index;
 dwell_s_ = 0.0;
 return true;
}

bool TaskExecutive::abandon(std::size_t index) {
 if (state_ != TaskState::Active || active_goal_ || index >= plan_.goals.size() ||
 completed(index) || abandoned(index) || plan_.goals[index].terminal) {
 return false;
 }
 abandoned_[index] = true;
 abandonment_order_.push_back(index);
 return true;
}

std::vector<std::string> TaskExecutive::completed_goal_ids() const {
 std::vector<std::string> ids;
 ids.reserve(completion_order_.size());
 for (const auto index : completion_order_)
 ids.push_back(plan_.goals[index].id);
 return ids;
}

std::vector<std::string> TaskExecutive::abandoned_goal_ids() const {
 std::vector<std::string> ids;
 ids.reserve(abandonment_order_.size());
 for (const auto index : abandonment_order_)
 ids.push_back(plan_.goals[index].id);
 return ids;
}

std::optional<TaskArrival> TaskExecutive::update(const TaskObservation& observation) {
 if (state_ != TaskState::Active || !active_goal_ || *active_goal_ >= plan_.goals.size() ||
 !std::isfinite(observation.position.x) || !std::isfinite(observation.position.y) ||
 !std::isfinite(observation.speed_mps) || !std::isfinite(observation.dt_s) ||
 observation.dt_s < 0.0) {
 return std::nullopt;
 }
 const std::size_t index = *active_goal_;
 const auto& goal = plan_.goals[index];
 if (std::hypot(observation.position.x - goal.position.x,
 observation.position.y - goal.position.y) > goal.radius_m) {
 dwell_s_ = 0.0;
 return std::nullopt;
 }
 if (goal.terminal && !observation.completion_allowed)
 return std::nullopt;
 if (std::abs(observation.speed_mps) > goal.max_arrival_speed_mps)
 return std::nullopt;
 if (goal.requires_stop) {
 dwell_s_ += observation.dt_s;
 if (dwell_s_ + 1e-9 < goal.dwell_s)
 return std::nullopt;
 }

 completed_[index] = true;
 completion_order_.push_back(index);
 ++completed_count_;
 active_goal_.reset();
 const bool complete = goal.terminal;
 if (complete) {
 state_ = TaskState::Succeeded;
 } else if (plan_.ordering == TaskOrdering::Authored) {
 for (std::size_t next = index + 1; next < plan_.goals.size(); ++next) {
 if (!completed(next) && !abandoned(next)) {
 active_goal_ = next;
 break;
 }
 }
 }
 dwell_s_ = 0.0;
 return TaskArrival{index, goal.id, complete};
}

} // namespace executive
