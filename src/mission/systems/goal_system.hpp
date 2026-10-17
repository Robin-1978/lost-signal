#pragma once

#include "mission/content.hpp"

#include <cstddef>
#include <optional>
#include <string>
#include <vector>

namespace mission {

struct GoalArrival {
 std::size_t index{0};
 GoalStep goal;
 bool mission_complete{false};
};

class GoalSystem {
 public:
 void reset(const ObjectiveDefinition& objective);
 std::size_t goals_reached() const { return completed_count_; }
 std::optional<std::size_t> active_goal_index() const { return active_goal_; }
 bool activate(const ObjectiveDefinition& objective, std::size_t index);
 bool abandon(const ObjectiveDefinition& objective, std::size_t index);
 bool completed(std::size_t index) const {
 return index < completed_.size() && completed_[index];
 }
 bool abandoned(std::size_t index) const {
 return index < abandoned_.size() && abandoned_[index];
 }
 std::vector<std::size_t> eligible_goals(const ObjectiveDefinition& objective) const;
 std::vector<std::string> completed_goal_ids(const ObjectiveDefinition& objective) const;
 std::vector<std::string> abandoned_goal_ids(const ObjectiveDefinition& objective) const;
 std::optional<GoalArrival> update(const ObjectiveDefinition& objective, const MapDefinition& map,
 sim::Vec2 robot_position, double robot_speed_mps, bool completion_allowed, double dt_s);

 private:
 std::vector<bool> completed_;
 std::vector<bool> abandoned_;
 std::vector<std::size_t> completion_order_;
 std::vector<std::size_t> abandonment_order_;
 std::optional<std::size_t> active_goal_;
 std::size_t completed_count_{0};
 double dwell_s_{0.0};
};

} // namespace mission
