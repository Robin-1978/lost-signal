#pragma once

#include "game/content.hpp"

#include <cstddef>
#include <optional>

namespace game {

struct GoalArrival {
 std::size_t index{0};
 GoalStep goal;
 bool mission_complete{false};
};

class GoalSystem {
 public:
 void reset() {
 next_goal_ = 0;
 dwell_s_ = 0.0;
 }
 std::size_t goals_reached() const { return next_goal_; }
 std::optional<GoalArrival> update(const ObjectiveDefinition& objective, const MapDefinition& map,
 sim::Vec2 robot_position, double robot_speed_mps, bool completion_allowed, double dt_s);

 private:
 std::size_t next_goal_{0};
 double dwell_s_{0.0};
};

} // namespace game
