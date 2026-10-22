#pragma once

#include "robot/executive/robot_executive.hpp"
#include "sim/game/mission/content.hpp"
#include "robot/runtime/map_provider.hpp"
#include "robot/runtime/robot_application.hpp"

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

// Scenario adapter around the deployable TaskExecutive. It translates map
// markers and game action metadata once; generic goal lifecycle stays outside
// the simulation layer.
class GoalSystem {
 public:
 void bind(sim::robot::RobotExecutive& executive) { executive_ = &executive; }
 // The normal SIM composition binds the application host. This keeps the
 // scenario adapter on the same public boundary as a future UI instead of
 // retaining a separate Executive ownership path.
 void bind(sim::robot::RobotApplication& application) {
 application_ = &application;
 executive_ = nullptr;
 }
 void reset(const ObjectiveDefinition& objective, const sim::runtime::IMapProvider& maps);
 std::size_t goals_reached() const { return robot_executive().goals_reached(); }
 std::optional<std::size_t> active_goal_index() const { return robot_executive().active_goal_index(); }
 bool activate(const ObjectiveDefinition& objective, std::size_t index);
 bool abandon(const ObjectiveDefinition& objective, std::size_t index);
 bool completed(std::size_t index) const { return robot_executive().completed(index); }
 bool abandoned(std::size_t index) const { return robot_executive().abandoned(index); }
 std::vector<std::size_t> eligible_goals(const ObjectiveDefinition& objective) const;
 std::vector<std::string> completed_goal_ids(const ObjectiveDefinition& objective) const;
 std::vector<std::string> abandoned_goal_ids(const ObjectiveDefinition& objective) const;
 std::optional<GoalArrival> update(const ObjectiveDefinition& objective,
 sim::Vec2 robot_position, double robot_speed_mps, bool completion_allowed, double dt_s);
 executive::LocalizationDirective localization_directive(
 sim::Vec2 robot_position, double robot_speed_mps,
 bool localization_valid, double localization_confidence,
 double localization_age_s) const;

 private:
 sim::robot::RobotExecutive& robot_executive() {
 if (application_)
 return application_->executive();
 return executive_ ? *executive_ : fallback_;
 }
 const sim::robot::RobotExecutive& robot_executive() const {
 if (application_)
 return application_->executive();
 return executive_ ? *executive_ : fallback_;
 }

 sim::robot::RobotExecutive fallback_;
 sim::robot::RobotExecutive* executive_{nullptr};
 sim::robot::RobotApplication* application_{nullptr};
};

} // namespace mission
