#include "mission/systems/goal_system.hpp"

#include <iostream>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

namespace {

mission::GoalStep goal(std::string id, std::string marker, std::string action) {
 mission::GoalStep value;
 value.id = std::move(id);
 value.marker = std::move(marker);
 value.label = value.id;
 value.action = std::move(action);
 value.radius_m = 0.5;
 return value;
}

mission::MapDefinition map_with_markers() {
 mission::MapDefinition map;
 map.markers.emplace("a", mission::Pose{1.0, 0.0, 0.0});
 map.markers.emplace("b", mission::Pose{2.0, 0.0, 0.0});
 map.markers.emplace("home", mission::Pose{0.0, 0.0, 0.0});
 return map;
}

} // namespace

int main() {
 const auto map = map_with_markers();

 mission::ObjectiveDefinition authored;
 authored.goal_order = "authored";
 authored.goals = {goal("first", "a", "pickup"), goal("finish", "home", "complete")};
 mission::GoalSystem ordered;
 ordered.reset(authored);
 CHECK(ordered.active_goal_index() && *ordered.active_goal_index() == 0);
 const auto first = ordered.update(authored, map, {1.0, 0.0}, 0.0, true, 0.02);
 CHECK(first && first->index == 0 && !first->mission_complete);
 CHECK(ordered.active_goal_index() && *ordered.active_goal_index() == 1);
 const auto finish = ordered.update(authored, map, {0.0, 0.0}, 0.0, true, 0.02);
 CHECK(finish && finish->mission_complete);
 CHECK(!ordered.active_goal_index());

 mission::ObjectiveDefinition stopped_task;
 auto inspect = goal("inspect", "a", "inspect");
 inspect.dwell_s = 0.10;
 stopped_task.goals = {inspect, goal("finish", "home", "complete")};
 mission::GoalSystem inspection;
 inspection.reset(stopped_task);
 // Entering the radius at driving speed never consumes stopped dwell time.
 for (int sample = 0; sample < 10; ++sample)
 CHECK(!inspection.update(stopped_task, map, {1.0, 0.0}, 0.25, true, 0.02));
 for (int sample = 0; sample < 4; ++sample)
 CHECK(!inspection.update(stopped_task, map, {1.0, 0.0}, 0.0, true, 0.02));
 const auto inspected = inspection.update(stopped_task, map, {1.0, 0.0}, 0.0, true, 0.02);
 CHECK(inspected && inspected->goal.action == "inspect");

 mission::ObjectiveDefinition autonomous;
 autonomous.goal_order = "autonomous";
 autonomous.goals = {
 goal("apple-a", "a", "collect"),
 goal("apple-b", "b", "collect"),
 goal("packing-pad", "home", "complete"),
 };
 mission::GoalSystem tasks;
 tasks.reset(autonomous);
 CHECK(!tasks.active_goal_index());
 CHECK(tasks.eligible_goals(autonomous) == std::vector<std::size_t>({0, 1}));
 CHECK(!tasks.activate(autonomous, 2));
 CHECK(tasks.activate(autonomous, 1));
 const auto second_first = tasks.update(autonomous, map, {2.0, 0.0}, 0.0, true, 0.02);
 CHECK(second_first && second_first->index == 1);
 CHECK(tasks.eligible_goals(autonomous) == std::vector<std::size_t>({0}));
 CHECK(tasks.activate(autonomous, 0));
 CHECK(tasks.update(autonomous, map, {1.0, 0.0}, 0.0, true, 0.02));
 CHECK(tasks.eligible_goals(autonomous) == std::vector<std::size_t>({2}));
 CHECK(tasks.completed_goal_ids(autonomous) ==
 std::vector<std::string>({"apple-b", "apple-a"}));
 CHECK(tasks.activate(autonomous, 2));
 CHECK(tasks.update(autonomous, map, {0.0, 0.0}, 0.0, true, 0.02)->mission_complete);

 mission::GoalSystem curtailed;
 curtailed.reset(autonomous);
 CHECK(curtailed.abandon(autonomous, 1));
 CHECK(curtailed.eligible_goals(autonomous) == std::vector<std::size_t>({0}));
 CHECK(curtailed.abandon(autonomous, 0));
 CHECK(curtailed.eligible_goals(autonomous) == std::vector<std::size_t>({2}));
 CHECK(curtailed.abandoned_goal_ids(autonomous) ==
 std::vector<std::string>({"apple-b", "apple-a"}));
 return 0;
}
