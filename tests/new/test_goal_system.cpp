#include "sim/game/mission/systems/goal_system.hpp"
#include "sim/game/mission/deployable_map_adapter.hpp"

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
 map.id = "test-map";
 map.size_m = {4.0, 2.0};
 map.grid_resolution_m = 0.5;
 map.markers.emplace("a", mission::Pose{1.0, 0.0, 0.0});
 map.markers.emplace("b", mission::Pose{2.0, 0.0, 0.0});
 map.markers.emplace("home", mission::Pose{0.0, 0.0, 0.0});
 return map;
}

} // namespace

int main() {
 const auto map = map_with_markers();
 sim::runtime::MapManager maps;
 CHECK(maps.configure(mission::make_deployable_map_snapshot(map, "test")));
 CHECK(maps.activate());

 mission::ObjectiveDefinition authored;
 authored.goal_order = "authored";
 authored.goals = {goal("first", "a", "pickup"), goal("finish", "home", "complete")};
 mission::GoalSystem ordered;
 ordered.reset(authored, maps);
 CHECK(ordered.active_goal_index() && *ordered.active_goal_index() == 0);
 const auto first = ordered.update(authored, {1.0, 0.0}, 0.0, true, 0.02);
 CHECK(first && first->index == 0 && !first->mission_complete);
 CHECK(ordered.active_goal_index() && *ordered.active_goal_index() == 1);
 const auto finish = ordered.update(authored, {0.0, 0.0}, 0.0, true, 0.02);
 CHECK(finish && finish->mission_complete);
 CHECK(!ordered.active_goal_index());

 mission::ObjectiveDefinition stopped_task;
 auto inspect = goal("inspect", "a", "inspect");
 inspect.dwell_s = 0.10;
 stopped_task.goals = {inspect, goal("finish", "home", "complete")};
 mission::GoalSystem inspection;
 inspection.reset(stopped_task, maps);
 // Entering the radius at driving speed never consumes stopped dwell time.
 for (int sample = 0; sample < 10; ++sample)
 CHECK(!inspection.update(stopped_task, {1.0, 0.0}, 0.25, true, 0.02));
 for (int sample = 0; sample < 4; ++sample)
 CHECK(!inspection.update(stopped_task, {1.0, 0.0}, 0.0, true, 0.02));
 const auto inspected = inspection.update(stopped_task, {1.0, 0.0}, 0.0, true, 0.02);
 CHECK(inspected && inspected->goal.action == "inspect");

 mission::ObjectiveDefinition autonomous;
 autonomous.goal_order = "autonomous";
 autonomous.goals = {
 goal("apple-a", "a", "collect"),
 goal("apple-b", "b", "collect"),
 goal("packing-pad", "home", "complete"),
 };
 mission::GoalSystem tasks;
 tasks.reset(autonomous, maps);
 CHECK(!tasks.active_goal_index());
 CHECK(tasks.eligible_goals(autonomous) == std::vector<std::size_t>({0, 1}));
 CHECK(!tasks.activate(autonomous, 2));
 CHECK(tasks.activate(autonomous, 1));
 const auto second_first = tasks.update(autonomous, {2.0, 0.0}, 0.0, true, 0.02);
 CHECK(second_first && second_first->index == 1);
 CHECK(tasks.eligible_goals(autonomous) == std::vector<std::size_t>({0}));
 CHECK(tasks.activate(autonomous, 0));
 CHECK(tasks.update(autonomous, {1.0, 0.0}, 0.0, true, 0.02));
 CHECK(tasks.eligible_goals(autonomous) == std::vector<std::size_t>({2}));
 CHECK(tasks.completed_goal_ids(autonomous) ==
 std::vector<std::string>({"apple-b", "apple-a"}));
 CHECK(tasks.activate(autonomous, 2));
 CHECK(tasks.update(autonomous, {0.0, 0.0}, 0.0, true, 0.02)->mission_complete);

 mission::ObjectiveDefinition player = autonomous;
 player.goal_order = "player";
 mission::GoalSystem directed;
 directed.reset(player, maps);
 CHECK(!directed.active_goal_index());
 CHECK(directed.eligible_goals(player) == std::vector<std::size_t>({0, 1}));
 CHECK(!directed.activate(player, 2));
 CHECK(directed.activate(player, 0));
 CHECK(directed.update(player, {1.0, 0.0}, 0.0, true, 0.02));
 CHECK(directed.eligible_goals(player) == std::vector<std::size_t>({1}));

 mission::GoalSystem curtailed;
 curtailed.reset(autonomous, maps);
 CHECK(curtailed.abandon(autonomous, 1));
 CHECK(curtailed.eligible_goals(autonomous) == std::vector<std::size_t>({0}));
 CHECK(curtailed.abandon(autonomous, 0));
 CHECK(curtailed.eligible_goals(autonomous) == std::vector<std::size_t>({2}));
 CHECK(curtailed.abandoned_goal_ids(autonomous) ==
 std::vector<std::string>({"apple-b", "apple-a"}));
 return 0;
}
