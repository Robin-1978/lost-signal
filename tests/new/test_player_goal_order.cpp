#include "sim/game/mission/content_catalog.hpp"
#include "sim/game/scenario/scenario_session.hpp"

#include <algorithm>
#include <filesystem>
#include <iostream>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

int main() {
 const auto catalog = mission::ContentCatalog::load(LOST_SIGNAL_TEST_ASSETS);
 const auto& presentation = catalog.presentation_for_level("stormglass_protocol");
 mission::ScenarioSession session(
 catalog, "stormglass_protocol", presentation.recommended_build);

 CHECK(session.level().objective.goal_order == "player");
 CHECK(!session.active_goal());
 CHECK(session.planned_route().empty());
 CHECK(session.eligible_goals() == std::vector<std::size_t>({0, 1, 2}));
 CHECK(presentation.prediction_choices.size() == 3);
 CHECK(presentation.prediction_choices[0].goal_id == "flood-core");

 session.start();
 CHECK(session.snapshot().status == mission::MissionStatus::Running);
 CHECK(!session.active_goal());
 CHECK(!session.select_goal(3));
 CHECK(!session.select_goal_near({0.0, 0.0}, 0.5));
 CHECK(session.select_goal("ridge-core"));
 CHECK(session.active_goal());
 CHECK(session.active_goal()->id == "ridge-core");
 CHECK(!session.planned_route().empty());
 CHECK(session.snapshot().active_goal_id == "ridge-core");
 CHECK(session.eligible_goals() == std::vector<std::size_t>({0, 1, 2}));
 CHECK(std::any_of(session.events().begin(), session.events().end(),
 [](const auto& event) { return event.code == "target-selected"; }));
 CHECK(!session.select_goal(0));
 return 0;
}
