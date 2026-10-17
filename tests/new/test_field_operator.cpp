#include "sim/game/mission/content_catalog.hpp"
#include "sim/game/mission/campaign_progress.hpp"
#include "sim/game/scenario/scenario_session.hpp"
#include "sim/game/mission/systems/field_operator_system.hpp"

#include <cmath>
#include <filesystem>
#include <iostream>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

namespace {

bool near(sim::Vec2 first, sim::Vec2 second, double tolerance = 1e-9) {
 return std::hypot(first.x - second.x, first.y - second.y) <= tolerance;
}

} // namespace

int main() {
 mission::MapDefinition test_map;
 test_map.size_m = {10.0, 8.0};
 test_map.obstacles.push_back({"wall", {4.0, 2.0}, {5.0, 6.0}, {}});

 // The operator state is deterministic, normalized for diagonal input and
 // collides with authored level geometry rather than walking through it.
 mission::FieldOperatorSystem first;
 mission::FieldOperatorSystem repeat;
 first.reset(test_map, {1.0, 1.0}, "field_test");
 repeat.reset(test_map, {1.0, 1.0}, "field_test");
 CHECK(near(first.state().position, repeat.state().position));
 CHECK(near(first.state().home_position, sim::Vec2{1.0, 1.0}));
 CHECK(!near(first.state().position, first.state().home_position));
 first.set_move_command({1.0, 1.0});
 repeat.set_move_command({1.0, 1.0});
 for (int step = 0; step < 20; ++step) {
 first.update(0.05, test_map, "field_test");
 repeat.update(0.05, test_map, "field_test");
 }
 CHECK(near(first.state().position, repeat.state().position));
 CHECK(first.state().moving);
 CHECK(std::abs(std::hypot(first.state().velocity.x, first.state().velocity.y) -
 mission::FieldOperatorSystem::walk_speed_mps) < 1e-9);

 // All five intentions form one revisioned command stream. Only target-based
 // commands require a world point, and acknowledgements apply to the matching
 // revision so stale planner results cannot confirm a newer command.
 CHECK(first.issue_intent(mission::FieldIntentKind::Follow));
 CHECK(first.intent().revision == 1);
 CHECK(first.intent().acknowledgement == mission::FieldIntentAcknowledgement::Pending);
 CHECK(near(first.intent().target, first.state().position));
 first.acknowledge(0, true);
 CHECK(first.intent().acknowledgement == mission::FieldIntentAcknowledgement::Pending);
 first.acknowledge(1, true);
 CHECK(first.intent().acknowledgement == mission::FieldIntentAcknowledgement::Accepted);
 CHECK(!first.issue_intent(mission::FieldIntentKind::GoTo));
 CHECK(first.intent().revision == 1);
 CHECK(first.issue_intent(mission::FieldIntentKind::GoTo, sim::Vec2{7.0, 1.0}));
 CHECK(first.intent().revision == 2);
 CHECK(first.issue_intent(mission::FieldIntentKind::Search, sim::Vec2{7.0, 7.0}));
 CHECK(first.intent().revision == 3);
 CHECK(first.issue_intent(mission::FieldIntentKind::Wait));
 CHECK(first.intent().revision == 4);
 CHECK(first.issue_intent(mission::FieldIntentKind::Return));
 CHECK(first.intent().revision == 5);
 CHECK(near(first.intent().target, first.state().home_position));

 const auto catalog = mission::ContentCatalog::load(LOST_SIGNAL_TEST_ASSETS);
 const auto build = catalog.presentation_for_level("boot").recommended_build;

 // Story/garage missions reject field controls. Expedition mode accepts them
 // and turns each intention into a route through the existing planner.
 mission::ScenarioSession story(catalog, "boot", build);
 CHECK(!story.set_field_operator_command(1.0, 0.0));
 CHECK(!story.issue_field_intent(mission::FieldIntentKind::Follow));

 // Operator-drive playgrounds start under human authority. A click-to-drive
 // target plans through the normal autonomy stack, and any later keyboard
 // command takes authority back immediately.
 mission::ScenarioSession proving(
 catalog, "vehicle_proving_ground",
 catalog.presentation_for_level("vehicle_proving_ground").recommended_build);
 proving.start();
 CHECK(proving.manual_control_active());
 CHECK(proving.issue_field_intent(mission::FieldIntentKind::GoTo, sim::Vec2{7.5, 13.0}));
 CHECK(!proving.manual_control_active());
 CHECK(!proving.planned_route().empty());
 proving.set_manual_command(0.0, 0.0);
 CHECK(!proving.manual_control_active());
 for (int step = 0; step < 150; ++step)
 proving.step();
 CHECK(proving.result().path_length_m > 0.05);
 proving.set_manual_command(1.0, 0.0);
 CHECK(proving.manual_control_active());
 for (int step = 0; step < 75; ++step)
 proving.step();
 CHECK(std::abs(proving.snapshot().robot.v) > 0.25);
 proving.set_manual_command(0.0, 0.0);
 for (int step = 0; step < 250 && !proving.result().braking_measurement_valid; ++step)
 proving.step();
 CHECK(proving.result().braking_measurement_valid);
 CHECK(proving.result().last_stop_start_speed_mps > 0.25);
 CHECK(proving.result().last_stop_time_s > 0.0);
 CHECK(proving.result().last_stop_distance_m > 0.0);

 // Expedition input can move the human and express intent, but it can never
 // take direct authority over the robot chassis. A high-level GoTo command
 // must then produce a real route and autonomous Bullet motion.
 mission::ScenarioSession control_guard(catalog, "boot", build);
 control_guard.set_endless_mode(true);
 control_guard.start();
 control_guard.set_manual_command(1.0, 1.0);
 for (int step = 0; step < 25; ++step)
 control_guard.step();
 CHECK(control_guard.planned_route().empty());
 CHECK(control_guard.result().path_length_m < 1e-3);
 CHECK(control_guard.issue_field_intent(mission::FieldIntentKind::GoTo,
 sim::Vec2{6.0, 4.0}));
 CHECK(!control_guard.planned_route().empty());
 for (int step = 0; step < 150; ++step)
 control_guard.step();
 CHECK(control_guard.result().path_length_m > 0.05);

 mission::ScenarioSession expedition(catalog, "boot", build);
 expedition.set_endless_mode(true);
 expedition.start();
 CHECK(expedition.set_field_operator_command(1.0, 0.0));
 for (int step = 0; step < 50; ++step)
 expedition.step();
 CHECK(expedition.set_field_operator_command(0.0, 0.0));
 CHECK(expedition.field_operator().position.x >
 expedition.field_operator().home_position.x + 1.0);

 CHECK(expedition.issue_field_intent(mission::FieldIntentKind::Follow));
 CHECK(!expedition.planned_route().empty());
 CHECK(expedition.field_intent().acknowledgement ==
 mission::FieldIntentAcknowledgement::Accepted);

 CHECK(expedition.issue_field_intent(mission::FieldIntentKind::GoTo, sim::Vec2{6.0, 4.0}));
 CHECK(!expedition.planned_route().empty());
 CHECK(expedition.field_intent().acknowledgement ==
 mission::FieldIntentAcknowledgement::Accepted);

 CHECK(expedition.issue_field_intent(mission::FieldIntentKind::Search, sim::Vec2{7.0, 7.0}));
 CHECK(!expedition.planned_route().empty());
 CHECK(expedition.field_intent().acknowledgement ==
 mission::FieldIntentAcknowledgement::Accepted);

 CHECK(expedition.issue_field_intent(mission::FieldIntentKind::Wait));
 CHECK(expedition.planned_route().empty());
 CHECK(expedition.field_intent().acknowledgement ==
 mission::FieldIntentAcknowledgement::Accepted);

 CHECK(expedition.issue_field_intent(mission::FieldIntentKind::Return));
 CHECK(!expedition.planned_route().empty());
 CHECK(expedition.field_intent().acknowledgement ==
 mission::FieldIntentAcknowledgement::Accepted);

 // The world save carries the human position and revisioned companion intent.
 // Restoring on the same level re-plans that intent from the fresh robot pose.
 const auto save_path = std::filesystem::temp_directory_path() /
 "lost-signal-field-expedition-test.json";
 mission::CampaignProgress progress;
 progress.set_field_expedition(expedition.field_expedition_checkpoint());
 progress.save(save_path, catalog);
 const auto loaded = mission::CampaignProgress::load(save_path, catalog);
 CHECK(loaded.field_expedition().has_value());
 CHECK(loaded.field_expedition()->level_id == "boot");
 CHECK(near(loaded.field_expedition()->field_operator.position,
 expedition.field_operator().position));
 CHECK(loaded.field_expedition()->intent.kind == mission::FieldIntentKind::Return);
 CHECK(loaded.field_expedition()->intent.revision == expedition.field_intent().revision);

 mission::ScenarioSession resumed(catalog, "boot", build);
 resumed.set_endless_mode(true);
 resumed.restore_field_expedition(*loaded.field_expedition());
 resumed.start();
 CHECK(near(resumed.field_operator().position, expedition.field_operator().position));
 CHECK(resumed.field_intent().kind == mission::FieldIntentKind::Return);
 CHECK(resumed.field_intent().acknowledgement ==
 mission::FieldIntentAcknowledgement::Accepted);
 CHECK(!resumed.planned_route().empty());
 std::filesystem::remove(save_path);

 return 0;
}
