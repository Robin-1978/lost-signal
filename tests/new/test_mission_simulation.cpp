#include "game/campaign_progress.hpp"
#include "game/content_catalog.hpp"
#include "game/mission_simulation.hpp"

#include <cmath>
#include <algorithm>
#include <filesystem>
#include <iostream>

#define CHECK(condition) do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
} while (false)

int main() {
 const auto catalog = game::ContentCatalog::load(LOST_SIGNAL_TEST_ASSETS);
 const auto build = catalog.presentation_for_level("lost_signal").recommended_build;

 game::MissionSimulation manual(
 catalog, "boot", catalog.presentation_for_level("boot").recommended_build);
 manual.start();
 manual.set_manual_command(1.0, 0.0);
 for (int step = 0; step < 50; ++step) manual.step();
 CHECK(manual.result().path_length_m > 0.1);

 for (const auto& level_id : catalog.campaign().levels) {
 const auto& level_presentation = catalog.presentation_for_level(level_id);
 game::MissionSimulation chapter_mission(
 catalog, level_id, level_presentation.recommended_build);
 chapter_mission.run_to_completion();
 CHECK(chapter_mission.result().status == game::MissionStatus::Succeeded);
 }

 game::MissionSimulation drifting(
 catalog, "localization", catalog.presentation_for_level("localization").default_build);
 drifting.run_to_completion();
 CHECK(drifting.result().status == game::MissionStatus::Succeeded);
 CHECK(!drifting.result().safe_score);

 game::MissionSimulation first(catalog, "lost_signal", build);
 CHECK(!first.planned_path().empty());
 first.run_to_completion();
 CHECK(first.result().status == game::MissionStatus::Succeeded);
 CHECK(first.result().via_reached);
 CHECK(first.result().collisions == 0);
 CHECK(first.result().safe_score);
 CHECK(first.result().min_dynamic_clearance_m >= first.level().safe_min_clearance_m);
 CHECK(first.result().astar_expanded > 0);
 CHECK(first.result().local_decisions > 0);
 CHECK(first.replay().size() > 10);
 CHECK(std::any_of(first.events().begin(), first.events().end(), [](const auto& event) {
 return event.code == "dynamic-detected";
 }));
 CHECK(std::any_of(first.events().begin(), first.events().end(), [](const auto& event) {
 return event.code == "waiting";
 }));
 CHECK(std::any_of(first.replay().begin(), first.replay().end(), [](const auto& snapshot) {
 return !snapshot.dynamic_obstacles.empty();
 }));
 CHECK(std::any_of(first.replay().begin(), first.replay().end(), [](const auto& snapshot) {
 return !snapshot.dynamic_obstacles.empty()
 && std::hypot(snapshot.dynamic_obstacles.front().estimated_velocity.x,
 snapshot.dynamic_obstacles.front().estimated_velocity.y) > 0.2;
 }));
  CHECK(std::any_of(first.replay().begin(), first.replay().end(), [](const auto& snapshot) {
  return snapshot.control.dwa_active && !snapshot.control.dwa_selected.empty();
  }));
  CHECK(std::any_of(first.replay().begin(), first.replay().end(), [](const auto& snapshot) {
  return std::abs(snapshot.control.heading_pid_p) > 0.01;
  }));
  CHECK(std::any_of(first.replay().begin(), first.replay().end(), [](const auto& snapshot) {
  return std::abs(snapshot.control.left_target_rpm) > 1.0
  || std::abs(snapshot.control.right_target_rpm) > 1.0;
  }));
  CHECK(std::any_of(first.replay().begin(), first.replay().end(), [](const auto& snapshot) {
  return std::abs(snapshot.control.left_pwm) > 0.01
  || std::abs(snapshot.control.right_pwm) > 0.01;
  }));
  CHECK(std::all_of(first.replay().begin(), first.replay().end(), [](const auto& snapshot) {
  return std::abs(snapshot.control.left_pwm) <= 1.0
  && std::abs(snapshot.control.right_pwm) <= 1.0;
  }));

 const auto progress_path = std::filesystem::temp_directory_path()
 / "lost-signal-progress-test.json";
 std::filesystem::remove(progress_path);
 game::CampaignProgress progress;
 progress.record_completion("lost_signal", first.result(), catalog);
 progress.save(progress_path, catalog);
 const auto loaded_progress = game::CampaignProgress::load(progress_path, catalog);
 CHECK(loaded_progress.completed_levels().size() == 1);
 CHECK(loaded_progress.completed_levels().front() == "lost_signal");
 CHECK(loaded_progress.best_scores().at("lost_signal").safe);
 std::filesystem::remove(progress_path);

 game::MissionSimulation default_attempt(
 catalog, "lost_signal", catalog.presentation_for_level("lost_signal").default_build);
 default_attempt.run_to_completion();
 CHECK(default_attempt.result().status == game::MissionStatus::Failed);
 CHECK(default_attempt.result().failure_reason == "collision-dynamic");
 CHECK(default_attempt.result().collisions == 1);

 game::BuildSelection mid_style = build;
 mid_style.style = 0.37;
 game::MissionSimulation mid_first(catalog, "lost_signal", mid_style);
 mid_first.run_to_completion();
 CHECK(mid_first.result().status == game::MissionStatus::Succeeded);
 game::MissionSimulation mid_second(catalog, "lost_signal", mid_style);
 mid_second.run_to_completion();
 CHECK(std::abs(mid_second.result().time_s - mid_first.result().time_s) < 1e-12);

 game::MissionSimulation second(catalog, "lost_signal", build);
 second.run_to_completion();
 CHECK(second.result().status == first.result().status);
 CHECK(std::abs(second.result().time_s - first.result().time_s) < 1e-12);
 CHECK(std::abs(second.result().path_length_m - first.result().path_length_m) < 1e-12);
 CHECK(second.events().size() == first.events().size());
 return 0;
}
