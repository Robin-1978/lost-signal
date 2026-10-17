#include "mission/content_catalog.hpp"
#include "scenario/scenario_session.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <string>
#include <vector>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

namespace {

void configure_orchard(const std::filesystem::path& assets, double time_limit_s,
 const std::string& spawn = "packing_pad") {
 const auto path = assets / "levels" / "orchard_scramble.json";
 std::ifstream input(path);
 nlohmann::json level;
 input >> level;
 level["objective"]["time_limit_s"] = time_limit_s;
 level["objective"]["spawn"] = spawn;
 std::ofstream output(path, std::ios::trunc);
 output << level.dump(2) << '\n';
}

bool has_event(const mission::ScenarioSession& run, const std::string& code) {
 return std::any_of(run.events().begin(), run.events().end(),
 [&](const auto& event) { return event.code == code; });
}

} // namespace

int main() {
 const auto catalog = mission::ContentCatalog::load(LOST_SIGNAL_TEST_ASSETS);
 const auto& playgrounds = catalog.campaign().playgrounds;
 CHECK(std::find(playgrounds.begin(), playgrounds.end(), "orchard_scramble") != playgrounds.end());
 CHECK(std::find(catalog.campaign().levels.begin(), catalog.campaign().levels.end(),
 "orchard_scramble") == catalog.campaign().levels.end());

 const auto base = catalog.presentation_for_level("orchard_scramble").recommended_build;
 CHECK(catalog.level("orchard_scramble").environment_keyframes.size() == 4);
 std::set<std::string> first_targets;
 for (const auto& policy : {"task.balanced", "task.safest", "task.fastest", "task.valuable"}) {
 auto build = base;
 build.task_policy = policy;
 mission::ScenarioSession run(catalog, "orchard_scramble", build, std::nullopt, {},
 mission::ComponentRegistry::builtins(), {}, mission::PhysicsMode::Domain);
 CHECK(run.payload_state().mass_kg == 0.0);
 run.run_to_completion();
 CHECK(run.result().status == mission::MissionStatus::Succeeded);
 CHECK(run.level().objective.task_policy == policy);
 CHECK(run.payload_state().history.size() == 5);
 CHECK(run.payload_state().item_count == 35);
 CHECK(std::abs(run.payload_state().mass_kg - 19.0) < 1e-12);
 CHECK(std::abs(run.payload_state().first_moment_kgm - 12.7) < 1e-12);
 CHECK(run.snapshot().abandoned_goal_ids.empty());
 CHECK(run.snapshot().completed_goal_ids.size() == 6);
 CHECK(run.snapshot().completed_goal_ids.back() == "packing-pad");
 CHECK(run.snapshot().completed_goal_ids.front() == run.payload_state().history.front().goal_id);
 CHECK(run.result().peak_wetness > 0.35);
 CHECK(run.result().min_visibility < 0.9);
 CHECK(has_event(run, "environment-changed"));
 first_targets.insert(run.snapshot().completed_goal_ids.front());

 double previous_mass = 0.0;
 double previous_wetness = 0.0;
 double previous_visibility = 1.0;
 std::size_t previous_pickups = 0;
 for (const auto& frame : run.replay()) {
 CHECK(frame.payload.mass_kg + 1e-12 >= previous_mass);
 CHECK(frame.payload.history.size() >= previous_pickups);
 CHECK(frame.environment.wetness + 1e-12 >= previous_wetness);
 CHECK(frame.environment.visibility <= previous_visibility + 1e-12);
 previous_mass = frame.payload.mass_kg;
 previous_wetness = frame.environment.wetness;
 previous_visibility = frame.environment.visibility;
 previous_pickups = frame.payload.history.size();
 }
 CHECK(std::abs(previous_mass - 19.0) < 1e-12);
 }
 // The player choice must change behavior, not only a label. This map gives
 // at least three distinct first targets across the four doctrines.
 CHECK(first_targets.size() >= 3);

 namespace fs = std::filesystem;
 const fs::path overlay = fs::temp_directory_path() / "lost_signal_orchard_reserve_assets";
 std::error_code error;
 fs::remove_all(overlay, error);
 error.clear();
 fs::copy(LOST_SIGNAL_TEST_ASSETS, overlay,
 fs::copy_options::recursive | fs::copy_options::overwrite_existing, error);
 CHECK(!error);

 // A constrained but feasible run keeps the most useful work, abandons the
 // low-value detours, and returns home. Repeating the same run must make
 // the same scheduling decision.
 configure_orchard(overlay, 125.0);
 const auto constrained_catalog = mission::ContentCatalog::load(overlay);
 auto constrained_build =
 constrained_catalog.presentation_for_level("orchard_scramble").recommended_build;
 constrained_build.task_policy = "task.valuable";
 mission::ScenarioSession constrained_a(
 constrained_catalog, "orchard_scramble", constrained_build, std::nullopt, {},
 mission::ComponentRegistry::builtins(), {}, mission::PhysicsMode::Domain);
 mission::ScenarioSession constrained_b(
 constrained_catalog, "orchard_scramble", constrained_build, std::nullopt, {},
 mission::ComponentRegistry::builtins(), {}, mission::PhysicsMode::Domain);
 constrained_a.run_to_completion();
 constrained_b.run_to_completion();
 CHECK(constrained_a.result().status == mission::MissionStatus::Succeeded);
 CHECK(constrained_a.snapshot().completed_goal_ids ==
 std::vector<std::string>({"ridge-pocket", "side-hill", "outer-row",
 "orchard-aisle", "packing-pad"}));
 CHECK(constrained_a.snapshot().abandoned_goal_ids ==
 std::vector<std::string>({"muddy-swale"}));
 CHECK(has_event(constrained_a, "task-abandoned"));
 CHECK(constrained_b.snapshot().completed_goal_ids == constrained_a.snapshot().completed_goal_ids);
 CHECK(constrained_b.snapshot().abandoned_goal_ids == constrained_a.snapshot().abandoned_goal_ids);
 CHECK(std::abs(constrained_b.result().time_s - constrained_a.result().time_s) < 1e-12);

 // If the robot starts away from home with too little time even for the
 // terminal return, report the violated reserve contract instead of a generic
 // route or timeout failure.
 configure_orchard(overlay, 5.0, "apple_ridge_pocket");
 const auto impossible_catalog = mission::ContentCatalog::load(overlay);
 auto impossible_build =
 impossible_catalog.presentation_for_level("orchard_scramble").recommended_build;
 mission::ScenarioSession impossible(
 impossible_catalog, "orchard_scramble", impossible_build, std::nullopt, {},
 mission::ComponentRegistry::builtins(), {}, mission::PhysicsMode::Domain);
 impossible.run_to_completion();
 CHECK(impossible.result().status == mission::MissionStatus::Failed);
 CHECK(impossible.result().failure_reason == "return-reserve");
 CHECK(has_event(impossible, "return-reserve"));

 fs::remove_all(overlay, error);
 return 0;
}
