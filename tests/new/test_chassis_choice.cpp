// Player chassis choice (backend slice; TGUI column deferred to desktop).
// Contract: presentation opt-in (absent = fixed, today's path), robot unlock
// gates, validated builds honored by the mission. boot opts in [ro1, ro2]
// (RO-2 verified to complete it below); all other levels stay fixed.

#include "game/campaign_progress.hpp"
#include "game/content_catalog.hpp"
#include "game/mission_simulation.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>

#define CHECK(condition)                                                                                     \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n";                            \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

namespace {

bool throws_like(const std::string& text) {
  try {
    throw;
  } catch (const std::exception& e) {
    return std::string(e.what()).find(text) != std::string::npos;
  } catch (...) {
    return false;
  }
}

} // namespace

int main() {
 const auto catalog = game::ContentCatalog::load(LOST_SIGNAL_TEST_ASSETS);
 // Registry/options content: boot offers ro1+ro2, ro2 locked behind its
 // ability, ro1/ro3 starter-or-absent as designed.
 {
  const auto& options = catalog.presentation_for_level("boot").chassis_options;
  CHECK(options.size() == 2 && options[0] == "robot.ro1" && options[1] == "robot.ro2");
  CHECK(catalog.robot("robot.ro2").unlock_ability == "ability.ackermann");
  CHECK(catalog.robot("robot.ro3").unlock_ability == "ability.tracked");
  CHECK(catalog.robot("robot.ro1").unlock_ability.empty());
  CHECK(catalog.presentation_for_level("lost_signal").chassis_options.empty());
 }
 // Default path bit-identical: empty request keeps presentation/companion.
 {
  game::MissionSimulation mission(catalog, "boot",
                                  catalog.presentation_for_level("boot").recommended_build);
  CHECK(mission.robot().drive_type == "differential");
  mission.run_to_completion();
  CHECK(mission.result().status == game::MissionStatus::Succeeded);
 }
 // Positive: RO-2 on boot with the ability owned (verified combo, pinned).
 {
  auto build = catalog.presentation_for_level("boot").recommended_build;
  build.chassis_robot = "robot.ro2";
  game::MissionSimulation mission(catalog, "boot", build, 1u, {"ability.ackermann"});
  CHECK(mission.robot().drive_type == "ackermann");
  mission.run_to_completion();
  CHECK(mission.result().status == game::MissionStatus::Succeeded);
  CHECK(mission.result().time_s < 10.0);
 }
 // Locked without the ability.
 {
  auto build = catalog.presentation_for_level("boot").recommended_build;
  build.chassis_robot = "robot.ro2";
  try {
   game::MissionSimulation mission(catalog, "boot", build, 1u);
   CHECK(false);
  } catch (const std::exception&) {
   CHECK(throws_like("ability.ackermann"));
  }
 }
 // Not offered on this level (ro3), even with its ability.
 {
  auto build = catalog.presentation_for_level("boot").recommended_build;
  build.chassis_robot = "robot.ro3";
  try {
   game::MissionSimulation mission(catalog, "boot", build, 1u, {"ability.tracked"});
   CHECK(false);
  } catch (const std::exception&) {
   CHECK(throws_like("not offered"));
  }
 }
 // Unknown robot id.
 {
  auto build = catalog.presentation_for_level("boot").recommended_build;
  build.chassis_robot = "robot.nope";
  try {
   game::MissionSimulation mission(catalog, "boot", build, 1u);
   CHECK(false);
  } catch (const std::exception&) {
   CHECK(true);
  }
 }
 // Fixed level refuses any override (lost_signal has no options).
 {
  auto build = catalog.presentation_for_level("lost_signal").recommended_build;
  build.chassis_robot = "robot.ro1";
  try {
   game::MissionSimulation mission(catalog, "lost_signal", build);
   CHECK(false);
  } catch (const std::exception&) {
   CHECK(throws_like("not offered"));
  }
 }
 // last_builds persist the chassis choice across save/load.
 {
  game::CampaignProgress progress;
  auto build = catalog.presentation_for_level("boot").recommended_build;
  build.chassis_robot = "robot.ro2";
  progress.record_build("boot", build);
  const std::filesystem::path save_path = "/tmp/opencode/progress_chassis.json";
  progress.save(save_path, catalog);
  const auto loaded = game::CampaignProgress::load(save_path, catalog);
  const auto* back = loaded.last_build("boot");
  CHECK(back != nullptr);
  CHECK(back->chassis_robot == "robot.ro2");
 }
 std::cerr << "chassis choice green\n";
 return 0;
}
