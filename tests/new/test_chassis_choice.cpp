// Player chassis choice (backend slice; TGUI column deferred to desktop).
// Contract: presentation opt-in (absent = fixed, today's path), robot unlock
// gates, validated builds honored by the mission. boot opts in [ro1, ro2]
// (RO-2 verified to complete it below); all other levels stay fixed.

#include "sim/game/mission/campaign_progress.hpp"
#include "sim/game/mission/content_catalog.hpp"
#include "sim/game/scenario/scenario_session.hpp"
#include "sim/hardware/physics/physics_world.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <vector>

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
 const auto catalog = mission::ContentCatalog::load(LOST_SIGNAL_TEST_ASSETS);
 // Registry/options content: boot exposes the differential, Ackermann and
 // both common holonomic chassis; Ackermann keeps its progression gate.
 {
  const auto& options = catalog.presentation_for_level("boot").chassis_options;
 CHECK(options.size() == 4 && options[0] == "robot.ro1" && options[1] == "robot.ro2" &&
 options[2] == "robot.ro4" && options[3] == "robot.ro5");
  CHECK(catalog.robot("robot.ro2").unlock_ability == "ability.ackermann");
  CHECK(catalog.robot("robot.ro3").unlock_ability == "ability.tracked");
 CHECK(catalog.robot("robot.ro4").drive_type == "swerve");
 CHECK(catalog.robot("robot.ro5").drive_type == "mecanum");
  CHECK(catalog.robot("robot.ro1").unlock_ability.empty());
  CHECK(catalog.presentation_for_level("lost_signal").chassis_options.empty());
 }
 // Default path bit-identical: empty request keeps presentation/companion.
 {
  mission::ScenarioSession mission(catalog, "boot",
                                  catalog.presentation_for_level("boot").recommended_build);
  CHECK(mission.robot().drive_type == "differential");
  mission.run_to_completion();
  CHECK(mission.result().status == mission::MissionStatus::Succeeded);
 }
 // Positive: RO-2 on boot with the ability owned (verified combo, pinned).
 {
  auto build = catalog.presentation_for_level("boot").recommended_build;
  build.chassis_robot = "robot.ro2";
  mission::ScenarioSession mission(catalog, "boot", build, 1u, {"ability.ackermann"});
  CHECK(mission.robot().drive_type == "ackermann");
  mission.run_to_completion();
  CHECK(mission.result().status == mission::MissionStatus::Succeeded);
  CHECK(mission.result().time_s < 10.0);
 }
 // Locked without the ability.
 {
  auto build = catalog.presentation_for_level("boot").recommended_build;
  build.chassis_robot = "robot.ro2";
  try {
   mission::ScenarioSession mission(catalog, "boot", build, 1u);
   CHECK(false);
  } catch (const std::exception&) {
   CHECK(throws_like("ability.ackermann"));
  }
 }
 // The proving ground is available to a fresh save with every chassis and
 // component family, while defaulting to the articulated RO-2.
 {
  const auto& level = catalog.level("vehicle_proving_ground");
  const auto& presentation = catalog.presentation_for_level("vehicle_proving_ground");
  CHECK(level.mode == "playground");
 CHECK(level.control_mode == "operator_drive");
 CHECK(level.loadout_access == "open");
  CHECK(presentation.robot_id == "robot.ro2");
 CHECK(presentation.chassis_options.size() == 5);
 CHECK(presentation.component_columns.size() == 5);
  CHECK(std::find(catalog.campaign().playgrounds.begin(), catalog.campaign().playgrounds.end(),
                  "vehicle_proving_ground") != catalog.campaign().playgrounds.end());
  CHECK(std::find(presentation.component_columns[1].options.begin(),
                  presentation.component_columns[1].options.end(),
                  "sensor.sixteen_line_lidar") != presentation.component_columns[1].options.end());
 CHECK(std::find(presentation.component_columns[3].options.begin(),
 presentation.component_columns[3].options.end(),
 "motor.speed") != presentation.component_columns[3].options.end());
  mission::ScenarioSession mission(catalog, "vehicle_proving_ground",
                                   presentation.recommended_build, 1u);
  CHECK(mission.robot().drive_type == "ackermann");
 }
 // Not offered on a fixed campaign level, even with its ability.
 {
 auto build = catalog.presentation_for_level("lost_signal").recommended_build;
  build.chassis_robot = "robot.ro3";
  try {
 mission::ScenarioSession mission(catalog, "lost_signal", build, 1u, {"ability.tracked"});
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
   mission::ScenarioSession mission(catalog, "boot", build, 1u);
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
   mission::ScenarioSession mission(catalog, "lost_signal", build);
   CHECK(false);
  } catch (const std::exception&) {
   CHECK(throws_like("not offered"));
  }
 }
 // last_builds persist the chassis choice across save/load.
 {
  mission::CampaignProgress progress;
  auto build = catalog.presentation_for_level("boot").recommended_build;
  build.chassis_robot = "robot.ro2";
  progress.record_build("boot", build);
 const auto save_path =
 std::filesystem::temp_directory_path() / "lost_signal_progress_chassis.json";
  progress.save(save_path, catalog);
  const auto loaded = mission::CampaignProgress::load(save_path, catalog);
  const auto* back = loaded.last_build("boot");
  CHECK(back != nullptr);
  CHECK(back->chassis_robot == "robot.ro2");
 }
 // The two holonomic chassis are selectable content and complete a real
 // playable level through the full planner/controller/chassis chain. A
 // Bullet build repeats the same proof with rigid-body pose authority.
 std::vector<mission::PhysicsMode> physics_modes{mission::PhysicsMode::Domain};
 if (sim::physics::bullet_physics_available()) {
 physics_modes.push_back(mission::PhysicsMode::Bullet);
 }
 for (const auto physics_mode : physics_modes) {
 for (const auto* robot_id : {"robot.ro4", "robot.ro5"}) {
 auto build = catalog.presentation_for_level("boot").recommended_build;
 build.chassis_robot = robot_id;
 mission::ScenarioSession mission(catalog, "boot", build, 1u, {},
 mission::ComponentRegistry::builtins(), {}, physics_mode);
 mission.run_to_completion();
 CHECK(mission.result().status == mission::MissionStatus::Succeeded);
 CHECK(mission.physics_mode() == physics_mode);
 if (std::string(robot_id) == "robot.ro4") {
 CHECK(std::any_of(mission.replay().begin(), mission.replay().end(), [](const auto& frame) {
 return frame.control.mount_actuations.size() == 4 &&
 std::any_of(frame.control.mount_actuations.begin(), frame.control.mount_actuations.end(),
 [](const auto& actuation) { return std::abs(actuation.yaw_rad) > 1e-6; });
 }));
 }
 }
 }
 std::cerr << "chassis choice green\n";
 return 0;
}
