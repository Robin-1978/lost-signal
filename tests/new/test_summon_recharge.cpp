// Summon/recharge goal coverage (overlay tree, shipped files untouched):
// a dormant actor wakes on a summon goal (miss branch would say otherwise)
// and a recharge goal tops the battery account. The overlay copies the
// shipped tree, adds one map + level + presentation, and runs headless.
#include "mission/campaign_progress.hpp"
#include "mission/content_catalog.hpp"
#include "scenario/scenario_session.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>

#define CHECK(condition)                                                                                     \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n";                           \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

namespace fs = std::filesystem;

const char kTestMap[] = R"({
  "schema_version": 1,
  "id": "test_summon_map",
  "scene_3d": "scene.research_base",
  "size_m": [12.0, 12.0],
  "grid_resolution_m": 0.5,
  "obstacles": [],
  "surfaces": [],
  "markers": {
    "test_start": [1.0, 1.0, 0.0],
    "test_meet": [3.0, 1.0, 0.0],
    "test_charge": [6.0, 1.0, 0.0],
    "test_home": [9.0, 9.0, 0.0]
  },
  "actors": [
    {
      "id": "test_cart",
      "type": "patrol",
      "path": [[5.0, 3.0], [5.0, 5.0]],
      "speed_mps": 0.5,
      "radius_m": 0.45,
      "active_in": ["test_summon"],
      "dormant_until_summoned": true,
      "collision": "fatal",
      "visual": {"model": "models/actors/forklift.glb", "ref_radius_m": 0.45}
    }
  ]
})";

const char kTestLevel[] = R"({
  "schema_version": 1,
  "id": "test_summon",
  "order": 99,
  "title": "Test Summon",
  "map": "test_summon_map",
  "control_mode": "autonomous",
  "localization_mode": "fused",
  "chapter": 1,
  "prerequisites": [],
  "seed": 7,
  "objective": {
    "type": "deliver_via",
    "spawn": "test_start",
    "via": "test_charge",
    "via_label": "Test Charge",
    "goal": "test_home",
    "goal_label": "Test Home",
    "goal_visual": "calibration_pad",
    "time_limit_s": 120,
    "max_collisions": 0,
    "arrive_tolerance_m": 0.62,
    "via_radius_m": 1.25,
    "goals": [
      {"marker": "test_meet", "label": "Test Meet", "action": "summon",
       "radius_m": 1.25, "summon_actor": "test_cart"},
      {"marker": "test_charge", "label": "Test Charge", "action": "recharge",
       "radius_m": 1.25, "recharge_wh": 5.0},
      {"marker": "test_home", "label": "Test Home", "action": "complete",
       "radius_m": 0.62}
    ]
  },
  "loadout_rules": {"budget": 8, "required_slots": ["tires"]},
  "teaching": {"concept": "test", "prediction_prompt": "t", "knowledge_card": "t"},
  "score": {"safe_min_clearance_m": 0.3, "fast_time_s": 100.0, "efficient_energy_wh": 10.0},
  "rewards": ["ability.basic_motion"]
})";

int main() {
 const fs::path overlay = fs::temp_directory_path() / "lost_signal_summon_assets";
 std::error_code error;
 fs::remove_all(overlay, error);
 fs::copy(LOST_SIGNAL_TEST_ASSETS, overlay,
          fs::copy_options::recursive | fs::copy_options::overwrite_existing, error);
 CHECK(!error);
 {
  std::ofstream out(overlay / "maps" / "test_summon_map.json", std::ios::trunc);
  CHECK(static_cast<bool>(out));
  out << kTestMap;
 }
 {
  std::ofstream out(overlay / "levels" / "test_summon.json", std::ios::trunc);
  CHECK(static_cast<bool>(out));
  out << kTestLevel;
 }
 // Presentation: clone boot's (valid builds, budget-compatible), repoint.
 {
  std::ifstream in(overlay / "presentation" / "boot.json");
  CHECK(static_cast<bool>(in));
  std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  std::string::size_type pos = text.find("\"presentation.boot\"");
  CHECK(pos != std::string::npos);
  text.replace(pos, std::string("\"presentation.boot\"").size(), "\"presentation.test_summon\"");
  pos = text.find("\"level\": \"boot\"");
  CHECK(pos != std::string::npos);
  text.replace(pos, std::string("\"level\": \"boot\"").size(), "\"level\": \"test_summon\"");
  // The clone must not steal the default slot (catalog rejects two defaults).
  pos = text.find("\"is_default\": true");
  CHECK(pos != std::string::npos);
  text.replace(pos, std::string("\"is_default\": true").size(), "\"is_default\": false");
  std::ofstream out(overlay / "presentation" / "test_summon.json", std::ios::trunc);
  CHECK(static_cast<bool>(out));
  out << text;
 }
 // Campaign must list every level exactly once (overlay copy is editable).
 {
  std::ifstream in(overlay / "campaign.json");
  CHECK(static_cast<bool>(in));
  std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  auto levels_pos = text.find("\"levels\"");
  CHECK(levels_pos != std::string::npos);
  auto open_pos = text.find("[", levels_pos);
  CHECK(open_pos != std::string::npos);
  auto close_pos = text.find("]", open_pos);
  CHECK(close_pos != std::string::npos);
  text.insert(close_pos, ", \"test_summon\"");
  std::ofstream out(overlay / "campaign.json", std::ios::trunc);
  CHECK(static_cast<bool>(out));
  out << text;
 }
 const auto catalog = mission::ContentCatalog::load(overlay.string());
 // Explicit goals parse (summon + recharge + complete, in order).
 const auto& objective = catalog.level("test_summon").objective;
 CHECK(objective.goals.size() == 3);
 CHECK(objective.goals[0].action == "summon");
 CHECK(objective.goals[0].summon_actor == "test_cart");
 CHECK(objective.goals[1].action == "recharge");
 CHECK(objective.goals[1].recharge_wh == 5.0);
 CHECK(objective.goals[2].action == "complete");

 // Tires only (like boot): stock motor drives, no sensor needed —
 // summon/recharge/complete are positional goals, not perception ones.
 mission::BuildSelection build;
 build.parts["tires"] = "tires.standard";
 build.style = 0.0;
 mission::ScenarioSession mission(catalog, "test_summon", build, 7u);
 mission.run_to_completion();
 CHECK(mission.result().status == mission::MissionStatus::Succeeded);
 bool summoned = false, recharged = false;
 for (const auto& event : mission.events()) {
  if (event.code == "summoned") {
 // The distinct event code proves that a dormant actor was found and woke.
   summoned = true;
  }
  if (event.code == "recharged") recharged = true;
 }
 CHECK(summoned);
 CHECK(recharged);
 // Recharge zeroes the account mid-run; only the post-charge leg remains.
 // (A 5Wh top-up on a ~0.1Wh run, clamped at zero, never negative.)
 CHECK(mission.result().energy_wh < 0.5);
 CHECK(mission.result().energy_wh >= 0.0);
 std::cerr << "summon/recharge green\n";
 return 0;
}
