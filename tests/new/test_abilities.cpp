// Abilities registry contract (data layer only; UI display deferred to desktop).
// Closes the silent-drop hole: 23 ability rewards previously vanished in
// record_completion (only components() were kept). Now registry-gated.

#include "game/campaign_progress.hpp"
#include "game/content_catalog.hpp"
#include "game/mission_simulation.hpp"

#include <cstdio>
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

int main() {
 const auto catalog = game::ContentCatalog::load(LOST_SIGNAL_TEST_ASSETS);
 // Registry: every level-granted ability.* resolves (no absolute count: the
 // registry grows with content, the loop below is the pin, not the number).
 CHECK(!catalog.abilities().empty());
 for (const auto& [level_id, level] : catalog.levels()) {
  for (const auto& reward : level.rewards) {
   if (reward.rfind("ability.", 0) == 0) {
    CHECK(catalog.abilities().count(reward) == 1);
    CHECK(!catalog.ability(reward).title.empty());
   }
  }
 }
 CHECK(catalog.ability("ability.signal_cross").title.size() > 0);
 // Completion records abilities (idempotent) and they survive save/load.
 {
  game::CampaignProgress progress;
  CHECK(progress.abilities().empty());
  game::MissionResult done;
  done.safe_score = true;
  progress.record_completion("signal_cross", done, catalog);
  CHECK(progress.abilities().size() == 1);
  CHECK(progress.abilities().front() == "ability.signal_cross");
  progress.record_completion("signal_cross", done, catalog);
  CHECK(progress.abilities().size() == 1);
  progress.record_completion("signal_hold", done, catalog);
  CHECK(progress.abilities().size() == 2);
 const auto save_path =
 std::filesystem::temp_directory_path() / "lost_signal_progress_abilities.json";
  progress.save(save_path, catalog);
  const auto loaded = game::CampaignProgress::load(save_path, catalog);
  CHECK(loaded.abilities().size() == 2);
  CHECK(loaded.abilities().front() == "ability.signal_cross");
  // Old saves without the key load with empty abilities (additive format).
 const auto legacy_path =
 std::filesystem::temp_directory_path() / "lost_signal_progress_abilities_legacy.json";
  {
   std::ofstream out(legacy_path, std::ios::trunc);
   out << "{\"schema_version\": 2, \"completed_levels\": [], \"inventory\": []}\n";
  }
  const auto legacy = game::CampaignProgress::load(legacy_path, catalog);
  CHECK(legacy.abilities().empty());
 }
 std::cerr << "abilities registry green\n";
 return 0;
}
