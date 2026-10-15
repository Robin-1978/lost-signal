// Contract reputation ledger (board economy, progress side): claim once,
// double-claim no-ops, spend never goes negative, save round-trips, old
// saves load at zero. The earn hook (mission completion -> claim) lands with
// the board UI, not here.
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

namespace fs = std::filesystem;

int main() {
 const auto catalog = game::ContentCatalog::load(LOST_SIGNAL_TEST_ASSETS);
 game::CampaignProgress progress;
 CHECK(progress.reputation() == 0);
 CHECK(progress.claimed_contracts().empty());
 // Claim credits once; repeats and junk are no-ops.
 CHECK(progress.claim_contract("w01-1", 2));
 CHECK(progress.reputation() == 2);
 CHECK(!progress.claim_contract("w01-1", 2));
 CHECK(progress.reputation() == 2);
 CHECK(!progress.claim_contract("", 2));
 CHECK(!progress.claim_contract("w01-2", 0));
 CHECK(!progress.claim_contract("w01-2", -1));
 CHECK(progress.claim_contract("w01-2", 3));
 CHECK(progress.reputation() == 5);
 // Spend guards the floor.
 CHECK(!progress.spend_reputation(6));
 CHECK(progress.reputation() == 5);
 CHECK(!progress.spend_reputation(0));
 CHECK(progress.spend_reputation(2));
 CHECK(progress.reputation() == 3);
 // Round-trip through the save file.
 const fs::path save_path = "/tmp/opencode/progress_rep.json";
 progress.save(save_path, catalog);
 auto loaded = game::CampaignProgress::load(save_path, catalog);
 CHECK(loaded.reputation() == 3);
 CHECK(loaded.claimed_contracts().size() == 2);
 CHECK(!loaded.claim_contract("w01-1", 2)); // still claimed after load.
 // Old saves without the keys load at zero (additive format).
 const fs::path legacy_path = "/tmp/opencode/progress_rep_legacy.json";
 {
  std::ofstream out(legacy_path, std::ios::trunc);
  out << "{\"schema_version\": 2, \"completed_levels\": [], \"inventory\": []}\n";
 }
 const auto legacy = game::CampaignProgress::load(legacy_path, catalog);
 CHECK(legacy.reputation() == 0);
 CHECK(legacy.claimed_contracts().empty());
 // Earn hook: completing the job's level with its axis scored claims rep
 // once; missing the axis pays nothing; replays never double-pay.
 {
  game::CampaignProgress earn;
  game::MissionResult fast_run;
  fast_run.fast_score = true;
  earn.record_completion("signal_cross", fast_run, catalog);
  CHECK(earn.reputation() == 4); // w01-3 fast job.
  earn.record_completion("signal_cross", fast_run, catalog);
  CHECK(earn.reputation() == 4);
  game::MissionResult slow_run;
  slow_run.safe_score = true; // fast axis missed: no pay.
  earn.record_completion("signal_cross", slow_run, catalog);
  CHECK(earn.reputation() == 4);
  // C++ catalog carries the same board the validator gates.
  CHECK(catalog.contracts().size() == 5);
 }
 std::cout << "contract rep green\n";
 return 0;
}
