// Contract reputation ledger (board economy, progress side): claim once,
// double-claim no-ops, spend never goes negative, save round-trips, old
// saves load at zero, and reputation buys persistent garage inventory.
#include "sim/game/mission/campaign_progress.hpp"
#include "sim/game/mission/content_catalog.hpp"
#include "sim/game/scenario/scenario_session.hpp"

#include <algorithm>
#include <cmath>
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

int main() {
 const auto catalog = mission::ContentCatalog::load(LOST_SIGNAL_TEST_ASSETS);
 mission::CampaignProgress progress;
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
 mission::WorldActivitySystem regional_world;
 regional_world.reset(123456);
 progress.set_regional_world(regional_world.checkpoint(0.0));
 // Spend guards the floor.
 CHECK(!progress.spend_reputation(6));
 CHECK(progress.reputation() == 5);
 CHECK(!progress.spend_reputation(0));
 CHECK(progress.spend_reputation(2));
 CHECK(progress.reputation() == 3);
 // Reputation has a concrete garage outlet: buying a component is atomic,
 // idempotent, and uses the same inventory persisted by the campaign.
 mission::CampaignProgress buyer;
 CHECK(buyer.claim_contract("shop-fund", 3));
 CHECK(mission::component_reputation_cost(catalog.component("motor.torque")) == 1);
 CHECK(buyer.purchase_component("motor.torque", catalog));
 CHECK(buyer.reputation() == 2);
 CHECK(std::find(buyer.inventory().begin(), buyer.inventory().end(), "motor.torque") !=
 buyer.inventory().end());
 CHECK(!buyer.purchase_component("motor.torque", catalog));
 CHECK(buyer.reputation() == 2);
 // Round-trip through the save file.
 const fs::path save_path = fs::temp_directory_path() / "lost_signal_progress_rep.json";
 progress.save(save_path, catalog);
 auto loaded = mission::CampaignProgress::load(save_path, catalog);
 CHECK(loaded.reputation() == 3);
 CHECK(loaded.claimed_contracts().size() == 2);
 CHECK(loaded.regional_world().has_value());
 CHECK(!loaded.regional_world()->rng_state.empty());
 CHECK(loaded.regional_world()->state.weather == mission::WeatherKind::Clear);
 CHECK(!loaded.claim_contract("w01-1", 2)); // still claimed after load.
 // Old saves without the keys load at zero (additive format).
 const fs::path legacy_path =
 fs::temp_directory_path() / "lost_signal_progress_rep_legacy.json";
 {
  std::ofstream out(legacy_path, std::ios::trunc);
 out << "{\"schema_version\": 3, \"completed_levels\": [], \"inventory\": []}\n";
 }
 const auto legacy = mission::CampaignProgress::load(legacy_path, catalog);
 CHECK(legacy.reputation() == 0);
 CHECK(legacy.claimed_contracts().empty());
 // Earn hook: completing the job's level with its axis scored claims rep
 // once; missing the axis pays nothing; replays never double-pay.
 {
  CHECK(!catalog.contracts().empty());
  const auto& job = catalog.contracts().front();
 const auto& base_level = catalog.level(job.level);
 const auto& contract_presentation = catalog.presentation_for_level(job.level);
 mission::ScenarioSession contracted(catalog, job.level, contract_presentation.recommended_build,
 std::nullopt, std::vector<std::string>{}, mission::ComponentRegistry::builtins(),
 std::shared_ptr<const autonomy::IGlobalPlanner>{},
 std::shared_ptr<sim::IHighLevelPolicy>{}, job.id);
 CHECK(contracted.active_contract() != nullptr);
 CHECK(contracted.active_contract()->id == job.id);
 CHECK(std::abs(contracted.payload_state().mass_kg -
 (base_level.payload_mass_kg + job.mass_kg)) < 1e-9);
  mission::CampaignProgress missed;
  mission::MissionResult missed_run;
  missed.record_completion(job.level, missed_run, catalog);
  CHECK(missed.reputation() == 0);
 CHECK(missed.completed_levels().empty());

 mission::MissionResult collided_run;
 collided_run.status = mission::MissionStatus::Succeeded;
 collided_run.collisions = 1;
 collided_run.performance.ranked = true;
 collided_run.performance.overall = 100.0;
 missed.record_completion(job.level, collided_run, catalog);
 CHECK(missed.completed_levels().empty());

  mission::CampaignProgress earn;
  mission::MissionResult qualifying_run;
 qualifying_run.status = mission::MissionStatus::Succeeded;
 qualifying_run.performance.ranked = true;
 qualifying_run.performance.overall = 82.0;
 qualifying_run.performance.safety = 81.0;
 qualifying_run.performance.pace = 80.0;
 qualifying_run.performance.efficiency = 79.0;
 qualifying_run.performance.smoothness = 78.0;
 qualifying_run.performance.decisiveness = 77.0;
  if (job.axis == "fast") {
   qualifying_run.fast_score = true;
  } else if (job.axis == "safe") {
   qualifying_run.safe_score = true;
  } else if (job.axis == "efficient") {
   qualifying_run.efficient_score = true;
  } else {
   CHECK(false);
  }
  earn.record_completion(job.level, qualifying_run, catalog);
  CHECK(earn.reputation() == job.rep);
 CHECK(earn.best_scores().at(job.level).best_overall == 82.0);
  earn.record_completion(job.level, qualifying_run, catalog);
  CHECK(earn.reputation() == job.rep);
  // C++ catalog carries every board the validator gates (relational pin:
  // job count grows with content, never an absolute number here).
  for (const auto& job : catalog.contracts()) {
   CHECK(catalog.levels().count(job.level) == 1);
   CHECK(!job.id.empty() && job.rep >= 1);
  }
 }
 std::cout << "contract rep green\n";
 return 0;
}
