#pragma once

#include "content.hpp"
#include "mission/systems/world_activity_system.hpp"
#include "mission/systems/field_operator_system.hpp"

#include <filesystem>
#include <string>
#include <optional>
#include <utility>
#include <unordered_map>
#include <vector>

namespace mission {

class ContentCatalog;
struct MissionResult;

struct BestScore {
 bool safe{false};
 bool fast{false};
 bool efficient{false};
 double best_time_s{0.0};
 double best_energy_wh{0.0};
};

int component_reputation_cost(const ComponentDefinition& component);

class CampaignProgress {
 public:
 static CampaignProgress load(const std::filesystem::path& path, const ContentCatalog& catalog);

 void record_completion(const std::string& level_id, const MissionResult& result,
                        const ContentCatalog& catalog);
 void save(const std::filesystem::path& path, const ContentCatalog& catalog) const;

 // DIY inventory: owned component ids. Starts at starting_inventory, grows by
 // component rewards (idempotent). Garage offers level columns ∩ inventory.
 const std::vector<std::string>& inventory() const { return inventory_; }
 // Unlocked ability ids (registry-gated: unknown ability.* rewards fail loudly
 // in record_completion instead of vanishing). Display owns the consumer.
 const std::vector<std::string>& abilities() const { return abilities_; }
 // Last attempted build per level; restored when re-entering the garage.
 void record_build(const std::string& level_id, const BuildSelection& build);
 const BuildSelection* last_build(const std::string& level_id) const;
 // Contract reputation ledger and garage economy:
 // claim_contract credits rep once per job id (repeat claims are no-ops);
 // spend_reputation returns false instead of going negative; component
 // purchases atomically debit reputation and extend persistent inventory.
 int reputation() const { return reputation_; }
 const std::vector<std::string>& claimed_contracts() const { return claimed_contracts_; }
 bool claim_contract(const std::string& job_id, int rep);
 bool spend_reputation(int amount);
 bool purchase_component(const std::string& component_id, const ContentCatalog& catalog);
 void set_regional_world(WorldActivityCheckpoint checkpoint) {
 regional_world_ = std::move(checkpoint);
 }
 const std::optional<WorldActivityCheckpoint>& regional_world() const { return regional_world_; }
 void set_field_expedition(FieldExpeditionCheckpoint checkpoint) {
 field_expedition_ = std::move(checkpoint);
 }
 const std::optional<FieldExpeditionCheckpoint>& field_expedition() const {
 return field_expedition_;
 }

 const std::vector<std::string>& completed_levels() const { return completed_levels_; }
 const std::unordered_map<std::string, BestScore>& best_scores() const { return best_scores_; }

 private:
 std::vector<std::string> completed_levels_;
 std::vector<std::string> inventory_;
 std::vector<std::string> abilities_;
 std::unordered_map<std::string, BuildSelection> last_builds_;
 std::unordered_map<std::string, BestScore> best_scores_;
 int reputation_{0};
 std::vector<std::string> claimed_contracts_;
 std::optional<WorldActivityCheckpoint> regional_world_;
 std::optional<FieldExpeditionCheckpoint> field_expedition_;
};

} // namespace mission
