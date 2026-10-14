#pragma once

#include "content.hpp"

#include <filesystem>
#include <string>
#include <unordered_map>
#include <vector>

namespace game {

class ContentCatalog;
struct MissionResult;

struct BestScore {
 bool safe{false};
 bool fast{false};
 bool efficient{false};
 double best_time_s{0.0};
 double best_energy_wh{0.0};
};

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

 const std::vector<std::string>& completed_levels() const { return completed_levels_; }
 const std::unordered_map<std::string, BestScore>& best_scores() const { return best_scores_; }

 private:
 std::vector<std::string> completed_levels_;
 std::vector<std::string> inventory_;
 std::vector<std::string> abilities_;
 std::unordered_map<std::string, BuildSelection> last_builds_;
 std::unordered_map<std::string, BestScore> best_scores_;
};

} // namespace game
