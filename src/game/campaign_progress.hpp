#pragma once

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
 static CampaignProgress load(
 const std::filesystem::path& path, const ContentCatalog& catalog);

 void record_completion(
 const std::string& level_id, const MissionResult& result,
 const ContentCatalog& catalog);
 void save(const std::filesystem::path& path, const ContentCatalog& catalog) const;

 const std::vector<std::string>& completed_levels() const { return completed_levels_; }
 const std::unordered_map<std::string, BestScore>& best_scores() const {
 return best_scores_;
 }

private:
 std::vector<std::string> completed_levels_;
 std::unordered_map<std::string, BestScore> best_scores_;
};

} // namespace game
