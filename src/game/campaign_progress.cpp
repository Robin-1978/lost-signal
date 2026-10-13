#include "campaign_progress.hpp"

#include "content_catalog.hpp"
#include "mission_simulation.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <fstream>
#include <limits>
#include <system_error>

namespace game {
namespace {

using json = nlohmann::json;

bool known_level(const ContentCatalog& catalog, const std::string& id) {
 return catalog.levels().contains(id);
}

} // namespace

CampaignProgress CampaignProgress::load(
 const std::filesystem::path& path, const ContentCatalog& catalog) {
 CampaignProgress progress;
 progress.inventory_ = catalog.campaign().starting_inventory;
 if (path.empty() || !std::filesystem::exists(path)) return progress;

 std::ifstream input(path);
 if (!input) throw ContentError("cannot open progress file: " + path.string());
 json value;
 try {
 input >> value;
 } catch (const json::exception& error) {
 throw ContentError(path.string() + ": invalid progress JSON: " + error.what());
 }
 if (value.value("schema_version", 0) != 2) {
 throw ContentError(path.string() + ": unsupported progress schema_version");
 }

 for (const auto& id : value.value("completed_levels", std::vector<std::string>{})) {
 if (known_level(catalog, id)
 && std::find(progress.completed_levels_.begin(), progress.completed_levels_.end(), id)
 == progress.completed_levels_.end()) {
 progress.completed_levels_.push_back(id);
 }
 }
 progress.inventory_ = catalog.campaign().starting_inventory;
 for (const auto& id : value.value("inventory", std::vector<std::string>{})) {
 if (catalog.components().contains(id)
 && std::find(progress.inventory_.begin(), progress.inventory_.end(), id)
 == progress.inventory_.end()) {
 progress.inventory_.push_back(id);
 }
 }
 const auto builds = value.value("last_builds", json::object());
 for (auto it = builds.begin(); it != builds.end(); ++it) {
 if (!known_level(catalog, it.key()) || !it.value().is_object()) continue;
 const auto& entry = it.value();
 BuildSelection build;
 build.tires = entry.value("tires", std::string{});
 build.sensor = entry.value("sensor", std::string{});
 build.computer = entry.value("computer", std::string{});
 build.style = std::clamp(entry.value("style", 0.5), 0.0, 1.0);
 bool known = true;
 for (const auto* id : {&build.tires, &build.sensor, &build.computer}) {
 if (!id->empty() && !catalog.components().contains(*id)) known = false;
 }
 if (known) progress.last_builds_.emplace(it.key(), build);
 }
 const auto scores = value.value("best_scores", json::object());
 for (auto it = scores.begin(); it != scores.end(); ++it) {
 if (!known_level(catalog, it.key()) || !it.value().is_object()) continue;
 progress.best_scores_.emplace(it.key(), BestScore{
 it.value().value("safe", false),
 it.value().value("fast", false),
 it.value().value("efficient", false),
 it.value().value("best_time_s", 0.0),
 it.value().value("best_energy_wh", 0.0),
 });
 }
 return progress;
}

void CampaignProgress::record_completion(
 const std::string& level_id, const MissionResult& result,
 const ContentCatalog& catalog) {
 (void)catalog.level(level_id);
 if (std::find(completed_levels_.begin(), completed_levels_.end(), level_id)
 == completed_levels_.end()) {
 completed_levels_.push_back(level_id);
 }
 for (const auto& reward : catalog.level(level_id).rewards) {
 if (catalog.components().contains(reward)
 && std::find(inventory_.begin(), inventory_.end(), reward) == inventory_.end()) {
 inventory_.push_back(reward);
 }
 }
 const auto& order = catalog.campaign().levels;
 std::sort(completed_levels_.begin(), completed_levels_.end(), [&](const auto& left, const auto& right) {
 return std::find(order.begin(), order.end(), left) < std::find(order.begin(), order.end(), right);
 });

 auto& score = best_scores_[level_id];
 score.safe = score.safe || result.safe_score;
 score.fast = score.fast || result.fast_score;
 score.efficient = score.efficient || result.efficient_score;
 if (score.best_time_s <= 0.0 || result.time_s < score.best_time_s) {
 score.best_time_s = result.time_s;
 }
 if (score.best_energy_wh <= 0.0 || result.energy_wh < score.best_energy_wh) {
 score.best_energy_wh = result.energy_wh;
 }
}

void CampaignProgress::record_build(const std::string& level_id, const BuildSelection& build) {
 last_builds_[level_id] = build;
}

const BuildSelection* CampaignProgress::last_build(const std::string& level_id) const {
 const auto found = last_builds_.find(level_id);
 return found == last_builds_.end() ? nullptr : &found->second;
}

void CampaignProgress::save(
 const std::filesystem::path& path, const ContentCatalog& catalog) const {
 if (path.empty()) return;
 if (!path.parent_path().empty()) std::filesystem::create_directories(path.parent_path());
 json builds = json::object();
 for (const auto& [level_id, build] : last_builds_) {
 builds[level_id] = {
 {"tires", build.tires},
 {"sensor", build.sensor},
 {"computer", build.computer},
 {"style", build.style},
 };
 }
 json scores = json::object();
 for (const auto& [level_id, score] : best_scores_) {
 scores[level_id] = {
 {"safe", score.safe},
 {"fast", score.fast},
 {"efficient", score.efficient},
 {"best_time_s", score.best_time_s},
 {"best_energy_wh", score.best_energy_wh},
 };
 }
 const json value = {
 {"schema_version", 2},
 {"content_version", catalog.campaign().content_version},
 {"completed_levels", completed_levels_},
 {"inventory", inventory_},
 {"last_builds", std::move(builds)},
 {"best_scores", std::move(scores)},
 };

 auto temporary = path;
 temporary += ".tmp";
 {
 std::ofstream output(temporary, std::ios::trunc);
 if (!output) throw ContentError("cannot write progress file: " + temporary.string());
 output << value.dump(2) << '\n';
 }
 std::error_code error;
 std::filesystem::rename(temporary, path, error);
 if (error) {
 std::filesystem::remove(path, error);
 error.clear();
 std::filesystem::rename(temporary, path, error);
 }
 if (error) {
 throw ContentError("cannot replace progress file: " + path.string() + ": " + error.message());
 }
}

} // namespace game
