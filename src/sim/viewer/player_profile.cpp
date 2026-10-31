#include "sim/viewer/player_profile.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>

#include "robot/core/json.hpp"

namespace lost_signal::sim::viewer {
namespace {
using robot::core::Json;

const Json* field(const Json& object, const char* key) { return object.find(key); }

bool finite_nonnegative(const Json* value, double& output) {
  if (value == nullptr || !value->is_num() || !std::isfinite(value->as_num()) ||
      value->as_num() < 0.0) return false;
  output = value->as_num();
  return true;
}

bool string_field(const Json& object, const char* key, std::string& output) {
  const Json* value = field(object, key);
  if (value == nullptr || !value->is_str() || value->as_str().empty()) return false;
  output = value->as_str();
  return true;
}

Json chapter_json(const ChapterProgress& chapter) {
  Json::Array cards;
  for (const auto& card : chapter.knowledge_cards) cards.push_back(Json::Str(card));
  return Json::Obj({
      {"experience_id", Json::Str(chapter.experience_id)},
      {"completions", Json::Num(static_cast<double>(chapter.completions))},
      {"stars", Json::Num(static_cast<double>(chapter.stars))},
      {"best_elapsed_s", Json::Num(chapter.best_elapsed_s)},
      {"best_clearance_m", Json::Num(chapter.best_clearance_m)},
      {"knowledge_cards", Json::Arr(std::move(cards))}});
}

Json archive_json(const TeachingArchive& archive) {
  return Json::Obj({
      {"experience_id", Json::Str(archive.experience_id)},
      {"report_path", Json::Str(archive.report_path)},
      {"baseline_replay_path", Json::Str(archive.baseline_replay_path)},
      {"variant_replay_path", Json::Str(archive.variant_replay_path)}});
}

bool decode_chapter(const Json& value, ChapterProgress& output) {
  if (!value.is_object() || !string_field(value, "experience_id", output.experience_id)) return false;
  const Json* completions = field(value, "completions");
  const Json* stars = field(value, "stars");
  double number = 0.0;
  if (completions == nullptr || !completions->is_num() || completions->as_num() < 0.0 ||
      std::floor(completions->as_num()) != completions->as_num() ||
      completions->as_num() > 4294967295.0 || stars == nullptr || !stars->is_num() ||
      stars->as_num() < 0.0 || stars->as_num() > 3.0 ||
      std::floor(stars->as_num()) != stars->as_num() ||
      !finite_nonnegative(field(value, "best_elapsed_s"), number)) return false;
  output.completions = static_cast<std::uint32_t>(completions->as_num());
  output.stars = static_cast<std::uint8_t>(stars->as_num());
  output.best_elapsed_s = number;
  if (!finite_nonnegative(field(value, "best_clearance_m"), output.best_clearance_m)) return false;
  const Json* cards = field(value, "knowledge_cards");
  if (cards == nullptr || !cards->is_array()) return false;
  std::set<std::string> unique;
  for (const auto& card : cards->as_array()) {
    if (!card.is_str() || card.as_str().empty() || !unique.insert(card.as_str()).second) return false;
    output.knowledge_cards.push_back(card.as_str());
  }
  return true;
}

bool decode_archive(const Json& value, TeachingArchive& output) {
  return value.is_object() &&
         string_field(value, "experience_id", output.experience_id) &&
         string_field(value, "report_path", output.report_path) &&
         string_field(value, "baseline_replay_path",
                      output.baseline_replay_path) &&
         string_field(value, "variant_replay_path",
                      output.variant_replay_path);
}
}  // namespace

std::string default_player_profile_path() {
  // An explicit environment override is useful for portable installs and CI;
  // otherwise follow the XDG state convention on Unix-like systems.  The
  // fallback remains local to the working directory when no home directory is
  // available (for example, a sandboxed test process).
  if (const char* override_path = std::getenv("LOST_SIGNAL_PROFILE_PATH");
      override_path != nullptr && *override_path != '\0') {
    return override_path;
  }
  if (const char* state_home = std::getenv("XDG_STATE_HOME");
      state_home != nullptr && *state_home != '\0') {
    return (std::filesystem::path(state_home) / "lost-signal" /
            "player-profile.json")
        .string();
  }
  if (const char* home = std::getenv("HOME"); home != nullptr && *home != '\0') {
    return (std::filesystem::path(home) / ".local" / "state" /
            "lost-signal" / "player-profile.json")
        .string();
  }
  return "player-profile.json";
}

std::string knowledge_card_id(const std::string& experience_id,
                              std::size_t lesson_index) {
  return experience_id + "/lesson/" + std::to_string(lesson_index);
}

std::uint8_t chapter_stars(const ChapterOutcome& outcome) {
  if (!outcome.safety_passed) return 0;
  if (!outcome.mission_passed) return 1;
  if (!outcome.experiment_passed) return 2;
  return 3;
}

void record_chapter(PlayerProfile& profile, const ChapterOutcome& outcome) {
  if (outcome.experience_id.empty()) return;
  auto it = std::find_if(profile.chapters.begin(), profile.chapters.end(),
                         [&](const ChapterProgress& chapter) {
                           return chapter.experience_id == outcome.experience_id;
                         });
  if (it == profile.chapters.end()) {
    profile.chapters.push_back({});
    it = std::prev(profile.chapters.end());
    it->experience_id = outcome.experience_id;
  }
  ChapterProgress& chapter = *it;
  ++chapter.completions;
  chapter.stars = std::max(chapter.stars, chapter_stars(outcome));
  if (std::isfinite(outcome.elapsed_s) && outcome.elapsed_s > 0.0 &&
      (chapter.best_elapsed_s <= 0.0 || outcome.elapsed_s < chapter.best_elapsed_s)) {
    chapter.best_elapsed_s = outcome.elapsed_s;
  }
  if (std::isfinite(outcome.minimum_clearance_m) && outcome.minimum_clearance_m >= 0.0) {
    chapter.best_clearance_m = std::max(chapter.best_clearance_m, outcome.minimum_clearance_m);
  }
  for (const auto& card : outcome.knowledge_cards) {
    if (card.empty() || std::find(chapter.knowledge_cards.begin(), chapter.knowledge_cards.end(), card) != chapter.knowledge_cards.end()) continue;
    chapter.knowledge_cards.push_back(card);
  }
}

void record_teaching_archive(PlayerProfile& profile,
                             const TeachingArchive& archive) {
  if (archive.experience_id.empty() || archive.report_path.empty() ||
      archive.baseline_replay_path.empty() ||
      archive.variant_replay_path.empty()) {
    return;
  }
  const auto existing = std::find_if(
      profile.archives.begin(), profile.archives.end(),
      [&](const TeachingArchive& candidate) {
        return candidate.report_path == archive.report_path;
      });
  if (existing != profile.archives.end()) {
    *existing = archive;
    return;
  }
  profile.archives.push_back(archive);
  // Keep the profile bounded even when a player exports every retry.
  constexpr std::size_t kMaximumArchives = 32;
  if (profile.archives.size() > kMaximumArchives) {
    profile.archives.erase(profile.archives.begin(),
                           profile.archives.begin() +
                               (profile.archives.size() - kMaximumArchives));
  }
}

bool load_player_profile(const std::string& path, PlayerProfile& profile,
                         std::string* error) {
  if (error != nullptr) error->clear();
  std::ifstream input(path, std::ios::binary);
  if (!input.is_open()) { profile = {}; return true; }
  std::ostringstream text;
  text << input.rdbuf();
  std::string parse_error;
  auto root = robot::core::json_parse(text.str(), &parse_error);
  if (root == nullptr || !root->is_object()) {
    profile = {};
    if (error != nullptr) *error = parse_error.empty() ? "invalid profile" : parse_error;
    return false;
  }
  const Json* version = field(*root, "version");
  const Json* format = field(*root, "format");
  const Json* chapters = field(*root, "chapters");
  if (format == nullptr || !format->is_str() ||
      format->as_str() != "lost-signal-player-profile/1" ||
      version == nullptr || !version->is_num() || version->as_num() != 1.0 ||
      chapters == nullptr || !chapters->is_array()) {
    profile = {};
    if (error != nullptr) *error = "profile schema mismatch";
    return false;
  }
  PlayerProfile decoded;
  std::set<std::string> chapter_ids;
  for (const auto& chapter : chapters->as_array()) {
    ChapterProgress progress;
    if (!decode_chapter(chapter, progress)) {
      profile = {};
      if (error != nullptr) *error = "invalid profile chapter";
      return false;
    }
    if (!chapter_ids.insert(progress.experience_id).second) {
      profile = {};
      if (error != nullptr) *error = "duplicate profile chapter";
      return false;
    }
    decoded.chapters.push_back(std::move(progress));
  }
  const Json* archives = field(*root, "archives");
  if (archives != nullptr) {
    if (!archives->is_array()) {
      profile = {};
      if (error != nullptr) *error = "invalid profile archives";
      return false;
    }
    std::set<std::string> report_paths;
    for (const auto& archive : archives->as_array()) {
      TeachingArchive decoded_archive;
      if (!decode_archive(archive, decoded_archive) ||
          !report_paths.insert(decoded_archive.report_path).second) {
        profile = {};
        if (error != nullptr) *error = "invalid profile archive";
        return false;
      }
      decoded.archives.push_back(std::move(decoded_archive));
    }
    constexpr std::size_t kMaximumArchives = 32;
    if (decoded.archives.size() > kMaximumArchives) {
      profile = {};
      if (error != nullptr) *error = "too many profile archives";
      return false;
    }
  }
  profile = std::move(decoded);
  return true;
}

bool save_player_profile(const std::string& path, const PlayerProfile& profile,
                         std::string* error) {
  if (error != nullptr) error->clear();
  if (path.empty()) {
    if (error != nullptr) *error = "profile path is empty";
    return false;
  }
  Json::Array chapters;
  for (const auto& chapter : profile.chapters) chapters.push_back(chapter_json(chapter));
  Json::Array archives;
  for (const auto& archive : profile.archives) {
    archives.push_back(archive_json(archive));
  }
  const std::filesystem::path target(path);
  std::error_code fs_error;
  if (!target.parent_path().empty()) std::filesystem::create_directories(target.parent_path(), fs_error);
  if (fs_error) { if (error != nullptr) *error = fs_error.message(); return false; }
  const std::filesystem::path temporary = target.string() + ".tmp";
  std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
  if (!output.is_open()) { if (error != nullptr) *error = "cannot open profile temporary file"; return false; }
  output << robot::core::json_dump(Json::Obj({
      {"format", Json::Str("lost-signal-player-profile/1")},
      {"version", Json::Num(1.0)},
      {"chapters", Json::Arr(std::move(chapters))},
      {"archives", Json::Arr(std::move(archives))}})) << '\n';
  output.close();
  if (!output) { if (error != nullptr) *error = "cannot write profile temporary file"; return false; }
  std::filesystem::rename(temporary, target, fs_error);
  if (fs_error) { if (error != nullptr) *error = fs_error.message(); return false; }
  return true;
}

}  // namespace lost_signal::sim::viewer
