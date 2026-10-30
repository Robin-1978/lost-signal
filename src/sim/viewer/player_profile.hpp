#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace lost_signal::sim::viewer {

struct ChapterProgress {
  std::string experience_id;
  std::uint32_t completions = 0;
  std::uint8_t stars = 0;
  double best_elapsed_s = 0.0;
  double best_clearance_m = 0.0;
  std::vector<std::string> knowledge_cards;
};

struct PlayerProfile {
  int version = 1;
  std::vector<ChapterProgress> chapters;
};

struct ChapterOutcome {
  std::string experience_id;
  bool robot_passed = false;
  bool safety_passed = false;
  bool mission_passed = false;
  bool experiment_passed = false;
  double elapsed_s = 0.0;
  double minimum_clearance_m = 0.0;
  std::vector<std::string> knowledge_cards;
};

// Returns the user-local profile location used by the Viewer when no explicit
// path is supplied.  This is presentation state only; it is never included in
// a runtime invariant digest and never enters ROBOT/SIM decisions.
std::string default_player_profile_path();
std::string knowledge_card_id(const std::string& experience_id,
                              std::size_t lesson_index);

// Stars are public progression only: safety is the first gate, mission is the
// second, and a complete controlled experiment is the third. No SIM truth,
// actor identity, future script, or score shortcut enters this calculation.
std::uint8_t chapter_stars(const ChapterOutcome& outcome);
void record_chapter(PlayerProfile& profile, const ChapterOutcome& outcome);

bool load_player_profile(const std::string& path, PlayerProfile& profile,
                         std::string* error = nullptr);
bool save_player_profile(const std::string& path, const PlayerProfile& profile,
                         std::string* error = nullptr);

}  // namespace lost_signal::sim::viewer
