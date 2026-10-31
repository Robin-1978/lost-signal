#include <cassert>
#include <filesystem>
#include <fstream>

#include "sim/viewer/player_profile.hpp"

int main() {
 using namespace lost_signal::sim::viewer;
 ChapterOutcome outcome;
 outcome.experience_id = "assembly_lab";
 outcome.safety_passed = true;
 outcome.mission_passed = true;
 outcome.experiment_passed = true;
 outcome.elapsed_s = 12.0;
 outcome.minimum_clearance_m = 0.42;
 outcome.knowledge_cards = {"wheel-contact", "lidar-timestamps"};
 assert(chapter_stars(outcome) == 3);
 PlayerProfile profile;
 record_chapter(profile, outcome);
 assert(profile.chapters.size() == 1);
 assert(profile.chapters.front().stars == 3);
 outcome.elapsed_s = 15.0;
 outcome.minimum_clearance_m = 0.20;
 outcome.knowledge_cards.push_back("wheel-contact");
 record_chapter(profile, outcome);
 assert(profile.chapters.front().completions == 2);
 assert(profile.chapters.front().best_elapsed_s == 12.0);
 assert(profile.chapters.front().best_clearance_m == 0.42);
 assert(profile.chapters.front().knowledge_cards.size() == 2);
 TeachingArchive archive{
     "assembly_lab", "/tmp/assembly.json",
     "/tmp/assembly.json.baseline.observations.jsonl",
     "/tmp/assembly.json.variant.observations.jsonl"};
 record_teaching_archive(profile, archive);
 record_teaching_archive(profile, archive);
 assert(profile.archives.size() == 1);
 const auto path = std::filesystem::temp_directory_path() / "lost_signal_player_profile_test.json";
 std::string error;
 assert(save_player_profile(path.string(), profile, &error));
 PlayerProfile loaded;
 assert(load_player_profile(path.string(), loaded, &error));
 assert(loaded.chapters.size() == 1 && loaded.chapters.front().stars == 3);
 assert(loaded.archives.size() == 1);
 assert(loaded.archives.front().report_path == "/tmp/assembly.json");
 std::filesystem::remove(path);
 std::ofstream duplicate_archive(path);
 duplicate_archive
     << R"({"format":"lost-signal-player-profile/1","version":1,"chapters":[],"archives":[)"
     << R"({"experience_id":"same","report_path":"/tmp/report.json","baseline_replay_path":"/tmp/base.jsonl","variant_replay_path":"/tmp/variant.jsonl"},)"
     << R"({"experience_id":"same","report_path":"/tmp/report.json","baseline_replay_path":"/tmp/base2.jsonl","variant_replay_path":"/tmp/variant2.jsonl"}]})";
 duplicate_archive.close();
 assert(!load_player_profile(path.string(), loaded, &error));
 assert(loaded.chapters.empty() && loaded.archives.empty());
 std::filesystem::remove(path);
 std::ofstream corrupt(path);
 corrupt << "{bad";
 corrupt.close();
 assert(!load_player_profile(path.string(), loaded, &error));
 assert(loaded.chapters.empty());
 std::filesystem::remove(path);
 std::ofstream wrong_format(path);
 wrong_format << R"({"format":"wrong/1","version":1,"chapters":[]})";
 wrong_format.close();
 assert(!load_player_profile(path.string(), loaded, &error));
 assert(loaded.chapters.empty());
 std::filesystem::remove(path);
 std::ofstream duplicate(path);
 duplicate << R"({"format":"lost-signal-player-profile/1","version":1,"chapters":[)"
              R"({"experience_id":"same","completions":1,"stars":1,"best_elapsed_s":1,"best_clearance_m":1,"knowledge_cards":[]},)"
              R"({"experience_id":"same","completions":1,"stars":1,"best_elapsed_s":1,"best_clearance_m":1,"knowledge_cards":[]}]})";
 duplicate.close();
 assert(!load_player_profile(path.string(), loaded, &error));
 assert(loaded.chapters.empty());
 std::filesystem::remove(path);
 return 0;
}
