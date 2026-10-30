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
 const auto path = std::filesystem::temp_directory_path() / "lost_signal_player_profile_test.json";
 std::string error;
 assert(save_player_profile(path.string(), profile, &error));
 PlayerProfile loaded;
 assert(load_player_profile(path.string(), loaded, &error));
 assert(loaded.chapters.size() == 1 && loaded.chapters.front().stars == 3);
 std::filesystem::remove(path);
 std::ofstream corrupt(path);
 corrupt << "{bad";
 corrupt.close();
 assert(!load_player_profile(path.string(), loaded, &error));
 assert(loaded.chapters.empty());
 std::filesystem::remove(path);
 return 0;
}
