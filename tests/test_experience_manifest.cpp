#include <cassert>
#include <filesystem>
#include <fstream>
#include <string>

#include "sim/viewer/experience_manifest.hpp"

namespace {

void write_text(const std::filesystem::path& path, const std::string& text) {
 std::ofstream output(path, std::ios::binary);
 assert(output.is_open());
 output << text;
 assert(output.good());
}

} // namespace

int main() {
 const std::filesystem::path root = TEST_SOURCE_DIR;
 lost_signal::sim::viewer::ExperienceManifest experience;
 std::string error;
 assert(lost_signal::sim::viewer::load_experience_manifest(
 (root / "configs/experiences/first_contact.json").string(), experience,
 &error));
 assert(experience.experience_id == "first_contact");
 assert(experience.objectives.size() == 1);
 assert(experience.objectives.front() == "glass_door");
 assert(experience.choices.size() == 3);
 assert(experience.walkaround.size() == 4);
 assert(experience.walkaround.front().component_id == "drive_wheel");
 assert(experience.experiments.size() == 1);
 assert(experience.experiments.front().controlled_variable ==
 "lidar.scan_period_s");
 assert(experience.experiments.front().variant_value == 0.10);
 assert(experience.experiments.front().step_budget == 1200);
 int correct = 0;
 for (const auto& choice : experience.choices) {
 if (choice.correct) ++correct;
 }
 assert(correct == 1);
 assert(std::filesystem::is_regular_file(experience.map_path));
 assert(std::filesystem::is_regular_file(experience.robot_path));

 const auto bad = root / "configs/experiences/bad_test_experience.json";
 write_text(bad,
 R"({"format":"lost-signal-experience/1","experience_id":"bad","title":"bad","subtitle":"bad","files":{"map":"../worlds/research_base.json","scene":"../worlds/research_base.scene.json","robot":"../robot/differential.json","sim":"../sim/default.json"},"objectives":["glass_door"],"briefing":["brief"],"prediction":{"question":"q","choices":[{"choice_id":"a","label":"a","explanation":"a","correct":true},{"choice_id":"b","label":"b","explanation":"b","correct":true}]},"running_prompt":"run","debrief":{"success_title":"ok","failure_title":"no","lessons":["lesson"]}})");
 assert(!lost_signal::sim::viewer::load_experience_manifest(
 bad.string(), experience, &error));
 assert(!error.empty());

 // Presentation manifests cannot silently bind two cards to the same
 // physical component; ambiguous camera targets are rejected at load time.
 write_text(bad,
 R"({"format":"lost-signal-experience/1","experience_id":"bad","title":"bad","subtitle":"bad","files":{"map":"../worlds/research_base.json","scene":"../worlds/research_base.scene.json","robot":"../robot/differential.json","sim":"../sim/default.json"},"objectives":["glass_door"],"briefing":["brief"],"walkaround":[{"component_id":"wheel","title":"a","lesson":"a"},{"component_id":"wheel","title":"b","lesson":"b"}],"prediction":{"question":"q","choices":[{"choice_id":"a","label":"a","explanation":"a","correct":true},{"choice_id":"b","label":"b","explanation":"b","correct":false}]},"running_prompt":"run","debrief":{"success_title":"ok","failure_title":"no","lessons":["lesson"]}})");
 assert(!lost_signal::sim::viewer::load_experience_manifest(
 bad.string(), experience, &error));
 assert(error == "experience walkaround component ids must be unique");

 write_text(bad,
 R"({"format":"lost-signal-experience/1","experience_id":"bad","title":"bad","subtitle":"bad","files":{"map":"../worlds/research_base.json","scene":"../worlds/research_base.scene.json","robot":"../robot/differential.json","sim":"../sim/default.json"},"objectives":["glass_door"],"briefing":["brief"],"experiments":[{"experiment_id":"bad","title":"bad","controlled_variable":"sim.truth_pose","variant_value":1.0,"step_budget":10}],"prediction":{"question":"q","choices":[{"choice_id":"a","label":"a","explanation":"a","correct":true},{"choice_id":"b","label":"b","explanation":"b","correct":false}]},"running_prompt":"run","debrief":{"success_title":"ok","failure_title":"no","lessons":["lesson"]}})");
 assert(!lost_signal::sim::viewer::load_experience_manifest(
 bad.string(), experience, &error));
 assert(error.find("supported variable") != std::string::npos);
 std::filesystem::remove(bad);
 return 0;
}
