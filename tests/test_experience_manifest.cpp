#include <cassert>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "sim/viewer/experience_flow.hpp"
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
 assert(std::filesystem::is_regular_file(experience.next_experience_path));
 assert(std::filesystem::path(experience.next_experience_path).filename() ==
 "safety_unknown_obstacle.json");
 assert(experience.objectives.size() == 1);
 assert(experience.objectives.front() == "glass_door");
 assert(experience.time_limit_steps == 1200);
 assert(experience.intro.size() == 4);
 double intro_duration_s = 0.0;
 for (const auto& shot : experience.intro) intro_duration_s += shot.duration_s;
 assert(std::abs(intro_duration_s - 6.0) < 1e-12);
 assert(experience.intro.front().component_id == "drive_wheel");
 assert(experience.intro.back().component_id == "mission");
 assert(experience.choices.size() == 3);
 assert(experience.choices.front().evidence_rule ==
 lost_signal::sim::viewer::ExperienceEvidenceRule::kComparisonTradeoff);
 assert(experience.walkaround.size() == 4);
 assert(experience.walkaround.front().component_id == "drive_wheel");
 assert(!experience.walkaround.front().required);
 assert(!experience.walkaround[1].required);
 assert(!experience.walkaround[2].required);
 std::vector<bool> required_walkaround;
 for (const auto& stop : experience.walkaround) {
 required_walkaround.push_back(stop.required);
 }
 lost_signal::sim::viewer::ExperienceFlow first_decision({
 experience.walkaround.size(), experience.choices.size(),
 experience.time_limit_steps, required_walkaround,
 static_cast<std::uint64_t>(experience.experiments.front().step_budget)});
 assert(first_decision.walkaround_complete());
 assert(first_decision.continue_to_prediction());
 assert(intro_duration_s < 10.0);
 const std::size_t english_briefing_count = experience.briefing.size();
 const std::size_t english_intro_count = experience.intro.size();
 const std::size_t english_walkaround_count = experience.walkaround.size();
 const std::size_t english_experiment_count = experience.experiments.size();
 assert(experience.experiments.size() == 1);
 assert(experience.experiments.front().controlled_variable ==
 lost_signal::sim::viewer::ExperienceVariable::kLidarScanPeriodS);
 assert(std::string(lost_signal::sim::viewer::experience_variable_spec(
 experience.experiments.front().controlled_variable).id) ==
 "lidar.scan_period_s");
 assert(experience.experiments.front().variant_value == 0.10);
 assert(experience.experiments.front().step_budget == 1200);
 assert(experience.experiments.front().minimum_elapsed_increase_s == 1.0);
 assert(experience.experiments.front().minimum_clearance_loss_m == 0.05);
 assert(experience.experiments.front().equivalent_elapsed_tolerance_s == 0.5);
 assert(experience.experiments.front().equivalent_clearance_tolerance_m ==
 0.02);
 int correct = 0;
 for (const auto& choice : experience.choices) {
 if (choice.correct) ++correct;
 }
 assert(correct == 0); // evidence decides the outcome after both runs
 assert(std::filesystem::is_regular_file(experience.map_path));
 assert(std::filesystem::is_regular_file(experience.robot_path));
 assert(experience.startup_mode ==
 lost_signal::sim::viewer::ExperienceStartupMode::kAutomatic);

 assert(lost_signal::sim::viewer::load_experience_manifest(
 (root / "configs/experiences/zh-CN/first_contact.json").string(),
 experience, &error));
 assert(experience.title == "初次接触：精确停靠");
 assert(experience.walkaround.front().title == "驱动轮 / 牵引力");
 assert(experience.briefing.size() == english_briefing_count);
 assert(experience.intro.size() == english_intro_count);
 assert(experience.walkaround.size() == english_walkaround_count);
 assert(experience.experiments.size() == english_experiment_count);
 assert(std::filesystem::path(experience.next_experience_path).filename() ==
 "safety_unknown_obstacle.json");

 assert(lost_signal::sim::viewer::load_experience_manifest(
 (root / "configs/experiences/safety_unknown_obstacle.json").string(),
 experience, &error));
 assert(experience.walkaround.size() == 3);
 for (const auto& stop : experience.walkaround) {
 assert(!stop.required);
 }
 std::vector<bool> safety_required;
 for (const auto& stop : experience.walkaround) {
 safety_required.push_back(stop.required);
 }
 lost_signal::sim::viewer::ExperienceFlow safety_flow({
 experience.walkaround.size(), experience.choices.size(),
 experience.time_limit_steps, safety_required});
 assert(safety_flow.required_walkaround_count() == 0);
 assert(safety_flow.walkaround_complete());
 assert(safety_flow.continue_to_prediction());
 assert(std::filesystem::path(experience.next_experience_path).filename() ==
 "slam_loop.json");
 assert(lost_signal::sim::viewer::load_experience_manifest(
 (root / "configs/experiences/slam_loop.json").string(), experience,
 &error));
 assert(experience.experience_id == "slam_loop");
 assert(experience.startup_mode ==
 lost_signal::sim::viewer::ExperienceStartupMode::kSlamOnly);
 assert(experience.objectives.size() == 4);
 assert(experience.objectives.back() == "loop_home");
 assert(lost_signal::sim::viewer::load_experience_manifest(
 (root / "configs/experiences/zh-CN/slam_loop.json").string(),
 experience, &error));
 assert(experience.title == "无先验地图闭环：建图、跟踪、校正");
 assert(experience.startup_mode ==
 lost_signal::sim::viewer::ExperienceStartupMode::kSlamOnly);

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

 {
 std::ofstream out(bad);
 out <<
 R"({"format":"lost-signal-experience/1","experience_id":"bad","title":"bad","subtitle":"bad","time_limit_steps":0,"files":{"map":"../worlds/research_base.json","scene":"../worlds/research_base.scene.json","robot":"../robot/differential.json","sim":"../sim/default.json"},"objectives":["glass_door"],"briefing":["brief"],"prediction":{"question":"q","choices":[{"choice_id":"a","label":"a","explanation":"a","correct":true},{"choice_id":"b","label":"b","explanation":"b","correct":false}]},"running_prompt":"run","debrief":{"success_title":"ok","failure_title":"no","lessons":["lesson"]}})";
 }
 assert(!lost_signal::sim::viewer::load_experience_manifest(
 bad.string(), experience, &error));
 assert(error ==
 "experience 'time_limit_steps' must be a positive integer");

 write_text(bad,
 R"({"format":"lost-signal-experience/1","experience_id":"bad","title":"bad","subtitle":"bad","files":{"map":"../worlds/research_base.json","scene":"../worlds/research_base.scene.json","robot":"../robot/differential.json","sim":"../sim/default.json"},"objectives":["glass_door"],"briefing":["brief"],"experiments":[{"experiment_id":"bad","title":"bad","controlled_variable":"sim.truth_pose","variant_value":1.0,"step_budget":10}],"prediction":{"question":"q","choices":[{"choice_id":"a","label":"a","explanation":"a","correct":true},{"choice_id":"b","label":"b","explanation":"b","correct":false}]},"running_prompt":"run","debrief":{"success_title":"ok","failure_title":"no","lessons":["lesson"]}})");
 assert(!lost_signal::sim::viewer::load_experience_manifest(
 bad.string(), experience, &error));
 assert(error.find("supported variable") != std::string::npos);

 write_text(bad,
 R"({"format":"lost-signal-experience/1","experience_id":"bad","startup_mode":"truth_pose","title":"bad","subtitle":"bad","files":{"map":"../worlds/research_base.json","scene":"../worlds/research_base.scene.json","robot":"../robot/differential.json","sim":"../sim/default.json"},"objectives":["glass_door"],"briefing":["brief"],"prediction":{"question":"q","choices":[{"choice_id":"a","label":"a","explanation":"a","correct":true},{"choice_id":"b","label":"b","explanation":"b","correct":false}]},"running_prompt":"run","debrief":{"success_title":"ok","failure_title":"no","lessons":["lesson"]}})");
 assert(!lost_signal::sim::viewer::load_experience_manifest(
 bad.string(), experience, &error));
 assert(error.find("startup_mode") != std::string::npos);

 // Campaign edges must be authored relative to the current manifest. They
 // are canonical regular files, so a typo or an absolute machine-local path
 // cannot destroy a completed chapter when Next is selected.
 write_text(bad,
 R"({"format":"lost-signal-experience/1","experience_id":"bad","next_experience":"/tmp/next.json","title":"bad","subtitle":"bad","files":{"map":"../worlds/research_base.json","scene":"../worlds/research_base.scene.json","robot":"../robot/differential.json","sim":"../sim/default.json"},"objectives":["glass_door"],"briefing":["brief"],"prediction":{"question":"q","choices":[{"choice_id":"a","label":"a","explanation":"a","correct":true},{"choice_id":"b","label":"b","explanation":"b","correct":false}]},"running_prompt":"run","debrief":{"success_title":"ok","failure_title":"no","lessons":["lesson"]}})");
 assert(!lost_signal::sim::viewer::load_experience_manifest(
 bad.string(), experience, &error));
 assert(error.find("relative file path") != std::string::npos);

 write_text(bad,
 R"({"format":"lost-signal-experience/1","experience_id":"bad","next_experience":"missing-experience.json","title":"bad","subtitle":"bad","files":{"map":"../worlds/research_base.json","scene":"../worlds/research_base.scene.json","robot":"../robot/differential.json","sim":"../sim/default.json"},"objectives":["glass_door"],"briefing":["brief"],"prediction":{"question":"q","choices":[{"choice_id":"a","label":"a","explanation":"a","correct":true},{"choice_id":"b","label":"b","explanation":"b","correct":false}]},"running_prompt":"run","debrief":{"success_title":"ok","failure_title":"no","lessons":["lesson"]}})");
 assert(!lost_signal::sim::viewer::load_experience_manifest(
 bad.string(), experience, &error));
 assert(error.find("next_experience") != std::string::npos);
 std::filesystem::remove(bad);

 // The Viewer catalog is a supported entry point, not a hand-maintained
 // README promise. Every published lab must load through the same manifest
 // contract and resolve its map/scene references.
 for (const char* id : {"signal_town", "signal_town_big", "industrial_200k", "terrain_lab"}) {
 const auto catalog = root / "configs/experiences" / (std::string(id) + ".json");
 assert(lost_signal::sim::viewer::load_experience_manifest(
 catalog.string(), experience, &error));
 assert(experience.experience_id == id);
 assert(experience.objectives.size() >= 2);
 assert(std::filesystem::is_regular_file(experience.map_path));
 assert(std::filesystem::is_regular_file(experience.scene_path));
 }
	 // The catalog includes the engineering worlds that used to be reachable
	 // only by guessing a --map path. They all use the same Runtime contract.
	 for (const char* id : {"proving_ground", "orchard_hillside",
	 "stormglass_station", "washout_canyon"}) {
	 const auto catalog = root / "configs/experiences" /
	 (std::string(id) + ".json");
	 assert(lost_signal::sim::viewer::load_experience_manifest(
	 catalog.string(), experience, &error));
	 assert(experience.experience_id == id);
	 assert(experience.objectives.size() == 1);
	 assert(std::filesystem::is_regular_file(experience.map_path));
	 assert(std::filesystem::is_regular_file(experience.scene_path));
	 }

 // The authored catalog is a graph, not a list of UI-only labels. Validate
 // every root-level chapter, resolve each edge, and prove the campaign edge
 // from first_contact terminates. This catches a renamed chapter before the
 // Viewer presents a dead Next button.
 const std::filesystem::path catalog_root = root / "configs/experiences";
 std::map<std::string, std::filesystem::path> catalog;
 for (const auto& entry : std::filesystem::directory_iterator(catalog_root)) {
 if (!entry.is_regular_file() || entry.path().extension() != ".json") continue;
 lost_signal::sim::viewer::ExperienceManifest manifest;
 assert(lost_signal::sim::viewer::load_experience_manifest(
 entry.path().string(), manifest, &error));
 assert(catalog.emplace(manifest.experience_id, entry.path()).second);
 }
 assert(catalog.size() >= 11);
 for (const auto& [id, path] : catalog) {
 lost_signal::sim::viewer::ExperienceManifest manifest;
 assert(lost_signal::sim::viewer::load_experience_manifest(
 path.string(), manifest, &error));
 if (!manifest.next_experience_path.empty()) {
 auto next = std::filesystem::canonical(manifest.next_experience_path);
 assert(std::filesystem::is_regular_file(next));
 lost_signal::sim::viewer::ExperienceManifest next_manifest;
 assert(lost_signal::sim::viewer::load_experience_manifest(
 next.string(), next_manifest, &error));
 assert(catalog.count(next_manifest.experience_id) == 1);
 }
 }
 std::set<std::string> visited;
 std::string current = "first_contact";
 while (!current.empty()) {
 assert(visited.insert(current).second);
 const auto it = catalog.find(current);
 assert(it != catalog.end());
 lost_signal::sim::viewer::ExperienceManifest manifest;
 assert(lost_signal::sim::viewer::load_experience_manifest(
 it->second.string(), manifest, &error));
 if (manifest.next_experience_path.empty()) break;
 lost_signal::sim::viewer::ExperienceManifest next_manifest;
 assert(lost_signal::sim::viewer::load_experience_manifest(
 manifest.next_experience_path, next_manifest, &error));
 current = next_manifest.experience_id;
 }
 assert(visited.count("signal_town") == 1);
 assert(visited.count("signal_town_big") == 1);
 assert(visited.count("industrial_200k") == 1);
 return 0;
}
