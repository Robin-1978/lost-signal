#pragma once

#include <string>
#include <vector>

namespace lost_signal::sim::viewer {

struct ExperienceChoice {
 std::string choice_id;
 std::string label;
 std::string explanation;
 bool correct = false;
};

struct ExperienceManifest {
 std::string experience_id;
 std::string title;
 std::string subtitle;
 std::string map_path;
 std::string scene_path;
 std::string robot_path;
 std::string sim_path;
 std::vector<std::string> objectives;
 std::vector<std::string> briefing;
 std::string prediction_question;
 std::vector<ExperienceChoice> choices;
 std::string running_prompt;
 std::string success_title;
 std::string failure_title;
 std::vector<std::string> lessons;
};

// Loads a presentation/mission manifest and resolves its file references
// relative to the manifest. It selects ordinary ROBOT/SIM configuration
// before startup; no field is published into a running RobotApplication.
bool load_experience_manifest(const std::string& path,
 ExperienceManifest& experience,
 std::string* error = nullptr);

} // namespace lost_signal::sim::viewer
