#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace lost_signal::sim::viewer {

enum class ExperienceEvidenceRule {
 kStaticAnswer,
 kIndependentAcceptance,
 kSimRejected,
 kIncomplete,
};

struct ExperienceChoice {
 std::string choice_id;
 std::string label;
 std::string explanation;
 bool correct = false;
 ExperienceEvidenceRule evidence_rule = ExperienceEvidenceRule::kStaticAnswer;
};

struct ExperienceWalkaroundStop {
 // Presentation-only semantic selector. The Viewer resolves it to a
 // prepared MuJoCo link pose; it is never forwarded to RobotApplication.
 std::string component_id;
 std::string title;
 std::string lesson;
};

struct ExperienceExperiment {
 std::string experiment_id;
 std::string title;
 std::string controlled_variable;
 double variant_value = 0.0;
 long step_budget = 0;
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
 // Presentation run budget. Reaching it pauses the field lab and opens a
 // failed debrief; it never changes ROBOT limits or scenario acceptance.
 std::uint64_t time_limit_steps = 0;
 std::vector<std::string> briefing;
 std::vector<ExperienceWalkaroundStop> walkaround;
 std::vector<ExperienceExperiment> experiments;
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
