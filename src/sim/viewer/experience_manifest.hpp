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
 kComparisonTradeoff,
 kComparisonEquivalent,
 kComparisonUnsafe,
};

// Whitelisted controlled variables. A chapter changes exactly one value;
// the runtime owns the typed patch and rejects values outside this registry.
// Adding a variable therefore requires an explicit implementation and test,
// rather than a stringly-typed branch in the raylib frontend.
enum class ExperienceVariable {
 kLidarScanPeriodS,
};

// Declares who establishes the map frame before an experience starts. This
// is orchestration input, not a gameplay hint: the runtime applies it before
// the first observation and the ROBOT localization stack owns all subsequent
// estimation. `kAutomatic` preserves the normal map/prior based selector.
enum class ExperienceStartupMode {
 kAutomatic,
 kSeedAndTrack,
 kGlobalSearch,
 kSlamOnly,
};

struct ExperienceVariableSpec {
 ExperienceVariable variable = ExperienceVariable::kLidarScanPeriodS;
 const char* id = "lidar.scan_period_s";
 const char* unit = "s";
 double minimum = 0.001;
 double maximum = 10.0;
 bool physics_grid = true;
};

const ExperienceVariableSpec& experience_variable_spec(
 ExperienceVariable variable);
bool parse_experience_variable(const std::string& id,
 ExperienceVariable& variable);

struct ExperienceChoice {
 std::string choice_id;
 std::string label;
 std::string explanation;
 // Used only by legacy static-answer questions. Evidence-driven choices are
 // assessed from the completed run and do not carry an authored answer key.
 bool correct = false;
 ExperienceEvidenceRule evidence_rule = ExperienceEvidenceRule::kStaticAnswer;
};

struct ExperienceWalkaroundStop {
 // Presentation-only semantic selector. The Viewer resolves it to a
 // prepared MuJoCo link pose; it is never forwarded to RobotApplication.
 std::string component_id;
 std::string title;
 std::string lesson;
 bool required = true;
};

// Presentation-only opening shot. `component_id` selects a real MuJoCo
// body/joint or configured sensor frame; "mission" is the authored route
// composition. Camera choreography never supplies state to ROBOT.
struct ExperienceHeroShot {
 std::string component_id;
 std::string title;
 std::string caption;
 double duration_s = 0.0;
};

struct ExperienceExperiment {
 std::string experiment_id;
 std::string title;
 ExperienceVariable controlled_variable =
 ExperienceVariable::kLidarScanPeriodS;
 double variant_value = 0.0;
 long step_budget = 0;
 // Presentation-only evidence classifier. These authored tolerances decide
 // whether two completed runs demonstrate a meaningful educational effect;
 // they never tune ROBOT or SIM acceptance.
 double minimum_elapsed_increase_s = 0.0;
 double minimum_clearance_loss_m = 0.0;
 double equivalent_elapsed_tolerance_s = 0.0;
 double equivalent_clearance_tolerance_m = 0.0;
};

struct ExperienceProgressionPolicy {
 // Safety and SIM validity are unconditional gates. A safety lesson may
 // deliberately fail its delivery mission after performing the correct
 // emergency stop, so mission completion is configurable per chapter.
 bool require_mission = true;
 // Defaults to true when an experiment is declared and false otherwise.
 bool require_experiment = false;
};

struct ExperienceManifest {
 std::string experience_id;
 // Optional campaign edge, resolved relative to this manifest and
 // canonicalized at load. Presentation uses it only after the current
 // run passes its independent safety acceptance.
 std::string next_experience_path;
 std::string title;
 std::string subtitle;
 std::string map_path;
 std::string scene_path;
 std::string robot_path;
 std::string sim_path;
 ExperienceStartupMode startup_mode = ExperienceStartupMode::kAutomatic;
 std::vector<std::string> objectives;
 // Presentation run budget. Reaching it pauses the field lab and opens a
 // failed debrief; it never changes ROBOT limits or scenario acceptance.
 std::uint64_t time_limit_steps = 0;
 std::vector<std::string> briefing;
 std::vector<ExperienceHeroShot> intro;
 std::vector<ExperienceWalkaroundStop> walkaround;
 std::vector<ExperienceExperiment> experiments;
 ExperienceProgressionPolicy progression;
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
