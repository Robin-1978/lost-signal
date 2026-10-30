#include "sim/viewer/experience_manifest.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <cmath>
#include <set>
#include <sstream>

#include "robot/core/json.hpp"

namespace lost_signal::sim::viewer {

const ExperienceVariableSpec& experience_variable_spec(
 ExperienceVariable variable) {
 static const ExperienceVariableSpec kLidarScanPeriod{
 ExperienceVariable::kLidarScanPeriodS,
 "lidar.scan_period_s", "s", 0.001, 10.0, true,
 ExperienceVariableKind::kContinuous};
 static const ExperienceVariableSpec kChassisKind{
 ExperienceVariable::kChassisKind, "chassis.kind", "category", 0.0, 0.0,
 false, ExperienceVariableKind::kCategorical};
 switch (variable) {
 case ExperienceVariable::kLidarScanPeriodS:
 return kLidarScanPeriod;
 case ExperienceVariable::kChassisKind:
 return kChassisKind;
 }
 return kLidarScanPeriod;
}

bool parse_experience_variable(const std::string& id,
 ExperienceVariable& variable) {
 for (const auto candidate : {ExperienceVariable::kLidarScanPeriodS,
                              ExperienceVariable::kChassisKind}) {
   const auto& spec = experience_variable_spec(candidate);
   if (id == spec.id) {
     variable = spec.variable;
     return true;
   }
 }
 return false;
}

ExperienceVariableKind experience_variable_kind(ExperienceVariable variable) {
 return experience_variable_spec(variable).kind;
}

bool experience_variable_category_valid(ExperienceVariable variable,
                                        const std::string& category) {
 if (experience_variable_kind(variable) !=
     ExperienceVariableKind::kCategorical) {
   return false;
 }
 // Chassis profiles are the deployable robot contracts already validated by
 // robot_config. Keeping this closed domain prevents a manifest from
 // smuggling an arbitrary path or string branch into the experiment runner.
 if (variable == ExperienceVariable::kChassisKind) {
   return category == "differential" || category == "ackermann" ||
          category == "mecanum" || category == "swerve" ||
          category == "tracked";
 }
 return false;
}

namespace {

using robot::core::Json;

bool read_text(const std::string& path, std::string& text,
 std::string& error) {
 std::ifstream input(path, std::ios::binary);
 if (!input.is_open()) {
 error = "cannot open experience manifest: " + path;
 return false;
 }
 std::ostringstream stream;
 stream << input.rdbuf();
 text = stream.str();
 return true;
}

bool take_string(const Json& object, const char* key, std::string& value,
 std::string& error) {
 const Json* node = object.find(key);
 if (node == nullptr || !node->is_str() || node->as_str().empty()) {
 error = std::string("experience '") + key +
 "' must be a non-empty string";
 return false;
 }
 value = node->as_str();
 return true;
}

bool take_strings(const Json& object, const char* key,
 std::vector<std::string>& values, std::string& error) {
 const Json* node = object.find(key);
 if (node == nullptr || !node->is_array() || node->as_array().empty()) {
 error = std::string("experience '") + key +
 "' must be a non-empty string array";
 return false;
 }
 std::set<std::string> unique;
 for (const Json& item : node->as_array()) {
 if (!item.is_str() || item.as_str().empty() ||
 !unique.insert(item.as_str()).second) {
 error = std::string("experience '") + key +
 "' contains an empty or duplicate value";
 return false;
 }
 values.push_back(item.as_str());
 }
 return true;
}

bool resolve_file(const std::filesystem::path& root,
 const std::string& authored, std::string& resolved, std::string& error) {
 const std::filesystem::path path = root / authored;
 std::error_code filesystem_error;
 const auto canonical = std::filesystem::canonical(path, filesystem_error);
 if (filesystem_error || !std::filesystem::is_regular_file(canonical)) {
 error = "experience file reference does not exist: " + authored;
 return false;
 }
 resolved = canonical.string();
 return true;
}

bool resolve_relative_file(const std::filesystem::path& root,
 const std::string& authored, const char* field,
 std::string& resolved, std::string& error) {
 const std::filesystem::path relative(authored);
 if (authored.empty() || relative.is_absolute()) {
 error = std::string("experience '") + field +
 "' must be a non-empty relative file path";
 return false;
 }
 if (!resolve_file(root, authored, resolved, error)) {
 error = std::string("experience '") + field + "': " + error;
 return false;
 }
 return true;
}

bool parse_evidence_rule(const std::string& text,
 ExperienceEvidenceRule& rule) {
 if (text == "independent_acceptance") {
 rule = ExperienceEvidenceRule::kIndependentAcceptance;
 } else if (text == "sim_rejected") {
 rule = ExperienceEvidenceRule::kSimRejected;
 } else if (text == "incomplete") {
 rule = ExperienceEvidenceRule::kIncomplete;
  } else if (text == "comparison_tradeoff") {
    rule = ExperienceEvidenceRule::kComparisonTradeoff;
  } else if (text == "comparison_equivalent") {
    rule = ExperienceEvidenceRule::kComparisonEquivalent;
  } else if (text == "comparison_unsafe") {
    rule = ExperienceEvidenceRule::kComparisonUnsafe;
 } else {
 return false;
 }
 return true;
}

bool parse_startup_mode(const std::string& text,
 ExperienceStartupMode& mode) {
 if (text == "auto") {
 mode = ExperienceStartupMode::kAutomatic;
 } else if (text == "seed_and_track") {
 mode = ExperienceStartupMode::kSeedAndTrack;
 } else if (text == "global_search") {
 mode = ExperienceStartupMode::kGlobalSearch;
 } else if (text == "slam_only") {
 mode = ExperienceStartupMode::kSlamOnly;
 } else {
 return false;
 }
 return true;
}

bool take_non_negative_number(const Json& object, const char* key,
                              double& value, std::string& error) {
  const Json* node = object.find(key);
  if (node == nullptr || !node->is_num() ||
      !std::isfinite(node->as_num()) || node->as_num() < 0.0) {
    error = std::string("experience '") + key +
            "' must be a finite non-negative number";
    return false;
  }
  value = node->as_num();
  return true;
}

bool take_optional_bool(const Json& object, const char* key, bool& value,
 std::string& error) {
 const Json* node = object.find(key);
 if (node == nullptr) return true;
 if (!node->is_bool()) {
 error = std::string("experience '") + key + "' must be boolean";
 return false;
 }
 value = node->as_bool();
 return true;
}

bool take_optional_string(const Json& object, const char* key,
                          std::string& value, std::string& error) {
 const Json* node = object.find(key);
 if (node == nullptr) return true;
 if (!node->is_str() || node->as_str().empty()) {
  error = std::string("experience '") + key +
          "' must be a non-empty string";
  return false;
 }
 value = node->as_str();
 return true;
}

} // namespace

bool load_experience_manifest(const std::string& path,
 ExperienceManifest& experience, std::string* error) {
 std::string local_error;
 std::string& out_error = error != nullptr ? *error : local_error;
 out_error.clear();
 std::string text;
 if (!read_text(path, text, out_error)) return false;
 std::string parse_error;
 auto root = robot::core::json_parse(text, &parse_error);
 if (root == nullptr || !root->is_object()) {
 out_error = "invalid experience manifest: " + parse_error;
 return false;
 }
 const Json* format = root->find("format");
 const Json* files = root->find("files");
 const Json* prediction = root->find("prediction");
 const Json* debrief = root->find("debrief");
 if (format == nullptr || !format->is_str() ||
 format->as_str() != "lost-signal-experience/1" ||
 files == nullptr || !files->is_object() ||
 prediction == nullptr || !prediction->is_object() ||
 debrief == nullptr || !debrief->is_object()) {
 out_error = "experience needs format, files, prediction, and debrief";
 return false;
 }
 ExperienceManifest parsed;
 std::string authored_map, authored_scene, authored_robot, authored_sim;
 if (!take_string(*root, "experience_id", parsed.experience_id,
 out_error) ||
 !take_string(*root, "title", parsed.title, out_error) ||
 !take_string(*root, "subtitle", parsed.subtitle, out_error) ||
 !take_string(*files, "map", authored_map, out_error) ||
 !take_string(*files, "scene", authored_scene, out_error) ||
 !take_string(*files, "robot", authored_robot, out_error) ||
 !take_string(*files, "sim", authored_sim, out_error) ||
 !take_strings(*root, "objectives", parsed.objectives, out_error) ||
 !take_strings(*root, "briefing", parsed.briefing, out_error) ||
 !take_string(*prediction, "question", parsed.prediction_question,
 out_error) ||
 !take_string(*root, "running_prompt", parsed.running_prompt,
 out_error) ||
 !take_string(*debrief, "success_title", parsed.success_title,
 out_error) ||
 !take_string(*debrief, "failure_title", parsed.failure_title,
 out_error) ||
 !take_strings(*debrief, "lessons", parsed.lessons, out_error)) {
 return false;
 }
 const Json* startup_mode = root->find("startup_mode");
 if (startup_mode != nullptr &&
 (!startup_mode->is_str() ||
 !parse_startup_mode(startup_mode->as_str(), parsed.startup_mode))) {
 out_error = "experience 'startup_mode' must be auto, seed_and_track, "
 "global_search, or slam_only";
 return false;
 }
 const Json* time_limit = root->find("time_limit_steps");
 if (time_limit != nullptr) {
 if (!time_limit->is_num() || !std::isfinite(time_limit->as_num()) ||
 time_limit->as_num() < 1.0 ||
 std::floor(time_limit->as_num()) != time_limit->as_num()) {
 out_error = "experience 'time_limit_steps' must be a positive integer";
 return false;
 }
 const Json* assembly = root->find("assembly");
 if (assembly != nullptr) {
  parsed.has_assembly = true;
  if (!assembly->is_object() ||
      !take_optional_string(*assembly, "wheel", parsed.assembly.wheel,
                            out_error) ||
      !take_optional_string(*assembly, "lidar", parsed.assembly.lidar,
                            out_error) ||
      !take_optional_string(*assembly, "compute", parsed.assembly.compute,
                            out_error) ||
      !validate_assembly_selection(parsed.assembly, &out_error)) {
   if (out_error.empty()) out_error = "experience assembly is invalid";
   return false;
  }
 }
 parsed.time_limit_steps =
 static_cast<std::uint64_t>(time_limit->as_num());
 }
 const Json* choices = prediction->find("choices");
 if (choices == nullptr || !choices->is_array() ||
 choices->as_array().size() < 2 || choices->as_array().size() > 4) {
 out_error = "experience prediction must contain two to four choices";
 return false;
 }
 std::set<std::string> choice_ids;
 int correct_choices = 0;
 bool has_static_choice = false;
 bool has_evidence_choice = false;
 bool has_terminal_evidence = false;
 bool has_comparison_evidence = false;
 std::set<int> evidence_rules;
 for (const Json& node : choices->as_array()) {
 if (!node.is_object()) {
 out_error = "experience choice must be an object";
 return false;
 }
 ExperienceChoice choice;
 if (!take_string(node, "choice_id", choice.choice_id, out_error) ||
 !take_string(node, "label", choice.label, out_error) ||
 !take_string(node, "explanation", choice.explanation, out_error)) {
 return false;
 }
 if (!choice_ids.insert(choice.choice_id).second) {
 out_error = "experience choice ids must be unique";
 return false;
 }
 const Json* evidence_rule = node.find("evidence_rule");
 const Json* correct = node.find("correct");
 if (evidence_rule != nullptr) {
 if (!evidence_rule->is_str() ||
 !parse_evidence_rule(evidence_rule->as_str(), choice.evidence_rule)) {
 out_error = "experience choice has unsupported evidence_rule";
 return false;
 }
 if (correct != nullptr) {
 out_error = "evidence-driven choice must not author 'correct'";
 return false;
 }
 has_evidence_choice = true;
 const bool comparison_rule =
 choice.evidence_rule == ExperienceEvidenceRule::kComparisonTradeoff ||
 choice.evidence_rule == ExperienceEvidenceRule::kComparisonEquivalent ||
 choice.evidence_rule == ExperienceEvidenceRule::kComparisonUnsafe;
 has_comparison_evidence = has_comparison_evidence || comparison_rule;
 has_terminal_evidence = has_terminal_evidence || !comparison_rule;
 if (!evidence_rules.insert(static_cast<int>(choice.evidence_rule)).second) {
 out_error = "experience evidence rules must be unique";
 return false;
 }
 } else {
 if (correct == nullptr || !correct->is_bool()) {
 out_error = "static choice needs boolean 'correct'";
 return false;
 }
 choice.correct = correct->as_bool();
 choice.evidence_rule = ExperienceEvidenceRule::kStaticAnswer;
 has_static_choice = true;
 }
 if (choice.correct) ++correct_choices;
 parsed.choices.push_back(std::move(choice));
 }
 if (has_static_choice && has_evidence_choice) {
 out_error = "experience cannot mix static and evidence-driven choices";
 return false;
 }
 if (has_terminal_evidence && has_comparison_evidence) {
 out_error = "experience cannot mix terminal and comparison evidence";
 return false;
 }
 if (has_terminal_evidence &&
 (evidence_rules.size() != 3 ||
 evidence_rules.count(static_cast<int>(
 ExperienceEvidenceRule::kIndependentAcceptance)) == 0 ||
 evidence_rules.count(static_cast<int>(
 ExperienceEvidenceRule::kSimRejected)) == 0 ||
 evidence_rules.count(static_cast<int>(
 ExperienceEvidenceRule::kIncomplete)) == 0)) {
 out_error = "terminal prediction must cover accepted, rejected, and incomplete";
 return false;
 }
 if (has_comparison_evidence &&
 (evidence_rules.size() != 3 ||
 evidence_rules.count(static_cast<int>(
 ExperienceEvidenceRule::kComparisonTradeoff)) == 0 ||
 evidence_rules.count(static_cast<int>(
 ExperienceEvidenceRule::kComparisonEquivalent)) == 0 ||
 evidence_rules.count(static_cast<int>(
 ExperienceEvidenceRule::kComparisonUnsafe)) == 0)) {
 out_error = "comparison prediction must cover tradeoff, equivalent, and unsafe";
 return false;
 }
 if (has_static_choice && correct_choices != 1) {
 out_error = "experience prediction must have exactly one correct choice";
 return false;
 }
 const Json* walkaround = root->find("walkaround");
 if (walkaround != nullptr) {
 if (!walkaround->is_array() || walkaround->as_array().size() < 2 ||
 walkaround->as_array().size() > 8) {
 out_error = "experience walkaround must contain two to eight stops";
 return false;
 }
 std::set<std::string> component_ids;
 for (const Json& node : walkaround->as_array()) {
 if (!node.is_object()) {
 out_error = "experience walkaround stop must be an object";
 return false;
 }
 ExperienceWalkaroundStop stop;
 if (!take_string(node, "component_id", stop.component_id, out_error) ||
 !take_string(node, "title", stop.title, out_error) ||
 !take_string(node, "lesson", stop.lesson, out_error) ||
 !component_ids.insert(stop.component_id).second) {
 if (out_error.empty()) {
 out_error = "experience walkaround component ids must be unique";
 }
 return false;
 }
 const Json* required = node.find("required");
 if (required != nullptr) {
 if (!required->is_bool()) {
 out_error = "experience walkaround 'required' must be boolean";
 return false;
 }
 stop.required = required->as_bool();
 }
 parsed.walkaround.push_back(std::move(stop));
 }
 }
 const Json* intro = root->find("intro");
 if (intro != nullptr) {
 if (!intro->is_object()) {
 out_error = "experience intro must be an object";
 return false;
 }
 const Json* shots = intro->find("shots");
 if (shots == nullptr || !shots->is_array() ||
 shots->as_array().size() < 2 || shots->as_array().size() > 6) {
 out_error = "experience intro must contain two to six shots";
 return false;
 }
 double total_duration_s = 0.0;
 for (const Json& node : shots->as_array()) {
 if (!node.is_object()) {
 out_error = "experience intro shot must be an object";
 return false;
 }
 ExperienceHeroShot shot;
 if (!take_string(node, "component_id", shot.component_id, out_error) ||
 !take_string(node, "title", shot.title, out_error) ||
 !take_string(node, "caption", shot.caption, out_error)) {
 return false;
 }
 const Json* duration = node.find("duration_s");
 if (duration == nullptr || !duration->is_num() ||
 !std::isfinite(duration->as_num()) || duration->as_num() < 0.5 ||
 duration->as_num() > 4.0) {
 out_error =
 "experience intro shot duration_s must be between 0.5 and 4 seconds";
 return false;
 }
 shot.duration_s = duration->as_num();
 total_duration_s += shot.duration_s;
 const auto is_live_component = [](const std::string& id) {
   if (id == "com") return true;
   constexpr const char* prefix = "arm_joint_";
   if (id.rfind(prefix, 0) != 0 || id.size() != 11) return false;
   const char ordinal = id.back();
   return ordinal >= '1' && ordinal <= '4';
 };
 const bool known_component = shot.component_id == "mission" ||
 is_live_component(shot.component_id) ||
 std::any_of(parsed.walkaround.begin(), parsed.walkaround.end(),
 [&](const ExperienceWalkaroundStop& stop) {
 return stop.component_id == shot.component_id;
 });
 if (!known_component) {
 out_error = "experience intro shot must reference a walkaround component";
 return false;
 }
 parsed.intro.push_back(std::move(shot));
 }
 if (total_duration_s < 5.0 || total_duration_s > 7.0) {
 out_error = "experience intro must last between 5 and 7 seconds";
 return false;
 }
 }
 const Json* experiments = root->find("experiments");
 if (experiments != nullptr) {
 // The current flow is one controlled A/B per chapter. Reject additional
 // entries instead of silently running only experiments.front().
 if (!experiments->is_array() || experiments->as_array().size() != 1) {
 out_error = "experience must contain exactly one controlled experiment";
 return false;
 }
 std::set<std::string> experiment_ids;
 for (const Json& node : experiments->as_array()) {
 if (!node.is_object()) {
 out_error = "experience experiment must be an object";
 return false;
 }
 ExperienceExperiment experiment;
 std::string controlled_variable_id;
 if (!take_string(node, "experiment_id", experiment.experiment_id,
 out_error) ||
 !take_string(node, "title", experiment.title, out_error) ||
 !take_string(node, "controlled_variable",
 controlled_variable_id, out_error)) {
 return false;
 }
 const Json* variant = node.find("variant_value");
 const Json* variant_category = node.find("variant_category");
 const Json* variant_robot = node.find("variant_robot");
 const Json* budget = node.find("step_budget");
 const ExperienceVariableSpec* variable_spec = nullptr;
 if (parse_experience_variable(
 controlled_variable_id, experiment.controlled_variable)) {
 variable_spec = &experience_variable_spec(experiment.controlled_variable);
 }
 const bool categorical = variable_spec != nullptr &&
     variable_spec->kind == ExperienceVariableKind::kCategorical;
 const bool valid_continuous = !categorical && variable_spec != nullptr &&
     variant != nullptr && variant->is_num() &&
     std::isfinite(variant->as_num()) &&
     variant->as_num() >= variable_spec->minimum &&
     variant->as_num() <= variable_spec->maximum &&
     (!variable_spec->physics_grid || std::abs(
         variant->as_num() * 1000.0 -
         std::round(variant->as_num() * 1000.0)) <= 1e-9);
 const bool valid_categorical = categorical && variant_category != nullptr &&
     variant_category->is_str() &&
     experience_variable_category_valid(experiment.controlled_variable,
                                        variant_category->as_str()) &&
     variant_robot != nullptr && variant_robot->is_str() &&
     !variant_robot->as_str().empty();
 if (!experiment_ids.insert(experiment.experiment_id).second ||
 variable_spec == nullptr || (!valid_continuous && !valid_categorical) ||
 budget == nullptr || !budget->is_num() ||
 !std::isfinite(budget->as_num()) || budget->as_num() < 1.0 ||
 std::floor(budget->as_num()) != budget->as_num()) {
 out_error = "experience experiment needs a unique id, supported variable, "
 "valid variant, and integer step budget";
 return false;
 }
 if (valid_continuous) {
   experiment.variant_value = variant->as_num();
 } else {
   experiment.variant_category = variant_category->as_str();
   experiment.variant_robot_path = variant_robot->as_str();
 }
 experiment.step_budget = static_cast<long>(budget->as_num());
    if (!take_non_negative_number(
            node, "minimum_elapsed_increase_s",
            experiment.minimum_elapsed_increase_s, out_error) ||
        !take_non_negative_number(
            node, "minimum_clearance_loss_m",
            experiment.minimum_clearance_loss_m, out_error) ||
        !take_non_negative_number(
            node, "equivalent_elapsed_tolerance_s",
            experiment.equivalent_elapsed_tolerance_s, out_error) ||
        !take_non_negative_number(
            node, "equivalent_clearance_tolerance_m",
            experiment.equivalent_clearance_tolerance_m, out_error)) {
      return false;
    }
 parsed.experiments.push_back(std::move(experiment));
 }
 }
 if (has_comparison_evidence && parsed.experiments.empty()) {
 out_error = "comparison evidence requires a controlled experiment";
 return false;
 }
 if (has_comparison_evidence) {
 const auto& experiment = parsed.experiments.front();
 if (experiment.minimum_elapsed_increase_s <=
 experiment.equivalent_elapsed_tolerance_s &&
 experiment.minimum_clearance_loss_m <=
 experiment.equivalent_clearance_tolerance_m) {
 out_error = "comparison tradeoff and equivalent evidence overlap";
 return false;
 }
 }
 parsed.progression.require_experiment = !parsed.experiments.empty();
 const Json* progression = root->find("progression");
 if (progression != nullptr) {
 if (!progression->is_object() || !take_optional_bool(
 *progression, "require_mission", parsed.progression.require_mission,
 out_error) || !take_optional_bool(
 *progression, "require_experiment",
 parsed.progression.require_experiment, out_error)) {
 if (out_error.empty()) out_error = "experience progression must be an object";
 return false;
 }
 if (parsed.progression.require_experiment && parsed.experiments.empty()) {
 out_error = "experience progression requires a missing experiment";
 return false;
 }
 }
 const std::filesystem::path manifest_root =
 std::filesystem::absolute(std::filesystem::path(path)).parent_path();
 if (!resolve_file(manifest_root, authored_map, parsed.map_path,
 out_error) ||
 !resolve_file(manifest_root, authored_scene, parsed.scene_path,
 out_error) ||
 !resolve_file(manifest_root, authored_robot, parsed.robot_path,
 out_error) ||
 !resolve_file(manifest_root, authored_sim, parsed.sim_path,
 out_error)) {
 return false;
 }
 for (auto& experiment : parsed.experiments) {
   if (experiment.variant_robot_path.empty()) continue;
   std::string resolved_variant_robot;
   if (!resolve_relative_file(manifest_root, experiment.variant_robot_path,
                              "variant_robot", resolved_variant_robot,
                              out_error)) {
     return false;
   }
   experiment.variant_robot_path = std::move(resolved_variant_robot);
 }
 const Json* next_experience = root->find("next_experience");
 if (next_experience != nullptr) {
 if (!next_experience->is_str() ||
 !resolve_relative_file(manifest_root, next_experience->as_str(),
 "next_experience", parsed.next_experience_path, out_error)) {
 if (out_error.empty()) {
 out_error = "experience 'next_experience' must be a relative file path";
 }
 return false;
 }
 }
 experience = std::move(parsed);
 out_error.clear();
 return true;
}

} // namespace lost_signal::sim::viewer
