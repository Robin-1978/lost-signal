#include "sim/viewer/experience_manifest.hpp"

#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>

#include "robot/core/json.hpp"

namespace lost_signal::sim::viewer {
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
 const Json* choices = prediction->find("choices");
 if (choices == nullptr || !choices->is_array() ||
 choices->as_array().size() < 2 || choices->as_array().size() > 4) {
 out_error = "experience prediction must contain two to four choices";
 return false;
 }
 std::set<std::string> choice_ids;
 int correct_choices = 0;
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
 const Json* correct = node.find("correct");
 if (correct == nullptr || !correct->is_bool() ||
 !choice_ids.insert(choice.choice_id).second) {
 out_error = "experience choice needs unique id and boolean correct";
 return false;
 }
 choice.correct = correct->as_bool();
 if (choice.correct) ++correct_choices;
 parsed.choices.push_back(std::move(choice));
 }
 if (correct_choices != 1) {
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
 parsed.walkaround.push_back(std::move(stop));
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
 experience = std::move(parsed);
 out_error.clear();
 return true;
}

} // namespace lost_signal::sim::viewer
