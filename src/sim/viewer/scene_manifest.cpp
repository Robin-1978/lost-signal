#include "sim/viewer/scene_manifest.hpp"

#include <cmath>
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
 error = "cannot open scene manifest: " + path;
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
 error = std::string("scene '") + key + "' must be a non-empty string";
 return false;
 }
 value = node->as_str();
 return true;
}

bool take_number(const Json& object, const char* key, double& value,
 std::string& error) {
 const Json* node = object.find(key);
 if (node == nullptr || !node->is_num() ||
 !std::isfinite(node->as_num())) {
 error = std::string("scene '") + key + "' must be a finite number";
 return false;
 }
 value = node->as_num();
 return true;
}

bool take_vec3(const Json& object, const char* key,
 std::array<double, 3>& value, std::string& error) {
 const Json* node = object.find(key);
 if (node == nullptr || !node->is_array() ||
 node->as_array().size() != value.size()) {
 error = std::string("scene '") + key + "' must contain three numbers";
 return false;
 }
 for (std::size_t i = 0; i < value.size(); ++i) {
 const Json& component = node->as_array()[i];
 if (!component.is_num() || !std::isfinite(component.as_num())) {
 error = std::string("scene '") + key + "' contains a non-finite value";
 return false;
 }
 value[i] = component.as_num();
 }
 return true;
}

bool take_rgba(const Json& object, const char* key,
 std::array<int, 4>& value, std::string& error) {
 const Json* node = object.find(key);
 if (node == nullptr || !node->is_array() ||
 node->as_array().size() != value.size()) {
 error = std::string("scene '") + key + "' must contain four channels";
 return false;
 }
 for (std::size_t i = 0; i < value.size(); ++i) {
 const Json& channel = node->as_array()[i];
 if (!channel.is_num() || channel.as_num() < 0.0 ||
 channel.as_num() > 255.0 ||
 channel.as_num() != std::floor(channel.as_num())) {
 error = std::string("scene '") + key + "' channels must be integers 0..255";
 return false;
 }
 value[i] = static_cast<int>(channel.as_num());
 }
 return true;
}

const world::StaticBox* find_box(
 const std::vector<world::StaticBox>& boxes, const std::string& id) {
 for (const auto& box : boxes) {
 if (box.box_id == id) return &box;
 }
 return nullptr;
}

bool parse_instance(const Json& node, const AssetManifest& assets,
 const std::vector<world::StaticBox>& boxes, SceneInstance& instance,
 std::string& error) {
 if (!node.is_object() ||
 !take_string(node, "instance_id", instance.instance_id, error) ||
 !take_string(node, "asset_id", instance.asset_id, error) ||
 !take_vec3(node, "position_m", instance.position_m, error) ||
 !take_vec3(node, "scale", instance.scale, error) ||
 !take_number(node, "yaw_deg", instance.yaw_deg, error)) {
 return false;
 }
 const AssetDefinition* asset = assets.find(instance.asset_id);
 if (asset == nullptr || asset->kind != "environment") {
 error = "scene instance references a missing/non-environment asset: " +
 instance.asset_id;
 return false;
 }
 for (double scale : instance.scale) {
 if (!(scale > 0.0) || scale > 100.0) {
 error = "scene instance scale must be in (0, 100]";
 return false;
 }
 }
 const Json* box_id = node.find("collision_box_id");
 const Json* surface_only = node.find("surface_only");
 if (box_id != nullptr) {
 if (!box_id->is_str() || box_id->as_str().empty()) {
 error = "scene collision_box_id must be a non-empty string";
 return false;
 }
 instance.collision_box_id = box_id->as_str();
 }
 if (surface_only != nullptr) {
 if (!surface_only->is_bool()) {
 error = "scene surface_only must be boolean";
 return false;
 }
 instance.surface_only = surface_only->as_bool();
 }
 if (instance.surface_only == !instance.collision_box_id.empty()) {
 error = "scene instance must declare exactly one of collision_box_id or surface_only";
 return false;
 }
 if (instance.surface_only) {
 if (std::abs(instance.position_m[2]) > 0.10) {
 error = "surface-only scene instance must stay within 0.10 m of ground";
 return false;
 }
 return true;
 }
 const world::StaticBox* box = find_box(boxes, instance.collision_box_id);
 if (box == nullptr || !(box->height_m > 0.0)) {
 error = "scene instance references a missing/non-solid collision box: " +
 instance.collision_box_id;
 return false;
 }
 const double center_x = (box->x0 + box->x1) * 0.5;
 const double center_y = (box->y0 + box->y1) * 0.5;
 if (std::hypot(center_x - instance.position_m[0],
 center_y - instance.position_m[1]) > 0.02) {
 error = "scene visual centre does not match collision box: " +
 instance.collision_box_id;
 return false;
 }
 return true;
}

} // namespace

const SceneInstance* SceneManifest::visual_for_box(
 const std::string& box_id) const {
 if (box_id.empty()) return nullptr;
 for (const auto& instance : instances) {
 if (instance.collision_box_id == box_id) return &instance;
 }
 return nullptr;
}

bool load_scene_manifest(const std::string& path,
 const AssetManifest& assets,
 const std::vector<world::StaticBox>& boxes,
 SceneManifest& scene,
 std::string* error) {
 std::string local_error;
 std::string& out_error = error != nullptr ? *error : local_error;
 std::string text;
 if (!read_text(path, text, out_error)) return false;
 std::string parse_error;
 auto root = robot::core::json_parse(text, &parse_error);
 if (root == nullptr || !root->is_object()) {
 out_error = "invalid scene manifest: " + parse_error;
 return false;
 }
 const Json* format = root->find("format");
 const Json* camera = root->find("camera");
 const Json* floor = root->find("floor");
 const Json* instances = root->find("instances");
 if (format == nullptr || !format->is_str() ||
 format->as_str() != "lost-signal-scene/1" ||
 camera == nullptr || !camera->is_object() ||
 floor == nullptr || !floor->is_object() ||
 instances == nullptr || !instances->is_array()) {
 out_error = "scene needs format, camera, floor, and instances";
 return false;
 }
 SceneManifest parsed;
 if (!take_string(*root, "scene_id", parsed.scene_id, out_error) ||
 !take_number(*camera, "distance_m", parsed.camera.distance_m,
 out_error) ||
 !take_number(*camera, "yaw_deg", parsed.camera.yaw_deg, out_error) ||
 !take_number(*camera, "pitch_deg", parsed.camera.pitch_deg,
 out_error) ||
 !take_number(*camera, "target_height_m",
 parsed.camera.target_height_m, out_error) ||
 !take_rgba(*floor, "base_rgba", parsed.floor.base_rgba,
 out_error) ||
 !take_rgba(*floor, "grid_rgba", parsed.floor.grid_rgba,
 out_error) ||
 !take_number(*floor, "grid_spacing_m", parsed.floor.grid_spacing_m,
 out_error)) {
 return false;
 }
 if (parsed.camera.distance_m < 0.6 || parsed.camera.distance_m > 100.0 ||
 parsed.camera.pitch_deg < 5.0 || parsed.camera.pitch_deg > 85.0 ||
 !(parsed.floor.grid_spacing_m > 0.0) ||
 parsed.floor.grid_spacing_m > 20.0) {
 out_error = "scene camera or floor values are outside supported bounds";
 return false;
 }
 std::set<std::string> ids;
 std::set<std::string> collision_bindings;
 for (const Json& node : instances->as_array()) {
 SceneInstance instance;
 if (!parse_instance(node, assets, boxes, instance, out_error)) {
 return false;
 }
 if (!ids.insert(instance.instance_id).second) {
 out_error = "duplicate scene instance id: " + instance.instance_id;
 return false;
 }
 if (!instance.collision_box_id.empty() &&
 !collision_bindings.insert(instance.collision_box_id).second) {
 out_error = "multiple scene visuals bind collision box: " +
 instance.collision_box_id;
 return false;
 }
 parsed.instances.push_back(std::move(instance));
 }
 scene = std::move(parsed);
 out_error.clear();
 return true;
}

} // namespace lost_signal::sim::viewer
