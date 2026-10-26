#include "sim/viewer/asset_manifest.hpp"

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
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
 error = "cannot open asset manifest: " + path;
 return false;
 }
 std::ostringstream stream;
 stream << input.rdbuf();
 text = stream.str();
 return true;
}

bool take_string(const Json& object, const char* key, std::string& value,
 std::string& error, bool allow_empty = false) {
 const Json* node = object.find(key);
 if (node == nullptr || !node->is_str() ||
 (!allow_empty && node->as_str().empty())) {
 error = std::string("asset '") + key + "' must be a non-empty string";
 return false;
 }
 value = node->as_str();
 return true;
}

bool take_vec3(const Json& object, const char* key,
 std::array<double, 3>& value, std::string& error) {
 const Json* node = object.find(key);
 if (node == nullptr || !node->is_array() ||
 node->as_array().size() != value.size()) {
 error = std::string("asset '") + key + "' must contain three numbers";
 return false;
 }
 for (std::size_t i = 0; i < value.size(); ++i) {
 const Json& component = node->as_array()[i];
 if (!component.is_num() || !std::isfinite(component.as_num())) {
 error = std::string("asset '") + key + "' contains a non-finite value";
 return false;
 }
 value[i] = component.as_num();
 }
 return true;
}

bool take_string_array(const Json& object, const char* key,
 std::vector<std::string>& values, std::string& error,
 bool allow_empty = false) {
 const Json* node = object.find(key);
 if (node == nullptr || !node->is_array() ||
 (!allow_empty && node->as_array().empty())) {
 error = std::string("asset '") + key + "' must be an array";
 return false;
 }
 std::set<std::string> unique;
 for (const Json& entry : node->as_array()) {
 if (!entry.is_str() || entry.as_str().empty() ||
 !unique.insert(entry.as_str()).second) {
 error = std::string("asset '") + key +
 "' must contain unique non-empty strings";
 return false;
 }
 values.push_back(entry.as_str());
 }
 return true;
}

bool take_vec3_array(const Json& object, const char* key,
 std::vector<std::array<double, 3>>& values, std::string& error) {
 const Json* node = object.find(key);
 if (node == nullptr || !node->is_array()) {
 error = std::string("asset '") + key + "' must be an array";
 return false;
 }
 for (const Json& entry : node->as_array()) {
 if (!entry.is_array() || entry.as_array().size() != 3) {
 error = std::string("asset '") + key +
 "' entries must contain three numbers";
 return false;
 }
 std::array<double, 3> point{};
 for (std::size_t axis = 0; axis < point.size(); ++axis) {
 if (!entry.as_array()[axis].is_num() ||
 !std::isfinite(entry.as_array()[axis].as_num())) {
 error = std::string("asset '") + key +
 "' contains a non-finite point";
 return false;
 }
 point[axis] = entry.as_array()[axis].as_num();
 }
 values.push_back(point);
 }
 return true;
}

bool take_positive_integer(const Json& object, const char* key,
 std::size_t& value, std::string& error) {
 const Json* node = object.find(key);
 if (node == nullptr || !node->is_num() || node->as_num() < 1.0 ||
 node->as_num() != std::floor(node->as_num())) {
 error = std::string("asset '") + key + "' must be a positive integer";
 return false;
 }
 value = static_cast<std::size_t>(node->as_num());
 return true;
}

bool valid_content_hash(const std::string& hash) {
 if (hash.size() != 24 || hash.rfind("fnv1a64:", 0) != 0) return false;
 for (std::size_t i = 8; i < hash.size(); ++i) {
 const char c = hash[i];
 if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f'))) return false;
 }
 return true;
}

std::string content_hash(const std::filesystem::path& path,
 std::string& error) {
 std::ifstream input(path, std::ios::binary);
 if (!input.is_open()) {
 error = "cannot read asset for hashing: " + path.string();
 return {};
 }
 std::uint64_t hash = 14695981039346656037ULL;
 char buffer[8192];
 while (input.good()) {
 input.read(buffer, sizeof(buffer));
 const std::streamsize count = input.gcount();
 for (std::streamsize i = 0; i < count; ++i) {
 hash ^= static_cast<unsigned char>(buffer[i]);
 hash *= 1099511628211ULL;
 }
 }
 if (!input.eof()) {
 error = "failed while hashing asset: " + path.string();
 return {};
 }
 std::ostringstream encoded;
 encoded << "fnv1a64:" << std::hex << std::setfill('0')
 << std::setw(16) << hash;
 return encoded.str();
}

bool valid_axis(const std::string& axis) {
 static const std::set<std::string> axes{
 "+X", "-X", "+Y", "-Y", "+Z", "-Z"};
 return axes.contains(axis);
}

char axis_dimension(const std::string& axis) {
 return axis.size() == 2 ? axis[1] : '?';
}

std::array<double, 3> axis_vector(const std::string& axis) {
 std::array<double, 3> vector{};
 const double sign = axis[0] == '-' ? -1.0 : 1.0;
 const std::size_t dimension = axis[1] == 'X' ? 0U
 : axis[1] == 'Y' ? 1U : 2U;
 vector[dimension] = sign;
 return vector;
}

std::array<double, 3> cross(const std::array<double, 3>& a,
 const std::array<double, 3>& b) {
 return {a[1] * b[2] - a[2] * b[1],
 a[2] * b[0] - a[0] * b[2],
 a[0] * b[1] - a[1] * b[0]};
}

bool equal_axis(const std::array<double, 3>& a,
 const std::array<double, 3>& b) {
 return a == b;
}

bool safe_relative_model_path(const std::string& path) {
 const std::filesystem::path candidate(path);
 if (candidate.empty() || candidate.is_absolute()) return false;
 for (const auto& component : candidate) {
 if (component == "..") return false;
 }
 const std::string extension = candidate.extension().string();
 return extension == ".glb" || extension == ".gltf";
}

bool path_is_within(const std::filesystem::path& root,
                    const std::filesystem::path& candidate) {
  auto root_it = root.begin();
  auto candidate_it = candidate.begin();
  for (; root_it != root.end(); ++root_it, ++candidate_it) {
    if (candidate_it == candidate.end() || *root_it != *candidate_it) {
      return false;
    }
  }
  return true;
}

bool parse_asset(const Json& node, AssetDefinition& asset,
 std::string& error) {
 if (!node.is_object()) {
 error = "asset entry is not an object";
 return false;
 }
 if (!take_string(node, "asset_id", asset.asset_id, error) ||
 !take_string(node, "kind", asset.kind, error) ||
 !take_string(node, "path", asset.relative_path, error) ||
 !take_string(node, "source", asset.source, error) ||
 !take_string(node, "source_artifact", asset.source_artifact, error) ||
 !take_string(node, "license", asset.license, error) ||
 !take_string(node, "content_hash", asset.content_hash, error) ||
 !take_string(node, "up_axis", asset.up_axis, error) ||
 !take_string(node, "forward_axis", asset.forward_axis, error) ||
 !take_string(node, "axle_axis", asset.axle_axis, error, true) ||
 !take_vec3(node, "pivot_m", asset.pivot_m, error) ||
 !take_string(node, "pivot_semantic", asset.pivot_semantic, error) ||
 !take_vec3(node, "asset_to_link_translation_m",
 asset.asset_to_link_translation_m, error) ||
 !take_vec3(node, "asset_to_link_rpy_rad",
 asset.asset_to_link_rpy_rad, error) ||
 !take_string(node, "link_binding", asset.link_binding, error) ||
 !take_vec3_array(node, "grounding_points_m", asset.grounding_points_m,
 error) ||
 !take_string_array(node, "material_roles", asset.material_roles, error) ||
 !take_positive_integer(node, "triangle_budget", asset.triangle_budget,
 error) ||
 !take_positive_integer(node, "material_budget", asset.material_budget,
 error)) {
 return false;
 }
 if (!valid_content_hash(asset.content_hash)) {
 error = "asset content_hash must be fnv1a64 plus 16 lowercase hex digits";
 return false;
 }
 const Json* optical = node.find("sensor_optical_center_m");
 if (optical != nullptr) {
 if (!take_vec3(node, "sensor_optical_center_m",
 asset.sensor_optical_center_m, error)) return false;
 asset.has_sensor_optical_center = true;
 }
 const Json* skeleton = node.find("skeleton_root");
 if (skeleton != nullptr &&
 !take_string(node, "skeleton_root", asset.skeleton_root, error)) {
 return false;
 }
 if (!take_string_array(node, "animation_clips", asset.animation_clips,
 error, true)) {
 return false;
 }
 const Json* lods = node.find("lods");
 if (lods == nullptr || !lods->is_array() || lods->as_array().empty()) {
 error = "asset lods must be a non-empty array";
 return false;
 }
 int expected_level = 0;
 double previous_distance = -1.0;
 for (const Json& lod_node : lods->as_array()) {
 if (!lod_node.is_object()) {
 error = "asset lod entry must be an object";
 return false;
 }
 const Json* level = lod_node.find("level");
 const Json* distance = lod_node.find("maximum_distance_m");
 AssetLod lod;
 if (level == nullptr || !level->is_num() ||
 level->as_num() != expected_level ||
 !take_string(lod_node, "path", lod.relative_path, error) ||
 distance == nullptr || !distance->is_num() ||
 !std::isfinite(distance->as_num()) || distance->as_num() < 0.0 ||
 (expected_level == 0 && distance->as_num() != 0.0) ||
 (expected_level > 0 && distance->as_num() <= previous_distance)) {
 error = "asset lod levels/distances must be ordered from LOD0";
 return false;
 }
 lod.level = expected_level++;
 lod.maximum_distance_m = distance->as_num();
 previous_distance = lod.maximum_distance_m;
 if (!safe_relative_model_path(lod.relative_path)) {
 error = "asset lod path must be a safe relative GLB/GLTF path";
 return false;
 }
 asset.lods.push_back(std::move(lod));
 }
 if (asset.lods.front().relative_path != asset.relative_path) {
 error = "asset LOD0 path must match path";
 return false;
 }
 const Json* version = node.find("version");
 const Json* units = node.find("meters_per_unit");
 if (version == nullptr || !version->is_num() ||
 version->as_num() < 1.0 ||
 version->as_num() != std::floor(version->as_num())) {
 error = "asset version must be a positive integer";
 return false;
 }
 if (units == nullptr || !units->is_num() ||
 !std::isfinite(units->as_num()) || !(units->as_num() > 0.0)) {
 error = "asset meters_per_unit must be finite and positive";
 return false;
 }
 asset.version = static_cast<int>(version->as_num());
 asset.meters_per_unit = units->as_num();
 if (!valid_axis(asset.up_axis) || !valid_axis(asset.forward_axis) ||
 (!asset.axle_axis.empty() && !valid_axis(asset.axle_axis))) {
 error = "asset axes must be one of +/-X, +/-Y, +/-Z";
 return false;
 }
 if (axis_dimension(asset.up_axis) == axis_dimension(asset.forward_axis) ||
 (!asset.axle_axis.empty() &&
 (axis_dimension(asset.axle_axis) == axis_dimension(asset.up_axis) ||
 axis_dimension(asset.axle_axis) == axis_dimension(asset.forward_axis)))) {
 error = "asset up, forward, and axle axes must use distinct dimensions";
 return false;
 }
 const auto forward = axis_vector(asset.forward_axis);
 const auto up = axis_vector(asset.up_axis);
 const auto left = asset.axle_axis.empty()
 ? cross(up, forward) : axis_vector(asset.axle_axis);
 if (!equal_axis(cross(forward, left), up)) {
 error = "asset forward, axle/left, and up axes must form a right-handed frame";
 return false;
 }
 if (!safe_relative_model_path(asset.relative_path)) {
 error = "asset path must be a relative .glb or .gltf path without '..'";
 return false;
 }
 const Json* bounds = node.find("expected_bounds_m");
 if (bounds == nullptr || !bounds->is_object() ||
 !take_vec3(*bounds, "min", asset.expected_bounds_m.minimum_m, error) ||
 !take_vec3(*bounds, "max", asset.expected_bounds_m.maximum_m, error)) {
 return false;
 }
 for (std::size_t i = 0; i < 3; ++i) {
 if (!(asset.expected_bounds_m.minimum_m[i] <
 asset.expected_bounds_m.maximum_m[i])) {
 error = "asset expected bounds must have positive extent on every axis";
 return false;
 }
 }
 const Json* radius = node.find("reference_radius_m");
 if (radius != nullptr) {
 if (!radius->is_num() || !std::isfinite(radius->as_num()) ||
 radius->as_num() < 0.0) {
 error = "asset reference_radius_m must be finite non-negative";
 return false;
 }
 asset.reference_radius_m = radius->as_num();
 }
 const Json* half_width = node.find("reference_half_width_m");
 if (half_width != nullptr) {
 if (!half_width->is_num() || !std::isfinite(half_width->as_num()) ||
 half_width->as_num() < 0.0) {
 error = "asset reference_half_width_m must be finite non-negative";
 return false;
 }
 asset.reference_half_width_m = half_width->as_num();
 }
 return true;
}

} // namespace

AssetFrameTransform asset_frame_transform(const AssetDefinition& asset) {
 const auto forward = axis_vector(asset.forward_axis);
 const auto up = axis_vector(asset.up_axis);
 const auto left = asset.axle_axis.empty()
 ? cross(up, forward) : axis_vector(asset.axle_axis);

 // Rows map authored components onto canonical link X-forward, Y-left,
 // Z-up. RPY is a conventional Rz(yaw)*Ry(pitch)*Rx(roll) residual.
 const std::array<double, 9> axes{
 forward[0], forward[1], forward[2],
 left[0], left[1], left[2],
 up[0], up[1], up[2]};
 const double roll = asset.asset_to_link_rpy_rad[0];
 const double pitch = asset.asset_to_link_rpy_rad[1];
 const double yaw = asset.asset_to_link_rpy_rad[2];
 const double cx = std::cos(roll);
 const double sx = std::sin(roll);
 const double cy = std::cos(pitch);
 const double sy = std::sin(pitch);
 const double cz = std::cos(yaw);
 const double sz = std::sin(yaw);
 const std::array<double, 9> rpy{
 cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx,
 sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx,
 -sy, cy * sx, cy * cx};

 AssetFrameTransform transform;
 for (std::size_t row = 0; row < 3; ++row) {
 for (std::size_t column = 0; column < 3; ++column) {
 for (std::size_t inner = 0; inner < 3; ++inner) {
 transform.rotation[row * 3 + column] +=
 rpy[row * 3 + inner] * axes[inner * 3 + column];
 }
 }
 }
 for (std::size_t row = 0; row < 3; ++row) {
 transform.translation_m[row] = asset.asset_to_link_translation_m[row];
 for (std::size_t column = 0; column < 3; ++column) {
 transform.translation_m[row] -=
 transform.rotation[row * 3 + column] * asset.pivot_m[column];
 }
 }
 return transform;
}

std::array<double, 3> transform_asset_point(
 const AssetFrameTransform& transform,
 const std::array<double, 3>& point_m) {
 std::array<double, 3> result = transform.translation_m;
 for (std::size_t row = 0; row < 3; ++row) {
 for (std::size_t column = 0; column < 3; ++column) {
 result[row] += transform.rotation[row * 3 + column] * point_m[column];
 }
 }
 return result;
}

std::array<double, 3> asset_point_to_render(
 const AssetDefinition& asset,
 const std::array<double, 3>& point_asset_units) {
 std::array<double, 3> point_m{};
 for (std::size_t axis = 0; axis < point_m.size(); ++axis) {
 point_m[axis] = point_asset_units[axis] * asset.meters_per_unit;
 }
 const auto link = transform_asset_point(asset_frame_transform(asset), point_m);
 // Link and MuJoCo use X-forward/Y-left/Z-up. raylib uses X-forward/Y-up
 // and Z-right, so this is the one link-to-render basis change.
 return {link[0], link[2], -link[1]};
}

AssetSelection asset_selection_for_robot(const std::string& robot_id,
 const std::string& chassis_kind) {
 // chassis_kind is authoritative because configuration files intentionally
 // reuse robot identities across hardware variants. Keep the identity only
 // for selecting optional sensor/appearance variants within that family.
 std::string family;
 if (chassis_kind == "ackermann") family = "ro2";
 else if (chassis_kind == "swerve") family = "ro4";
 else if (chassis_kind == "mecanum") family = "ro5";
 else if (chassis_kind == "tracked") family = "ro3";
 else if (chassis_kind == "differential") family = "ro1";
 else if (robot_id == "ro2" || robot_id == "ro3" ||
 robot_id == "ro4" || robot_id == "ro5") family = robot_id;
 else family = "ro1";

 AssetSelection selection;
 selection.chassis = family == "ro3"
 ? "robots.ro3.hull" : "robots." + family + ".chassis";
 selection.wheel = family == "ro4"
 ? "robots.ro4.swerve_module"
 : family == "ro5" ? "robots.ro5.roller_left"
 : "robots.ro1.wheel_standard";
 selection.wheel_right = family == "ro5"
 ? "robots.ro5.roller_right" : selection.wheel;
 selection.steering_knuckle_left = family == "ro2"
 ? "robots.ro2.front_left_knuckle" : "";
 selection.steering_knuckle_right = family == "ro2"
 ? "robots.ro2.front_right_knuckle" : "";
 selection.lidar = "robots.ro1.lidar_standard";
 selection.caster_fork = "robots.ro1.caster_fork";
 selection.caster_wheel = "robots.ro1.caster_wheel";
 selection.track = "robots.ro3.track";
 if (robot_id == "ro6") {
 selection.arm_turret = "robots.ro6.turret";
 selection.arm_upper = "robots.ro6.upper_arm";
 selection.arm_forearm = "robots.ro6.forearm";
 selection.arm_tool = "robots.ro6.tool";
 }
 return selection;
}

const AssetDefinition* AssetManifest::find(
 const std::string& asset_id) const {
 for (const auto& asset : assets_) {
 if (asset.asset_id == asset_id) return &asset;
 }
 return nullptr;
}

const AssetDefinition* AssetManifest::find_by_path(
    const std::string& relative_path) const {
  for (const auto& asset : assets_) {
    if (asset.relative_path == relative_path) return &asset;
  }
  return nullptr;
}

bool load_asset_manifest(const std::string& path, AssetManifest& manifest,
 std::string* error) {
 std::string local_error;
 std::string& out_error = error != nullptr ? *error : local_error;
 std::string text;
 if (!read_text(path, text, out_error)) return false;
 std::string parse_error;
 auto root = robot::core::json_parse(text, &parse_error);
 if (root == nullptr || !root->is_object()) {
 out_error = "invalid asset manifest: " + parse_error;
 return false;
 }
 const Json* format = root->find("format");
 const Json* assets = root->find("assets");
 if (format == nullptr || !format->is_str() ||
 format->as_str() != "lost-signal-assets/2") {
 out_error = "asset manifest format must be lost-signal-assets/2";
 return false;
 }
 if (assets == nullptr || !assets->is_array() ||
 assets->as_array().empty()) {
 out_error = "asset manifest must contain a non-empty assets array";
 return false;
 }
 AssetManifest parsed;
 std::set<std::string> ids;
 std::set<std::string> paths;
  std::error_code filesystem_error;
  const std::filesystem::path models_root = std::filesystem::canonical(
      std::filesystem::absolute(std::filesystem::path(path)).parent_path(),
      filesystem_error);
  if (filesystem_error) {
    out_error = "cannot resolve asset manifest directory: " +
                filesystem_error.message();
    return false;
  }
 for (const Json& node : assets->as_array()) {
 AssetDefinition asset;
 if (!parse_asset(node, asset, out_error)) return false;
 if (!ids.insert(asset.asset_id).second) {
 out_error = "duplicate asset id: " + asset.asset_id;
 return false;
 }
 if (!paths.insert(asset.relative_path).second) {
 out_error = "duplicate asset path: " + asset.relative_path;
 return false;
 }
    const auto asset_path = std::filesystem::canonical(
        models_root / asset.relative_path, filesystem_error);
    if (filesystem_error || !path_is_within(models_root, asset_path) ||
        !std::filesystem::is_regular_file(asset_path)) {
 out_error = "asset file does not exist: " + asset.relative_path;
 return false;
 }
 const std::string measured_hash = content_hash(asset_path, out_error);
 if (measured_hash.empty()) return false;
 if (measured_hash != asset.content_hash) {
 out_error = "asset content hash differs: " + asset.relative_path;
 return false;
 }
 for (const auto& lod : asset.lods) {
 const auto lod_path = std::filesystem::canonical(
 models_root / lod.relative_path, filesystem_error);
 if (filesystem_error || !path_is_within(models_root, lod_path) ||
 !std::filesystem::is_regular_file(lod_path)) {
 out_error = "asset LOD file does not exist: " + lod.relative_path;
 return false;
 }
 }
 parsed.assets_.push_back(std::move(asset));
 }
 manifest = std::move(parsed);
 return true;
}
} // namespace lost_signal::sim::viewer
