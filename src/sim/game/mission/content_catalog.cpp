#include "content_catalog.hpp"
#include "robot/autonomy/planning/task/task_ordering.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
#include <set>
#include <sstream>
#include <unordered_set>

namespace mission {
namespace {

using json = nlohmann::json;

json read_json(const std::filesystem::path& path) {
 std::ifstream input(path);
 if (!input) {
  throw ContentError("cannot open content file: " + path.string());
 }
 try {
  return json::parse(input);
 } catch (const json::exception& error) {
  throw ContentError(path.string() + ": " + error.what());
 }
}

json read_localized_json(const std::filesystem::path& path, const std::filesystem::path& root,
                         const std::string& locale) {
 auto value = read_json(path);
 const auto localized_path = root / "locales" / locale / std::filesystem::relative(path, root);
 if (std::filesystem::is_regular_file(localized_path)) {
  try {
   const auto overlay = read_json(localized_path);
   if (overlay.value("schema_version", 0) != 1 || !overlay.at("translations").is_object())
    throw ContentError(localized_path.string() + ": invalid locale overlay");
   for (const auto& [pointer_text, translated] : overlay.at("translations").items()) {
    if (!translated.is_string())
     throw ContentError(localized_path.string() + ": translation must be a string at " + pointer_text);
    const json::json_pointer pointer(pointer_text);
    if (!value.contains(pointer) || !value.at(pointer).is_string())
     throw ContentError(localized_path.string() + ": translation points to a non-string at " + pointer_text);
    value.at(pointer) = translated;
   }
  } catch (const json::exception& error) {
   throw ContentError(localized_path.string() + ": cannot apply locale overlay: " + error.what());
  }
 }
 return value;
}

void require_schema_v1(const json& value, const std::filesystem::path& path) {
 if (value.value("schema_version", 0) != 1) {
  throw ContentError(path.string() + ": unsupported schema_version");
 }
}

void require_schema_version(const json& value, const std::filesystem::path& path, int expected) {
 if (value.value("schema_version", 0) != expected) {
  throw ContentError(path.string() + ": unsupported schema_version");
 }
}

std::vector<std::filesystem::path> json_files(const std::filesystem::path& directory) {
 if (!std::filesystem::is_directory(directory)) {
  throw ContentError("missing content directory: " + directory.string());
 }
 std::vector<std::filesystem::path> result;
 for (const auto& entry : std::filesystem::directory_iterator(directory)) {
  if (entry.is_regular_file() && entry.path().extension() == ".json") {
   result.push_back(entry.path());
  }
 }
 std::sort(result.begin(), result.end());
 return result;
}

sim::Vec2 vec2(const json& value) {
 return {value.at(0).get<double>(), value.at(1).get<double>()};
}

Vec3 vec3(const json& value) {
 return {value.at(0).get<double>(), value.at(1).get<double>(), value.at(2).get<double>()};
}

Vec3 rotate_rpy_degrees(Vec3 value, Vec3 rotation_deg) {
 constexpr double kDegreesToRadians = sim::kPi / 180.0;
 const double roll = rotation_deg.x * kDegreesToRadians;
 const double pitch = rotation_deg.y * kDegreesToRadians;
 const double yaw = rotation_deg.z * kDegreesToRadians;
 const double sr = std::sin(roll);
 const double cr = std::cos(roll);
 const double sp = std::sin(pitch);
 const double cp = std::cos(pitch);
 const double sy = std::sin(yaw);
 const double cy = std::cos(yaw);
 const Vec3 rolled{value.x, cr * value.y - sr * value.z, sr * value.y + cr * value.z};
 const Vec3 pitched{cp * rolled.x + sp * rolled.z, rolled.y, -sp * rolled.x + cp * rolled.z};
 return {cy * pitched.x - sy * pitched.y, sy * pitched.x + cy * pitched.y, pitched.z};
}

double norm(Vec3 value) {
 return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

Vec3 subtract(Vec3 left, Vec3 right) {
 return {left.x - right.x, left.y - right.y, left.z - right.z};
}

Vec3 add(Vec3 left, Vec3 right) {
 return {left.x + right.x, left.y + right.y, left.z + right.z};
}

double dot(Vec3 left, Vec3 right) {
 return left.x * right.x + left.y * right.y + left.z * right.z;
}

double inertia_about_model_z(const RigidLinkDefinition& link) {
 const Vec3 local_x = rotate_rpy_degrees({1.0, 0.0, 0.0}, link.rotation_deg);
 const Vec3 local_y = rotate_rpy_degrees({0.0, 1.0, 0.0}, link.rotation_deg);
 const Vec3 local_z = rotate_rpy_degrees({0.0, 0.0, 1.0}, link.rotation_deg);
 return link.inertia_diagonal_kgm2.x * local_x.z * local_x.z +
        link.inertia_diagonal_kgm2.y * local_y.z * local_y.z +
        link.inertia_diagonal_kgm2.z * local_z.z * local_z.z;
}

std::array<unsigned char, 4> rgba(const std::string& value) {
 if (value.size() != 7 && value.size() != 9) {
  throw ContentError("visual color must use #RRGGBB or #RRGGBBAA");
 }
 const auto channel = [&](std::size_t offset) {
  return static_cast<unsigned char>(std::stoul(value.substr(offset, 2), nullptr, 16));
 };
 const auto alpha = value.size() == 9 ? channel(7) : static_cast<unsigned char>(255);
 return {channel(1), channel(3), channel(5), alpha};
}

VisualDefinition visual(const json& value) {
 return {
   value.at("primitive_2d").get<std::string>(),
   vec3(value.at("size_m")),
   rgba(value.at("color").get<std::string>()),
   value.at("model_3d").get<std::string>(),
 };
}

void require_visual_asset(const VisualDefinition& value, const std::filesystem::path& root,
                          const std::filesystem::path& source) {
 if (!std::filesystem::is_regular_file(root / value.model_3d)) {
  throw ContentError(source.string() + ": missing 3D model '" + value.model_3d + "'");
 }
}

Pose pose(const json& value) {
 return {value.at(0).get<double>(), value.at(1).get<double>(), value.at(2).get<double>()};
}

Properties properties(const json& value) {
 Properties result;
 for (auto it = value.begin(); it != value.end(); ++it) {
  result.emplace(it.key(), it.value().get<double>());
 }
 return result;
}

BuildSelection build(const json& value) {
 // Slot names come from the JSON keys themselves: a new hardware slot needs
 // no loader change (schema constrains the spelling, validator the semantics).
 BuildSelection selection;
 for (auto it = value.begin(); it != value.end(); ++it) {
  if (it.key() == "style" || it.key() == "task_policy") {
   continue;
  }
  selection.parts[it.key()] = it.value().get<std::string>();
 }
 selection.style = value.at("style").get<double>();
 selection.task_policy = value.value("task_policy", std::string{});
 return selection;
}

template <typename Value>
void insert_unique(std::unordered_map<std::string, Value>& values, Value value,
                   const std::filesystem::path& path) {
 const auto id = value.id;
 if (!values.emplace(id, std::move(value)).second) {
  throw ContentError(path.string() + ": duplicate id '" + id + "'");
 }
}

bool contains(const std::vector<std::string>& values, const std::string& value) {
 return std::find(values.begin(), values.end(), value) != values.end();
}

double cross(sim::Vec2 origin, sim::Vec2 a, sim::Vec2 b) {
 return (a.x - origin.x) * (b.y - origin.y) - (a.y - origin.y) * (b.x - origin.x);
}

std::vector<sim::Vec2> convex_hull(std::vector<sim::Vec2> points) {
 std::sort(points.begin(), points.end(), [](const auto& left, const auto& right) {
  return left.x < right.x || (left.x == right.x && left.y < right.y);
 });
 points.erase(std::unique(points.begin(), points.end(),
                          [](const auto& left, const auto& right) {
                           return std::abs(left.x - right.x) < 1e-9 && std::abs(left.y - right.y) < 1e-9;
                          }),
              points.end());
 if (points.size() < 3)
  return points;
 std::vector<sim::Vec2> hull;
 for (const auto& point : points) {
  while (hull.size() >= 2 && cross(hull[hull.size() - 2], hull.back(), point) <= 1e-9)
   hull.pop_back();
  hull.push_back(point);
 }
 const std::size_t lower_size = hull.size();
 for (auto it = points.rbegin() + 1; it != points.rend(); ++it) {
  while (hull.size() > lower_size && cross(hull[hull.size() - 2], hull.back(), *it) <= 1e-9)
   hull.pop_back();
  hull.push_back(*it);
 }
 hull.pop_back();
 return hull;
}

double origin_support_margin(const std::vector<sim::Vec2>& contacts) {
 const auto hull = convex_hull(contacts);
 if (hull.size() < 3)
  return -1.0;
 double margin = std::numeric_limits<double>::infinity();
 for (std::size_t index = 0; index < hull.size(); ++index) {
  const auto& from = hull[index];
  const auto& to = hull[(index + 1) % hull.size()];
  const double edge_length = std::hypot(to.x - from.x, to.y - from.y);
  const double inward_distance = cross(from, to, {0.0, 0.0}) / std::max(1e-9, edge_length);
  margin = std::min(margin, inward_distance);
 }
 return margin;
}

bool valid_locale_name(const std::string& value) {
 if (value.empty() || value.front() == '-' || value.back() == '-')
  return false;
 bool previous_hyphen = false;
 for (const unsigned char character : value) {
  const bool hyphen = character == '-';
  if ((!std::isalnum(character) && !hyphen) || (hyphen && previous_hyphen))
   return false;
  previous_hyphen = hyphen;
 }
 return true;
}

double point_segment_distance(sim::Vec2 point, sim::Vec2 from, sim::Vec2 to) {
 const sim::Vec2 segment{to.x - from.x, to.y - from.y};
 const double length_squared = segment.x * segment.x + segment.y * segment.y;
 if (length_squared <= 1e-18)
 return std::hypot(point.x - from.x, point.y - from.y);
 const sim::Vec2 relative{point.x - from.x, point.y - from.y};
 const double fraction = std::clamp(
 (relative.x * segment.x + relative.y * segment.y) / length_squared,
 0.0, 1.0);
 const sim::Vec2 nearest{
 from.x + fraction * segment.x,
 from.y + fraction * segment.y,
 };
 return std::hypot(point.x - nearest.x, point.y - nearest.y);
}

bool point_in_actor_swept_volume(sim::Vec2 point, const ActorDefinition& actor,
 double additional_radius_m) {
 if (actor.path.empty() || actor.radius_m < 0.0 || additional_radius_m < 0.0)
 return false;
 const double envelope_radius_m = actor.radius_m + additional_radius_m;
 if (actor.path.size() == 1)
 return point_segment_distance(point, actor.path.front(), actor.path.front()) <= envelope_radius_m;
 if (actor.kind == "patrol") {
 return point_segment_distance(point, actor.path[0], actor.path[1]) <= envelope_radius_m;
 }
 for (std::size_t index = 0; index < actor.path.size(); ++index) {
 if (point_segment_distance(point, actor.path[index],
 actor.path[(index + 1) % actor.path.size()]) <= envelope_radius_m) {
 return true;
 }
 }
 return false;
}

} // namespace

double property(const Properties& properties, const std::string& name) {
 const auto found = properties.find(name);
 if (found == properties.end()) {
  throw ContentError("missing behavior property '" + name + "'");
 }
 return found->second;
}

bool active_in_level(const std::vector<std::string>& levels, const std::string& level_id) {
 return levels.empty() || contains(levels, level_id);
}

ContentCatalog ContentCatalog::load(const std::filesystem::path& assets_root,
                                    const std::string& requested_locale) {
 ContentCatalog catalog;
 catalog.root_ = std::filesystem::canonical(assets_root);

 const auto locales_path = catalog.root_ / "locales.json";
 const auto locales = read_json(locales_path);
 require_schema_v1(locales, locales_path);
 catalog.supported_locales_ = locales.at("supported_locales").get<std::vector<std::string>>();
 if (catalog.supported_locales_.empty())
  throw ContentError(locales_path.string() + ": supported_locales must not be empty");
 std::set<std::string> unique_locales;
 for (const auto& locale : catalog.supported_locales_) {
  if (!valid_locale_name(locale))
   throw ContentError(locales_path.string() + ": invalid locale code '" + locale + "'");
  if (!unique_locales.insert(locale).second)
   throw ContentError(locales_path.string() + ": duplicate locale code '" + locale + "'");
 }
 catalog.locale_ =
   requested_locale.empty() ? locales.at("default_locale").get<std::string>() : requested_locale;
 if (!contains(catalog.supported_locales_, catalog.locale_)) {
  std::ostringstream message;
  message << "unsupported locale '" << catalog.locale_ << "' (supported:";
  for (const auto& locale : catalog.supported_locales_)
   message << ' ' << locale;
  message << ')';
  throw ContentError(message.str());
 }

 const auto campaign_path = catalog.root_ / "campaign.json";
 const auto campaign_json = read_localized_json(campaign_path, catalog.root_, catalog.locale_);
 require_schema_v1(campaign_json, campaign_path);
 const auto& companion = campaign_json.at("companion");
 catalog.campaign_.id = campaign_json.at("id").get<std::string>();
 catalog.campaign_.title = campaign_json.at("title").get<std::string>();
 catalog.campaign_.content_version = campaign_json.at("content_version").get<std::string>();
 catalog.campaign_.companion_code = companion.at("unit_code").get<std::string>();
 catalog.campaign_.companion_name = companion.at("display_name").get<std::string>();
 catalog.campaign_.companion_robot = companion.at("robot").get<std::string>();
 catalog.campaign_.starting_level = campaign_json.at("starting_level").get<std::string>();
 catalog.campaign_.levels = campaign_json.at("levels").get<std::vector<std::string>>();
 catalog.campaign_.playgrounds = campaign_json.at("playgrounds").get<std::vector<std::string>>();
 for (const auto& collection : campaign_json.at("collections")) {
 catalog.campaign_.collections.push_back({
 collection.at("id").get<std::string>(),
 collection.at("eyebrow").get<std::string>(),
 collection.at("title").get<std::string>(),
 collection.at("subtitle").get<std::string>(),
 collection.at("accent").get<std::string>(),
 collection.at("levels").get<std::vector<std::string>>(),
 });
 }
 catalog.campaign_.starting_inventory =
 campaign_json.at("starting_inventory").get<std::vector<std::string>>();
 catalog.campaign_.starting_policies =
 campaign_json.at("starting_policies").get<std::vector<std::string>>();

 const auto task_policies_path = catalog.root_ / "task_policies.json";
 const auto task_policies_json = read_localized_json(task_policies_path, catalog.root_, catalog.locale_);
 require_schema_v1(task_policies_json, task_policies_path);
 const auto task_algorithms = autonomy::TaskOrderingRegistry::with_builtins();
 for (const auto& item : task_policies_json.at("task_policies")) {
  TaskPolicyDefinition policy{
    item.at("id").get<std::string>(),          item.at("title").get<std::string>(),
    item.at("description").get<std::string>(), item.at("algorithm").get<std::string>(),
    properties(item.at("parameters")),
  };
  if (!task_algorithms.contains(policy.algorithm)) {
   throw ContentError(task_policies_path.string() + ": unknown task-ordering algorithm '" + policy.algorithm +
                      "'");
  }
  insert_unique(catalog.task_policies_, std::move(policy), task_policies_path);
 }

 {
 const auto path = catalog.root_ / "construction_pieces.json";
 const auto value = read_localized_json(path, catalog.root_, catalog.locale_);
require_schema_v1(value, path);
 for (const auto& item : value.at("pieces")) {
 const auto& collision = item.at("collision");
 const auto& support = item.at("support");
 const auto& presentation = item.at("presentation");
 ConstructionPieceDefinition piece;
 piece.id = item.at("id").get<std::string>();
 piece.title = item.at("title").get<std::string>();
 piece.description = item.at("description").get<std::string>();
 piece.size_m = vec3(collision.at("size_m"));
 piece.mass_kg = item.at("mass_kg").get<double>();
 piece.surface_material = item.at("top_surface").at("material").get<std::string>();
 piece.required_endpoint_contacts = support.at("required_endpoint_contacts").get<int>();
 piece.max_span_m = support.at("max_span_m").get<double>();
 piece.allowed_socket_types = item.at("allowed_socket_types").get<std::vector<std::string>>();
 piece.color = rgba(presentation.at("color").get<std::string>());
 piece.top_color = rgba(presentation.value("top_color", presentation.at("color").get<std::string>()));
 piece.accent_color = rgba(presentation.value("accent_color", std::string{"#27E0CAEB"}));
 piece.surface_pattern = presentation.value("surface_pattern", std::string{"plate"});
 piece.model_3d = presentation.value("model_3d", std::string{});
 if (!piece.model_3d.empty() && !std::filesystem::is_regular_file(catalog.root_ / piece.model_3d)) {
 throw ContentError(path.string() + ": missing construction model '" + piece.model_3d + "'");
 }
 insert_unique(catalog.construction_pieces_, std::move(piece), path);
 }
 }

 for (const auto& path : json_files(catalog.root_ / "levels")) {
  const auto value = read_localized_json(path, catalog.root_, catalog.locale_);
  require_schema_v1(value, path);
  const auto& objective = value.at("objective");
  const auto& loadout = value.at("loadout_rules");
  const auto& teaching = value.at("teaching");
  const auto& score = value.at("score");
  LevelDefinition level;
  level.id = value.at("id").get<std::string>();
  level.order = value.at("order").get<int>();
  level.chapter = value.at("chapter").get<int>();
  level.mode = value.value("mode", std::string{"experiment"});
  level.title = value.at("title").get<std::string>();
  level.map = value.at("map").get<std::string>();
  level.control_mode = value.at("control_mode").get<std::string>();
  level.localization_mode = value.at("localization_mode").get<std::string>();
  if (level.mode != "mainline" && level.mode != "contract" && level.mode != "experiment" &&
      level.mode != "playground") {
   throw ContentError(path.string() + ": unsupported mode");
  }
 if (level.control_mode != "manual_calibration" && level.control_mode != "operator_drive" &&
 level.control_mode != "autonomous") {
   throw ContentError(path.string() + ": unsupported control_mode");
  }
 if (level.localization_mode != "odometry" && level.localization_mode != "known_map") {
   throw ContentError(path.string() + ": unsupported localization_mode");
  }
  level.prerequisites = value.at("prerequisites").get<std::vector<std::string>>();
  level.seed = value.at("seed").get<unsigned>();
  level.objective = {
    objective.at("type").get<std::string>(),
    objective.at("spawn").get<std::string>(),
    objective.value("via", std::string{}),
    objective.at("goal").get<std::string>(),
    objective.value("via_label", std::string{}),
    objective.at("goal_label").get<std::string>(),
    objective.at("goal_visual").get<std::string>(),
    objective.at("time_limit_s").get<double>(),
    objective.at("max_collisions").get<int>(),
    objective.at("arrive_tolerance_m").get<double>(),
    objective.value("max_arrival_speed_mps", 999.0),
    objective.value("via_radius_m", 1.25),
    {},
  };
  level.objective.goal_order = objective.value("goal_order", std::string{"authored"});
  level.objective.task_policy = objective.value("task_policy", std::string{"task.balanced"});
  level.objective.return_reserve_s = objective.value("return_reserve_s", 0.0);
 if (level.objective.goal_order != "authored" && level.objective.goal_order != "autonomous" &&
 level.objective.goal_order != "player") {
   throw ContentError(path.string() + ": unknown goal_order " + level.objective.goal_order);
  }
  // Unified goals: explicit list wins; otherwise derive from legacy fields so
  // reach = [complete] and deliver_via = [pickup, complete]. Same arrivals,
  // same order, same radii — derivation is behavior-preserving by construction.
  if (objective.contains("goals")) {
   for (const auto& entry : objective.at("goals")) {
    GoalStep step;
    step.marker = entry.at("marker").get<std::string>();
    step.label = entry.value("label", std::string{});
    step.action = entry.value("action", "complete");
    step.radius_m = entry.value("radius_m", level.objective.arrive_tolerance_m);
    step.summon_actor = entry.value("summon_actor", std::string{});
    step.recharge_wh = entry.value("recharge_wh", 0.0);
    step.dwell_s = entry.value("dwell_s", 0.0);
    step.id = entry.value("id", step.marker);
    step.visual = entry.value("visual", std::string{});
    if (const auto item = entry.find("item"); item != entry.end()) {
     step.item_count = item->at("count").get<int>();
     step.item_mass_kg = item->at("mass_kg").get<double>();
     step.item_cog_height_m = item->at("cog_height_m").get<double>();
     step.item_value = item->at("value").get<double>();
     step.item_fragility = item->at("fragility").get<double>();
    }
    if (step.action != "pickup" && step.action != "complete" && step.action != "summon" &&
        step.action != "recharge" && step.action != "inspect" && step.action != "collect") {
     throw ContentError(path.string() + ": unknown goal action " + step.action);
    }
    if (step.action == "collect" && (step.item_count <= 0 || step.item_mass_kg <= 0.0)) {
     throw ContentError(path.string() + ": collect goal requires positive item count and mass");
    }
    if (step.action == "inspect" && step.dwell_s <= 0.0) {
     throw ContentError(path.string() + ": inspect goal requires dwell_s > 0");
    }
    level.objective.goals.push_back(std::move(step));
   }
   if (level.objective.goals.empty()) {
    throw ContentError(path.string() + ": empty goals list");
   }
   std::set<std::string> task_ids;
   for (const auto& step : level.objective.goals) {
    if (!task_ids.insert(step.id.empty() ? step.marker : step.id).second) {
     throw ContentError(path.string() + ": duplicate goal id " + step.id);
    }
   }
  } else if (!level.objective.via.empty()) {
   GoalStep pickup;
   pickup.id = level.objective.via;
   pickup.marker = level.objective.via;
   pickup.label = level.objective.via_label;
   pickup.action = "pickup";
   pickup.radius_m = level.objective.via_radius_m;
   level.objective.goals.push_back(std::move(pickup));
   GoalStep complete;
   complete.id = level.objective.goal;
   complete.marker = level.objective.goal;
   complete.label = level.objective.goal_label;
   complete.action = "complete";
   complete.radius_m = level.objective.arrive_tolerance_m;
   level.objective.goals.push_back(std::move(complete));
  } else {
   GoalStep complete;
   complete.id = level.objective.goal;
   complete.marker = level.objective.goal;
   complete.label = level.objective.goal_label;
   complete.action = "complete";
   complete.radius_m = level.objective.arrive_tolerance_m;
   level.objective.goals.push_back(std::move(complete));
  }
  level.budget = loadout.at("budget").get<int>();
 level.loadout_access =
 loadout.value("access", level.mode == "playground" ? std::string{"open"} : std::string{"campaign"});
 if (level.loadout_access != "campaign" && level.loadout_access != "open") {
 throw ContentError(path.string() + ": unsupported loadout access");
 }
  level.required_slots = loadout.at("required_slots").get<std::vector<std::string>>();
  level.concept_id = teaching.at("concept").get<std::string>();
  level.prediction_prompt = teaching.at("prediction_prompt").get<std::string>();
  level.knowledge_card = teaching.at("knowledge_card").get<std::string>();
  level.safe_min_clearance_m = score.at("safe_min_clearance_m").get<double>();
  level.safe_clearance_scope = score.value("safe_clearance_scope", "all");
  if (level.safe_clearance_scope != "all" && level.safe_clearance_scope != "static" &&
      level.safe_clearance_scope != "dynamic") {
   throw ContentError(path.string() + ": invalid safe_clearance_scope");
  }
  level.max_localization_error_m = score.value("max_localization_error_m", 999.0);
  if (const auto planning = value.find("planning"); planning != value.end()) {
   level.planning_localization_margin_m = planning->at("localization_margin_m").get<double>();
  }
  level.fast_time_s = score.at("fast_time_s").get<double>();
  level.efficient_energy_wh = score.at("efficient_energy_wh").get<double>();
  level.rewards = value.at("rewards").get<std::vector<std::string>>();
  if (const auto payload = value.find("payload"); payload != value.end()) {
   level.payload_mass_kg = payload->at("mass_kg").get<double>();
   level.payload_cog_height_m = payload->at("cog_height_m").get<double>();
  }
  if (const auto environment = value.find("environment"); environment != value.end()) {
   double previous_time_s = -1.0;
   for (const auto& item : environment->at("keyframes")) {
    EnvironmentKeyframeDefinition keyframe;
    keyframe.id = item.at("id").get<std::string>();
    keyframe.label = item.at("label").get<std::string>();
    keyframe.time_s = item.at("time_s").get<double>();
    keyframe.wetness = item.at("wetness").get<double>();
    keyframe.visibility = item.at("visibility").get<double>();
    keyframe.daylight = item.at("daylight").get<double>();
    keyframe.material_grip = item.at("material_grip").get<std::unordered_map<std::string, double>>();
    if (keyframe.time_s <= previous_time_s) {
     throw ContentError(path.string() + ": environment keyframe times must increase");
    }
    previous_time_s = keyframe.time_s;
    level.environment_keyframes.push_back(std::move(keyframe));
   }
   if (!level.environment_keyframes.empty() && std::abs(level.environment_keyframes.front().time_s) > 1e-9) {
    throw ContentError(path.string() + ": environment timeline must start at 0 s");
   }
  }
 if (const auto maneuvers = value.find("maneuvers"); maneuvers != value.end()) {
 std::set<std::string> ids;
 for (const auto& item : *maneuvers) {
 ManeuverDefinition maneuver{
 item.at("id").get<std::string>(),
 item.at("title").get<std::string>(),
 item.at("description").get<std::string>(),
 item.at("spawn").get<std::string>(),
 item.at("goal").get<std::string>(),
 item.at("control").get<std::string>(),
 item.at("metrics").get<std::vector<std::string>>(),
 };
 if (!ids.insert(maneuver.id).second ||
 (maneuver.control != "manual" && maneuver.control != "autonomous") ||
 maneuver.metrics.empty()) {
 throw ContentError(path.string() + ": invalid maneuver '" + maneuver.id + "'");
 }
 level.maneuvers.push_back(std::move(maneuver));
 }
 }
 if (const auto construction = value.find("construction"); construction != value.end()) {
 ConstructionScenarioDefinition scenario;
 scenario.id = construction->at("id").get<std::string>();
 scenario.snap_tolerance_m = construction->at("snap_tolerance_m").get<double>();
 scenario.inventory = construction->at("inventory").get<std::unordered_map<std::string, int>>();
 std::set<std::string> socket_ids;
 for (const auto& item : construction->at("sockets")) {
 ConstructionSocketDefinition socket{
 item.at("id").get<std::string>(),
 item.at("type").get<std::string>(),
 vec3(item.at("position_m")),
 };
 if (!socket_ids.insert(socket.id).second)
 throw ContentError(path.string() + ": duplicate construction socket '" + socket.id + "'");
 scenario.sockets.push_back(std::move(socket));
 }
 scenario.default_plan = construction->at("default_plan").get<std::string>();
 std::set<std::string> plan_ids;
 for (const auto& item : construction->at("plans")) {
 ConstructionPlanOptionDefinition option;
 option.id = item.at("id").get<std::string>();
 if (!plan_ids.insert(option.id).second)
 throw ContentError(path.string() + ": duplicate construction plan option '" + option.id + "'");
 for (const auto& encoded : item.at("placements")) {
 option.placements.push_back({
 encoded.at("instance_id").get<std::string>(),
 encoded.at("piece_id").get<std::string>(),
 encoded.at("start_socket").get<std::string>(),
 encoded.at("end_socket").get<std::string>(),
 });
 }
 scenario.plans.push_back(std::move(option));
 }
 if (!plan_ids.count(scenario.default_plan))
 throw ContentError(path.string() + ": construction default_plan is not a plan option");
 for (const auto& [piece_id, count] : scenario.inventory) {
 if (count <= 0 || catalog.construction_pieces_.find(piece_id) == catalog.construction_pieces_.end()) {
 throw ContentError(path.string() + ": invalid construction inventory piece '" + piece_id + "'");
 }
 }
 level.construction = std::move(scenario);
 }
 if (const auto manipulation = value.find("manipulation"); manipulation != value.end()) {
 ManipulationProgramDefinition program;
 program.success_position_tolerance_m =
 manipulation->value("success_position_tolerance_m", 0.03);
 double previous_time = -1.0;
 for (const auto& item : manipulation->at("waypoints")) {
 ManipulationWaypointDefinition waypoint{
 item.at("time_s").get<double>(), vec3(item.at("target_m")),
 item.at("tool_pitch_rad").get<double>()};
 if (!(waypoint.time_s > previous_time))
 throw ContentError(path.string() + ": manipulation waypoint times must increase");
 previous_time = waypoint.time_s;
 program.waypoints.push_back(waypoint);
 }
 if (program.waypoints.empty() || !(program.success_position_tolerance_m > 0.0))
 throw ContentError(path.string() + ": invalid manipulation program");
 level.manipulation = std::move(program);
 }
  insert_unique(catalog.levels_, std::move(level), path);
 }

 {
  const auto path = catalog.root_ / "surfaces.json";
  const auto value = read_localized_json(path, catalog.root_, catalog.locale_);
require_schema_version(value, path, 2);
  for (const auto& item : value.at("materials")) {
   SurfaceMaterialDefinition material;
   material.id = item.at("id").get<std::string>();
   material.title = item.at("title").get<std::string>();
   material.description = item.at("description").get<std::string>();
   material.friction_mu = item.at("friction_mu").get<double>();
   material.texture_albedo = item.value("texture", std::string{});
 material.albedo_tint = rgba(item.value("albedo_tint", std::string{"#FFFFFF"}));
   material.rolling_drag = item.at("rolling_drag").get<double>();
   material.traction_scale = item.at("traction_scale").get<double>();
material.contact_time_constant_s = item.value("contact_time_constant_s", 0.0025);
material.contact_damping_ratio = item.value("contact_damping_ratio", 1.0);
   material.lidar_reflectivity = item.at("lidar_reflectivity").get<double>();
   if (const auto effect = item.find("drive_effect"); effect != item.end()) {
    material.drive_effect = effect->at("kind").get<std::string>();
    material.effect_color = rgba(effect->at("color").get<std::string>());
    material.effect_intensity = effect->at("intensity").get<double>();
   }
   insert_unique(catalog.surface_materials_, std::move(material), path);
  }
 }
 {
  const auto path = catalog.root_ / "abilities.json";
  const auto value = read_localized_json(path, catalog.root_, catalog.locale_);
  require_schema_v1(value, path);
  for (const auto& item : value.at("abilities")) {
   AbilityDefinition ability;
   ability.id = item.at("id").get<std::string>();
   ability.title = item.at("title").get<std::string>();
   ability.description = item.at("description").get<std::string>();
   insert_unique(catalog.abilities_, std::move(ability), path);
  }
 }
 {
  const auto path = catalog.root_ / "strings.json";
  const auto value = read_localized_json(path, catalog.root_, catalog.locale_);
  require_schema_v1(value, path);
  for (const auto& [key, text] : value.at("strings").items()) {
   if (!catalog.strings_.emplace(key, text.get<std::string>()).second) {
    throw ContentError(path.string() + ": duplicate string '" + key + "'");
   }
  }
 }
 for (const auto& path : json_files(catalog.root_ / "contracts")) {
  const auto value = read_localized_json(path, catalog.root_, catalog.locale_);
  require_schema_v1(value, path);
  for (const auto& item : value.at("jobs")) {
   ContractJob job;
   job.id = item.at("id").get<std::string>();
   job.level = item.at("level").get<std::string>();
   job.cargo = item.at("cargo").get<std::string>();
   job.axis = item.at("axis").get<std::string>();
   job.rep = item.at("rep").get<int>();
   job.mass_kg = item.at("mass_kg").get<double>();
   job.cog_height_m = item.at("cog_height_m").get<double>();
   job.fragility = item.at("fragility").get<double>();
   job.blurb = item.at("blurb").get<std::string>();
   // Unknown levels fail loudly here (validator pins the same rule in Python).
   if (catalog.levels_.find(job.level) == catalog.levels_.end()) {
    throw ContentError(path.string() + ": unknown contract level '" + job.level + "'");
   }
   for (const auto& known : catalog.contracts_) {
    if (known.id == job.id) {
     throw ContentError(path.string() + ": duplicate contract job '" + job.id + "'");
    }
   }
   catalog.contracts_.push_back(std::move(job));
  }
 }
 for (const auto& path : json_files(catalog.root_ / "maps")) {
  const auto value = read_localized_json(path, catalog.root_, catalog.locale_);
 require_schema_version(value, path, 2);
  MapDefinition map;
  map.id = value.at("id").get<std::string>();
  map.scene_3d = value.at("scene_3d").get<std::string>();
  map.size_m = vec2(value.at("size_m"));
  map.grid_resolution_m = value.at("grid_resolution_m").get<double>();
  for (const auto& item : value.at("obstacles")) {
   map.obstacles.push_back({
     item.at("id").get<std::string>(),
     vec2(item.at("min")),
     vec2(item.at("max")),
     item.value("active_in", std::vector<std::string>{}),
   });
  }
  for (const auto& item : value.at("surfaces")) {
   const auto& material = catalog.surface_material(item.at("material").get<std::string>());
   SurfaceDefinition surface;
   surface.id = item.at("id").get<std::string>();
   surface.min = vec2(item.at("min"));
   surface.max = vec2(item.at("max"));
   surface.material = material.id;
   surface.active_in = item.value("active_in", std::vector<std::string>{});
   surface.label = item.at("label").get<std::string>();
   surface.traction_scale = material.traction_scale;
   surface.friction_mu = material.friction_mu;
   surface.rolling_drag = material.rolling_drag;
   surface.contact_time_constant_s = material.contact_time_constant_s;
   surface.contact_damping_ratio = material.contact_damping_ratio;
   surface.lidar_reflectivity = material.lidar_reflectivity;
   map.surfaces.push_back(std::move(surface));
  }
  for (const auto& item : value.value("terrain", nlohmann::json::array())) {
   TerrainPatchDefinition patch;
   patch.id = item.at("id").get<std::string>();
   patch.min = vec2(item.at("min"));
   patch.max = vec2(item.at("max"));
   patch.base_height_m = item.at("base_height_m").get<double>();
   patch.grade_x = item.at("grade_x").get<double>();
   patch.grade_y = item.at("grade_y").get<double>();
   patch.roughness = item.value("roughness", 0.0);
   patch.active_in = item.value("active_in", std::vector<std::string>{});
 patch.grade_xy_per_m = item.value("grade_xy_per_m", 0.0);
   if (!(patch.min.x < patch.max.x && patch.min.y < patch.max.y)) {
    throw ContentError(path.string() + ": terrain patch '" + patch.id + "' has an invalid box");
   }
   map.terrain.push_back(std::move(patch));
  }
  for (const auto& item : value.value("conflict_zones", nlohmann::json::array())) {
   ConflictZoneDefinition zone;
   zone.id = item.at("id").get<std::string>();
   zone.min = vec2(item.at("min"));
   zone.max = vec2(item.at("max"));
   zone.authority = item.at("authority").get<std::string>();
   zone.active_in = item.value("active_in", std::vector<std::string>{});
 for (const auto& approach_item :
 item.value("observation_approaches", nlohmann::json::array())) {
 ConflictObservationApproachDefinition approach;
 for (const auto& point : approach_item.at("centerline"))
 approach.centerline.push_back(vec2(point));
 approach.half_width_m = approach_item.at("half_width_m").get<double>();
 if (approach.centerline.size() < 2 || !(approach.half_width_m > 0.0)) {
 throw ContentError(path.string() + ": conflict zone '" + zone.id +
 "' has an invalid observation approach");
 }
 zone.observation_approaches.push_back(std::move(approach));
 }
   if (!(zone.min.x < zone.max.x && zone.min.y < zone.max.y)) {
    throw ContentError(path.string() + ": conflict zone '" + zone.id + "' has an invalid box");
   }
 constexpr double kApproachBoundaryToleranceM = 1e-6;
 for (const auto& approach : zone.observation_approaches) {
 const auto& start = approach.centerline.front();
 const bool within_x = start.x >= zone.min.x - kApproachBoundaryToleranceM &&
 start.x <= zone.max.x + kApproachBoundaryToleranceM;
 const bool within_y = start.y >= zone.min.y - kApproachBoundaryToleranceM &&
 start.y <= zone.max.y + kApproachBoundaryToleranceM;
 const bool on_x_edge = std::abs(start.x - zone.min.x) <= kApproachBoundaryToleranceM ||
 std::abs(start.x - zone.max.x) <= kApproachBoundaryToleranceM;
 const bool on_y_edge = std::abs(start.y - zone.min.y) <= kApproachBoundaryToleranceM ||
 std::abs(start.y - zone.max.y) <= kApproachBoundaryToleranceM;
 if (!(within_x && within_y && (on_x_edge || on_y_edge))) {
 throw ContentError(path.string() + ": conflict zone '" + zone.id +
 "' observation approach must start on the zone boundary");
 }
 for (std::size_t point = 1; point < approach.centerline.size(); ++point) {
 const auto& value = approach.centerline[point];
 const bool strictly_inside =
 value.x > zone.min.x + kApproachBoundaryToleranceM &&
 value.x < zone.max.x - kApproachBoundaryToleranceM &&
 value.y > zone.min.y + kApproachBoundaryToleranceM &&
 value.y < zone.max.y - kApproachBoundaryToleranceM;
 if (strictly_inside) {
 throw ContentError(path.string() + ": conflict zone '" + zone.id +
 "' observation approach must extend outward from its boundary");
 }
 }
 }
   if (zone.authority != "shadow" && zone.authority != "control") {
    throw ContentError(path.string() + ": conflict zone '" + zone.id + "' has unknown authority '" +
                       zone.authority + "'");
   }
   map.conflict_zones.push_back(std::move(zone));
  }
 std::set<std::string> world_object_ids;
 for (const auto& item : value.value("world_objects", nlohmann::json::array())) {
 WorldObjectDefinition object;
 object.id = item.at("id").get<std::string>();
 if (!world_object_ids.insert(object.id).second) {
 throw ContentError(path.string() + ": duplicate world object '" + object.id + "'");
 }
 object.prefab = item.at("prefab").get<std::string>();
 object.position_m = vec3(item.at("position_m"));
 object.scale = vec3(item.value("scale", json::array({1.0, 1.0, 1.0})));
 if (!(object.scale.x > 0.0 && object.scale.y > 0.0 && object.scale.z > 0.0)) {
 throw ContentError(path.string() + ": world object '" + object.id + "' has invalid scale");
 }
 object.yaw_degrees = item.value("yaw_degrees", 0.0);
 object.active_in = item.value("active_in", std::vector<std::string>{});
 for (const auto& active_id : object.active_in) {
 (void)catalog.level(active_id);
 }
 for (const auto& collision : item.at("collisions")) {
 if (collision.at("type").get<std::string>() != "box") {
 throw ContentError(path.string() + ": world object '" + object.id +
 "' uses an unsupported collision type");
 }
 const auto material = collision.at("material").get<std::string>();
 const auto& surface = catalog.surface_material(material);
 const auto size_m = vec3(collision.at("size_m"));
 if (!(size_m.x > 0.0 && size_m.y > 0.0 && size_m.z > 0.0)) {
 throw ContentError(path.string() + ": world object '" + object.id +
 "' has invalid collision size");
 }
 object.collisions.push_back({
 vec3(collision.at("position_m")),
 size_m,
 collision.value("yaw_degrees", 0.0),
 material,
 surface.friction_mu,
 surface.rolling_drag,
 surface.contact_time_constant_s,
 surface.contact_damping_ratio,
 surface.lidar_reflectivity,
 });
 }
 map.world_objects.push_back(std::move(object));
 }
  for (auto it = value.at("markers").begin(); it != value.at("markers").end(); ++it) {
   map.markers.emplace(it.key(), pose(it.value()));
  }
  for (const auto& item : value.at("actors")) {
   ActorDefinition actor;
   actor.id = item.at("id").get<std::string>();
   actor.kind = item.at("type").get<std::string>();
   if (actor.kind != "patrol" && actor.kind != "wander" && actor.kind != "dwell" &&
       actor.kind != "seeded-wander") {
    throw ContentError(path.string() + ": unknown actor kind " + actor.kind);
   }
   for (const auto& point : item.at("path")) {
    actor.path.push_back(vec2(point));
   }
   actor.speed_mps = item.at("speed_mps").get<double>();
   actor.radius_m = item.at("radius_m").get<double>();
   actor.active_in = item.at("active_in").get<std::vector<std::string>>();
   actor.dormant_until_summoned = item.value("dormant_until_summoned", false);
   actor.waits = item.value("waits", std::vector<double>{});
   const std::string collision = item.at("collision").get<std::string>();
   if (collision != "fatal" && collision != "social") {
    throw ContentError(path.string() + ": unknown actor collision " + collision);
   }
   actor.collision_social = (collision == "social");
   const auto& visual = item.at("visual");
   actor.visual_model = visual.at("model").get<std::string>();
   actor.visual_kind = visual.value("kind", std::string{"vehicle"});
   if (actor.visual_kind != "vehicle" && actor.visual_kind != "pedestrian") {
    throw ContentError(path.string() + ": unknown actor visual kind " + actor.visual_kind);
   }
   actor.visual_ref_radius_m = visual.at("ref_radius_m").get<double>();
   if (!std::filesystem::is_regular_file(catalog.root_ / actor.visual_model)) {
    throw ContentError(path.string() + ": missing actor model '" + actor.visual_model + "'");
   }
   map.actors.push_back(std::move(actor));
  }
  insert_unique(catalog.maps_, std::move(map), path);
 }

 for (const auto& path : json_files(catalog.root_ / "scenes")) {
  const auto value = read_localized_json(path, catalog.root_, catalog.locale_);
  require_schema_v1(value, path);
  const auto& camera = value.at("camera");
  const auto& lighting = value.at("lighting");
  const auto& effects = value.at("effects");
  const auto& floor = value.at("floor");
  SceneDefinition scene;
  scene.id = value.at("id").get<std::string>();
  scene.camera = {
    vec3(camera.at("offset_m")),
    camera.at("target_height_m").get<double>(),
    camera.at("fov_degrees").get<double>(),
    camera.at("focus_weights").at("robot").get<double>(),
    camera.at("focus_weights").at("actor").get<double>(),
    camera.at("focus_weights").at("goal").get<double>(),
    {},
    {},
    camera.at("transition_s").get<double>(),
  };
  const auto read_shot = [&](const json& shot) {
   const auto& weights = shot.at("focus_weights");
   return SceneShotDefinition{
     vec3(shot.at("offset_m")),
     shot.at("target_height_m").get<double>(),
     shot.at("fov_degrees").get<double>(),
     weights.at("robot").get<double>(),
     weights.at("actor").get<double>(),
     weights.at("goal").get<double>(),
   };
  };
  scene.camera.event_shot = read_shot(camera.at("event_shot"));
  scene.camera.success_shot = read_shot(camera.at("success_shot"));
  const auto require_focus_sum = [&](double robot, double actor, double goal, const std::string& shot) {
   if (std::abs(robot + actor + goal - 1.0) > 1e-6) {
    throw ContentError(path.string() + ": " + shot + " focus weights must sum to 1");
   }
  };
  require_focus_sum(scene.camera.robot_focus_weight, scene.camera.actor_focus_weight,
                    scene.camera.goal_focus_weight, "camera");
  require_focus_sum(scene.camera.event_shot.robot_focus_weight, scene.camera.event_shot.actor_focus_weight,
                    scene.camera.event_shot.goal_focus_weight, "event shot");
  require_focus_sum(scene.camera.success_shot.robot_focus_weight,
                    scene.camera.success_shot.actor_focus_weight, scene.camera.success_shot.goal_focus_weight,
                    "success shot");
  if (scene.camera.transition_s <= 0.0) {
   throw ContentError(path.string() + ": camera transition must be positive");
  }
  scene.lighting = {
    vec3(lighting.at("direction")),
    rgba(lighting.at("directional_color").get<std::string>()),
    lighting.at("directional_intensity").get<double>(),
    rgba(lighting.at("ambient_color").get<std::string>()),
    rgba(lighting.at("fog_color").get<std::string>()),
    lighting.at("fog_start_m").get<double>(),
    lighting.at("fog_end_m").get<double>(),
    rgba(lighting.at("robot_active_color").get<std::string>()),
    rgba(lighting.at("robot_waiting_color").get<std::string>()),
    rgba(lighting.at("goal_color").get<std::string>()),
    lighting.at("shadow_map_resolution").get<int>(),
    lighting.at("shadow_extent_m").get<double>(),
    lighting.at("shadow_strength").get<double>(),
    lighting.at("exposure").get<double>(),
    lighting.at("bloom_threshold").get<double>(),
    lighting.at("bloom_strength").get<double>(),
    lighting.at("vignette_strength").get<double>(),
  };
  if (scene.lighting.fog_end_m <= scene.lighting.fog_start_m) {
   throw ContentError(path.string() + ": fog_end_m must exceed fog_start_m");
  }
  if (scene.lighting.shadow_map_resolution != 0 && scene.lighting.shadow_map_resolution != 1024 &&
      scene.lighting.shadow_map_resolution != 2048 && scene.lighting.shadow_map_resolution != 4096) {
   throw ContentError(path.string() + ": unsupported shadow_map_resolution");
  }
  if (scene.lighting.shadow_extent_m <= 0.0 || scene.lighting.shadow_strength < 0.0 ||
      scene.lighting.shadow_strength > 1.0 || scene.lighting.exposure < 0.25 ||
      scene.lighting.exposure > 4.0 || scene.lighting.bloom_threshold < 0.0 ||
      scene.lighting.bloom_threshold > 4.0 || scene.lighting.bloom_strength < 0.0 ||
      scene.lighting.bloom_strength > 1.0 || scene.lighting.vignette_strength < 0.0 ||
      scene.lighting.vignette_strength > 1.0) {
   throw ContentError(path.string() + ": invalid lighting quality value");
  }
  scene.effects = {
    effects.at("robot_scale").get<double>(),          effects.at("bob_amplitude_m").get<double>(),
    effects.at("bob_frequency_hz").get<double>(),     effects.at("eye_height_m").get<double>(),
    effects.at("scan_radius_m").get<double>(),        effects.at("perception_link_height_m").get<double>(),
    effects.at("prediction_horizon_s").get<double>(), effects.at("success_ring_radius_m").get<double>(),
    effects.value("trail_duration_s", 1.6),           effects.value("surface_particle_count", 18),
    effects.value("surface_particle_scale", 1.0),     effects.value("risk_indicator_radius_m", 0.82),
  };
  scene.floor = {
    floor.at("albedo_texture").get<std::string>(),   rgba(floor.at("base_color").get<std::string>()),
    rgba(floor.at("tile_color").get<std::string>()), rgba(floor.at("boundary_color").get<std::string>()),
    floor.at("tile_size_m").get<double>(),           floor.at("seam_m").get<double>(),
    floor.at("color_variation").get<int>(),          floor.at("boundary_height_m").get<double>(),
    floor.at("boundary_width_m").get<double>(),
  };
  if (!std::filesystem::is_regular_file(catalog.root_ / scene.floor.albedo_texture)) {
   throw ContentError(path.string() + ": missing floor texture '" + scene.floor.albedo_texture + "'");
  }
  if (scene.floor.seam_m >= scene.floor.tile_size_m) {
   throw ContentError(path.string() + ": floor seam must be smaller than a tile");
  }
  for (const auto& prefab_value : value.at("prefabs")) {
   ScenePrefabDefinition prefab;
   prefab.id = prefab_value.at("id").get<std::string>();
   for (const auto& part_value : prefab_value.at("parts")) {
    ScenePartDefinition part;
    part.primitive = part_value.value("primitive", std::string{});
    part.model_3d = part_value.value("model_3d", std::string{});
    part.position_m = vec3(part_value.at("position_m"));
    part.size_m = vec3(part_value.at("size_m"));
    part.yaw_degrees = part_value.value("yaw_degrees", 0.0);
    part.color = rgba(part_value.at("color").get<std::string>());
    if (!part.model_3d.empty() && !std::filesystem::is_regular_file(catalog.root_ / part.model_3d)) {
     throw ContentError(path.string() + ": missing scene model '" + part.model_3d + "'");
    }
    prefab.parts.push_back(std::move(part));
   }
   if (!scene.prefabs.emplace(prefab.id, std::move(prefab)).second) {
    throw ContentError(path.string() + ": duplicate scene prefab");
   }
  }
  for (const auto& visual_value : value.value("goal_visuals", json::array())) {
   SceneGoalVisualDefinition visual;
   visual.id = visual_value.at("id").get<std::string>();
   visual.state_prefabs = visual_value.at("states").get<std::unordered_map<std::string, std::string>>();
   if (!scene.goal_visuals.emplace(visual.id, std::move(visual)).second) {
    throw ContentError(path.string() + ": duplicate goal visual");
   }
  }
  if (const auto payload_value = value.find("payload_visual"); payload_value != value.end()) {
   const auto& pickup_effect = payload_value->at("pickup_effect");
   const auto& damage_warning = payload_value->at("damage_warning");
   scene.payload_visual = {
     payload_value->at("mount").get<std::string>(),
     payload_value->at("container_prefab").get<std::string>(),
     payload_value->at("item_prefab").get<std::string>(),
     payload_value->at("damaged_item_prefab").get<std::string>(),
     vec3(payload_value->at("item_origin_m")),
     vec3(payload_value->at("item_spacing_m")),
     payload_value->at("grid_columns").get<int>(),
     payload_value->at("grid_rows").get<int>(),
     payload_value->at("items_per_visual").get<int>(),
     payload_value->at("max_visible_items").get<int>(),
     payload_value->at("damaged_below_integrity").get<double>(),
     pickup_effect.at("duration_s").get<double>(),
     pickup_effect.at("particles").get<int>(),
     rgba(pickup_effect.at("color").get<std::string>()),
     rgba(pickup_effect.at("accent_color").get<std::string>()),
     pickup_effect.at("radius_m").get<double>(),
     pickup_effect.at("rise_m").get<double>(),
     damage_warning.at("below_integrity").get<double>(),
     rgba(damage_warning.at("color").get<std::string>()),
     damage_warning.at("height_m").get<double>(),
   };
  }
  std::set<std::string> instance_ids;
  for (const auto& instance_value : value.at("instances")) {
   const auto instance_id = instance_value.at("id").get<std::string>();
   if (!instance_ids.insert(instance_id).second) {
    throw ContentError(path.string() + ": duplicate scene instance '" + instance_id + "'");
   }
 const auto role = instance_value.at("role").get<std::string>();
 if (role != "visual_only") {
 throw ContentError(path.string() + ": scene instance '" + instance_id +
 "' must declare role 'visual_only'");
 }
   scene.instances.push_back({
     instance_id,
     instance_value.at("prefab").get<std::string>(),
 role,
     vec3(instance_value.at("position_m")),
     vec3(instance_value.value("scale", json::array({1.0, 1.0, 1.0}))),
     instance_value.value("yaw_degrees", 0.0),
     instance_value.value("active_in", std::vector<std::string>{}),
   });
   for (const auto& active_id : scene.instances.back().active_in) {
    (void)catalog.level(active_id);
   }
  }
  for (const auto& instance : scene.instances) {
   if (!scene.prefabs.contains(instance.prefab)) {
    throw ContentError(path.string() + ": scene instance references unknown prefab '" + instance.prefab +
                       "'");
   }
  }
  for (const auto& [visual_id, visual] : scene.goal_visuals) {
   for (const auto& [state, prefab] : visual.state_prefabs) {
    if (state != "available" && state != "active" && state != "completed" && state != "abandoned") {
     throw ContentError(path.string() + ": goal visual '" + visual_id + "' uses unknown state '" + state +
                        "'");
    }
    if (!scene.prefabs.contains(prefab)) {
     throw ContentError(path.string() + ": goal visual '" + visual_id + "' references unknown prefab '" +
                        prefab + "'");
    }
   }
  }
  if (!scene.payload_visual.mount.empty()) {
   for (const auto& prefab : {scene.payload_visual.container_prefab, scene.payload_visual.item_prefab,
                              scene.payload_visual.damaged_item_prefab}) {
    if (!scene.prefabs.contains(prefab)) {
     throw ContentError(path.string() + ": payload visual references unknown prefab '" + prefab + "'");
    }
   }
  }
  insert_unique(catalog.scenes_, std::move(scene), path);
 }

 for (const auto& path : json_files(catalog.root_ / "components")) {
  const auto value = read_localized_json(path, catalog.root_, catalog.locale_);
  if (path.filename() == "policies.json") {
   require_schema_v1(value, path);
   for (const auto& item : value.at("policies")) {
    PolicyDefinition policy{
      item.at("id").get<std::string>(),          item.at("title").get<std::string>(),
      item.at("description").get<std::string>(), item.at("style").get<double>(),
      properties(item.at("behavior")),
    };
    insert_unique(catalog.policies_, std::move(policy), path);
   }
   continue;
  }
 require_schema_version(value, path, 3);
  const auto slot = value.at("slot").get<std::string>();
  const auto& profile = value.at("model_profile");
  for (const auto& item : value.at("components")) {
   ComponentDefinition component{
     item.at("id").get<std::string>(),
     slot,
     item.at("title").get<std::string>(),
     item.at("description").get<std::string>(),
     item.at("cost").get<int>(),
     properties(item.at("behavior")),
     visual(item.at("visual")),
     ModelProfileDefinition{
       profile.at("fidelity").get<std::string>(),
       profile.at("basis").get<std::string>(),
       profile.at("source_refs").get<std::vector<std::string>>(),
     },
   };
   require_visual_asset(component.visual, catalog.root_, path);
   insert_unique(catalog.components_, std::move(component), path);
  }
 }

 for (const auto& path : json_files(catalog.root_ / "robots")) {
  const auto value = read_localized_json(path, catalog.root_, catalog.locale_);
 require_schema_version(value, path, 8);
  const auto& drive = value.at("drive");
  const auto& motor = drive.at("motor");
  const auto& steering = drive.at("steering");
  const auto& energy = drive.at("energy");
  const auto& encoder = drive.at("encoder");
  const auto& collision = value.at("collision");
  RobotDefinition robot;
  robot.id = value.at("id").get<std::string>();
  robot.title = value.at("title").get<std::string>();
  const auto& robot_profile = value.at("model_profile");
  robot.model_profile = {
    robot_profile.at("fidelity").get<std::string>(),
    robot_profile.at("basis").get<std::string>(),
    robot_profile.at("source_refs").get<std::vector<std::string>>(),
  };
  robot.drive_type = drive.at("type").get<std::string>();
  robot.drive_label_key = drive.value("label_key", "garage.drive." + robot.drive_type);
  robot.wheel_radius_m = drive.at("wheel_radius_m").get<double>();
  robot.wheel_base_m = drive.at("wheel_base_m").get<double>();
 robot.rated_speed_mps = drive.at("rated_speed_mps").get<double>();
  robot.effective_track_width_m = drive.value("effective_track_width_m", robot.wheel_base_m);
  robot.motor = {
    motor.at("max_wheel_speed_mps").get<double>(),
    motor.at("max_wheel_accel_mps2").get<double>(),
    motor.at("stall_force_N").get<double>(),
    motor.at("free_speed_mps").get<double>(),
    motor.at("pwm_limit").get<double>(),
    motor.at("integral_limit").get<double>(),
    motor.at("wheel_equiv_mass_kg").get<double>(),
    motor.at("controller_bandwidth_radps").get<double>(),
    motor.at("controller_integral_time_s").get<double>(),
    motor.at("controller_derivative_time_s").get<double>(),
 motor.at("static_friction_feedforward_duty").get<double>(),
    motor.at("coulomb_smoothing_speed_mps").get<double>(),
    motor.at("viscous_drag_N_per_mps").get<double>(),
  };
  robot.mass_kg = drive.at("mass_kg").get<double>();
  robot.yaw_inertia_kgm2 = drive.at("yaw_inertia_kgm2").get<double>();
  robot.max_yaw_accel_radps2 = drive.at("max_yaw_accel_radps2").get<double>();
  robot.tracked_max_yaw_rate_rads = drive.value("max_yaw_rate_rads", 1.5);
  robot.track_length_m = drive.value("track_length_m", 0.7);
  robot.track_width_m = drive.value("track_width_m", 0.58);
 if (!(robot.effective_track_width_m > 0.0) || !(robot.rated_speed_mps > 0.0)) {
 throw ContentError(path.string() + ": track width and rated speed must be positive");
  }
  robot.unlock_ability = value.value("unlock_ability", std::string{});
  robot.steering = {
    steering.at("integral_limit").get<double>(),
    steering.value("max_steer_rad", 0.6),
    steering.value("max_steer_rate_radps", 2.5),
    steering.value("servo_response_s", 0.4),
 steering.value("stall_torque_Nm", 8.0),
  };
  robot.energy_base_w = energy.at("base_w").get<double>();
  robot.energy_per_v_w = energy.at("per_v_w").get<double>();
  robot.energy_per_w_w = energy.at("per_w_w").get<double>();
  robot.encoder = {encoder.at("counts_per_revolution").get<int>()};
 if (const auto brake = drive.find("service_brake"); brake != drive.end()) {
 robot.service_brake = {
 brake->at("enabled").get<bool>(),
 brake->at("maximum_torque_Nm").get<double>(),
 brake->at("apply_time_s").get<double>(),
 brake->at("release_time_s").get<double>(),
 brake->at("low_speed_smoothing_radps").get<double>(),
 };
 }
  Vec3 articulated_center_of_mass_in_base;
  double articulated_center_of_mass_height_m = 0.0;
  double articulated_base_link_height_m = 0.0;
  bool has_articulated_center_of_mass = false;
  if (const auto articulation = value.find("articulation"); articulation != value.end()) {
   robot.articulation.root_link = articulation->at("root_link").get<std::string>();
   std::set<std::string> link_ids;
   for (const auto& item : articulation->at("links")) {
    const auto& collision_shape = item.at("collision");
    RigidLinkDefinition link;
    link.id = item.at("id").get<std::string>();
    link.shape = collision_shape.at("shape").get<std::string>();
    link.size_m = collision_shape.contains("size_m") ? vec3(collision_shape.at("size_m")) : Vec3{};
    link.radius_m = collision_shape.value("radius_m", 0.0);
    link.length_m = collision_shape.value("length_m", 0.0);
    link.position_m = vec3(item.at("position_m"));
    link.rotation_deg = vec3(item.at("rotation_deg"));
    link.mass_kg = item.at("mass_kg").get<double>();
    link.inertia_diagonal_kgm2 = vec3(item.at("inertia_diagonal_kgm2"));
    link.friction = item.value("friction", 0.8);
    link.spinning_resistance = item.value("spinning_resistance", 0.0);
    link.contact_time_constant_s = item.value("contact_time_constant_s", 0.0);
    link.contact_damping_ratio = item.value("contact_damping_ratio", 0.0);
 if (link.contact_time_constant_s > 0.0 &&
 (link.contact_time_constant_s < 0.002 || !(link.contact_damping_ratio > 0.0))) {
 throw ContentError(path.string() + ": articulated link '" + link.id +
 "' has invalid MuJoCo contact response");
 }
    link.contact_axle_axis = item.contains("contact_axle_axis") ? vec3(item.at("contact_axle_axis")) : Vec3{};
    if (!link_ids.insert(link.id).second)
     throw ContentError(path.string() + ": duplicate articulated link '" + link.id + "'");
    robot.articulation.links.push_back(std::move(link));
   }
   if (!link_ids.count(robot.articulation.root_link))
    throw ContentError(path.string() + ": articulation root_link is unknown");
   std::set<std::string> joint_ids;
   for (const auto& item : articulation->at("joints")) {
    RigidJointDefinition joint{
      item.at("id").get<std::string>(),     item.at("type").get<std::string>(),
      item.at("parent").get<std::string>(), item.at("child").get<std::string>(),
      vec3(item.at("pivot_in_parent_m")),   vec3(item.at("pivot_in_child_m")),
      vec3(item.at("axis_in_parent")),      vec3(item.at("axis_in_child")),
 item.at("type").get<std::string>() == "revolute",
 item.value("lower_limit_rad", 0.0), item.value("upper_limit_rad", 0.0),
 item.value("damping_Nms_per_rad", 0.0), item.value("friction_loss_Nm", 0.0),
    };
 if (!std::isfinite(joint.damping_Nms_per_rad) || joint.damping_Nms_per_rad < 0.0 ||
 !std::isfinite(joint.friction_loss_Nm) || joint.friction_loss_Nm < 0.0) {
 throw ContentError(path.string() + ": articulated joint '" + joint.id +
 "' damping and friction loss must be finite and non-negative");
 }
    if (!joint_ids.insert(joint.id).second || !link_ids.count(joint.parent) || !link_ids.count(joint.child) ||
        joint.parent == joint.child) {
     throw ContentError(path.string() + ": invalid articulated joint '" + joint.id + "'");
    }
    if (joint.type == "revolute" && !(joint.lower_limit_rad < joint.upper_limit_rad)) {
 throw ContentError(path.string() + ": bounded revolute joint '" + joint.id +
 "' must have lower_limit_rad < upper_limit_rad");
 }
 robot.articulation.joints.push_back(std::move(joint));
   }
   const auto link_for = [&](const std::string& id) -> const RigidLinkDefinition& {
    const auto found = std::find_if(robot.articulation.links.begin(), robot.articulation.links.end(),
                                    [&](const auto& link) { return link.id == id; });
    if (found == robot.articulation.links.end())
     throw ContentError(path.string() + ": unknown articulated link '" + id + "'");
    return *found;
   };
   const auto& root_link = link_for(robot.articulation.root_link);
   if (norm(root_link.rotation_deg) > 1e-9) {
    throw ContentError(path.string() + ": articulated root_link must align with the declared base_link axes");
   }
   if (robot.articulation.joints.size() + 1 != robot.articulation.links.size()) {
    throw ContentError(path.string() + ": articulation must be a single rooted tree");
   }
   std::set<std::string> child_links;
   std::map<std::string, std::vector<std::string>> children;
   for (const auto& joint : robot.articulation.joints) {
    if (joint.child == robot.articulation.root_link || !child_links.insert(joint.child).second) {
     throw ContentError(path.string() + ": every non-root articulated link must have exactly one parent");
    }
    children[joint.parent].push_back(joint.child);
    const auto& parent = link_for(joint.parent);
    const auto& child = link_for(joint.child);
    const Vec3 parent_pivot =
      add(parent.position_m, rotate_rpy_degrees(joint.pivot_in_parent_m, parent.rotation_deg));
    const Vec3 child_pivot =
      add(child.position_m, rotate_rpy_degrees(joint.pivot_in_child_m, child.rotation_deg));
    if (norm(subtract(parent_pivot, child_pivot)) > 1e-6) {
     throw ContentError(path.string() + ": articulated joint '" + joint.id +
                        "' has separated parent and child pivots at rest");
    }
    const Vec3 parent_axis = rotate_rpy_degrees(joint.axis_in_parent, parent.rotation_deg);
    const Vec3 child_axis = rotate_rpy_degrees(joint.axis_in_child, child.rotation_deg);
    const double parent_axis_norm = norm(parent_axis);
    const double child_axis_norm = norm(child_axis);
    if (parent_axis_norm < 1e-9 || child_axis_norm < 1e-9 ||
        std::abs(std::abs(dot(parent_axis, child_axis) / (parent_axis_norm * child_axis_norm)) - 1.0) >
          1e-6) {
     throw ContentError(path.string() + ": articulated joint '" + joint.id + "' axes do not align at rest");
    }
   }
   std::set<std::string> reachable;
   std::vector<std::string> pending{robot.articulation.root_link};
   while (!pending.empty()) {
    const auto current = pending.back();
    pending.pop_back();
    if (!reachable.insert(current).second)
     throw ContentError(path.string() + ": articulated joint graph contains a cycle");
    const auto found = children.find(current);
    if (found != children.end())
     pending.insert(pending.end(), found->second.begin(), found->second.end());
   }
   if (reachable.size() != robot.articulation.links.size())
    throw ContentError(path.string() + ": articulated joint graph is disconnected");

   double articulated_mass_kg = 0.0;
   Vec3 first_moment;
   for (const auto& link : robot.articulation.links) {
    const auto inertia = link.inertia_diagonal_kgm2;
    if (inertia.x > inertia.y + inertia.z + 1e-9 || inertia.y > inertia.x + inertia.z + 1e-9 ||
        inertia.z > inertia.x + inertia.y + 1e-9) {
     throw ContentError(path.string() + ": articulated link '" + link.id +
                        "' has a non-physical inertia tensor");
    }
    articulated_mass_kg += link.mass_kg;
    first_moment.x += link.mass_kg * link.position_m.x;
    first_moment.y += link.mass_kg * link.position_m.y;
    first_moment.z += link.mass_kg * link.position_m.z;
   }
   if (std::abs(articulated_mass_kg - robot.mass_kg) > 1e-6 * std::max(1.0, robot.mass_kg)) {
    throw ContentError(path.string() + ": articulated link masses must sum to drive.mass_kg");
   }
   const Vec3 assembly_center{first_moment.x / articulated_mass_kg, first_moment.y / articulated_mass_kg,
                              first_moment.z / articulated_mass_kg};
   articulated_center_of_mass_in_base = subtract(assembly_center, root_link.position_m);
   articulated_center_of_mass_height_m = assembly_center.z;
   articulated_base_link_height_m = root_link.position_m.z;
   has_articulated_center_of_mass = true;
   double assembly_yaw_inertia = 0.0;
   for (const auto& link : robot.articulation.links) {
    const double dx = link.position_m.x - assembly_center.x;
    const double dy = link.position_m.y - assembly_center.y;
    assembly_yaw_inertia += inertia_about_model_z(link) + link.mass_kg * (dx * dx + dy * dy);
   }
   std::set<std::string> actuator_channels;
   for (const auto& item : articulation->at("actuators")) {
    JointActuatorDefinition actuator{
      item.at("channel").get<std::string>(), item.at("joint").get<std::string>(),
      item.at("effort_limit_Nm").get<double>(), item.value("armature_inertia_kgm2", 0.0)};
    if (!actuator_channels.insert(actuator.channel).second || !joint_ids.count(actuator.joint))
     throw ContentError(path.string() + ": invalid articulated actuator channel '" + actuator.channel + "'");
    if (actuator.armature_inertia_kgm2 > 0.0) {
     const auto driven_joint =
       std::find_if(robot.articulation.joints.begin(), robot.articulation.joints.end(),
                    [&](const auto& joint) { return joint.id == actuator.joint; });
     const auto axis = driven_joint->axis_in_child;
     const double axis_norm = norm(axis);
     const int principal_components = static_cast<int>(std::abs(axis.x) > 1e-9) +
                                      static_cast<int>(std::abs(axis.y) > 1e-9) +
                                      static_cast<int>(std::abs(axis.z) > 1e-9);
     if (std::abs(axis_norm - 1.0) > 1e-6 || principal_components != 1) {
      throw ContentError(path.string() + ": actuator armature for '" + actuator.channel +
                         "' requires a unit principal joint axis");
     }
     const auto& child = link_for(driven_joint->child);
     RigidLinkDefinition armature;
     armature.rotation_deg = child.rotation_deg;
     const double transverse = 0.5 * actuator.armature_inertia_kgm2;
     armature.inertia_diagonal_kgm2 = {
       std::abs(axis.x) > 0.5 ? actuator.armature_inertia_kgm2 : transverse,
       std::abs(axis.y) > 0.5 ? actuator.armature_inertia_kgm2 : transverse,
       std::abs(axis.z) > 0.5 ? actuator.armature_inertia_kgm2 : transverse,
     };
     assembly_yaw_inertia += inertia_about_model_z(armature);
    }
    robot.articulation.actuators.push_back(std::move(actuator));
   }
   if (std::abs(assembly_yaw_inertia - robot.yaw_inertia_kgm2) >
       0.001 * std::max(assembly_yaw_inertia, robot.yaw_inertia_kgm2)) {
    throw ContentError(path.string() +
                       ": drive.yaw_inertia_kgm2 must match the articulated assembly and armatures");
   }
  }
 if (const auto manipulator = value.find("manipulator"); manipulator != value.end()) {
 ManipulatorDefinition definition;
 definition.type = manipulator->at("type").get<std::string>();
 definition.channels = manipulator->at("channels").get<std::array<std::string, 4>>();
 definition.shoulder_height_m = manipulator->at("shoulder_height_m").get<double>();
 definition.upper_arm_length_m = manipulator->at("upper_arm_length_m").get<double>();
 definition.forearm_length_m = manipulator->at("forearm_length_m").get<double>();
 definition.tool_length_m = manipulator->at("tool_length_m").get<double>();
 definition.lower_rad = manipulator->at("lower_rad").get<std::array<double, 4>>();
 definition.upper_rad = manipulator->at("upper_rad").get<std::array<double, 4>>();
 definition.home_rad = manipulator->at("home_rad").get<std::array<double, 4>>();
 definition.kp = manipulator->at("kp").get<std::array<double, 4>>();
 definition.kd = manipulator->at("kd").get<std::array<double, 4>>();
 definition.maximum_effort_Nm =
 manipulator->at("maximum_effort_Nm").get<std::array<double, 4>>();
 definition.maximum_velocity_radps =
 manipulator->at("maximum_velocity_radps").get<std::array<double, 4>>();
 definition.gravity_moment_Nm =
 manipulator->at("gravity_moment_Nm").get<std::array<double, 3>>();
 definition.end_effector_frame = manipulator->at("end_effector_frame").get<std::string>();
 if (definition.type != "yaw_planar_3r")
 throw ContentError(path.string() + ": unsupported manipulator type '" + definition.type + "'");
 std::set<std::string> unique_channels;
 for (std::size_t index = 0; index < definition.channels.size(); ++index) {
 const auto actuator = std::find_if(robot.articulation.actuators.begin(),
 robot.articulation.actuators.end(), [&](const auto& candidate) {
 return candidate.channel == definition.channels[index];
 });
 if (!unique_channels.insert(definition.channels[index]).second ||
 actuator == robot.articulation.actuators.end()) {
 throw ContentError(path.string() + ": manipulator references an invalid actuator channel");
 }
 if (!(definition.lower_rad[index] < definition.home_rad[index] &&
 definition.home_rad[index] < definition.upper_rad[index]) ||
 !(definition.kp[index] > 0.0) || definition.kd[index] < 0.0 ||
 !(definition.maximum_effort_Nm[index] > 0.0) ||
 !(definition.maximum_velocity_radps[index] > 0.0)) {
 throw ContentError(path.string() + ": manipulator has invalid joint parameters");
 }
 }
 for (const double moment : definition.gravity_moment_Nm) {
 if (!(std::isfinite(moment) && moment >= 0.0))
 throw ContentError(path.string() + ": manipulator has invalid gravity moments");
 }
 if (!(definition.shoulder_height_m > 0.0 && definition.upper_arm_length_m > 0.0 &&
 definition.forearm_length_m > 0.0 && definition.tool_length_m >= 0.0)) {
 throw ContentError(path.string() + ": manipulator has invalid kinematic dimensions");
 }
 if (definition.end_effector_frame.empty())
 throw ContentError(path.string() + ": manipulator end_effector_frame is empty");
 robot.manipulator = std::move(definition);
 }
  robot.collision_radius_m = collision.at("planning_radius_m").get<double>();
  const auto& rigid_body = collision.at("rigid_body");
  robot.rigid_body_shape = rigid_body.at("shape").get<std::string>();
  robot.rigid_body_size_m = vec3(rigid_body.at("size_m"));
  robot.collision_height_m = robot.rigid_body_size_m.z;
  robot.base_link_height_m = robot.collision_height_m * 0.5;
  robot.center_of_mass_height_m = robot.collision_height_m * 0.5;
 robot.center_of_mass_in_base_m = {0.0, 0.0, robot.center_of_mass_height_m};
  if (has_articulated_center_of_mass) {
   robot.base_link_height_m = articulated_base_link_height_m;
   robot.center_of_mass_height_m = articulated_center_of_mass_height_m;
 robot.center_of_mass_in_base_m = articulated_center_of_mass_in_base;
  }
  if (robot.rigid_body_shape != "box" || robot.rigid_body_size_m.x <= 0.0 ||
      robot.rigid_body_size_m.y <= 0.0 || robot.rigid_body_size_m.z <= 0.0) {
   throw ContentError(path.string() + ": rigid_body must be a positive box");
  }
  if (std::hypot(robot.rigid_body_size_m.x * 0.5, robot.rigid_body_size_m.y * 0.5) >
      robot.collision_radius_m + 1e-9) {
   throw ContentError(path.string() + ": planning_radius_m must contain the rigid body footprint");
  }
  std::set<std::string> mount_ids;
  std::set<std::string> frame_ids;
  for (const auto& item : value.at("mounts")) {
   MountDefinition mount{
     item.at("id").get<std::string>(),    vec3(item.at("position_m")),
     vec3(item.at("rotation_deg")),       item.at("motion").get<std::string>(),
     item.value("rolling_radius_m", 0.0), item.value("frame_id", item.at("id").get<std::string>() + "_link"),
   };
   if (mount.motion != "fixed" && mount.motion != "wheel" && mount.motion != "track" &&
       mount.motion != "caster") {
    throw ContentError(path.string() + ": unknown mount motion '" + mount.motion + "'");
   }
   if (mount.motion == "wheel" && mount.rolling_radius_m <= 0.0) {
    throw ContentError(path.string() + ": rolling mount '" + mount.id + "' requires rolling_radius_m > 0");
   }
   if (!mount_ids.insert(mount.id).second) {
    throw ContentError(path.string() + ": duplicate robot mount '" + mount.id + "'");
   }
   if (!frame_ids.insert(mount.frame_id).second) {
    throw ContentError(path.string() + ": duplicate robot frame '" + mount.frame_id + "'");
   }
   if (mount.frame_id == "map" || mount.frame_id == "odom" || mount.frame_id == "base_footprint" ||
       mount.frame_id == "base_link") {
    throw ContentError(path.string() + ": mount frame uses reserved frame '" + mount.frame_id + "'");
   }
   robot.mounts.push_back(std::move(mount));
  }
 if (robot.manipulator && !frame_ids.contains(robot.manipulator->end_effector_frame)) {
 throw ContentError(path.string() + ": manipulator end_effector_frame has no matching mount");
 }
  if (!robot.articulation.links.empty()) {
   const auto& root =
     *std::find_if(robot.articulation.links.begin(), robot.articulation.links.end(),
                   [&](const auto& link) { return link.id == robot.articulation.root_link; });
   for (const auto& link : robot.articulation.links) {
    if (link.id == robot.articulation.root_link)
     continue;
    const auto mount = std::find_if(robot.mounts.begin(), robot.mounts.end(),
                                    [&](const auto& candidate) { return candidate.frame_id == link.id; });
    if (mount == robot.mounts.end()) {
     throw ContentError(path.string() + ": articulated link '" + link.id + "' has no matching mount/frame");
    }
    const Vec3 expected_position = subtract(link.position_m, root.position_m);
    if (norm(subtract(mount->position_m, expected_position)) > 1e-6 ||
        norm(subtract(mount->rotation_deg, link.rotation_deg)) > 1e-6) {
     throw ContentError(path.string() + ": mount/frame for articulated link '" + link.id +
                        "' does not match its rest transform");
    }
 if (mount->motion == "wheel" &&
 (link.shape == "cylinder" || link.shape == "ellipsoid" ||
 link.shape == "crowned_wheel") &&
        std::abs(mount->rolling_radius_m - link.radius_m) > 1e-9) {
     throw ContentError(path.string() + ": rolling radius for articulated link '" + link.id +
                        "' does not match its collision radius");
    }
   }
  }
  if (const auto supports = value.find("passive_supports"); supports != value.end()) {
   for (const auto& item : *supports) {
    PassiveSupportDefinition support{
      item.at("steer_mount").get<std::string>(),
      item.at("roll_mount").get<std::string>(),
    };
    const auto steer = std::find_if(robot.mounts.begin(), robot.mounts.end(),
                                    [&](const auto& mount) { return mount.id == support.steer_mount; });
    const auto roll = std::find_if(robot.mounts.begin(), robot.mounts.end(),
                                   [&](const auto& mount) { return mount.id == support.roll_mount; });
    if (steer == robot.mounts.end() || roll == robot.mounts.end()) {
     throw ContentError(path.string() + ": passive support references an unknown mount");
    }
    if (support.steer_mount == support.roll_mount || steer->motion != "caster" || roll->motion != "wheel" ||
        roll->rolling_radius_m <= 0.0) {
     throw ContentError(path.string() +
                        ": passive support requires distinct caster steer and rolling wheel mounts");
    }
    robot.passive_supports.push_back(std::move(support));
   }
  }
  for (const auto& item : value.at("base_parts")) {
   RobotPartDefinition part{
     item.at("id").get<std::string>(),
     item.at("mount").get<std::string>(),
     visual(item.at("visual")),
   };
   if (!mount_ids.count(part.mount)) {
    throw ContentError(path.string() + ": base part references unknown mount '" + part.mount + "'");
   }
   require_visual_asset(part.visual, catalog.root_, path);
   robot.base_parts.push_back(std::move(part));
  }
  std::set<std::string> slot_ids;
  for (const auto& item : value.at("slots")) {
   RobotSlotDefinition slot{
     item.at("slot").get<std::string>(),
     item.at("mounts").get<std::vector<std::string>>(),
   };
   if (!slot_ids.insert(slot.slot).second) {
    throw ContentError(path.string() + ": duplicate robot slot '" + slot.slot + "'");
   }
   for (const auto& mount : slot.mounts) {
    if (!mount_ids.count(mount)) {
     throw ContentError(path.string() + ": slot references unknown mount '" + mount + "'");
    }
   }
   robot.slots.push_back(std::move(slot));
  }
 for (const auto* required : {"sensor", "computer"}) {
   if (!slot_ids.count(required)) {
    throw ContentError(path.string() + ": missing robot slot '" + required + "'");
   }
  }
 const bool has_tires = slot_ids.count("tires") != 0;
 const bool has_tracks = slot_ids.count("tracks") != 0;
 if (has_tires == has_tracks) {
 throw ContentError(path.string() +
 ": robot must define exactly one ground-contact slot: 'tires' or 'tracks'");
 }
  if (robot.drive_type == "differential") {
   std::vector<sim::Vec2> contacts;
 const auto traction = std::find_if(robot.slots.begin(), robot.slots.end(),
 [](const auto& slot) { return slot.slot == "tires" || slot.slot == "tracks"; });
 if (traction != robot.slots.end()) {
 for (const auto& mount_id : traction->mounts) {
     const auto mount = std::find_if(robot.mounts.begin(), robot.mounts.end(),
                                     [&](const auto& candidate) { return candidate.id == mount_id; });
     if (mount != robot.mounts.end())
      contacts.push_back({mount->position_m.x, mount->position_m.y});
    }
   }
   for (const auto& support : robot.passive_supports) {
    const auto mount = std::find_if(robot.mounts.begin(), robot.mounts.end(), [&](const auto& candidate) {
     return candidate.id == support.roll_mount;
    });
    if (mount != robot.mounts.end())
     contacts.push_back({mount->position_m.x, mount->position_m.y});
   }
   constexpr double kMinimumSupportMarginM = 0.02;
   if (has_articulated_center_of_mass) {
    for (auto& contact : contacts) {
     contact.x -= articulated_center_of_mass_in_base.x;
     contact.y -= articulated_center_of_mass_in_base.y;
    }
   }
   if (origin_support_margin(contacts) < kMinimumSupportMarginM) {
    throw ContentError(
      path.string() +
      ": differential support polygon must contain the centre of mass with at least 0.02 m margin");
   }
  }
  insert_unique(catalog.robots_, std::move(robot), path);
 }

 for (const auto& path : json_files(catalog.root_ / "presentation")) {
  const auto value = read_localized_json(path, catalog.root_, catalog.locale_);
  require_schema_v1(value, path);
  const auto& garage = value.at("garage");
  const auto& prediction = value.at("prediction");
  const auto& running = value.at("running");
  const auto& debrief = value.at("debrief");
  PresentationDefinition presentation;
  presentation.id = value.at("id").get<std::string>();
  presentation.is_default = value.at("is_default").get<bool>();
  presentation.level = value.at("level").get<std::string>();
  presentation.robot_id = value.value("robot", std::string{});
  presentation.chassis_options = value.value("chassis_options", std::vector<std::string>{});
  presentation.task_policies = value.value("task_policies", std::vector<std::string>{});
  presentation.garage_heading = garage.at("heading").get<std::string>();
  presentation.garage_brief = garage.at("brief").get<std::string>();
  for (const auto& item : garage.at("component_columns")) {
   presentation.component_columns.push_back({
     item.at("slot").get<std::string>(),
     item.at("title").get<std::string>(),
     item.at("options").get<std::vector<std::string>>(),
     item.value("player_choice", true),
   });
  }
  presentation.policy_title = garage.at("policy_title").get<std::string>();
  presentation.policies = garage.at("policies").get<std::vector<std::string>>();
  presentation.default_build = build(garage.at("default_build"));
  presentation.recommended_build = build(garage.at("recommended_build"));
  presentation.prediction_heading = prediction.at("heading").get<std::string>();
  presentation.prediction_hint = prediction.at("hint").get<std::string>();
  for (const auto& item : prediction.at("choices")) {
   presentation.prediction_choices.push_back({
     item.at("id").get<std::string>(),
     item.at("text").get<std::string>(),
     item.at("feedback").get<std::string>(),
 item.value("goal_id", std::string{}),
   });
  }
  presentation.running_heading = running.at("heading").get<std::string>();
  presentation.normal_line = running.at("normal_line").get<std::string>();
  presentation.detected_line = running.at("detected_line").get<std::string>();
 presentation.evidence_fields = running.value("evidence_fields", std::vector<std::string>{});
 for (const auto& field : presentation.evidence_fields) {
 static const std::unordered_set<std::string> allowed{
 "speed", "braking", "steering", "turn_radius", "grade", "slip", "lidar", "manipulator"};
 if (!allowed.count(field))
 throw ContentError(path.string() + ": unknown running evidence field '" + field + "'");
 }
  presentation.goal_lines = running.value("goal_lines", std::vector<std::string>{});
  presentation.success_title = debrief.at("success_title").get<std::string>();
  presentation.success_summary = debrief.at("success_summary").get<std::string>();
  presentation.failure_summary = debrief.at("failure_summary").get<std::string>();
  presentation.evidence_heading = debrief.at("evidence_heading").get<std::string>();
  presentation.evidence_caption = debrief.at("evidence_caption").get<std::string>();
  presentation.knowledge_title = debrief.at("knowledge_title").get<std::string>();
  presentation.failure_titles =
    debrief.at("failure_titles").get<std::unordered_map<std::string, std::string>>();
  if (presentation.is_default) {
   if (!catalog.default_presentation_id_.empty()) {
    throw ContentError("more than one default presentation");
   }
   catalog.default_presentation_id_ = presentation.id;
  }
  insert_unique(catalog.presentations_, std::move(presentation), path);
 }

 std::set<std::string> story_ids;
 for (const auto& path : json_files(catalog.root_ / "story")) {
  const auto value = read_localized_json(path, catalog.root_, catalog.locale_);
  require_schema_v1(value, path);
  for (const auto& item : value.at("beats")) {
   StoryBeatDefinition beat{
     item.at("id").get<std::string>(),   item.at("level").get<std::string>(),
     item.at("when").get<std::string>(), item.at("speaker").get<std::string>(),
     item.at("text").get<std::string>(),
   };
   if (!story_ids.insert(beat.id).second) {
    throw ContentError(path.string() + ": duplicate story beat '" + beat.id + "'");
   }
   (void)catalog.level(beat.level);
   catalog.story_beats_.push_back(std::move(beat));
  }
 }

 {
  const auto chapters_path = catalog.root_ / "chapters.json";
  const auto chapters_value = read_localized_json(chapters_path, catalog.root_, catalog.locale_);
  require_schema_v1(chapters_value, chapters_path);
  std::set<int> chapter_ids;
  for (const auto& item : chapters_value.at("chapters")) {
   ChapterDefinition chapter;
   chapter.id = item.at("id").get<int>();
   chapter.title = item.at("title").get<std::string>();
   chapter.blurb = item.at("blurb").get<std::string>();
   if (!chapter_ids.insert(chapter.id).second) {
    throw ContentError(chapters_path.string() + ": duplicate chapter id");
   }
   catalog.chapters_.push_back(std::move(chapter));
  }
  if (catalog.chapters_.empty()) {
   throw ContentError("at least one chapter is required");
  }
 }

 if (catalog.default_presentation_id_.empty()) {
  throw ContentError("no default presentation found");
 }

 (void)catalog.robot(catalog.campaign_.companion_robot);

 std::vector<std::string> experience_ids = catalog.campaign_.levels;
 experience_ids.insert(experience_ids.end(), catalog.campaign_.playgrounds.begin(),
                       catalog.campaign_.playgrounds.end());
 std::set<std::string> unique_experiences(experience_ids.begin(), experience_ids.end());
 if (unique_experiences.size() != experience_ids.size()) {
  throw ContentError("guided levels and playgrounds must be disjoint and unique");
 }
 std::set<int> orders;
 for (const auto& id : experience_ids) {
  const auto& level = catalog.level(id);
  (void)catalog.task_policy(level.objective.task_policy);
  if (!orders.insert(level.order).second) {
   throw ContentError("duplicate level order");
  }
  const bool known_chapter =
    std::any_of(catalog.chapters_.begin(), catalog.chapters_.end(),
                [&](const ChapterDefinition& chapter) { return chapter.id == level.chapter; });
  if (!known_chapter) {
   throw ContentError("level '" + id + "' references an unknown chapter");
  }
  (void)catalog.map(level.map);
  const auto& map = catalog.map(level.map);
 const auto& scene = catalog.scene(map.scene_3d);
 for (const auto& object : map.world_objects) {
 if (!scene.prefabs.contains(object.prefab)) {
 throw ContentError("map '" + map.id + "' world object '" + object.id +
 "' references unknown prefab '" + object.prefab + "'");
 }
 }
  if (!map.markers.count(level.objective.spawn) || !map.markers.count(level.objective.goal) ||
      (!level.objective.via.empty() && !map.markers.count(level.objective.via))) {
   throw ContentError("level '" + id + "' references an unknown map marker");
  }
 // Goal markers describe intended service/arrival poses. They may sit beyond
 // a controlled crossing, but their centres must lie outside the physical
 // swept volume of every fatal actor. Validate the largest selectable
 // chassis because a content selection must not turn an otherwise valid task
 // into an unavoidable overlap. Score clearance is deliberately absent: a
 // debrief target may be missed without making task geometry invalid.
 const auto& presentation = catalog.presentation_for_level(id);
 double largest_robot_radius_m = 0.0;
 const auto include_robot_radius = [&](const std::string& robot_id) {
 largest_robot_radius_m =
 std::max(largest_robot_radius_m, catalog.robot(robot_id).collision_radius_m);
 };
 include_robot_radius(presentation.robot_id.empty() ? catalog.campaign_.companion_robot
 : presentation.robot_id);
 for (const auto& chassis_id : presentation.chassis_options)
 include_robot_radius(chassis_id);
 const double service_envelope_m = largest_robot_radius_m;
 for (const auto& goal : level.objective.goals) {
 const auto marker = map.markers.find(goal.marker);
 if (marker == map.markers.end()) {
 throw ContentError("level '" + id + "' references an unknown goal marker '" +
 goal.marker + "'");
 }
 const sim::Vec2 service_point{marker->second.x, marker->second.y};
 for (const auto& actor : map.actors) {
 if (actor.collision_social || !active_in_level(actor.active_in, id))
 continue;
 if (point_in_actor_swept_volume(service_point, actor, service_envelope_m)) {
 throw ContentError("level '" + id + "' goal marker '" + goal.marker +
 "' lies inside fatal actor '" + actor.id + "' swept safety envelope");
 }
 }
 }
 for (const auto& maneuver : level.maneuvers) {
 if (!map.markers.count(maneuver.spawn) || !map.markers.count(maneuver.goal)) {
 throw ContentError("level '" + id + "' maneuver '" + maneuver.id +
 "' references an unknown map marker");
 }
 }
 }
 for (const auto& [id, piece] : catalog.construction_pieces_) {
 (void)id;
 if (catalog.surface_materials_.find(piece.surface_material) == catalog.surface_materials_.end()) {
 throw ContentError("construction piece '" + piece.id + "' references unknown surface material '" +
 piece.surface_material + "'");
 }
 }
 if (experience_ids.size() != catalog.levels_.size()) {
  throw ContentError("guided levels and playgrounds must list every level exactly once");
 }
 std::set<std::string> collection_ids;
 std::set<std::string> collected_levels;
 for (const auto& collection : catalog.campaign_.collections) {
 if (!collection_ids.insert(collection.id).second) {
 throw ContentError("duplicate experience collection '" + collection.id + "'");
 }
 for (const auto& level_id : collection.levels) {
 (void)catalog.level(level_id);
 if (!collected_levels.insert(level_id).second) {
 throw ContentError("experience collections repeat level '" + level_id + "'");
 }
 }
 }
 if (collected_levels != unique_experiences) {
 throw ContentError("experience collections must present every level exactly once");
 }
 for (const auto& [id, presentation] : catalog.presentations_) {
 const auto& level = catalog.level(presentation.level);
  if (!presentation.robot_id.empty()) {
   (void)catalog.robot(presentation.robot_id);
  }
  for (const auto& chassis : presentation.chassis_options) {
   (void)catalog.robot(chassis);
  }
  for (const auto& policy : presentation.task_policies) {
   (void)catalog.task_policy(policy);
  }
 for (const auto& choice : presentation.prediction_choices) {
 if (choice.goal_id.empty())
 continue;
 if (level.objective.goal_order != "player") {
 throw ContentError("presentation '" + id + "' binds prediction choice '" + choice.id +
 "' to a goal in a mission that is not player ordered");
 }
 const bool known_goal = std::any_of(
 level.objective.goals.begin(), level.objective.goals.end(), [&](const GoalStep& goal) {
 return (goal.id.empty() ? goal.marker : goal.id) == choice.goal_id;
 });
 if (!known_goal) {
 throw ContentError("presentation '" + id + "' prediction choice '" + choice.id +
 "' references unknown goal '" + choice.goal_id + "'");
 }
 }
  catalog.validate_build(presentation.level, presentation.default_build);
  catalog.validate_build(presentation.level, presentation.recommended_build);
 }
 if (catalog.presentations_.size() != catalog.levels_.size()) {
  throw ContentError("every level must have exactly one presentation");
 }
 for (const auto& level_id : catalog.campaign_.levels) {
  (void)catalog.presentation_for_level(level_id);
 }
 for (const auto& level_id : catalog.campaign_.playgrounds) {
  (void)catalog.presentation_for_level(level_id);
 }
 return catalog;
}
const LevelDefinition& ContentCatalog::level(const std::string& id) const {
 const auto found = levels_.find(id);
 if (found == levels_.end()) {
  throw ContentError("unknown level '" + id + "'");
 }
 return found->second;
}

const MapDefinition& ContentCatalog::map(const std::string& id) const {
 const auto found = maps_.find(id);
 if (found == maps_.end()) {
  throw ContentError("unknown map '" + id + "'");
 }
 return found->second;
}

const SceneDefinition& ContentCatalog::scene(const std::string& id) const {
 const auto found = scenes_.find(id);
 if (found == scenes_.end()) {
  throw ContentError("unknown scene '" + id + "'");
 }
 return found->second;
}

const RobotDefinition& ContentCatalog::robot(const std::string& id) const {
 const auto found = robots_.find(id);
 if (found == robots_.end()) {
  throw ContentError("unknown robot '" + id + "'");
 }
 return found->second;
}

const ComponentDefinition& ContentCatalog::component(const std::string& id) const {
 const auto found = components_.find(id);
 if (found == components_.end()) {
  throw ContentError("unknown component '" + id + "'");
 }
 return found->second;
}

const ConstructionPieceDefinition& ContentCatalog::construction_piece(const std::string& id) const {
 const auto found = construction_pieces_.find(id);
 if (found == construction_pieces_.end()) {
 throw ContentError("unknown construction piece '" + id + "'");
 }
 return found->second;
}

const AbilityDefinition& ContentCatalog::ability(const std::string& id) const {
 const auto found = abilities_.find(id);
 if (found == abilities_.end()) {
  throw ContentError("unknown ability '" + id + "'");
 }
 return found->second;
}

const std::string& ContentCatalog::tr(const std::string& key) const {
 const auto found = strings_.find(key);
 if (found == strings_.end()) {
  throw ContentError("unknown UI string '" + key + "'");
 }
 return found->second;
}

std::string ContentCatalog::trf(const std::string& key, const std::vector<std::string>& args) const {
 std::string out = tr(key);
 for (std::size_t i = 0; i < args.size(); ++i) {
  const std::string slot = "{" + std::to_string(i) + "}";
  std::size_t at = 0;
  while ((at = out.find(slot, at)) != std::string::npos) {
   out.replace(at, slot.size(), args[i]);
   at += args[i].size();
  }
 }
 return out;
}

const PolicyDefinition& ContentCatalog::policy(const std::string& id) const {
 const auto found = policies_.find(id);
 if (found == policies_.end()) {
  throw ContentError("unknown policy '" + id + "'");
 }
 return found->second;
}

const TaskPolicyDefinition& ContentCatalog::task_policy(const std::string& id) const {
 const auto found = task_policies_.find(id);
 if (found == task_policies_.end()) {
  throw ContentError("unknown task policy '" + id + "'");
 }
 return found->second;
}

const PresentationDefinition& ContentCatalog::presentation(const std::string& id) const {
 const auto found = presentations_.find(id);
 if (found == presentations_.end()) {
  throw ContentError("unknown presentation '" + id + "'");
 }
 return found->second;
}

const SurfaceMaterialDefinition& ContentCatalog::surface_material(const std::string& id) const {
 const auto found = surface_materials_.find(id);
 if (found == surface_materials_.end()) {
  throw ContentError("unknown surface material '" + id + "'");
 }
 return found->second;
}

const PresentationDefinition& ContentCatalog::presentation_for_level(const std::string& level_id) const {
 const PresentationDefinition* result = nullptr;
 for (const auto& [id, value] : presentations_) {
  (void)id;
  if (value.level != level_id)
   continue;
  if (result) {
   throw ContentError("more than one presentation for level '" + level_id + "'");
  }
  result = &value;
 }
 if (!result) {
  throw ContentError("no presentation for level '" + level_id + "'");
 }
 return *result;
}

const PresentationDefinition& ContentCatalog::default_presentation() const {
 return presentation(default_presentation_id_);
}

const StoryBeatDefinition* ContentCatalog::story_beat(const std::string& level_id,
                                                      const std::string& when) const {
 const StoryBeatDefinition* result = nullptr;
 for (const auto& beat : story_beats_) {
  if (beat.level != level_id || beat.when != when)
   continue;
  if (result) {
   throw ContentError("more than one '" + when + "' story beat for level '" + level_id + "'");
  }
  result = &beat;
 }
 return result;
}

int ContentCatalog::build_cost(const BuildSelection& build) const {
 int result = 0;
 for (const auto& [slot, id] : build.parts) {
  (void)slot;
  if (!id.empty())
   result += component(id).cost;
 }
 return result;
}

void ContentCatalog::validate_build(const std::string& level_id, const BuildSelection& build,
                                    const std::vector<std::string>& owned_abilities) const {
 const auto& level_value = level(level_id);
 if (!build.chassis_robot.empty()) {
  // Player chassis choice: opt-in per level (absent = fixed, today's path),
  // robot must exist, unlock gate enforced against caller-held abilities.
  const auto& options = presentation_for_level(level_id).chassis_options;
  if (std::find(options.begin(), options.end(), build.chassis_robot) == options.end()) {
   throw ContentError("chassis '" + build.chassis_robot + "' is not offered for this level");
  }
  const auto& unlock = robot(build.chassis_robot).unlock_ability;
 if (level_value.loadout_access != "open" && !unlock.empty() &&
      std::find(owned_abilities.begin(), owned_abilities.end(), unlock) == owned_abilities.end()) {
   throw ContentError("chassis '" + build.chassis_robot + "' is locked behind '" + unlock + "'");
  }
 }
 for (const auto& slot : level_value.required_slots) {
  if (build.part(slot).empty()) {
   throw ContentError("required slot '" + slot + "' is empty");
  }
 }
 for (const auto& [slot, id] : build.parts) {
  if (id.empty()) {
   continue;
  }
  if (!contains(level_value.required_slots, slot)) {
   throw ContentError("slot '" + slot + "' is not enabled for this level");
  }
  const auto& selected = component(id);
  if (selected.slot != slot) {
   throw ContentError("component '" + id + "' does not belong in slot '" + slot + "'");
  }
 }
 if (!(build.style >= 0.0 && build.style <= 1.0)) {
  throw ContentError("driving style must be within [0, 1]");
 }
 if (!build.task_policy.empty() && level_value.objective.goal_order != "autonomous") {
  throw ContentError("task policy requires an autonomous goal set");
 }
 if (!build.task_policy.empty()) {
  (void)task_policy(build.task_policy);
  const auto& offered = presentation_for_level(level_id).task_policies;
  if (!contains(offered, build.task_policy)) {
   throw ContentError("task policy '" + build.task_policy + "' is not offered for this level");
  }
 }
 if (build_cost(build) > level_value.budget) {
  throw ContentError("selected build exceeds level budget");
 }
}

Properties ContentCatalog::style_behavior(double style) const {
 std::vector<const PolicyDefinition*> anchors;
 for (const auto& [id, policy] : policies_) {
  (void)id;
  anchors.push_back(&policy);
 }
 if (anchors.empty()) {
  throw ContentError("no driving style anchors");
 }
 std::sort(anchors.begin(), anchors.end(), [](const PolicyDefinition* left, const PolicyDefinition* right) {
  return left->style_anchor < right->style_anchor;
 });
 const double clamped = std::clamp(style, anchors.front()->style_anchor, anchors.back()->style_anchor);
 const PolicyDefinition* lower = anchors.front();
 const PolicyDefinition* upper = anchors.back();
 for (std::size_t index = 0; index + 1 < anchors.size(); ++index) {
  if (clamped >= anchors[index]->style_anchor && clamped <= anchors[index + 1]->style_anchor) {
   lower = anchors[index];
   upper = anchors[index + 1];
   break;
  }
 }
 const double span = upper->style_anchor - lower->style_anchor;
 const double mix = span > 0.0 ? (clamped - lower->style_anchor) / span : 0.0;
 std::set<std::string> keys;
 for (const auto& [name, value] : lower->behavior) {
  (void)value;
  keys.insert(name);
 }
 for (const auto& [name, value] : upper->behavior) {
  (void)value;
  keys.insert(name);
 }
 Properties result;
 for (const auto& name : keys) {
  const auto low = lower->behavior.find(name);
  const auto high = upper->behavior.find(name);
  const double low_value = low != lower->behavior.end()    ? low->second
                           : high != upper->behavior.end() ? high->second
                                                           : 0.0;
  const double high_value = high != upper->behavior.end() ? high->second : low_value;
  result.emplace(name, low_value + (high_value - low_value) * mix);
 }
 return result;
}

} // namespace mission
