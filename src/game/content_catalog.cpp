#include "content_catalog.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <set>
#include <sstream>

namespace game {
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

void require_schema_v1(const json& value, const std::filesystem::path& path) {
 if (value.value("schema_version", 0) != 1) {
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
  if (it.key() == "style") {
   continue;
  }
  selection.parts[it.key()] = it.value().get<std::string>();
 }
 selection.style = value.at("style").get<double>();
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

ContentCatalog ContentCatalog::load(const std::filesystem::path& assets_root) {
 ContentCatalog catalog;
 catalog.root_ = std::filesystem::canonical(assets_root);

 const auto campaign_path = catalog.root_ / "campaign.json";
 const auto campaign_json = read_json(campaign_path);
 require_schema_v1(campaign_json, campaign_path);
 const auto& companion = campaign_json.at("companion");
 catalog.campaign_ = {
   campaign_json.at("id").get<std::string>(),
   campaign_json.at("title").get<std::string>(),
   campaign_json.at("content_version").get<std::string>(),
   companion.at("unit_code").get<std::string>(),
   companion.at("display_name").get<std::string>(),
   companion.at("robot").get<std::string>(),
   campaign_json.at("starting_level").get<std::string>(),
   campaign_json.at("levels").get<std::vector<std::string>>(),
   campaign_json.at("starting_inventory").get<std::vector<std::string>>(),
   campaign_json.at("starting_policies").get<std::vector<std::string>>(),
 };

 for (const auto& path : json_files(catalog.root_ / "levels")) {
  const auto value = read_json(path);
  require_schema_v1(value, path);
  const auto& objective = value.at("objective");
  const auto& loadout = value.at("loadout_rules");
  const auto& teaching = value.at("teaching");
  const auto& score = value.at("score");
  LevelDefinition level;
  level.id = value.at("id").get<std::string>();
  level.order = value.at("order").get<int>();
  level.chapter = value.at("chapter").get<int>();
  level.title = value.at("title").get<std::string>();
  level.map = value.at("map").get<std::string>();
  level.control_mode = value.at("control_mode").get<std::string>();
  level.localization_mode = value.at("localization_mode").get<std::string>();
  if (level.control_mode != "manual_calibration" && level.control_mode != "autonomous") {
   throw ContentError(path.string() + ": unsupported control_mode");
  }
  if (level.localization_mode != "odometry" && level.localization_mode != "fused") {
   throw ContentError(path.string() + ": unsupported localization_mode");
  }
  level.prerequisites = value.at("prerequisites").get<std::vector<std::string>>();
  level.seed = value.at("seed").get<unsigned>();
  level.objective = {
    objective.at("type").get<std::string>(),        objective.at("spawn").get<std::string>(),
    objective.value("via", std::string{}),          objective.at("goal").get<std::string>(),
    objective.value("via_label", std::string{}),    objective.at("goal_label").get<std::string>(),
    objective.at("goal_visual").get<std::string>(), objective.at("time_limit_s").get<double>(),
    objective.at("max_collisions").get<int>(),      objective.at("arrive_tolerance_m").get<double>(),
    objective.value("via_radius_m", 1.25),
  };
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
    if (step.action != "pickup" && step.action != "complete" && step.action != "summon" &&
        step.action != "recharge") {
     throw ContentError(path.string() + ": unknown goal action " + step.action);
    }
    level.objective.goals.push_back(std::move(step));
   }
   if (level.objective.goals.empty()) {
    throw ContentError(path.string() + ": empty goals list");
   }
  } else if (!level.objective.via.empty()) {
   level.objective.goals.push_back({level.objective.via, level.objective.via_label, "pickup",
                                    level.objective.via_radius_m});
   level.objective.goals.push_back({level.objective.goal, level.objective.goal_label, "complete",
                                    level.objective.arrive_tolerance_m});
  } else {
   level.objective.goals.push_back({level.objective.goal, level.objective.goal_label, "complete",
                                    level.objective.arrive_tolerance_m});
  }
  level.budget = loadout.at("budget").get<int>();
  level.required_slots = loadout.at("required_slots").get<std::vector<std::string>>();
  level.concept = teaching.at("concept").get<std::string>();
  level.prediction_prompt = teaching.at("prediction_prompt").get<std::string>();
  level.knowledge_card = teaching.at("knowledge_card").get<std::string>();
  level.safe_min_clearance_m = score.at("safe_min_clearance_m").get<double>();
  level.max_localization_error_m = score.value("max_localization_error_m", 999.0);
  level.fast_time_s = score.at("fast_time_s").get<double>();
  level.efficient_energy_wh = score.at("efficient_energy_wh").get<double>();
  level.rewards = value.at("rewards").get<std::vector<std::string>>();
  insert_unique(catalog.levels_, std::move(level), path);
 }

 {
  const auto path = catalog.root_ / "surfaces.json";
  const auto value = read_json(path);
  require_schema_v1(value, path);
  for (const auto& item : value.at("materials")) {
   SurfaceMaterialDefinition material;
   material.id = item.at("id").get<std::string>();
   material.title = item.at("title").get<std::string>();
   material.description = item.at("description").get<std::string>();
   material.friction_mu = item.at("friction_mu").get<double>();
   material.texture_albedo = item.value("texture", std::string{});
   material.rolling_drag = item.at("rolling_drag").get<double>();
   material.traction_scale = item.at("traction_scale").get<double>();
   insert_unique(catalog.surface_materials_, std::move(material), path);
  }
 }
 {
  const auto path = catalog.root_ / "abilities.json";
  const auto value = read_json(path);
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
  const auto value = read_json(path);
  require_schema_v1(value, path);
  for (const auto& [key, text] : value.at("strings").items()) {
   if (!catalog.strings_.emplace(key, text.get<std::string>()).second) {
    throw ContentError(path.string() + ": duplicate string '" + key + "'");
   }
  }
 }
 for (const auto& path : json_files(catalog.root_ / "contracts")) {
  const auto value = read_json(path);
  require_schema_v1(value, path);
  for (const auto& item : value.at("jobs")) {
   ContractJob job;
   job.id = item.at("id").get<std::string>();
   job.level = item.at("level").get<std::string>();
   job.cargo = item.at("cargo").get<std::string>();
   job.axis = item.at("axis").get<std::string>();
   job.rep = item.at("rep").get<int>();
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
  const auto value = read_json(path);
  require_schema_v1(value, path);
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
   map.surfaces.push_back(std::move(surface));
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
   actor.visual_ref_radius_m = visual.at("ref_radius_m").get<double>();
   if (!std::filesystem::is_regular_file(catalog.root_ / actor.visual_model)) {
    throw ContentError(path.string() + ": missing actor model '" + actor.visual_model + "'");
   }
   map.actors.push_back(std::move(actor));
  }
  insert_unique(catalog.maps_, std::move(map), path);
 }

 for (const auto& path : json_files(catalog.root_ / "scenes")) {
  const auto value = read_json(path);
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
  std::set<std::string> instance_ids;
  for (const auto& instance_value : value.at("instances")) {
   const auto instance_id = instance_value.at("id").get<std::string>();
   if (!instance_ids.insert(instance_id).second) {
    throw ContentError(path.string() + ": duplicate scene instance '" + instance_id + "'");
   }
   scene.instances.push_back({
     instance_id,
     instance_value.at("prefab").get<std::string>(),
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
  insert_unique(catalog.scenes_, std::move(scene), path);
 }

 for (const auto& path : json_files(catalog.root_ / "components")) {
  const auto value = read_json(path);
  require_schema_v1(value, path);
  if (path.filename() == "policies.json") {
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
  const auto slot = value.at("slot").get<std::string>();
  for (const auto& item : value.at("components")) {
   ComponentDefinition component{
     item.at("id").get<std::string>(),    slot,
     item.at("title").get<std::string>(), item.at("description").get<std::string>(),
     item.at("cost").get<int>(),          properties(item.at("behavior")),
     visual(item.at("visual")),
   };
   require_visual_asset(component.visual, catalog.root_, path);
   insert_unique(catalog.components_, std::move(component), path);
  }
 }

 for (const auto& path : json_files(catalog.root_ / "robots")) {
  const auto value = read_json(path);
  require_schema_v1(value, path);
  const auto& drive = value.at("drive");
  const auto& motor = drive.at("motor");
  const auto& steering = drive.at("steering");
  const auto& energy = drive.at("energy");
  const auto& encoder = drive.at("encoder");
  const auto& collision = value.at("collision");
  RobotDefinition robot;
  robot.id = value.at("id").get<std::string>();
  robot.title = value.at("title").get<std::string>();
  robot.drive_type = drive.at("type").get<std::string>();
  robot.wheel_radius_m = drive.at("wheel_radius_m").get<double>();
  robot.wheel_base_m = drive.at("wheel_base_m").get<double>();
  robot.motor = {
    motor.at("max_wheel_speed_mps").get<double>(), motor.at("max_wheel_accel_mps2").get<double>(),
    motor.at("stall_force_N").get<double>(),       motor.at("free_speed_mps").get<double>(),
    motor.at("pwm_limit").get<double>(),           motor.at("integral_limit").get<double>(),
    motor.at("wheel_equiv_mass_kg").get<double>(),
  };
  robot.mass_kg = drive.at("mass_kg").get<double>();
  robot.yaw_inertia_kgm2 = drive.at("yaw_inertia_kgm2").get<double>();
  robot.max_yaw_accel_radps2 = drive.at("max_yaw_accel_radps2").get<double>();
  robot.tracked_max_yaw_rate_rads = drive.value("max_yaw_rate_rads", 1.5);
  robot.track_length_m = drive.value("track_length_m", 0.7);
  robot.track_width_m = drive.value("track_width_m", 0.58);
  robot.unlock_ability = value.value("unlock_ability", std::string{});
  robot.steering = {
    steering.at("integral_limit").get<double>(),
    steering.value("max_steer_rad", 0.6),
    steering.value("max_steer_rate_radps", 2.5),
    steering.value("servo_response_s", 0.4),
  };
  robot.energy_base_w = energy.at("base_w").get<double>();
  robot.energy_per_v_w = energy.at("per_v_w").get<double>();
  robot.energy_per_w_w = energy.at("per_w_w").get<double>();
  robot.encoder = {encoder.at("counts_per_revolution").get<int>()};
  robot.collision_radius_m = collision.at("radius_m").get<double>();
  robot.collision_height_m = collision.at("height_m").get<double>();
  std::set<std::string> mount_ids;
  for (const auto& item : value.at("mounts")) {
   MountDefinition mount{
     item.at("id").get<std::string>(),
     vec3(item.at("position_m")),
     vec3(item.at("rotation_deg")),
   };
   if (!mount_ids.insert(mount.id).second) {
    throw ContentError(path.string() + ": duplicate robot mount '" + mount.id + "'");
   }
   robot.mounts.push_back(std::move(mount));
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
  for (const auto* required : {"tires", "sensor", "computer"}) {
   if (!slot_ids.count(required)) {
    throw ContentError(path.string() + ": missing robot slot '" + required + "'");
   }
  }
  insert_unique(catalog.robots_, std::move(robot), path);
 }

 for (const auto& path : json_files(catalog.root_ / "presentation")) {
  const auto value = read_json(path);
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
  presentation.garage_heading = garage.at("heading").get<std::string>();
  presentation.garage_brief = garage.at("brief").get<std::string>();
  for (const auto& item : garage.at("component_columns")) {
   presentation.component_columns.push_back({
     item.at("slot").get<std::string>(),
     item.at("title").get<std::string>(),
     item.at("options").get<std::vector<std::string>>(),
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
   });
  }
  presentation.running_heading = running.at("heading").get<std::string>();
  presentation.normal_line = running.at("normal_line").get<std::string>();
  presentation.detected_line = running.at("detected_line").get<std::string>();
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
  const auto value = read_json(path);
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
  const auto chapters_value = read_json(chapters_path);
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

 std::set<int> orders;
 for (const auto& id : catalog.campaign_.levels) {
  const auto& level = catalog.level(id);
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
  (void)catalog.scene(map.scene_3d);
  if (!map.markers.count(level.objective.spawn) || !map.markers.count(level.objective.goal) ||
      (!level.objective.via.empty() && !map.markers.count(level.objective.via))) {
   throw ContentError("level '" + id + "' references an unknown map marker");
  }
 }
 if (catalog.campaign_.levels.size() != catalog.levels_.size()) {
  throw ContentError("campaign must list every level exactly once");
 }
 for (const auto& [id, presentation] : catalog.presentations_) {
  (void)catalog.level(presentation.level);
  if (!presentation.robot_id.empty()) {
   (void)catalog.robot(presentation.robot_id);
  }
  for (const auto& chassis : presentation.chassis_options) {
   (void)catalog.robot(chassis);
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
  if (!unlock.empty() && std::find(owned_abilities.begin(), owned_abilities.end(), unlock) ==
                            owned_abilities.end()) {
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

} // namespace game