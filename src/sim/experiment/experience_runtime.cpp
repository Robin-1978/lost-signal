#include "sim/experiment/experience_runtime.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <utility>

#include "lost_signal/build_revision.hpp"
#include "sim/eval/robot_config.hpp"
#include "sim/eval/world_io.hpp"
#include "sim/scenario/actor_sidecar.hpp"

namespace lost_signal::sim::experiment {
namespace {

constexpr std::uint64_t kFnvOffset = 1469598103934665603ULL;
constexpr std::uint64_t kFnvPrime = 1099511628211ULL;

void hash_bytes(std::uint64_t& hash, const void* data, std::size_t size) {
 const auto* bytes = static_cast<const unsigned char*>(data);
 for (std::size_t i = 0; i < size; ++i) {
 hash ^= bytes[i];
 hash *= kFnvPrime;
 }
}

void hash_string(std::uint64_t& hash, const std::string& value) {
 const std::uint64_t size = static_cast<std::uint64_t>(value.size());
 hash_bytes(hash, &size, sizeof(size));
 hash_bytes(hash, value.data(), value.size());
}

bool hash_file(std::uint64_t& hash, const char* role,
 const std::string& path, bool optional, std::string& error) {
 hash_string(hash, role);
 std::ifstream input(path, std::ios::binary);
 if (!input.is_open()) {
 if (optional) {
 hash_string(hash, "absent");
 return true;
 }
 error = "cannot hash experience input: " + path;
 return false;
 }
 hash_string(hash, "present");
 char chunk[4096];
 while (input.good()) {
 input.read(chunk, sizeof(chunk));
 const std::streamsize count = input.gcount();
 if (count > 0) {
 hash_bytes(hash, chunk, static_cast<std::size_t>(count));
 }
 }
 if (!input.eof()) {
 error = "cannot read experience input: " + path;
 return false;
 }
 return true;
}

std::string actor_sidecar_path(const std::string& map_path) {
 std::filesystem::path sidecar(map_path);
 sidecar.replace_extension(".actors.json");
 return sidecar.string();
}

bool compute_invariant_digest(const ExperienceRuntimeConfig& config,
 const ExperienceRuntime& runtime, std::uint64_t& digest,
 std::string& error) {
 digest = kFnvOffset;
 hash_string(digest, "lost-signal-experiment-invariant/2");
 // The robot profile is the controlled input for a categorical chassis
 // experiment. Hash the authored baseline path in the invariant digest so
 // baseline and variant remain comparable; the selected variant profile is
 // recorded separately in the EpisodeArtifact.
 if (!hash_file(digest, "robot", config.robot_path, false, error) ||
 !hash_file(digest, "sim", runtime.used_sim_path, false, error) ||
 !hash_file(digest, "world", config.map_path, false, error) ||
 !hash_file(digest, "actors", actor_sidecar_path(config.map_path), true,
 error) ||
 !hash_file(digest, "scene", config.scene_path, true, error) ||
 !hash_file(digest, "asset_manifest", config.asset_manifest_path, true,
 error)) {
 return false;
 }
 hash_string(digest, "code_revision");
 hash_string(digest, runtime.code_revision);
 hash_string(digest, "objectives");
 for (const auto& objective : config.objectives) {
 hash_string(digest, objective);
 }
 hash_string(digest, "startup_mode");
 hash_string(digest, experience_startup_mode_id(config.startup_mode));
 hash_string(digest, "assembly");
 hash_string(digest, config.has_assembly ? "selected" : "none");
 if (config.has_assembly) {
   hash_string(digest, config.assembly.wheel);
   hash_string(digest, config.assembly.lidar);
   hash_string(digest, config.assembly.compute);
 }
 return digest != 0;
}

} // namespace

ExperienceRuntimeConfig experience_runtime_config(
 const viewer::ExperienceManifest& experience,
 const std::string& asset_manifest_path) {
 ExperienceRuntimeConfig config;
 config.robot_path = experience.robot_path;
 config.sim_path = experience.sim_path;
 config.map_path = experience.map_path;
 config.scene_path = experience.scene_path;
 config.asset_manifest_path = asset_manifest_path;
 config.objectives = experience.objectives;
 config.assembly = experience.assembly;
 config.has_assembly = experience.has_assembly;
 config.startup_mode = experience.startup_mode;
 return config;
}

const char* experience_startup_mode_id(
 viewer::ExperienceStartupMode mode) {
 switch (mode) {
 case viewer::ExperienceStartupMode::kAutomatic: return "auto";
 case viewer::ExperienceStartupMode::kSeedAndTrack: return "seed_and_track";
 case viewer::ExperienceStartupMode::kGlobalSearch: return "global_search";
 case viewer::ExperienceStartupMode::kSlamOnly: return "slam_only";
 }
 return "invalid";
}

void apply_experience_startup_mode(
 viewer::ExperienceStartupMode mode,
 scenario::ScenarioSession& session) {
 using robot::autonomy::StartupMode;
 switch (mode) {
 case viewer::ExperienceStartupMode::kAutomatic:
 return;
 case viewer::ExperienceStartupMode::kSeedAndTrack:
 session.set_startup_mode(StartupMode::kSeedAndTrack);
 return;
 case viewer::ExperienceStartupMode::kGlobalSearch:
 session.set_startup_mode(StartupMode::kGlobalSearch);
 return;
 case viewer::ExperienceStartupMode::kSlamOnly:
 session.set_startup_mode(StartupMode::kSlamOnly);
 return;
 }
}

double experience_variable_value(viewer::ExperienceVariable variable,
 const eval::SimConfig& config) {
 switch (variable) {
 case viewer::ExperienceVariable::kLidarScanPeriodS:
 return config.scan_period_s;
 case viewer::ExperienceVariable::kChassisKind:
  return 0.0;
 }
 return 0.0;
}

bool apply_assembly(const viewer::AssemblySelection& selection,
                    robot::platform::RobotConfig& robot_config,
                    eval::SimConfig& sim_config, std::string& error) {
  if (!viewer::validate_assembly_selection(selection, &error)) return false;
  const auto* wheel = viewer::find_assembly("wheel", selection.wheel);
  const auto* lidar = viewer::find_assembly("lidar", selection.lidar);
  const auto* compute = viewer::find_assembly("compute", selection.compute);
  if (wheel == nullptr || lidar == nullptr || compute == nullptr) {
    error = "assembly registry is incomplete";
    return false;
  }
  // These are typed profile consequences, not scene truth.  The same wheel
  // friction and rolling-resistance values feed native description and SIM
  // motor construction before the first observation is published.
  robot_config.profile.tire_friction_mu = wheel->tire_friction_mu;
  robot_config.profile.rolling_resist = wheel->rolling_resist;
  sim_config.lidar_max_range_m = static_cast<float>(lidar->lidar_max_range_m);
  sim_config.lidar_beams = lidar->lidar_beams;
  sim_config.scan_period_s = lidar->scan_period_s;
  sim_config.decision_period_s = compute->decision_period_s;
  sim_config.planning_period_s = compute->decision_period_s;
  return true;
}

bool build_experience_runtime(const ExperienceRuntimeConfig& config,
 ExperienceRuntime& runtime, std::string* error) {
 std::string local_error;
 std::string& out_error = error != nullptr ? *error : local_error;
 out_error.clear();

 ExperienceRuntime built;
 built.code_revision = LOST_SIGNAL_BUILD_REVISION;
 std::string config_error;
 std::string robot_path = config.robot_path;
 if (config.variable_override.has_value() &&
     config.variable_override->variable ==
         viewer::ExperienceVariable::kChassisKind) {
   const auto& override = *config.variable_override;
   if (!viewer::experience_variable_category_valid(
           override.variable, override.category) ||
       override.robot_path.empty()) {
     out_error = "categorical chassis override is incomplete or unsupported";
     return false;
   }
   robot_path = override.robot_path;
 }
 built.robot_config = robot::platform::load_default_robot_config(
 robot_path, &built.used_robot_path, &config_error);
 if (!robot_path.empty() && !built.robot_config.has_value()) {
 out_error = "robot config: " + config_error;
 return false;
 }
 built.sim_config = eval::load_default_sim_config(
 config.sim_path, &built.used_sim_path, &config_error);
 if (!config.sim_path.empty() && !built.sim_config.has_value()) {
 out_error = "sim config: " + config_error;
 return false;
 }
 if (config.has_assembly && built.robot_config.has_value() &&
     built.sim_config.has_value()) {
   if (!apply_assembly(config.assembly, *built.robot_config,
                       *built.sim_config, config_error)) {
     out_error = "assembly: " + config_error;
     return false;
   }
 }
 if (built.robot_config.has_value()) {
 built.session = std::make_unique<scenario::ScenarioSession>(
 robot::platform::description_from_config(*built.robot_config),
 eval::motor_from_config(*built.robot_config));
 } else {
 built.session = std::make_unique<scenario::ScenarioSession>();
 }
 if (built.sim_config.has_value()) {
 if (config.variable_override.has_value()) {
 const auto& override = *config.variable_override;
 const auto& spec = viewer::experience_variable_spec(override.variable);
 if (!std::isfinite(override.value) || override.value < spec.minimum ||
 override.value > spec.maximum ||
 (spec.physics_grid && !scenario::on_physics_grid(override.value))) {
 out_error = "experiment override is outside its typed contract";
 return false;
 }
 switch (override.variable) {
 case viewer::ExperienceVariable::kLidarScanPeriodS:
 built.sim_config->scan_period_s = override.value;
 break;
 case viewer::ExperienceVariable::kChassisKind:
  // The typed chassis profile was selected before ROBOT construction. It has
  // no SIM scalar patch; the resulting plant is derived from robot_config.
  break;
 }
 }
 built.session->apply_sim_config(*built.sim_config);
 } else if (config.variable_override.has_value()) {
 out_error = "experiment override needs a SIM configuration";
 return false;
 }
 if (config.map_path.empty()) {
 out_error = "experience runtime needs a map file";
 return false;
 }

 eval::WorldFileContent content;
 if (!eval::load_world_file(config.map_path, content, &config_error)) {
 out_error = "map: " + config_error;
 return false;
 }
 std::vector<std::string> objective_ids;
 if (config.objectives.empty()) {
 for (const auto& place : content.places) {
 objective_ids.push_back(place.place_id);
 }
 } else {
 for (const auto& requested : config.objectives) {
 const bool found = std::any_of(
 content.places.begin(), content.places.end(),
 [&](const world::NamedPlace& place) {
 return place.place_id == requested;
 });
 if (!found) {
 out_error = "experience objective missing from map: " + requested;
 return false;
 }
 objective_ids.push_back(requested);
 }
 }

 auto& world = built.session->setup_world();
 world.set_survey_resolution_m(content.survey_resolution_m);
 world.set_static_boxes(std::move(content.boxes));
 world.set_places(std::move(content.places));
 world.set_materials(std::move(content.materials));
 world.set_material_zones(std::move(content.zones));
 world.set_terrain_patches(std::move(content.patches));
 world.set_support_slabs(std::move(content.slabs));
 world.set_environment(std::move(content.environment));
 world.set_initial_pose_prior(content.initial_pose);
 if (content.initial_pose.has_value()) {
 const auto& prior = *content.initial_pose;
 built.session->set_initial_pose(
 prior.x_m, prior.y_m, prior.yaw_rad, prior.position_sigma_m,
 prior.heading_sigma_rad);
 }
 built.session->set_objectives(std::move(objective_ids));
 apply_experience_startup_mode(config.startup_mode, *built.session);
 built.session->fit_map_window_to_world();
 if (scenario::load_actors_sidecar(
 config.map_path, *built.session, {}, &config_error) < 0) {
 out_error = "actors sidecar: " + config_error;
 return false;
 }
 if (!compute_invariant_digest(
 config, built, built.invariant_digest, out_error)) {
 return false;
 }
 built.session->prepare_world();

 runtime = std::move(built);
 return true;
}

} // namespace lost_signal::sim::experiment
