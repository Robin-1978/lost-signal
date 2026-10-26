#include "sim/experiment/experience_runtime.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include "sim/eval/robot_config.hpp"
#include "sim/eval/world_io.hpp"
#include "sim/scenario/actor_sidecar.hpp"

namespace lost_signal::sim::experiment {

double experience_variable_value(viewer::ExperienceVariable variable,
 const eval::SimConfig& config) {
 switch (variable) {
 case viewer::ExperienceVariable::kLidarScanPeriodS:
 return config.scan_period_s;
 }
 return 0.0;
}

bool build_experience_runtime(const ExperienceRuntimeConfig& config,
 ExperienceRuntime& runtime, std::string* error) {
 std::string local_error;
 std::string& out_error = error != nullptr ? *error : local_error;
 out_error.clear();

 ExperienceRuntime built;
 std::string config_error;
 built.robot_config = robot::platform::load_default_robot_config(
 config.robot_path, &built.used_robot_path, &config_error);
 if (!config.robot_path.empty() && !built.robot_config.has_value()) {
 out_error = "robot config: " + config_error;
 return false;
 }
 if (built.robot_config.has_value()) {
 built.session = std::make_unique<scenario::ScenarioSession>(
 robot::platform::description_from_config(*built.robot_config),
 eval::motor_from_config(*built.robot_config));
 } else {
 built.session = std::make_unique<scenario::ScenarioSession>();
 }

 built.sim_config = eval::load_default_sim_config(
 config.sim_path, &built.used_sim_path, &config_error);
 if (!config.sim_path.empty() && !built.sim_config.has_value()) {
 out_error = "sim config: " + config_error;
 return false;
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
 built.session->fit_map_window_to_world();
 if (scenario::load_actors_sidecar(
 config.map_path, *built.session, {}, &config_error) < 0) {
 out_error = "actors sidecar: " + config_error;
 return false;
 }
 built.session->prepare_world();

 runtime = std::move(built);
 return true;
}

} // namespace lost_signal::sim::experiment
