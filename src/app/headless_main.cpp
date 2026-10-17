#include "app/run_artifact.hpp"
#include "core/sha256.hpp"
#include "mission/content_catalog.hpp"
#include "mission/mission_simulation.hpp"
#include "physics/physics_world.hpp"

#include <nlohmann/json.hpp>

#include <cmath>
#include <filesystem>
#include <fstream>
#include <optional>
#include <iostream>
#include <string>

namespace {

void usage(const char* executable) {
 std::cout << "Usage: " << executable
 << " [--assets PATH] [--level ID] [--default|--recommended] [--seed N]"
 " [--chassis ROBOT_ID] [--task-policy TASK_POLICY_ID]"
 " [--physics domain|shadow|bullet|auto] [--endless [--duration SECONDS]"
 " [--intent follow|go-to|search|wait|return] [--target X Y]] [--json]"
 " [--artifact FILE] [--reproduce FILE]\n";
}

mission::FieldIntentKind parse_field_intent(const std::string& value) {
 if (value == "follow") return mission::FieldIntentKind::Follow;
 if (value == "go-to" || value == "go_to") return mission::FieldIntentKind::GoTo;
 if (value == "search") return mission::FieldIntentKind::Search;
 if (value == "wait") return mission::FieldIntentKind::Wait;
 if (value == "return") return mission::FieldIntentKind::Return;
 throw std::runtime_error("unknown field intent: " + value);
}

} // namespace

int main(int argc, char** argv) {
 try {
  std::filesystem::path assets = "assets";
  std::string level_id;
  bool recommended = true;
  bool json_output = false;
 bool endless = false;
 std::filesystem::path artifact_path;
 std::filesystem::path reproduce_path;
 bool explicit_run_configuration = false;
 double duration_s = 60.0;
  std::optional<uint64_t> seed_override;
 std::string requested_chassis;
 std::string requested_task_policy;
 std::optional<mission::FieldIntentKind> field_intent;
 std::optional<sim::Vec2> field_target;
 mission::PhysicsMode physics_mode = mission::PhysicsMode::Bullet;
  for (int index = 1; index < argc; ++index) {
   const std::string argument = argv[index];
   if (argument == "--assets" && index + 1 < argc) {
    assets = argv[++index];
   } else if (argument == "--level" && index + 1 < argc) {
    level_id = argv[++index];
 explicit_run_configuration = true;
   } else if (argument == "--default") {
    recommended = false;
 explicit_run_configuration = true;
   } else if (argument == "--recommended") {
    recommended = true;
 explicit_run_configuration = true;
   } else if (argument == "--seed" && index + 1 < argc) {
    seed_override = std::stoull(argv[++index]);
 explicit_run_configuration = true;
 } else if (argument == "--chassis" && index + 1 < argc) {
 requested_chassis = argv[++index];
 explicit_run_configuration = true;
 } else if (argument == "--task-policy" && index + 1 < argc) {
 requested_task_policy = argv[++index];
 explicit_run_configuration = true;
   } else if (argument == "--json") {
    json_output = true;
 } else if (argument == "--artifact" && index + 1 < argc) {
 artifact_path = argv[++index];
 } else if (argument == "--reproduce" && index + 1 < argc) {
 reproduce_path = argv[++index];
 } else if (argument == "--endless") {
 endless = true;
 explicit_run_configuration = true;
 } else if (argument == "--duration" && index + 1 < argc) {
 duration_s = std::stod(argv[++index]);
 explicit_run_configuration = true;
 if (!(duration_s > 0.0))
 throw std::runtime_error("endless duration must be positive");
 } else if (argument == "--intent" && index + 1 < argc) {
 field_intent = parse_field_intent(argv[++index]);
 explicit_run_configuration = true;
 } else if (argument == "--target" && index + 2 < argc) {
 field_target = sim::Vec2{std::stod(argv[++index]), std::stod(argv[++index])};
 explicit_run_configuration = true;
 } else if (argument == "--physics" && index + 1 < argc) {
 physics_mode = mission::parse_physics_mode(argv[++index]);
 explicit_run_configuration = true;
   } else if (argument == "--help" || argument == "-h") {
    usage(argv[0]);
    return 0;
   } else {
    throw std::runtime_error("unknown argument: " + argument);
   }
  }

 const auto catalog = mission::ContentCatalog::load(assets);
 mission::BuildSelection build;
 std::string selection_label = recommended ? "recommended" : "default";
 if (!reproduce_path.empty()) {
 if (explicit_run_configuration)
 throw std::runtime_error("--reproduce cannot be combined with run configuration options");
 std::ifstream input(reproduce_path);
 if (!input)
 throw std::runtime_error("cannot read run artifact: " + reproduce_path.string());
 nlohmann::json artifact;
 input >> artifact;
 if (artifact.value("schema", nlohmann::json{}) !=
 nlohmann::json{{"name", "lost-signal.run"}, {"version", app::kRunArtifactSchemaVersion}}) {
 throw std::runtime_error("--reproduce requires lost-signal.run schema version " +
 std::to_string(app::kRunArtifactSchemaVersion));
 }
 const auto& configuration = artifact.at("configuration");
 const auto& comparable = configuration.at("comparable");
 if (configuration.at("digest").get<std::string>() != sim::sha256(comparable.dump()))
 throw std::runtime_error("run artifact configuration digest is invalid");
 if (comparable.at("content_sha256").get<std::string>() != app::run_content_digest(assets))
 throw std::runtime_error("run artifact content hash does not match --assets");
 const auto& experiment = comparable.at("experiment");
 if (experiment.at("condition_id") != "authored-base" || experiment.at("mode") != "mission")
 throw std::runtime_error("Run Artifact v2 reproduction supports authored mission runs only");
 if (std::abs(experiment.at("fixed_step_s").get<double>() - mission::MissionSimulation::fixed_step_s) > 1e-12)
 throw std::runtime_error("run artifact fixed timestep is incompatible with this build");
 if (experiment.at("runtime_provider") != "channel.in-process")
 throw std::runtime_error("run artifact requires an unavailable runtime provider");
 level_id = experiment.at("mission_id").get<std::string>();
 seed_override = experiment.at("seed").get<std::uint64_t>();
 physics_mode = mission::parse_physics_mode(experiment.at("physics_backend").get<std::string>());
 const auto& hardware = comparable.at("hardware");
 build.parts = hardware.at("components").get<std::map<std::string, std::string>>();
 build.style = hardware.at("style").get<double>();
 build.chassis_robot = hardware.value("requested_chassis", std::string{});
 build.task_policy = comparable.at("algorithms").value("task_ordering", std::string{});
 selection_label = "reproduced";
 } else {
 if (level_id.empty())
 level_id = catalog.campaign().starting_level;
 const auto& presentation = catalog.presentation_for_level(level_id);
 build = recommended ? presentation.recommended_build : presentation.default_build;
 if (!requested_chassis.empty())
 build.chassis_robot = requested_chassis;
 if (!requested_task_policy.empty())
 build.task_policy = requested_task_policy;
 }
 if (field_intent && !endless)
 throw std::runtime_error("--intent requires --endless");
 mission::MissionSimulation mission(catalog, level_id, build, seed_override, {},
 mission::ComponentRegistry::builtins(), {}, physics_mode);
 if (endless) {
 mission.set_endless_mode(true);
 if (field_intent && !mission.issue_field_intent(*field_intent, field_target))
 throw std::runtime_error("field intent requires a valid target");
 mission.start();
 const int steps = static_cast<int>(std::ceil(duration_s / mission::MissionSimulation::fixed_step_s));
 for (int step = 0; step < steps; ++step)
 mission.step();
 } else {
 mission.run_to_completion();
 }
  const auto& result = mission.result();
 const auto runtime_health = mission.runtime_health();
 const auto& environment_keyframes = mission.level().environment_keyframes;
 const std::string environment_stage = environment_keyframes.empty()
 ? std::string{}
 : environment_keyframes[mission.snapshot().environment.keyframe_index].id;

 if (json_output || !artifact_path.empty()) {
   nlohmann::json output = {
     {"level", level_id},
     {"seed", mission.effective_seed()},
 {"physics_backend", mission.physics_backend_name()},
 {"runtime",
 {
 {"interface", "lsri"},
 {"api_version", mission.runtime_api_version()},
 {"provider", mission.runtime_provider_id()},
 {"state", sim::runtime::lifecycle_state_name(mission.runtime_state())},
 {"health", sim::runtime::health_level_name(runtime_health.level)},
 {"dropped_observations", runtime_health.dropped_observations},
 {"rejected_observations", runtime_health.rejected_observations},
 {"rejected_commands", runtime_health.rejected_commands},
 {"watchdog_stops", runtime_health.watchdog_stops},
 }},
     {"bullet_available", sim::physics::bullet_physics_available()},
     {"status", mission::mission_status_name(result.status)},
     {"failure_reason", result.failure_reason},
     {"time_s", result.time_s},
     {"path_length_m", result.path_length_m},
     {"energy_wh", result.energy_wh},
     {"min_clearance_m", result.min_clearance_m},
     {"scored_clearance_m", result.scored_clearance_m},
     {"min_static_clearance_m", result.min_static_clearance_m},
     {"min_static_clearance_time_s", result.min_static_clearance_time_s},
     {"min_static_robot_position",
      {
        result.min_static_robot_position.x,
        result.min_static_robot_position.y,
      }},
     {"min_dynamic_clearance_m", result.min_dynamic_clearance_m},
     {"min_dynamic_clearance_time_s", result.min_dynamic_clearance_time_s},
     {"min_dynamic_robot_position",
      {
        result.min_dynamic_robot_position.x,
        result.min_dynamic_robot_position.y,
      }},
     {"min_dynamic_actor_position",
      {
        result.min_dynamic_actor_position.x,
        result.min_dynamic_actor_position.y,
      }},
     {"odometry_error_m", result.odometry_error_m},
     {"max_odometry_error_m", result.max_odometry_error_m},
 {"odometry_heading_error_rad", result.odometry_heading_error_rad},
 {"max_odometry_heading_error_rad", result.max_odometry_heading_error_rad},
 {"fused_pose_error_m", result.fused_pose_error_m},
 {"max_fused_pose_error_m", result.max_fused_pose_error_m},
 {"fused_heading_error_rad", result.fused_heading_error_rad},
 {"max_fused_heading_error_rad", result.max_fused_heading_error_rad},
     {"collisions", result.collisions},
 {"rigid_contacts", result.rigid_contacts},
 {"shadow_error",
 {
 {"max_position_m", result.max_shadow_position_error_m},
 {"max_heading_rad", result.max_shadow_heading_error_rad},
 {"max_height_m", result.max_shadow_height_error_m},
 }},
     {"slip_events", result.slip_events},
     {"astar_expanded", result.astar_expanded},
     {"local_decisions", result.local_decisions},
     {"robot",
      {
        {"x", mission.snapshot().robot.x},
        {"y", mission.snapshot().robot.y},
        {"z", mission.snapshot().robot_height_m},
        {"heading", mission.snapshot().robot.theta},
        {"roll_rad", mission.snapshot().robot_roll_rad},
        {"pitch_rad", mission.snapshot().robot_pitch_rad},
      }},
     {"terrain",
      {
        {"current_grade", mission.snapshot().terrain_grade},
        {"current_cross_slope", mission.snapshot().cross_slope},
        {"max_grade", result.max_grade},
        {"max_abs_roll_rad", result.max_abs_roll_rad},
        {"current_rollover_risk", mission.snapshot().rollover_risk},
        {"max_rollover_risk", result.max_rollover_risk},
        {"traction_saturated", mission.snapshot().traction_saturated},
 {"traction_slip_observed", mission.snapshot().traction_slip_observed},
        {"traction_saturation_occurred", result.slip_events > 0},
 {"max_drive_slip_ratio", result.max_drive_slip_ratio},
      }},
 {"environment",
 {
 {"stage", environment_stage},
 {"wetness", mission.snapshot().environment.wetness},
 {"visibility", mission.snapshot().environment.visibility},
 {"daylight", mission.snapshot().environment.daylight},
 {"peak_wetness", result.peak_wetness},
 {"min_visibility", result.min_visibility},
 }},
 {"endless",
 {
 {"enabled", mission.endless_mode()},
 {"autonomous", mission.endless_mode()},
 {"dispatch_active", mission.field_dispatch().active},
 {"dispatch_completed", mission.field_dispatch().completed_count},
 {"route_points", mission.planned_route().size()},
 {"field_operator", {
 {"x", mission.field_operator().position.x},
 {"y", mission.field_operator().position.y},
 {"heading_rad", mission.field_operator().heading_rad},
 {"moving", mission.field_operator().moving},
 }},
 {"field_intent", {
 {"active", mission.field_intent().active},
 {"kind", mission::field_intent_kind_id(mission.field_intent().kind)},
 {"revision", mission.field_intent().revision},
 {"acknowledgement",
 mission::field_intent_acknowledgement_id(mission.field_intent().acknowledgement)},
 {"target", {mission.field_intent().target.x, mission.field_intent().target.y}},
 }},
 }},
     {"via_reached", result.via_reached},
 {"tasks",
 {
 {"goal_order", mission.level().objective.goal_order},
 {"policy", mission.level().objective.task_policy},
 {"completed_count", mission.snapshot().goals_reached},
 {"active_goal_id", mission.snapshot().active_goal_id},
 {"completed_goal_ids", mission.snapshot().completed_goal_ids},
 {"abandoned_goal_ids", mission.snapshot().abandoned_goal_ids},
 }},
 {"payload",
 {
 {"mass_kg", mission.payload_state().mass_kg},
 {"collected_mass_kg", mission.payload_state().collected_mass_kg},
 {"first_moment_kgm", mission.payload_state().first_moment_kgm},
 {"cog_height_m", mission.payload_state().cog_height_m},
 {"collected_items", result.collected_items},
 {"collected_value", result.collected_value},
 {"cargo_integrity", result.cargo_integrity},
 {"max_shock_mps2", mission.payload_state().max_shock_mps2},
 {"max_jerk_mps3", mission.payload_state().max_jerk_mps3},
 {"history", nlohmann::json::array()},
 }},
     {"events", nlohmann::json::array()},
 {"decisions", nlohmann::json::array()},
     {"scores",
      {
        {"safe", result.safe_score},
        {"fast", result.fast_score},
        {"efficient", result.efficient_score},
      }},
   };
 for (const auto& pickup : mission.payload_state().history) {
 output["payload"]["history"].push_back({
 {"goal_id", pickup.goal_id},
 {"time_s", pickup.time_s},
 {"item_count", pickup.item_count},
 {"mass_kg", pickup.mass_kg},
 {"cog_height_m", pickup.cog_height_m},
 {"value", pickup.value},
 {"fragility", pickup.fragility},
 });
 }
   for (const auto& event : mission.events()) {
    output["events"].push_back({
      {"time_s", event.time_s},
      {"code", event.code},
      {"message", event.message},
    });
   }
 for (const auto& decision : mission.decisions()) {
 output["decisions"].push_back({
 {"sequence", decision.sequence},
 {"decision_time_s", decision.decision_time_s},
 {"end_time_s", decision.end_time_s},
 {"observation_time_s", decision.observation_time_s},
 {"observation_refs", decision.observation_refs},
 {"prediction_horizon_s", decision.prediction_horizon_s},
 {"measured_separation_m", decision.measured_separation_m},
 {"predicted_separation_m", decision.predicted_separation_m},
 {"constraint", decision.constraint_code},
 {"reason", decision.reason_code},
 {"action", decision.action_code},
 {"target_speed_mps", decision.target_speed_mps},
 {"target_yaw_rate_radps", decision.target_yaw_rate_radps},
 {"outcome", decision.outcome_code},
 {"superseded_by", decision.superseded_by},
 });
 }
 if (!artifact_path.empty()) {
 const auto artifact = app::make_run_artifact(
 catalog, mission, assets, output, selection_label, duration_s);
 if (!artifact_path.parent_path().empty())
 std::filesystem::create_directories(artifact_path.parent_path());
 std::ofstream artifact_file(artifact_path);
 if (!artifact_file)
 throw std::runtime_error("cannot write run artifact: " + artifact_path.string());
 artifact_file << artifact.dump(2) << '\n';
 if (!artifact_file)
 throw std::runtime_error("failed while writing run artifact: " + artifact_path.string());
 }
 if (json_output)
 std::cout << output.dump(2) << '\n';
 }
 if (!json_output) {
   std::cout << "[" << level_id << "] " << mission::mission_status_name(result.status)
             << "  time=" << result.time_s << "s"
             << "  path=" << result.path_length_m << "m"
             << "  energy=" << result.energy_wh << "Wh"
             << " clearance=" << result.min_clearance_m << "m"
 << " physics=" << mission.physics_backend_name() << " runtime=" << mission.runtime_provider_id()
             << " z=" << mission.snapshot().robot_height_m << "m"
             << " roll=" << mission.snapshot().robot_roll_rad << "rad"
             << " pitch=" << mission.snapshot().robot_pitch_rad << "rad"
             << " grade=" << mission.snapshot().terrain_grade << " max_grade=" << result.max_grade
             << " max_roll=" << result.max_abs_roll_rad << "rad"
             << " rollover_risk=" << result.max_rollover_risk
 << " max_drive_slip_ratio=" << result.max_drive_slip_ratio
             << " traction_saturated=" << (mission.snapshot().traction_saturated ? "yes" : "no");
   if (!result.failure_reason.empty()) {
    std::cout << "  reason=" << result.failure_reason;
   }
   std::cout << '\n';
  }
 return endless || result.status == mission::MissionStatus::Succeeded ? 0 : 2;
 } catch (const std::exception& error) {
  std::cerr << "error: " << error.what() << '\n';
  return 1;
 }
}
