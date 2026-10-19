#include "sim/game/app/run_artifact.hpp"
#include "robot/core/sha256.hpp"
#include "sim/game/mission/content_catalog.hpp"
#include "sim/game/scenario/scenario_session.hpp"
#include "sim/hardware/physics/physics_world.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <optional>
#include <iostream>
#include <string>
#include <vector>

namespace {

void usage(const char* executable) {
 std::cout << "Usage: " << executable
 << " [--assets PATH] [--level ID] [--default|--recommended] [--seed N]"
 " [--chassis ROBOT_ID] [--component SLOT=ID] [--task-policy TASK_POLICY_ID]"
 " [--construction-plan PLAN_ID]"
 " [--goal-sequence GOAL_ID,GOAL_ID,...]"
 " [--maneuver ID]"
 " [--endless [--duration SECONDS]"
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
 std::map<std::string, std::string> component_overrides;
 std::string requested_task_policy;
 std::string requested_maneuver;
 std::string requested_construction_plan;
 std::string reproduced_construction_hash;
 std::optional<mission::FieldIntentKind> field_intent;
 std::optional<sim::Vec2> field_target;
 std::vector<std::string> requested_goal_sequence;
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
 } else if (argument == "--component" && index + 1 < argc) {
 const std::string override = argv[++index];
 const auto separator = override.find('=');
 if (separator == std::string::npos || separator == 0 || separator + 1 == override.size())
 throw std::runtime_error("--component requires SLOT=ID");
 component_overrides[override.substr(0, separator)] = override.substr(separator + 1);
 explicit_run_configuration = true;
 } else if (argument == "--task-policy" && index + 1 < argc) {
 requested_task_policy = argv[++index];
 explicit_run_configuration = true;
 } else if (argument == "--maneuver" && index + 1 < argc) {
 requested_maneuver = argv[++index];
 explicit_run_configuration = true;
 } else if (argument == "--construction-plan" && index + 1 < argc) {
 requested_construction_plan = argv[++index];
 explicit_run_configuration = true;
 } else if (argument == "--goal-sequence" && index + 1 < argc) {
 std::string sequence = argv[++index];
 std::size_t begin = 0;
 while (begin <= sequence.size()) {
 const std::size_t end = sequence.find(',', begin);
 const std::string id = sequence.substr(begin, end == std::string::npos
 ? std::string::npos : end - begin);
 if (id.empty())
 throw std::runtime_error("--goal-sequence contains an empty goal id");
 requested_goal_sequence.push_back(id);
 if (end == std::string::npos)
 break;
 begin = end + 1;
 }
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
 throw std::runtime_error("Run Artifact v3 reproduction supports authored mission runs only");
 if (std::abs(experiment.at("fixed_step_s").get<double>() - mission::ScenarioSession::fixed_step_s) > 1e-12)
 throw std::runtime_error("run artifact fixed timestep is incompatible with this build");
 if (experiment.at("runtime_provider") != "channel.in-process")
 throw std::runtime_error("run artifact requires an unavailable runtime provider");
 level_id = experiment.at("mission_id").get<std::string>();
 seed_override = experiment.at("seed").get<std::uint64_t>();
 if (experiment.at("physics_backend") != "mujoco")
 throw std::runtime_error("run artifact requires the MuJoCo physics engine");
 const auto& hardware = comparable.at("hardware");
 build.parts = hardware.at("components").get<std::map<std::string, std::string>>();
 build.style = hardware.at("style").get<double>();
 build.chassis_robot = hardware.value("requested_chassis", std::string{});
 build.task_policy = comparable.at("algorithms").value("task_ordering", std::string{});
 if (const auto construction = comparable.find("construction");
 construction != comparable.end() && !construction->is_null()) {
 reproduced_construction_hash = construction->at("plan_sha256").get<std::string>();
 }
 selection_label = "reproduced";
 } else {
 if (level_id.empty())
 level_id = catalog.campaign().starting_level;
 const auto& presentation = catalog.presentation_for_level(level_id);
 build = recommended ? presentation.recommended_build : presentation.default_build;
 if (!requested_chassis.empty())
 build.chassis_robot = requested_chassis;
 for (const auto& [slot, component_id] : component_overrides)
 build.parts[slot] = component_id;
 if (!requested_task_policy.empty())
 build.task_policy = requested_task_policy;
 }
 if (field_intent && !endless && catalog.level(level_id).control_mode != "operator_drive")
 throw std::runtime_error("--intent requires --endless or an operator-drive level");
 if (!requested_maneuver.empty() && endless)
 throw std::runtime_error("--maneuver cannot be combined with --endless");
 if (!requested_maneuver.empty() && !artifact_path.empty())
 throw std::runtime_error("maneuver artifacts require a future explicit artifact schema field");
 if (!requested_goal_sequence.empty() && !artifact_path.empty())
 throw std::runtime_error("player-directed artifacts require a future explicit artifact schema field");
 mission::ScenarioSession mission(catalog, level_id, build, seed_override, {},
 mission::ComponentRegistry::builtins());
 if (mission.level().construction) {
 const auto& construction = *mission.level().construction;
 std::string option_id = requested_construction_plan.empty()
 ? construction.default_plan : requested_construction_plan;
 if (!reproduced_construction_hash.empty()) {
 option_id.clear();
 for (const auto& option : construction.plans) {
 const auto candidate = mission::construction_plan_from_option(construction, option.id);
 if (mission::construction_plan_sha256(candidate) == reproduced_construction_hash) {
 option_id = option.id;
 break;
 }
 }
 if (option_id.empty())
 throw std::runtime_error("run artifact construction plan is unavailable in this content");
 }
 std::string reason;
 if (!mission.commit_construction_plan(
 mission::construction_plan_from_option(construction, option_id), &reason)) {
 throw std::runtime_error("construction plan rejected: " + reason);
 }
 } else if (!requested_construction_plan.empty() || !reproduced_construction_hash.empty()) {
 throw std::runtime_error("construction plan supplied for a level without construction");
 }
 if (!requested_maneuver.empty() && !mission.activate_maneuver(requested_maneuver))
 throw std::runtime_error("unknown or unavailable maneuver: " + requested_maneuver);
 if (endless)
 mission.set_endless_mode(true);

 // Operator intent is an input to an active ROBOT session. Publishing it
 // before start made the headless path differ from a click in the graphical
 // client and could leave the target outside the live control loop.
 if (!requested_goal_sequence.empty() && mission.level().objective.goal_order != "player")
 throw std::runtime_error("--goal-sequence requires a player-ordered level");
 if (requested_goal_sequence.empty() && mission.level().objective.goal_order == "player")
 throw std::runtime_error("player-ordered headless runs require --goal-sequence");
 if (endless || field_intent || !requested_goal_sequence.empty())
 mission.start();
 if (field_intent && !mission.issue_field_intent(*field_intent, field_target))
 throw std::runtime_error("field intent requires a valid target");
 if (endless) {
 const int steps = static_cast<int>(std::ceil(duration_s / mission::ScenarioSession::fixed_step_s));
 for (int step = 0; step < steps; ++step)
 mission.step();
 } else if (field_intent) {
 // An operator-drive intent is a bounded command inside a continuing
 // playground session, not the authored level objective. Stop the headless
 // run when that command completes or is rejected instead of waiting for the
 // unrelated playground timeout.
 while (mission.snapshot().status == mission::MissionStatus::Running &&
 mission.field_intent().active &&
 mission.field_intent().acknowledgement !=
 mission::FieldIntentAcknowledgement::Rejected) {
 mission.step();
 }
 } else if (!requested_goal_sequence.empty()) {
 std::size_t next_goal = 0;
 while (mission.snapshot().status == mission::MissionStatus::Running) {
 if (!mission.active_goal()) {
 if (next_goal >= requested_goal_sequence.size())
 throw std::runtime_error("--goal-sequence ended before the mission completed");
 const std::string& requested = requested_goal_sequence[next_goal++];
 const auto& goals = mission.level().objective.goals;
 const auto found = std::find_if(goals.begin(), goals.end(), [&](const auto& goal) {
 return (goal.id.empty() ? goal.marker : goal.id) == requested;
 });
 if (found == goals.end() ||
 !mission.select_goal(static_cast<std::size_t>(std::distance(goals.begin(), found)))) {
 throw std::runtime_error("goal is unknown, ineligible, or has no feasible route: " + requested);
 }
 }
 mission.step();
 }
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
 {"maneuver", mission.active_maneuver_id()},
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
     {"mujoco_available", sim::physics::mujoco_physics_available()},
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
     {"slip_events", result.slip_events},
 {"braking",
 {
 {"valid", result.braking_measurement_valid},
 {"start_speed_mps", result.last_stop_start_speed_mps},
 {"stop_time_s", result.last_stop_time_s},
 {"stop_distance_m", result.last_stop_distance_m},
 }},
     {"astar_expanded", result.astar_expanded},
     {"local_decisions", result.local_decisions},
 {"local_planner",
 {
 {"target", {mission.snapshot().control.local_target.x,
 mission.snapshot().control.local_target.y}},
 {"accepted_candidates", mission.snapshot().control.dwa_accepted_candidates},
 {"trajectory_validated", mission.snapshot().control.trajectory_validated},
 {"trajectory_verdict", mission.snapshot().control.trajectory_verdict},
 {"trajectory_diagnostic", mission.snapshot().control.trajectory_diagnostic},
 {"wheel_speed_mps", {mission.snapshot().control.left_measured_mps,
 mission.snapshot().control.right_measured_mps}},
 {"governor_hold", mission.snapshot().control.governor_hold},
 {"crossing_relevant", mission.snapshot().control.crossing_assessed},
 {"crossing_phase", mission.snapshot().control.crossing_phase},
{"crossing_reason", mission.snapshot().control.crossing_reason},
{"crossing_ready_to_commit", mission.snapshot().control.crossing_ready_to_commit},
{"crossing_safe_to_commit", mission.snapshot().control.crossing_safe_to_commit},
{"crossing_position_ready", mission.snapshot().control.crossing_position_ready},
{"crossing_speed_ready", mission.snapshot().control.crossing_speed_ready},
{"crossing_heading_ready", mission.snapshot().control.crossing_heading_ready},
{"crossing_prediction_evidence_available",
 mission.snapshot().control.crossing_prediction_evidence_available},
{"crossing_threats_moving_away",
 mission.snapshot().control.crossing_threats_moving_away},
{"crossing_execution_duration_s",
 mission.snapshot().control.crossing_execution_duration_s},
{"crossing_execution_min_gap_m",
 mission.snapshot().control.crossing_execution_min_gap_m},
{"crossing_execution_required_gap_m",
 mission.snapshot().control.crossing_execution_required_gap_m},
 }},
     {"robot",
      {
        {"x", mission.snapshot().robot.x},
        {"y", mission.snapshot().robot.y},
        {"z", mission.snapshot().robot_height_m},
        {"heading", mission.snapshot().robot.theta},
 {"linear_mps", mission.snapshot().robot.v},
 {"yaw_rate_radps", mission.snapshot().robot.w},
 {"estimated_forward_mps", mission.snapshot().estimated_forward_mps},
 {"estimated_lateral_mps", mission.snapshot().estimated_lateral_mps},
 {"estimated_yaw_rate_radps", mission.snapshot().estimated_yaw_rate_radps},
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
 {"operator_intent_completed",
 mission.field_intent().acknowledgement ==
 mission::FieldIntentAcknowledgement::Completed},
 }},
     {"via_reached", result.via_reached},
 {"tasks",
 {
 {"goal_order", mission.level().objective.goal_order},
 {"policy", mission.level().objective.task_policy},
 {"requested_sequence", requested_goal_sequence},
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
 if (mission.snapshot().manipulator.present) {
 const auto& arm = mission.snapshot().manipulator;
 output["manipulator"] = {
 {"target_reachable", arm.target_reachable},
 {"target_joint_rad", arm.target_joint_rad},
 {"measured_joint_rad", arm.measured_joint_rad},
 {"measured_velocity_radps", arm.measured_velocity_radps},
 {"target_position_m", {arm.target_pose.x_m, arm.target_pose.y_m, arm.target_pose.z_m}},
 {"measured_position_m", {arm.measured_pose.x_m, arm.measured_pose.y_m, arm.measured_pose.z_m}},
 {"position_error_m", arm.position_error_m},
 {"maximum_joint_speed_radps", result.max_manipulator_joint_speed_radps},
 };
 }
 if (mission.committed_construction()) {
 std::vector<std::string> piece_ids;
 for (const auto& piece : mission.committed_construction()->pieces)
 piece_ids.push_back(piece.placement.piece_id);
 output["construction"] = {
 {"plan_sha256", mission.committed_construction()->plan_sha256},
 {"map_revision", mission.map_revision()},
 {"piece_ids", piece_ids},
  {"contact_observed", result.construction_contact_observed},
  {"surface_traversed", result.construction_surface_traversed},
 };
 }
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
 const bool operator_intent_completed = field_intent && !endless &&
 mission.field_intent().acknowledgement ==
 mission::FieldIntentAcknowledgement::Completed;
 return endless || operator_intent_completed ||
 result.status == mission::MissionStatus::Succeeded ? 0 : 2;
 } catch (const std::exception& error) {
  std::cerr << "error: " << error.what() << '\n';
  return 1;
 }
}
