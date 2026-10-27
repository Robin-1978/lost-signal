#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "robot/platform/robot_config.hpp"
#include "sim/eval/robot_config.hpp"
#include "sim/eval/sim_config.hpp"
#include "sim/eval/world_io.hpp"
#include "sim/scenario/actor_sidecar.hpp"
#include "sim/scenario/scenario_session.hpp"

int main(int argc, char** argv) {
 using namespace lost_signal;
 const std::string config_dir = TEST_CONFIG_DIR;
 const std::string robot_path = config_dir + "/robot/ackermann.json";
 const std::string sim_path = config_dir + "/sim/default.json";
 const std::string map_path = config_dir + "/worlds/industrial_200k.json";

 robot::platform::RobotConfig robot_config;
 sim::eval::SimConfig sim_config;
 sim::eval::WorldFileContent world;
 std::string error;
 if (!robot::platform::load_robot_config(
 robot_path, robot_config, &error) ||
 !sim::eval::load_sim_config(sim_path, sim_config, &error) ||
 !sim::eval::load_world_file(map_path, world, &error)) {
 std::fprintf(stderr, "industrial_200k_mission: setup failed: %s\n",
 error.c_str());
 return 1;
 }
 std::fprintf(stderr,
 "ackermann config: wheelbase=%.3f steer=%.3f footprint=%.3f "
 "clearance=%.3f soft=%.3f\n",
 robot_config.profile.wheel_base_m, robot_config.limits.max_steer_rad,
 robot_config.limits.footprint_radius_m,
 robot_config.navigation.clearance_margin_m,
 robot_config.navigation.soft_inflation_radius_m);

 sim::scenario::ScenarioSession session(
 robot::platform::description_from_config(robot_config),
 sim::eval::motor_from_config(robot_config));
 session.apply_sim_config(sim_config);
 session.set_episode_history_enabled(false);
 sim::eval::apply_world_content(session.setup_world(), world);
 if (world.initial_pose.has_value()) {
 const auto& prior = *world.initial_pose;
 session.set_initial_pose(
 prior.x_m, prior.y_m, prior.yaw_rad,
 prior.position_sigma_m, prior.heading_sigma_rad);
 }
 session.fit_map_window_to_world();
 if (sim::scenario::load_actors_sidecar(
 map_path, session, {}, &error) != 4) {
 std::fprintf(stderr,
 "industrial_200k_mission: actor setup failed: %s\n", error.c_str());
 return 1;
 }

 std::vector<std::string> mission{
 "west_hub", "north_depot", "central_gate",
 "east_terminal", "south_yard"};
 bool trace = false;
 int mission_step_budget = 100'000;
 int trace_interval = 1000;
 std::vector<std::string> requested_places;
 std::string previous_localization_action;
 for (int index = 1; index < argc; ++index) {
 const std::string argument = argv[index];
 if (argument == "--trace") {
 trace = true;
 } else if (argument.rfind("--steps=", 0) == 0) {
 const long parsed = std::strtol(argument.c_str() + 8, nullptr, 10);
 if (parsed > 0 && parsed <= 10'000'000) {
 mission_step_budget = static_cast<int>(parsed);
 }
 } else if (argument.rfind("--trace-interval=", 0) == 0) {
 const long parsed = std::strtol(argument.c_str() + 17, nullptr, 10);
 if (parsed > 0 && parsed <= 1'000'000) {
 trace_interval = static_cast<int>(parsed);
 }
 } else {
 requested_places.push_back(argument);
 }
 }
 if (!requested_places.empty()) mission = std::move(requested_places);
 session.set_objectives(mission, 0.5);
 int steps = 0;
 std::string previous_reason;
 while (steps < mission_step_budget && !session.objectives_done()) {
 session.step_once();
 ++steps;
 if (trace && steps % trace_interval == 0) {
 if (const auto* sample = session.last_eval_step()) {
 const auto& estimate = sample->diag.estimate;
 const auto& odom = sample->diag.odometry_estimate;
 const auto& evidence = sample->diag.evidence;
 std::fprintf(stderr,
 " step=%d objective=%s truth=(%.2f,%.2f,%.2f) "
 "estimate=(%.2f,%.2f,%.2f) odom=(%.2f,%.2f,%.2f) "
        "v=%.2f sigma=(%.3f,%.3f) loc=%s slip=%s/%.2f grip=%.2f "
 "track=(%.3f,%.3f,%s) cmd=(%.3f,%.3f)->(%.3f,%.3f) "
 "clear=(validator=%.3f,dynamic=%.3f,safety=%.3f/%.3f/%.3f) "
 "dynamic-gate=(%.3f/%.3f) tracks=%zu "
 "route=(%zu/%zu g=%d err=%.3f move=%zu cusp=%s/%s/%.3f) "
 "decision=%s/%s/%s\n",
 steps, session.objectives().active_place().c_str(),
 sample->truth_after.body_x_m, sample->truth_after.body_y_m,
 sample->truth_after.body_yaw_rad, estimate.x_m, estimate.y_m,
 estimate.yaw_rad, odom.x_m, odom.y_m, odom.yaw_rad,
 sample->cmd.forward_mps,
        estimate.position_sigma_m, estimate.heading_sigma_rad,
        sample->diag.localization_action.c_str(),
        sample->diag.slip_quality.c_str(), sample->diag.slip_confidence,
        sample->diag.grip_factor, sample->diag.track_correction_m,
        sample->diag.track_correction_rad,
        sample->diag.track_diagnostic.c_str(),
 sample->cmd.forward_mps, sample->cmd.yaw_rate_radps,
 evidence.proposed_speed_mps, evidence.proposed_yaw_rate_radps,
 evidence.validator_static_clearance_m,
 evidence.predicted_separation_m,
 evidence.safety_hold_clearance_m,
 evidence.safety_command_clearance_m,
 evidence.safety_terminal_clearance_m,
 evidence.safety_dynamic_hold_clearance_m,
 evidence.safety_dynamic_command_clearance_m,
 sample->diag.dynamic_tracks.size(),
 evidence.route_segment, evidence.route_point_count,
 evidence.requested_gear, evidence.route_error_m,
 evidence.moving_candidate_count,
 evidence.gear_change_pending ? "pending" : "none",
 evidence.gear_change_braking_armed ? "armed" : "approach",
 evidence.gear_change_endpoint_distance_m,
 evidence.constraint_code.c_str(), evidence.reason_code.c_str(),
 evidence.action_code.c_str());
 for (const auto& track : sample->diag.dynamic_tracks) {
 std::fprintf(stderr,
 "  track id=%u p=(%.3f,%.3f) v=(%.3f,%.3f) "
 "r=%.3f u=%.3f ready=%s age=%.2f\n",
 track.id, track.x_m, track.y_m, track.vx_mps, track.vy_mps,
 track.radius_m, track.position_uncertainty_m,
 track.prediction_ready ? "yes" : "no", track.age_s);
 }
 // SIM truth is emitted only by this offline trace harness so a failed
 // scenario can distinguish actor-policy deadlock from ROBOT perception.
 // It never enters an observation, estimate, planner, or safety decision.
 for (const auto& actor : session.setup_world().actors()) {
 std::fprintf(stderr,
 " sim_actor id=%s p=(%.3f,%.3f) v=(%.3f,%.3f) yield=%.3f\n",
 actor.id.c_str(), actor.x, actor.y, actor.vx_mps, actor.vy_mps,
 actor.behavior.yield_speed_mps_);
 }
 }
 }
 if (trace) {
 if (const auto* sample = session.last_eval_step()) {
 const std::string& action = sample->diag.localization_action;
 if (action != previous_localization_action &&
 (action.find("recovery") != std::string::npos ||
 action.find("global_relocalization") != std::string::npos)) {
 const auto& estimate = sample->diag.estimate;
 std::fprintf(stderr,
 " localization_event step=%d objective=%s truth=(%.3f,%.3f,%.3f) "
 "estimate=(%.3f,%.3f,%.3f) sigma=(%.3f,%.3f) %s\n",
 steps, session.objectives().active_place().c_str(),
 sample->truth_after.body_x_m, sample->truth_after.body_y_m,
 sample->truth_after.body_yaw_rad, estimate.x_m, estimate.y_m,
 estimate.yaw_rad, estimate.position_sigma_m,
 estimate.heading_sigma_rad,
 sample->diag.localization_action.c_str());
 previous_localization_action = action;
 }
 if (sample->diag.localization_action.rfind(
 "global_relocalization:accepted", 0) == 0) {
 const auto& estimate = sample->diag.estimate;
 std::fprintf(stderr,
 " recovery step=%d truth=(%.3f,%.3f,%.3f) "
 "estimate=(%.3f,%.3f,%.3f) sigma=(%.3f,%.3f) %s\n",
 steps, sample->truth_after.body_x_m, sample->truth_after.body_y_m,
 sample->truth_after.body_yaw_rad, estimate.x_m, estimate.y_m,
 estimate.yaw_rad, estimate.position_sigma_m,
 estimate.heading_sigma_rad,
 sample->diag.localization_action.c_str());
 }
 if (sample->diag.evidence.reason_code != previous_reason) {
 previous_reason = sample->diag.evidence.reason_code;
 if (previous_reason == "global-plan-failed") {
 const auto& estimate = sample->diag.estimate;
 std::fprintf(stderr,
 " planning_failure step=%d objective=%s "
 "estimate=(%.9f,%.9f,%.9f) sigma=(%.3f,%.3f)\n",
 steps, session.objectives().active_place().c_str(), estimate.x_m,
 estimate.y_m, estimate.yaw_rad, estimate.position_sigma_m,
 estimate.heading_sigma_rad);
 }
 }
 }
 }
 }

 const auto score = session.score();
 std::fprintf(stderr,
 "industrial_200k_mission: steps=%d successful=%zu/%zu "
 "active=%s collisions=%zu distance=%.2f max_pos_err=%.3f\n",
 steps, session.objectives().successful(), session.objectives().total(),
 session.objectives().active_place().c_str(), score.collision_steps,
 score.distance_m, score.max_pos_err_m);
 for (const auto& result : session.objectives().results()) {
 std::fprintf(stderr,
 " objective=%s robot_done=%s truth_arrived=%s miss=%.3f\n",
 result.place_id.c_str(), result.robot_done ? "yes" : "no",
 result.truth_arrived ? "yes" : "no", result.truth_miss_m);
 }
 if (const auto* last = session.last_eval_step()) {
 const auto& estimate = last->diag.estimate;
 const auto& evidence = last->diag.evidence;
 std::fprintf(stderr,
 " final truth=(%.3f,%.3f,%.3f) estimate=(%.3f,%.3f,%.3f) "
 "cmd=(%.3f,%.3f) safety=%s decision=%s/%s/%s\n",
 last->truth_after.body_x_m, last->truth_after.body_y_m,
 last->truth_after.body_yaw_rad, estimate.x_m, estimate.y_m,
 estimate.yaw_rad, last->cmd.forward_mps, last->cmd.yaw_rate_radps,
 last->diag.safety_reason.c_str(), evidence.constraint_code.c_str(),
 evidence.reason_code.c_str(), evidence.action_code.c_str());
 }
 const bool passed = session.objectives_done() &&
 session.objectives_passed() &&
 session.objectives().successful() == mission.size() &&
 score.collision_steps == 0;
 return passed ? 0 : 1;
}
