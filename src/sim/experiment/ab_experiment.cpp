#include "sim/experiment/ab_experiment.hpp"

#include <cmath>
#include <cstring>
#include <fstream>
#include <memory>

#include "robot/core/json.hpp"
#include "robot/platform/robot_config.hpp"
#include "sim/eval/robot_config.hpp"
#include "sim/eval/sim_config.hpp"
#include "sim/eval/world_io.hpp"
#include "sim/scenario/actor_sidecar.hpp"
#include "sim/scenario/scenario_session.hpp"
#include "sim/viewer/experience_manifest.hpp"

namespace lost_signal::sim::experiment {
namespace {

void hash_bytes(std::uint64_t& hash, const void* data, std::size_t size) {
 const auto* bytes = static_cast<const unsigned char*>(data);
 for (std::size_t i = 0; i < size; ++i) {
 hash ^= bytes[i];
 hash *= 1099511628211ULL;
 }
}

void hash_command(std::uint64_t& hash,
 const robot::iface::MotionCommand& command) {
 const int space = static_cast<int>(command.space);
 hash_bytes(hash, &space, sizeof(space));
 hash_bytes(hash, &command.stamp_ns, sizeof(command.stamp_ns));
 hash_bytes(hash, &command.expiry_ns, sizeof(command.expiry_ns));
 hash_bytes(hash, &command.forward_mps, sizeof(command.forward_mps));
 hash_bytes(hash, &command.lateral_mps, sizeof(command.lateral_mps));
 hash_bytes(hash, &command.yaw_rate_radps, sizeof(command.yaw_rate_radps));
 hash_bytes(hash, &command.steer_angle_rad,
 sizeof(command.steer_angle_rad));
}

bool hash_file(std::uint64_t& hash, const std::string& path) {
 std::ifstream input(path, std::ios::binary);
 if (!input.is_open()) return false;
 char chunk[4096];
 while (input.good()) {
 input.read(chunk, sizeof(chunk));
 const std::streamsize count = input.gcount();
 if (count > 0) hash_bytes(hash, chunk, static_cast<std::size_t>(count));
 }
 return input.eof();
}

double seconds(robot::core::MonotonicNs stamp_ns) {
 return stamp_ns == robot::core::kInvalidTime
 ? -1.0 : static_cast<double>(stamp_ns) * 1e-9;
}

void capture_event(ExperimentEvent& event, const eval::EvalStep& step,
 robot::core::MonotonicNs stamp_ns) {
 if (event.observed) return;
 event.observed = true;
 event.step = step.step;
 event.stamp_s = seconds(stamp_ns);
}

bool run_variant(const viewer::ExperienceManifest& experience,
 double scan_period_s, long step_budget, VariantResult& result,
 std::string& error) {
 robot::platform::RobotConfig robot_config;
 if (!robot::platform::load_robot_config(
 experience.robot_path, robot_config, &error)) return false;
 eval::SimConfig sim_config;
 if (!eval::load_sim_config(experience.sim_path, sim_config, &error)) {
 return false;
 }
 sim_config.scan_period_s = scan_period_s;
 scenario::ScenarioSession session(
 robot::platform::description_from_config(robot_config),
 eval::motor_from_config(robot_config));
 session.apply_sim_config(sim_config);
 session.set_episode_history_enabled(false);

 eval::WorldFileContent world;
 if (!eval::load_world_file(experience.map_path, world, &error)) return false;
 eval::apply_world_content(session.setup_world(), world);
 if (world.initial_pose.has_value()) {
 const auto& prior = *world.initial_pose;
 session.set_initial_pose(prior.x_m, prior.y_m, prior.yaw_rad,
 prior.position_sigma_m, prior.heading_sigma_rad);
 }
 session.set_objectives(experience.objectives);
 session.fit_map_window_to_world();
 if (scenario::load_actors_sidecar(
 experience.map_path, session, {}, &error) < 0) return false;

 std::uint64_t actor_seed_digest = 1469598103934665603ULL;
 for (const auto& actor : session.setup_world().actors()) {
 hash_bytes(actor_seed_digest, actor.id.data(), actor.id.size());
 hash_bytes(actor_seed_digest, &actor.behavior.seed,
 sizeof(actor.behavior.seed));
 }

 std::uint64_t command_digest = 1469598103934665603ULL;
 std::uint64_t config_digest = 1469598103934665603ULL;
 if (!hash_file(config_digest, experience.robot_path) ||
 !hash_file(config_digest, experience.sim_path) ||
 !hash_file(config_digest, experience.map_path)) {
 error = "cannot hash experiment configuration";
 return false;
 }
 for (const auto& objective : experience.objectives) {
 hash_bytes(config_digest, objective.data(), objective.size());
 }
 hash_bytes(config_digest, &scan_period_s, sizeof(scan_period_s));
 ExperimentEvent first_scan;
 ExperimentEvent first_brake;
 ExperimentEvent stable_stop;
 bool have_motion_command = false;
 int stable_samples = 0;
 for (long step = 0; step < step_budget && !session.objectives_done(); ++step) {
 session.step_once();
 const eval::EvalStep* recorded = session.last_eval_step();
 if (recorded == nullptr) continue;
 hash_command(command_digest, recorded->cmd);
 if (!first_scan.observed && recorded->obs.scan_valid) {
 const auto header = robot::iface::scan_measurement_header(recorded->obs);
 if (header.valid()) capture_event(first_scan, *recorded, header.stamp_ns);
 }
 if (!recorded->cmd.is_safe_stop()) have_motion_command = true;
 if (have_motion_command && recorded->cmd.is_safe_stop()) {
 capture_event(first_brake, *recorded, recorded->cmd.stamp_ns);
 }
 const bool settled = first_brake.observed &&
 recorded->diag.has_observation &&
 std::abs(recorded->diag.estimate.forward_mps) <=
 robot_config.arrival.maximum_forward_speed_mps &&
 std::abs(recorded->diag.estimate.yaw_rate_radps) <=
 robot_config.arrival.maximum_yaw_rate_radps;
 stable_samples = settled ? stable_samples + 1 : 0;
 if (stable_samples >= robot_config.arrival.settle_cycles) {
 capture_event(stable_stop, *recorded,
 recorded->diag.estimate.stamp_ns);
 }
 }
 const eval::EvalStep* final_step = session.last_eval_step();
 if (final_step == nullptr) {
 error = "experiment produced no recorded step";
 return false;
 }
 result.scan_period_s = scan_period_s;
 result.delivery_seed = sim_config.delivery.seed;
 result.imu_seed = sim_config.imu_realism.seed;
 result.actor_seed_digest = actor_seed_digest;
 result.config_digest = config_digest;
 result.command_digest = command_digest;
 result.first_scan = first_scan;
 result.first_brake = first_brake;
 result.stable_stop = stable_stop;
 result.objectives_done = session.objectives_done();
 result.objectives_passed = session.objectives_passed();
 result.score = session.score();
 result.final_truth = final_step->truth_after;
 return true;
}

robot::core::Json variant_json(const VariantResult& result) {
 using robot::core::Json;
 Json::Object score;
 score.emplace_back("steps", Json::Num(static_cast<double>(result.score.steps)));
 score.emplace_back("elapsed_s", Json::Num(result.score.elapsed_s));
 score.emplace_back("collision_steps",
 Json::Num(static_cast<double>(result.score.collision_steps)));
 score.emplace_back("path_length_m", Json::Num(result.score.path_length_m));
 score.emplace_back("min_clearance_m", Json::Num(result.score.min_clearance_m));
 score.emplace_back("mean_pos_err_m", Json::Num(result.score.mean_pos_err_m));
 score.emplace_back("max_pos_err_m", Json::Num(result.score.max_pos_err_m));
 score.emplace_back("energy_proxy", Json::Num(result.score.energy_proxy));
 Json::Object out;
 const auto event_json = [](const ExperimentEvent& event) {
 Json::Object object;
 object.emplace_back("observed", Json::Bool(event.observed));
 object.emplace_back("step", Json::Num(static_cast<double>(event.step)));
 object.emplace_back("stamp_s", Json::Num(event.stamp_s));
 return Json::Obj(std::move(object));
 };
 out.emplace_back("scan_period_s", Json::Num(result.scan_period_s));
 out.emplace_back("delivery_seed",
 Json::Num(static_cast<double>(result.delivery_seed)));
 out.emplace_back("imu_seed", Json::Num(static_cast<double>(result.imu_seed)));
 out.emplace_back("actor_seed_digest",
 Json::Str(std::to_string(result.actor_seed_digest)));
 out.emplace_back("config_digest",
 Json::Str(std::to_string(result.config_digest)));
 out.emplace_back("command_digest",
 Json::Str(std::to_string(result.command_digest)));
 out.emplace_back("first_scan", event_json(result.first_scan));
 out.emplace_back("first_brake", event_json(result.first_brake));
 out.emplace_back("stable_stop", event_json(result.stable_stop));
 out.emplace_back("objectives_done", Json::Bool(result.objectives_done));
 out.emplace_back("objectives_passed", Json::Bool(result.objectives_passed));
 out.emplace_back("score", Json::Obj(std::move(score)));
 return Json::Obj(std::move(out));
}

} // namespace

bool run_scan_period_comparison(const std::string& experience_path,
 double variant_scan_period_s, long step_budget,
 ScanPeriodComparison& comparison, std::string* error) {
 std::string local_error;
 std::string& out_error = error != nullptr ? *error : local_error;
 out_error.clear();
 if (!std::isfinite(variant_scan_period_s) ||
 !(variant_scan_period_s > 0.0) || step_budget <= 0) {
 out_error = "scan period and step budget must be positive";
 return false;
 }
 viewer::ExperienceManifest experience;
 if (!viewer::load_experience_manifest(
 experience_path, experience, &out_error)) return false;
 eval::SimConfig sim_config;
 if (!eval::load_sim_config(experience.sim_path, sim_config, &out_error)) {
 return false;
 }
 ScanPeriodComparison result;
 result.experience_id = experience.experience_id;
 if (!run_variant(experience, sim_config.scan_period_s, step_budget,
 result.baseline, out_error) ||
 !run_variant(experience, variant_scan_period_s, step_budget,
 result.variant, out_error)) {
 return false;
 }
 if (result.baseline.delivery_seed != result.variant.delivery_seed ||
 result.baseline.imu_seed != result.variant.imu_seed ||
 result.baseline.actor_seed_digest != result.variant.actor_seed_digest) {
 out_error = "experiment seed mismatch";
 return false;
 }
 comparison = std::move(result);
 return true;
}

bool write_scan_period_comparison(const std::string& path,
 const ScanPeriodComparison& comparison, std::string* error) {
 using robot::core::Json;
 Json::Object root;
 root.emplace_back("format", Json::Str("lost-signal-ab/1"));
 root.emplace_back("experience_id", Json::Str(comparison.experience_id));
 root.emplace_back("controlled_variable", Json::Str("lidar.scan_period_s"));
 root.emplace_back("baseline", variant_json(comparison.baseline));
 root.emplace_back("variant", variant_json(comparison.variant));
 std::ofstream output(path, std::ios::binary | std::ios::trunc);
 if (!output.is_open()) {
 if (error != nullptr) *error = "cannot open comparison output: " + path;
 return false;
 }
 output << robot::core::json_dump(Json::Obj(std::move(root))) << '\n';
 output.flush();
 if (!output.good()) {
 if (error != nullptr) *error = "cannot write comparison output: " + path;
 return false;
 }
 if (error != nullptr) error->clear();
 return true;
}

} // namespace lost_signal::sim::experiment
