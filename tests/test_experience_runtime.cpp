#include <cassert>
#include <cstdio>
#include <fstream>
#include <string>
#include <utility>

#include "sim/experiment/experience_runtime.hpp"
#include "sim/experiment/episode_pair.hpp"
#include "sim/providers/replay.hpp"
#include "sim/viewer/experience_manifest.hpp"

int main() {
 using namespace lost_signal;
 sim::viewer::ExperienceManifest experience;
 std::string error;
 assert(sim::viewer::load_experience_manifest(
 std::string(TEST_SOURCE_DIR) + "/configs/experiences/first_contact.json",
 experience, &error));

 sim::experiment::ExperienceRuntimeConfig config;
 config.robot_path = experience.robot_path;
 config.sim_path = experience.sim_path;
 config.map_path = experience.map_path;
 config.scene_path = experience.scene_path;
 config.asset_manifest_path =
 std::string(TEST_SOURCE_DIR) + "/assets/models/manifest.json";
 config.objectives = experience.objectives;
 config.startup_mode = experience.startup_mode;
 const auto manifest_config = sim::experiment::experience_runtime_config(
 experience, config.asset_manifest_path);
 assert(manifest_config.robot_path == experience.robot_path);
 assert(manifest_config.objectives == experience.objectives);
 assert(manifest_config.startup_mode == experience.startup_mode);
 assert(!manifest_config.has_assembly);
 sim::viewer::ExperienceManifest slam_experience;
 assert(sim::viewer::load_experience_manifest(
 std::string(TEST_SOURCE_DIR) + "/configs/experiences/slam_loop.json",
 slam_experience, &error));
 const auto slam_config = sim::experiment::experience_runtime_config(
 slam_experience, config.asset_manifest_path);
 assert(slam_config.startup_mode ==
 sim::viewer::ExperienceStartupMode::kSlamOnly);
 assert(slam_config.objectives == slam_experience.objectives);
 sim::experiment::ExperienceRuntime first;
 sim::experiment::ExperienceRuntime retry;
 assert(sim::experiment::build_experience_runtime(config, first, &error));
 assert(sim::experiment::build_experience_runtime(config, retry, &error));
 assert(first.session->episode().steps().empty());
 assert(retry.session->episode().steps().empty());
 assert(first.session->score().steps == 0);
 assert(retry.session->score().steps == 0);
 assert(first.session->setup_world().truth().step == 0);
 assert(retry.session->setup_world().truth().step == 0);
 assert(first.invariant_digest != 0);
 assert(first.invariant_digest == retry.invariant_digest);
 assert(!first.code_revision.empty());
 assert(first.code_revision == retry.code_revision);

 // A chapter assembly is applied before the first model build.  It changes
 // typed wheel/SIM device parameters and the invariant digest together; it
 // never reads or publishes simulator truth.
 auto assembly_config = config;
 assembly_config.has_assembly = true;
 assembly_config.assembly.wheel = "robots.ro1.wheel_grip";
 assembly_config.assembly.lidar = "robots.ro1.lidar_fast";
 assembly_config.assembly.compute = "robots.ro1.compute_performance";
 sim::experiment::ExperienceRuntime assembly_runtime;
 assert(sim::experiment::build_experience_runtime(
     assembly_config, assembly_runtime, &error));
 assert(assembly_runtime.robot_config.has_value());
 assert(assembly_runtime.sim_config.has_value());
 assert(assembly_runtime.robot_config->profile.tire_friction_mu == 1.2);
 assert(assembly_runtime.robot_config->profile.rolling_resist == 0.014);
 assert(assembly_runtime.sim_config->lidar_max_range_m == 5.0f);
 assert(assembly_runtime.sim_config->lidar_beams == 48);
 assert(assembly_runtime.sim_config->scan_period_s == 0.05);
 assert(assembly_runtime.sim_config->decision_period_s == 0.10);
 assert(assembly_runtime.invariant_digest != first.invariant_digest);

 auto changed_startup_config = config;
 changed_startup_config.startup_mode =
 sim::viewer::ExperienceStartupMode::kSlamOnly;
 sim::experiment::ExperienceRuntime changed_startup;
 assert(sim::experiment::build_experience_runtime(
 changed_startup_config, changed_startup, &error));
 assert(changed_startup.invariant_digest != first.invariant_digest);

 const std::string changed_scene_path =
 "/tmp/lost_signal_experience_changed_scene.json";
 {
 std::ofstream changed_scene(changed_scene_path,
 std::ios::binary | std::ios::trunc);
 changed_scene << "{\"scene_id\":\"changed-for-digest-test\"}\n";
 }
 auto changed_scene_config = config;
 changed_scene_config.scene_path = changed_scene_path;
 sim::experiment::ExperienceRuntime changed_scene;
 assert(sim::experiment::build_experience_runtime(
 changed_scene_config, changed_scene, &error));
 assert(changed_scene.invariant_digest != first.invariant_digest);
 std::remove(changed_scene_path.c_str());

 for (std::uint64_t transition = 0;
 transition < experience.time_limit_steps &&
 (!first.session->objectives_done() ||
 !retry.session->objectives_done());
 ++transition) {
 assert(first.session->objectives_done() ==
 retry.session->objectives_done());
 first.session->step_once();
 retry.session->step_once();
 const auto* lhs = first.session->last_eval_step();
 const auto* rhs = retry.session->last_eval_step();
 assert(lhs != nullptr && rhs != nullptr);
 assert(lhs->step == rhs->step);
 assert(lhs->obs.sequence == rhs->obs.sequence);
 assert(lhs->obs.stamp_ns == rhs->obs.stamp_ns);
 assert(lhs->obs.ranges_m == rhs->obs.ranges_m);
 assert(lhs->obs.scan_header.source == rhs->obs.scan_header.source);
 assert(lhs->obs.motion_header.source == rhs->obs.motion_header.source);
 assert(lhs->obs.imu_header.source == rhs->obs.imu_header.source);
 assert(!lhs->obs.scan_static_valid);
 assert(lhs->obs.static_ranges_m.empty());
 assert(lhs->cmd.space == rhs->cmd.space);
 assert(lhs->cmd.forward_mps == rhs->cmd.forward_mps);
 assert(lhs->cmd.lateral_mps == rhs->cmd.lateral_mps);
 assert(lhs->cmd.yaw_rate_radps == rhs->cmd.yaw_rate_radps);
 assert(lhs->truth_after.body_x_m == rhs->truth_after.body_x_m);
 assert(lhs->truth_after.body_y_m == rhs->truth_after.body_y_m);
 assert(lhs->truth_after.body_yaw_rad == rhs->truth_after.body_yaw_rad);
 assert(lhs->truth_after.drive_angle_rad ==
 rhs->truth_after.drive_angle_rad);
 assert(lhs->diag.estimate.x_m == rhs->diag.estimate.x_m);
 assert(lhs->diag.estimate.y_m == rhs->diag.estimate.y_m);
 assert(lhs->diag.evidence.action_code == rhs->diag.evidence.action_code);
 assert(lhs->judge.static_collision == rhs->judge.static_collision);
 assert(lhs->judge.dynamic_collision == rhs->judge.dynamic_collision);
 }

 assert(first.session->objectives_done());
 assert(retry.session->objectives_done());
 assert(first.session->objectives_passed() ==
 retry.session->objectives_passed());
 const auto first_score = first.session->score();
 const auto retry_score = retry.session->score();
 assert(first_score.steps == retry_score.steps);
 assert(first_score.elapsed_s == retry_score.elapsed_s);
 assert(first_score.collision_steps == retry_score.collision_steps);
 assert(first_score.path_length_m == retry_score.path_length_m);
 assert(first_score.min_clearance_m == retry_score.min_clearance_m);
 assert(first_score.mean_pos_err_m == retry_score.mean_pos_err_m);

 // A real Retry is constructed after the old run has completed. Its first
 // transition must still match the original first transition, proving that
 // clocks, physics, sensors, ROBOT state, episode history, and seeded actor
 // streams were replaced rather than partially cleared in place.
 sim::experiment::ExperienceRuntime later_retry;
 assert(sim::experiment::build_experience_runtime(
 config, later_retry, &error));
 assert(later_retry.session->episode().steps().empty());
 assert(later_retry.session->score().steps == 0);
 assert(later_retry.session->setup_world().truth().step == 0);
 later_retry.session->step_once();
 const auto& original_first = first.session->episode().steps().front();
 const auto* rebuilt_first = later_retry.session->last_eval_step();
 assert(rebuilt_first != nullptr);
 assert(original_first.obs.sequence == rebuilt_first->obs.sequence);
 assert(original_first.obs.stamp_ns == rebuilt_first->obs.stamp_ns);
 assert(original_first.obs.ranges_m == rebuilt_first->obs.ranges_m);
 assert(original_first.cmd.space == rebuilt_first->cmd.space);
 assert(original_first.cmd.stamp_ns == rebuilt_first->cmd.stamp_ns);
 assert(original_first.cmd.forward_mps == rebuilt_first->cmd.forward_mps);
 assert(original_first.cmd.yaw_rate_radps ==
 rebuilt_first->cmd.yaw_rate_radps);
 assert(original_first.truth_after.body_x_m ==
 rebuilt_first->truth_after.body_x_m);
 assert(original_first.truth_after.body_y_m ==
 rebuilt_first->truth_after.body_y_m);
 assert(original_first.truth_after.body_yaw_rad ==
 rebuilt_first->truth_after.body_yaw_rad);
 const auto rebuilt_score = later_retry.session->score();
 assert(rebuilt_score.steps == 1);
 assert(rebuilt_score.elapsed_s == 0.02);
 assert(rebuilt_score.collision_steps == 0);

 // A visible controlled variant is a complete fresh runtime. Only its scan
 // cadence changes; motion and IMU cadence, seeds, world, and zero-time plant
 // remain authored and deterministic.
 sim::experiment::ExperienceRuntimeConfig variant_config = config;
 variant_config.variable_override = sim::experiment::ExperienceVariableOverride{
 sim::viewer::ExperienceVariable::kLidarScanPeriodS, 0.10};
 sim::experiment::ExperienceRuntime variant;
 assert(sim::experiment::build_experience_runtime(
 variant_config, variant, &error));
 assert(variant.sim_config.has_value());
 assert(variant.sim_config->scan_period_s == 0.10);
 assert(first.sim_config.has_value());
 assert(variant.sim_config->motion_period_s ==
 first.sim_config->motion_period_s);
 assert(variant.sim_config->imu_period_s == first.sim_config->imu_period_s);
 assert(variant.session->setup_world().truth().step == 0);
 assert(variant.session->setup_world().actors().size() ==
 first.session->setup_world().actors().size());
 assert(variant.invariant_digest == first.invariant_digest);

 variant_config.variable_override = sim::experiment::ExperienceVariableOverride{
 sim::viewer::ExperienceVariable::kLidarScanPeriodS, 0.0105};
 assert(!sim::experiment::build_experience_runtime(
 variant_config, variant, &error));
 assert(error == "experiment override is outside its typed contract");

 // Categorical chassis experiments use an explicit profile name and path.
 // The variant plant changes, while the invariant digest remains identical
 // because the chassis is the declared controlled variable.
 sim::viewer::ExperienceManifest chassis_experience;
 assert(sim::viewer::load_experience_manifest(
     std::string(TEST_SOURCE_DIR) + "/configs/experiences/chassis_lab.json",
     chassis_experience, &error));
 auto chassis_config = sim::experiment::experience_runtime_config(
     chassis_experience, config.asset_manifest_path);
 sim::experiment::ExperienceRuntime chassis_baseline;
 assert(sim::experiment::build_experience_runtime(
     chassis_config, chassis_baseline, &error));
 auto chassis_variant_config = chassis_config;
 const auto& chassis_definition = chassis_experience.experiments.front();
 chassis_variant_config.variable_override =
     sim::experiment::ExperienceVariableOverride{
         chassis_definition.controlled_variable, 0.0,
         chassis_definition.variant_category,
         chassis_definition.variant_robot_path};
 sim::experiment::ExperienceRuntime chassis_variant;
 assert(sim::experiment::build_experience_runtime(
     chassis_variant_config, chassis_variant, &error));
 assert(chassis_baseline.robot_config->profile.chassis_kind ==
        "differential");
 assert(chassis_variant.robot_config->profile.chassis_kind == "ackermann");
 assert(chassis_baseline.invariant_digest == chassis_variant.invariant_digest);
 assert(chassis_variant.session->setup_world().truth().step == 0);
 for (std::uint64_t step = 0;
      step < chassis_experience.time_limit_steps &&
      (!chassis_baseline.session->objectives_done() ||
       !chassis_variant.session->objectives_done());
      ++step) {
   if (!chassis_baseline.session->objectives_done()) {
     chassis_baseline.session->step_once();
   }
   if (!chassis_variant.session->objectives_done()) {
     chassis_variant.session->step_once();
   }
 }
 assert(chassis_baseline.session->objectives_done());
 assert(chassis_variant.session->objectives_done());
 sim::viewer::KeyframeReviewConfig chassis_review;
 auto chassis_baseline_artifact = sim::experiment::capture_episode_artifact(
     "baseline", &sim::viewer::experience_variable_spec(
         sim::viewer::ExperienceVariable::kChassisKind), 0.0,
     chassis_baseline.invariant_digest, chassis_baseline.code_revision,
     chassis_review, chassis_experience.time_limit_steps, false,
     *chassis_baseline.session, "differential");
 auto chassis_variant_artifact = sim::experiment::capture_episode_artifact(
     "variant", &sim::viewer::experience_variable_spec(
         sim::viewer::ExperienceVariable::kChassisKind), 0.0,
     chassis_variant.invariant_digest, chassis_variant.code_revision,
     chassis_review,
     static_cast<std::uint64_t>(chassis_definition.step_budget), false,
     *chassis_variant.session, "ackermann");
 sim::experiment::EpisodePair chassis_pair;
 chassis_pair.spec = sim::experiment::make_experiment_spec(
     chassis_experience, 0.0, chassis_baseline.invariant_digest,
     chassis_baseline.code_revision, &error);
 assert(chassis_pair.spec.has_value());
 chassis_pair.baseline = std::move(chassis_baseline_artifact);
 chassis_pair.variant = std::move(chassis_variant_artifact);
 assert(chassis_pair.experiment_valid());

 // A completed run is copied into an immutable review artifact before a
 // second runtime replaces the live session. The four progression verdicts
 // remain independent: SIM validity, safety, mission, and experiment.
 sim::viewer::KeyframeReviewConfig review_config;
 review_config.maximum_forward_speed_mps =
 first.robot_config->arrival.maximum_forward_speed_mps;
 review_config.maximum_yaw_rate_radps =
 first.robot_config->arrival.maximum_yaw_rate_radps;
 review_config.settle_samples = first.robot_config->arrival.settle_cycles;
 auto baseline_artifact = sim::experiment::capture_episode_artifact(
 "baseline", &sim::viewer::experience_variable_spec(
 sim::viewer::ExperienceVariable::kLidarScanPeriodS),
 first.sim_config->scan_period_s, first.invariant_digest,
 first.code_revision, review_config, experience.time_limit_steps,
 false, *first.session);
 assert(baseline_artifact.simulation_valid);
 assert(baseline_artifact.robot_verdict_passed);
 assert(baseline_artifact.safety_passed());
 assert(baseline_artifact.mission_passed());
 assert(baseline_artifact.episode.size() == first_score.steps);
 sim::experiment::EpisodePair pair;
 pair.spec = sim::experiment::make_experiment_spec(
 experience, first.sim_config->scan_period_s, first.invariant_digest,
 first.code_revision, &error);
 assert(pair.spec.has_value());
 pair.baseline = baseline_artifact;
 auto waiting = sim::experiment::evaluate_experience_gate(
 experience.progression, *pair.baseline, pair);
 assert(waiting.evidence_valid);
 assert(waiting.robot_passed);
 assert(waiting.sim_safety_passed);
 assert(waiting.mission_passed);
 assert(!waiting.experiment_passed);
 assert(!waiting.can_advance);
 pair.variant = sim::experiment::capture_episode_artifact(
 "variant", &sim::viewer::experience_variable_spec(
 sim::viewer::ExperienceVariable::kLidarScanPeriodS),
 variant.sim_config->scan_period_s, variant.invariant_digest,
 variant.code_revision, review_config,
 static_cast<std::uint64_t>(experience.experiments.front().step_budget),
 false, *first.session);
 auto complete = sim::experiment::evaluate_experience_gate(
 experience.progression, *pair.variant, pair);
 assert(pair.experiment_valid());
 assert(complete.evidence_valid);
 assert(complete.robot_passed);
 assert(complete.sim_safety_passed);
 assert(complete.mission_passed);
 assert(complete.experiment_passed);
 assert(complete.can_advance);
 const std::string report_path = "/tmp/lost_signal_episode_pair_test.json";
 const std::string baseline_replay_path =
 report_path + ".baseline.observations.jsonl";
 const std::string variant_replay_path =
 report_path + ".variant.observations.jsonl";
 assert(sim::experiment::write_episode_pair_bundle(
 report_path, pair, &error));
 std::ifstream report_input(report_path, std::ios::binary);
 const std::string report_text(
 (std::istreambuf_iterator<char>(report_input)),
 std::istreambuf_iterator<char>());
 assert(report_text.find("lost-signal-episode-pair/2") !=
 std::string::npos);
 assert(report_text.find("\"experiment_spec\"") != std::string::npos);
 assert(report_text.find("\"robot_passed\":true") != std::string::npos);
 assert(report_text.find("\"sim_safety_passed\":true") !=
 std::string::npos);
 assert(report_text.find(first.code_revision) != std::string::npos);
 sim::providers::ReplayProvider baseline_replay;
 sim::providers::ReplayProvider variant_replay;
 assert(baseline_replay.load(baseline_replay_path, &error));
 assert(variant_replay.load(variant_replay_path, &error));
 assert(baseline_replay.size() == pair.baseline->episode.size());
 assert(variant_replay.size() == pair.variant->episode.size());
 std::remove(report_path.c_str());
 std::remove(baseline_replay_path.c_str());
 std::remove(variant_replay_path.c_str());
 const auto synchronized =
 sim::experiment::synchronize_episode_pair_by_time(
 pair, /*reference_is_variant=*/true,
 pair.variant->episode.size() / 2);
 assert(synchronized.has_value());
 assert(synchronized->variant_index == pair.variant->episode.size() / 2);
 assert(synchronized->baseline_stamp_ns == synchronized->variant_stamp_ns);

 // Pair synchronization follows acquisition time rather than record index.
 // This matters when a controlled variable changes a sensor cadence or one
 // run contains additional recorded controller transitions.
 sim::experiment::EpisodePair uneven_pair;
 uneven_pair.baseline = baseline_artifact;
 uneven_pair.variant = *pair.variant;
 uneven_pair.baseline->episode.clear();
 uneven_pair.variant->episode.clear();
 for (const robot::core::MonotonicNs stamp_ns : {100, 200, 300}) {
 sim::eval::EvalStep step;
 step.obs.stamp_ns = stamp_ns;
 uneven_pair.baseline->episode.record(std::move(step));
 }
 for (const robot::core::MonotonicNs stamp_ns : {90, 170, 290, 410}) {
 sim::eval::EvalStep step;
 step.obs.stamp_ns = stamp_ns;
 uneven_pair.variant->episode.record(std::move(step));
 }
 const auto nearest_baseline =
 sim::experiment::synchronize_episode_pair_by_time(
 uneven_pair, /*reference_is_variant=*/true, 1);
 assert(nearest_baseline.has_value());
 assert(nearest_baseline->variant_index == 1);
 assert(nearest_baseline->baseline_index == 1);
 assert(nearest_baseline->variant_stamp_ns == 170);
 assert(nearest_baseline->baseline_stamp_ns == 200);
 const auto nearest_variant =
 sim::experiment::synchronize_episode_pair_by_time(
 uneven_pair, /*reference_is_variant=*/false, 2);
 assert(nearest_variant.has_value());
 assert(nearest_variant->baseline_index == 2);
 assert(nearest_variant->variant_index == 2);

 const auto immutable_evidence = sim::experiment::episode_evidence(
 *pair.variant, pair);
 assert(immutable_evidence.terminal);
 assert(immutable_evidence.comparison_complete);
 assert(immutable_evidence.comparison_valid);
 assert(immutable_evidence.all_robot_done);
 assert(immutable_evidence.all_sim_accepted);

 auto mismatched_pair = pair;
 ++mismatched_pair.variant->invariant_digest;
 const auto mismatched = sim::experiment::evaluate_experience_gate(
 experience.progression, *mismatched_pair.variant, mismatched_pair);
 assert(mismatched.evidence_valid);
 assert(mismatched.robot_passed);
 assert(mismatched.sim_safety_passed);
 assert(mismatched.mission_passed);
 assert(!mismatched.experiment_passed);
 assert(!mismatched.can_advance);
 auto wrong_budget_pair = pair;
 ++wrong_budget_pair.variant->step_budget;
 assert(!wrong_budget_pair.experiment_valid());
 auto wrong_condition_pair = pair;
 wrong_condition_pair.variant->condition_id = "baseline";
 assert(!wrong_condition_pair.experiment_valid());
 auto missing_spec_pair = pair;
 missing_spec_pair.spec.reset();
 assert(!missing_spec_pair.experiment_valid());
 assert(!sim::experiment::write_episode_pair_bundle(
 report_path, missing_spec_pair, &error));
 assert(error == "episode pair has no experiment spec");

 auto safe_stop_lesson = baseline_artifact;
 safe_stop_lesson.timed_out = true;
 safe_stop_lesson.objectives_done = false;
 safe_stop_lesson.objectives_passed = false;
 sim::viewer::ExperienceProgressionPolicy safety_policy;
 safety_policy.require_mission = false;
 safety_policy.require_experiment = false;
 const auto safety_gate = sim::experiment::evaluate_experience_gate(
 safety_policy, safe_stop_lesson, {});
 assert(safety_gate.evidence_valid);
 assert(safety_gate.robot_passed);
 assert(safety_gate.sim_safety_passed);
 assert(!safety_gate.mission_passed);
 assert(safety_gate.can_advance);
 return 0;
}
