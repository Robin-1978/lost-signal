#include <cassert>
#include <string>

#include "sim/experiment/experience_runtime.hpp"
#include "sim/experiment/episode_pair.hpp"
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
 config.objectives = experience.objectives;
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

 // A completed run is copied into an immutable review artifact before a
 // second runtime replaces the live session. The four progression verdicts
 // remain independent: SIM validity, safety, mission, and experiment.
 auto baseline_artifact = sim::experiment::capture_episode_artifact(
 "baseline", &sim::viewer::experience_variable_spec(
 sim::viewer::ExperienceVariable::kLidarScanPeriodS),
 first.sim_config->scan_period_s, first.invariant_digest, false,
 *first.session);
 assert(baseline_artifact.simulation_valid);
 assert(baseline_artifact.safety_passed());
 assert(baseline_artifact.mission_passed());
 assert(baseline_artifact.episode.size() == first_score.steps);
 sim::experiment::EpisodePair pair;
 pair.baseline = baseline_artifact;
 auto waiting = sim::experiment::evaluate_experience_gate(
 experience.progression, *pair.baseline, pair);
 assert(waiting.simulation_valid == false);
 assert(!waiting.experiment_passed);
 pair.variant = sim::experiment::capture_episode_artifact(
 "variant", &sim::viewer::experience_variable_spec(
 sim::viewer::ExperienceVariable::kLidarScanPeriodS),
 variant.sim_config->scan_period_s, variant.invariant_digest, false,
 *first.session);
 auto complete = sim::experiment::evaluate_experience_gate(
 experience.progression, *pair.variant, pair);
 assert(complete.simulation_valid);
 assert(complete.safety_passed);
 assert(complete.mission_passed);
 assert(complete.experiment_passed);
 assert(complete.can_advance);

 auto mismatched_pair = pair;
 ++mismatched_pair.variant->invariant_digest;
 const auto mismatched = sim::experiment::evaluate_experience_gate(
 experience.progression, *mismatched_pair.variant, mismatched_pair);
 assert(mismatched.simulation_valid);
 assert(mismatched.safety_passed);
 assert(mismatched.mission_passed);
 assert(!mismatched.experiment_passed);
 assert(!mismatched.can_advance);

 auto safe_stop_lesson = baseline_artifact;
 safe_stop_lesson.timed_out = true;
 safe_stop_lesson.objectives_done = false;
 safe_stop_lesson.objectives_passed = false;
 sim::viewer::ExperienceProgressionPolicy safety_policy;
 safety_policy.require_mission = false;
 safety_policy.require_experiment = false;
 const auto safety_gate = sim::experiment::evaluate_experience_gate(
 safety_policy, safe_stop_lesson, {});
 assert(safety_gate.simulation_valid);
 assert(safety_gate.safety_passed);
 assert(!safety_gate.mission_passed);
 assert(safety_gate.can_advance);
 return 0;
}
