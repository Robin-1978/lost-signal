#include "sim/experiment/episode_pair.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>

#include "robot/core/json.hpp"
#include "sim/providers/replay.hpp"
#include "sim/viewer/keyframe_review.hpp"
namespace lost_signal::sim::experiment {
namespace {

robot::core::MonotonicNs step_stamp(const eval::EvalStep& step) {
 if (step.obs.stamp_ns != robot::core::kInvalidTime) return step.obs.stamp_ns;
 if (step.diag.estimate.stamp_ns != robot::core::kInvalidTime) {
 return step.diag.estimate.stamp_ns;
 }
 return step.cmd.stamp_ns;
}

std::size_t nearest_step(const eval::EpisodeRecorder& episode,
 robot::core::MonotonicNs stamp_ns) {
 const auto& steps = episode.steps();
 if (steps.empty()) return 0;
 const auto found = std::lower_bound(
 steps.begin(), steps.end(), stamp_ns,
 [](const eval::EvalStep& step, robot::core::MonotonicNs stamp) {
 return step_stamp(step) < stamp;
 });
 if (found == steps.begin()) return 0;
 if (found == steps.end()) return steps.size() - 1;
 const auto previous = std::prev(found);
 const auto before = stamp_ns - step_stamp(*previous);
 const auto after = step_stamp(*found) - stamp_ns;
 return static_cast<std::size_t>(
 (before <= after ? previous : found) - steps.begin());
}

robot::core::Json score_json(const eval::EpisodeScore& score) {
 using robot::core::Json;
 Json::Object object;
 object.emplace_back("steps", Json::Num(static_cast<double>(score.steps)));
 object.emplace_back("elapsed_s", Json::Num(score.elapsed_s));
 object.emplace_back("collisions",
 Json::Num(static_cast<double>(score.collision_steps)));
 object.emplace_back("terrain_collisions",
 Json::Num(static_cast<double>(score.terrain_collision_steps)));
 object.emplace_back("distance_m", Json::Num(score.distance_m));
 object.emplace_back("path_length_m", Json::Num(score.path_length_m));
 object.emplace_back("min_clearance_m", Json::Num(score.min_clearance_m));
 object.emplace_back("mean_pos_err_m", Json::Num(score.mean_pos_err_m));
 object.emplace_back("max_pos_err_m", Json::Num(score.max_pos_err_m));
 object.emplace_back("energy_proxy", Json::Num(score.energy_proxy));
 return Json::Obj(std::move(object));
}

viewer::KeyframeReviewOutcome review_outcome(const EpisodeArtifact& artifact) {
 viewer::KeyframeReviewOutcome outcome;
 outcome.terminal = true;
 outcome.timed_out = artifact.timed_out;
 outcome.sim_accepted = artifact.safety_passed() && artifact.mission_passed();
 if (artifact.timed_out) {
 outcome.rejection_reasons.push_back("experiment_time_limit");
 }
 if (!artifact.objectives_done || !artifact.objectives_passed) {
 outcome.rejection_reasons.push_back("mission_rejected");
 }
 if (!artifact.safety_passed()) {
 outcome.rejection_reasons.push_back("safety_rejected");
 }
 return outcome;
}

robot::core::Json keyframes_json(const EpisodeArtifact& artifact) {
 using robot::core::Json;
 const auto keyframes = viewer::extract_review_keyframes(
 artifact.episode, artifact.review_config, review_outcome(artifact));
 Json::Array array;
 array.reserve(keyframes.size());
 for (const auto& keyframe : keyframes) {
 Json::Object object;
 object.emplace_back("kind", Json::Num(static_cast<int>(keyframe.kind)));
 object.emplace_back("episode_index",
 Json::Num(static_cast<double>(keyframe.episode_index)));
 object.emplace_back("step", Json::Num(static_cast<double>(keyframe.step)));
 object.emplace_back("stamp_ns",
 Json::Num(static_cast<double>(keyframe.stamp_ns)));
 object.emplace_back("title", Json::Str(keyframe.title));
 object.emplace_back("evidence", Json::Str(keyframe.evidence));
 array.push_back(Json::Obj(std::move(object)));
 }
 return Json::Arr(std::move(array));
}

robot::core::Json artifact_json(const EpisodeArtifact& artifact,
 const std::string& replay_path) {
 using robot::core::Json;
 Json::Object object;
 object.emplace_back("condition_id", Json::Str(artifact.condition_id));
 object.emplace_back("controlled_variable",
 Json::Str(artifact.controlled_variable));
 object.emplace_back("controlled_value",
 Json::Num(artifact.controlled_value));
 object.emplace_back("invariant_digest",
 Json::Str(std::to_string(artifact.invariant_digest)));
 object.emplace_back("code_revision", Json::Str(artifact.code_revision));
 object.emplace_back("map_id", Json::Str(artifact.map_id));
 object.emplace_back("map_revision",
 Json::Str(std::to_string(artifact.map_revision)));
 object.emplace_back("observation_replay", Json::Str(replay_path));
 object.emplace_back("recorded_steps",
 Json::Num(static_cast<double>(artifact.episode.size())));
 object.emplace_back("simulation_valid",
 Json::Bool(artifact.simulation_valid));
 object.emplace_back("safety_passed", Json::Bool(artifact.safety_passed()));
 object.emplace_back("mission_passed", Json::Bool(artifact.mission_passed()));
 object.emplace_back("timed_out", Json::Bool(artifact.timed_out));
 object.emplace_back("objectives_done", Json::Bool(artifact.objectives_done));
 object.emplace_back("objectives_passed",
 Json::Bool(artifact.objectives_passed));
 object.emplace_back("score", score_json(artifact.score));
 object.emplace_back("keyframes", keyframes_json(artifact));
 return Json::Obj(std::move(object));
}

} // namespace

bool EpisodeArtifact::safety_passed() const {
 return simulation_valid && score.collision_steps == 0 &&
 score.terrain_collision_steps == 0;
}

bool EpisodeArtifact::mission_passed() const {
 return simulation_valid && !timed_out && objectives_done &&
 objectives_passed;
}

EpisodeArtifact capture_episode_artifact(
 const std::string& condition_id,
 const viewer::ExperienceVariableSpec* variable,
 double controlled_value, std::uint64_t invariant_digest,
 const std::string& code_revision,
 viewer::KeyframeReviewConfig review_config, bool timed_out,
 const scenario::ScenarioSession& session) {
 EpisodeArtifact artifact;
 artifact.condition_id = condition_id;
 artifact.controlled_variable = variable != nullptr ? variable->id : "";
 artifact.code_revision = code_revision;
 const auto& active_map = session.active_map();
 artifact.map_id = active_map.map_id;
 artifact.map_revision = active_map.revision;
 artifact.controlled_value = controlled_value;
 artifact.invariant_digest = invariant_digest;
 artifact.review_config = review_config;
 artifact.episode = session.episode();
 artifact.score = session.score();
 artifact.timed_out = timed_out;
 artifact.objectives_done = session.objectives_done();
 artifact.objectives_passed = session.objectives_passed();
 artifact.simulation_valid = artifact.episode.complete() &&
 artifact.score.steps > 0 &&
 artifact.episode.total_recorded() == artifact.score.steps &&
 artifact.episode.size() == artifact.score.steps &&
 std::isfinite(artifact.score.elapsed_s) &&
 std::isfinite(artifact.score.path_length_m) &&
 std::isfinite(artifact.score.mean_pos_err_m);
 return artifact;
}

ExperienceGateVerdict evaluate_experience_gate(
 const viewer::ExperienceProgressionPolicy& policy,
 const EpisodeArtifact& active, const EpisodePair& pair) {
 ExperienceGateVerdict verdict;
 if (policy.require_experiment) {
 if (!pair.complete()) return verdict;
 const auto& baseline = *pair.baseline;
 const auto& variant = *pair.variant;
 verdict.experiment_passed = !baseline.controlled_variable.empty() &&
 baseline.controlled_variable == variant.controlled_variable &&
 baseline.invariant_digest != 0 &&
 baseline.invariant_digest == variant.invariant_digest &&
 std::isfinite(baseline.controlled_value) &&
 std::isfinite(variant.controlled_value) &&
 baseline.controlled_value != variant.controlled_value;
 verdict.simulation_valid = baseline.simulation_valid &&
 variant.simulation_valid;
 verdict.safety_passed = baseline.safety_passed() &&
 variant.safety_passed();
 verdict.mission_passed = baseline.mission_passed() &&
 variant.mission_passed();
 } else {
 verdict.experiment_passed = true;
 verdict.simulation_valid = active.simulation_valid;
 verdict.safety_passed = active.safety_passed();
 verdict.mission_passed = active.mission_passed();
 }
 verdict.can_advance = verdict.simulation_valid && verdict.safety_passed &&
 verdict.experiment_passed &&
 (!policy.require_mission || verdict.mission_passed);
 return verdict;
}

std::optional<SynchronizedEpisodeStep> synchronize_episode_pair_by_time(
 const EpisodePair& pair, bool reference_is_variant,
 std::size_t reference_index) {
 if (!pair.complete()) return std::nullopt;
 const auto& baseline = pair.baseline->episode;
 const auto& variant = pair.variant->episode;
 if (baseline.steps().empty() || variant.steps().empty()) return std::nullopt;
 const auto& reference = reference_is_variant ? variant : baseline;
 if (reference_index >= reference.steps().size()) return std::nullopt;
 const robot::core::MonotonicNs stamp_ns =
 step_stamp(reference.steps()[reference_index]);
 if (stamp_ns == robot::core::kInvalidTime) return std::nullopt;
 SynchronizedEpisodeStep synchronized;
 if (reference_is_variant) {
 synchronized.variant_index = reference_index;
 synchronized.baseline_index = nearest_step(baseline, stamp_ns);
 } else {
 synchronized.baseline_index = reference_index;
 synchronized.variant_index = nearest_step(variant, stamp_ns);
 }
 synchronized.baseline_stamp_ns =
 step_stamp(baseline.steps()[synchronized.baseline_index]);
 synchronized.variant_stamp_ns =
 step_stamp(variant.steps()[synchronized.variant_index]);
 return synchronized;
}

bool write_episode_pair_bundle(const std::string& report_path,
 const EpisodePair& pair, std::string* error) {
 std::string local_error;
 std::string& out_error = error != nullptr ? *error : local_error;
 out_error.clear();
 if (!pair.complete()) {
 out_error = "episode pair is incomplete";
 return false;
 }
 if (report_path.empty()) {
 out_error = "episode pair report path is empty";
 return false;
 }
 const std::filesystem::path report(report_path);
 std::error_code filesystem_error;
 if (!report.parent_path().empty()) {
 std::filesystem::create_directories(report.parent_path(), filesystem_error);
 if (filesystem_error) {
 out_error = "cannot create experiment output directory: " +
 filesystem_error.message();
 return false;
 }
 }
 const std::filesystem::path baseline_path =
 std::filesystem::path(report_path + ".baseline.observations.jsonl");
 const std::filesystem::path variant_path =
 std::filesystem::path(report_path + ".variant.observations.jsonl");
 if (!providers::export_episode(
 baseline_path.string(), pair.baseline->map_id,
 pair.baseline->map_revision, pair.baseline->episode, &out_error) ||
 !providers::export_episode(
 variant_path.string(), pair.variant->map_id,
 pair.variant->map_revision, pair.variant->episode, &out_error)) {
 return false;
 }

 using robot::core::Json;
 Json::Object root;
 root.emplace_back("format", Json::Str("lost-signal-episode-pair/1"));
 root.emplace_back("baseline", artifact_json(
 *pair.baseline, baseline_path.filename().string()));
 root.emplace_back("variant", artifact_json(
 *pair.variant, variant_path.filename().string()));
 const bool invariant_match = pair.baseline->invariant_digest != 0 &&
 pair.baseline->invariant_digest == pair.variant->invariant_digest;
 root.emplace_back("invariant_match", Json::Bool(invariant_match));
 root.emplace_back("controlled_variable_match", Json::Bool(
 pair.baseline->controlled_variable == pair.variant->controlled_variable));
 const bool experiment_passed = invariant_match &&
 !pair.baseline->controlled_variable.empty() &&
 pair.baseline->controlled_variable == pair.variant->controlled_variable &&
 std::isfinite(pair.baseline->controlled_value) &&
 std::isfinite(pair.variant->controlled_value) &&
 pair.baseline->controlled_value != pair.variant->controlled_value;
 root.emplace_back("experiment_passed", Json::Bool(experiment_passed));
 std::ofstream output(report, std::ios::binary | std::ios::trunc);
 if (!output.is_open()) {
 out_error = "cannot open experiment report: " + report.string();
 return false;
 }
 output << robot::core::json_dump(Json::Obj(std::move(root))) << '\n';
 output.flush();
 if (!output.good()) {
 out_error = "cannot write experiment report: " + report.string();
 return false;
 }
 return true;
}

} // namespace lost_signal::sim::experiment
