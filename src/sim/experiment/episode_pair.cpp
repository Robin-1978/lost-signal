#include "sim/experiment/episode_pair.hpp"

#include <algorithm>
#include <cmath>

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
 double controlled_value, std::uint64_t invariant_digest, bool timed_out,
 const scenario::ScenarioSession& session) {
 EpisodeArtifact artifact;
 artifact.condition_id = condition_id;
 artifact.controlled_variable = variable != nullptr ? variable->id : "";
 artifact.controlled_value = controlled_value;
 artifact.invariant_digest = invariant_digest;
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

} // namespace lost_signal::sim::experiment
