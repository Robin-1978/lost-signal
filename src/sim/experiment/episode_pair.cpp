#include "sim/experiment/episode_pair.hpp"

#include <cmath>

namespace lost_signal::sim::experiment {

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
 double controlled_value, bool timed_out,
 const scenario::ScenarioSession& session) {
 EpisodeArtifact artifact;
 artifact.condition_id = condition_id;
 artifact.controlled_variable = variable != nullptr ? variable->id : "";
 artifact.controlled_value = controlled_value;
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

} // namespace lost_signal::sim::experiment
