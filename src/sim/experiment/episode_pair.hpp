#pragma once

#include <optional>
#include <string>

#include "sim/eval/episode.hpp"
#include "sim/eval/scoring.hpp"
#include "sim/scenario/scenario_session.hpp"
#include "sim/viewer/experience_manifest.hpp"

namespace lost_signal::sim::experiment {

// Immutable-by-convention completed run artifact. The full recorded episode
// is copied before the next runtime is constructed, so baseline and variant
// can be reviewed, aligned, hashed, or exported without reading live state.
struct EpisodeArtifact {
 std::string condition_id;
 std::string controlled_variable;
 double controlled_value = 0.0;
 eval::EpisodeRecorder episode;
 eval::EpisodeScore score;
 bool timed_out = false;
 bool objectives_done = false;
 bool objectives_passed = false;
 bool simulation_valid = false;

 bool safety_passed() const;
 bool mission_passed() const;
};

EpisodeArtifact capture_episode_artifact(
 const std::string& condition_id,
 const viewer::ExperienceVariableSpec* variable,
 double controlled_value, bool timed_out,
 const scenario::ScenarioSession& session);

struct EpisodePair {
 std::optional<EpisodeArtifact> baseline;
 std::optional<EpisodeArtifact> variant;

 bool complete() const {
 return baseline.has_value() && variant.has_value();
 }
};

struct ExperienceGateVerdict {
 bool simulation_valid = false;
 bool safety_passed = false;
 bool mission_passed = false;
 bool experiment_passed = false;
 bool can_advance = false;
};

// Safety and simulation validity always gate progression. Mission and
// controlled-comparison completion follow the chapter policy.
ExperienceGateVerdict evaluate_experience_gate(
 const viewer::ExperienceProgressionPolicy& policy,
 const EpisodeArtifact& active, const EpisodePair& pair);

} // namespace lost_signal::sim::experiment
