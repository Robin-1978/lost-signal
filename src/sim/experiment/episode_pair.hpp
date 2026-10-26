#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "sim/eval/episode.hpp"
#include "sim/eval/scoring.hpp"
#include "sim/scenario/scenario_session.hpp"
#include "sim/viewer/experience_manifest.hpp"
#include "sim/viewer/keyframe_review.hpp"

namespace lost_signal::sim::experiment {

// Immutable-by-convention completed run artifact. The full recorded episode
// is copied before the next runtime is constructed, so baseline and variant
// can be reviewed, aligned, hashed, or exported without reading live state.
struct EpisodeArtifact {
 std::string condition_id;
 std::string controlled_variable;
 std::string code_revision;
 std::string map_id;
 std::uint64_t map_revision = 0;
 double controlled_value = 0.0;
 std::uint64_t invariant_digest = 0;
 viewer::KeyframeReviewConfig review_config;
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
 double controlled_value, std::uint64_t invariant_digest,
 const std::string& code_revision,
 viewer::KeyframeReviewConfig review_config, bool timed_out,
 const scenario::ScenarioSession& session);

struct EpisodePair {
 std::optional<EpisodeArtifact> baseline;
 std::optional<EpisodeArtifact> variant;

 bool complete() const {
 return baseline.has_value() && variant.has_value();
 }
};

struct SynchronizedEpisodeStep {
 std::size_t baseline_index = 0;
 std::size_t variant_index = 0;
 robot::core::MonotonicNs baseline_stamp_ns = robot::core::kInvalidTime;
 robot::core::MonotonicNs variant_stamp_ns = robot::core::kInvalidTime;
};

// Aligns the two immutable recordings by simulation/acquisition time. The
// selected reference index remains exact; the other run chooses its nearest
// recorded step. Wall time, render rate, and playback rate never participate.
std::optional<SynchronizedEpisodeStep> synchronize_episode_pair_by_time(
 const EpisodePair& pair, bool reference_is_variant,
 std::size_t reference_index);

// Writes a reproducible comparison report and one v4 observation replay for
// each run. The JSON report keeps typed inputs, input/code identity, scores,
// verdicts, and causal keyframes; the replay streams can be fed back through
// the ordinary ROBOT provider boundary.
bool write_episode_pair_bundle(const std::string& report_path,
 const EpisodePair& pair, std::string* error = nullptr);

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
