#pragma once

#include <cstdint>
#include <optional>
#include <string>

#include "sim/eval/episode.hpp"
#include "sim/eval/scoring.hpp"
#include "sim/scenario/scenario_session.hpp"
#include "sim/viewer/experience_flow.hpp"
#include "sim/viewer/experience_manifest.hpp"
#include "sim/viewer/keyframe_review.hpp"

namespace lost_signal::sim::experiment {

// Frozen contract for one controlled comparison. Values come from the
// validated manifest plus the actual baseline runtime before either Episode
// is captured. The digest identifies every execution input except the one
// declared variable.
struct ExperimentSpec {
 std::string experience_id;
 std::string experiment_id;
 viewer::ExperienceVariable controlled_variable =
 viewer::ExperienceVariable::kLidarScanPeriodS;
 double baseline_value = 0.0;
 double variant_value = 0.0;
 std::string baseline_value_label;
 std::string variant_value_label;
 std::uint64_t baseline_step_budget = 0;
 std::uint64_t variant_step_budget = 0;
 std::uint64_t input_digest = 0;
 std::string code_revision;
 double minimum_elapsed_increase_s = 0.0;
 double minimum_clearance_loss_m = 0.0;
 double equivalent_elapsed_tolerance_s = 0.0;
 double equivalent_clearance_tolerance_m = 0.0;

 bool valid() const;
};

std::optional<ExperimentSpec> make_experiment_spec(
 const viewer::ExperienceManifest& experience, double baseline_value,
 std::uint64_t input_digest, const std::string& code_revision,
 std::string* error = nullptr);

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
 std::string controlled_value_label;
 std::uint64_t invariant_digest = 0;
 std::uint64_t step_budget = 0;
 viewer::KeyframeReviewConfig review_config;
 eval::EpisodeRecorder episode;
 eval::EpisodeScore score;
 bool timed_out = false;
 bool objectives_done = false;
 bool objectives_passed = false;
 bool robot_verdict_passed = false;
 bool mission_verdict_passed = false;
 bool simulation_valid = false;

 bool safety_passed() const;
 bool mission_passed() const;
};

EpisodeArtifact capture_episode_artifact(
 const std::string& condition_id,
 const viewer::ExperienceVariableSpec* variable,
 double controlled_value, std::uint64_t invariant_digest,
 const std::string& code_revision,
 viewer::KeyframeReviewConfig review_config, std::uint64_t step_budget,
 bool timed_out,
 const scenario::ScenarioSession& session,
 const std::string& controlled_value_label = {});

struct EpisodePair {
 std::optional<ExperimentSpec> spec;
 std::optional<EpisodeArtifact> baseline;
 std::optional<EpisodeArtifact> variant;

 bool complete() const {
 return baseline.has_value() && variant.has_value();
 }

 bool experiment_valid() const;
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

// Writes a reproducible v2 comparison report and one v4 observation replay
// for each run. The JSON report keeps the frozen spec, input/code identity,
// four public verdicts, scores, and causal keyframes; replay streams use the
// ordinary ROBOT provider boundary.
bool write_episode_pair_bundle(const std::string& report_path,
 const EpisodePair& pair, std::string* error = nullptr);

struct ExperienceGateVerdict {
 // Evidence validity is an internal precondition. The four public verdicts
 // below keep ROBOT, physical safety, mission acceptance, and experiment
 // integrity separate.
 bool evidence_valid = false;
 bool robot_passed = false;
 bool sim_safety_passed = false;
 bool mission_passed = false;
 bool experiment_passed = false;
 bool can_advance = false;
};

// Safety and simulation validity always gate progression. Mission and
// controlled-comparison completion follow the chapter policy.
ExperienceGateVerdict evaluate_experience_gate(
 const viewer::ExperienceProgressionPolicy& policy,
 const EpisodeArtifact& active, const EpisodePair& pair);

// Builds prediction inputs only from captured Episode artifacts and the
// frozen experiment spec. It never reads a live ScenarioSession.
viewer::ExperienceEvidence episode_evidence(
 const EpisodeArtifact& active, const EpisodePair& pair);

} // namespace lost_signal::sim::experiment
