#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include "sim/eval/episode.hpp"

namespace lost_signal::sim::viewer {

enum class ReviewKeyframeKind {
 kFirstObservation,
 kFirstMotion,
 kBrakeCommand,
 kStableStop,
 kRobotDone,
 kCollision,
 kTimeout,
 kSimAccepted,
 kSimRejected,
};

struct ReviewKeyframe {
 ReviewKeyframeKind kind = ReviewKeyframeKind::kFirstObservation;
 std::size_t episode_index = 0;
 std::uint64_t step = 0;
 robot::core::MonotonicNs stamp_ns = robot::core::kInvalidTime;
 std::string title;
 std::string evidence;
};

struct ReviewKeyframeAlignment {
 std::size_t baseline_keyframe_index = 0;
 std::size_t variant_keyframe_index = 0;
};

// Canonical teaching chain for the debrief drawer. Missing stages stay empty:
// the presentation must never manufacture a ROBOT observation, command, goal
// verdict, or SIM acceptance merely to make a successful-looking timeline.
struct ReviewCausalChain {
 std::optional<std::size_t> first_observation;
 std::optional<std::size_t> braking_command;
 std::optional<std::size_t> stable_stop;
 std::optional<std::size_t> robot_done;
 std::optional<std::size_t> sim_verdict;
};

ReviewCausalChain build_review_causal_chain(
 const std::vector<ReviewKeyframe>& keyframes);

// Matches causal milestones across completed runs. Accepted and rejected SIM
// verdicts belong to the same terminal event class, so a failed variant can
// still be compared with a successful baseline at the moment each judge made
// its decision.
std::optional<ReviewKeyframeAlignment> align_review_keyframes_by_event(
 const std::vector<ReviewKeyframe>& baseline,
 const std::vector<ReviewKeyframe>& variant,
 ReviewKeyframeKind selected_variant_kind);

struct KeyframeReviewConfig {
 // Use the deployed arrival policy's speed limits and settle count. The
 // review must not invent a second definition of physical standstill.
 double maximum_forward_speed_mps = 0.08;
 double maximum_yaw_rate_radps = 0.08;
 int settle_samples = 3;
};

// Terminal context is assembled by SIM presentation after the run. It is
// deliberately separate from EpisodeStep ROBOT diagnostics: objective truth,
// collision, and timeout verdicts can be reviewed but cannot enter a live
// RobotApplication decision.
struct KeyframeReviewOutcome {
 bool terminal = false;
 bool timed_out = false;
 bool sim_accepted = false;
 std::vector<std::string> rejection_reasons;
};

// Extracts presentation-only bookmarks from a completed episode. It reads
// recorded observations, ROBOT estimates, commands, and evidence; it never
// mutates or republishes them and never participates in a live decision.
std::vector<ReviewKeyframe> extract_review_keyframes(
 const eval::EpisodeRecorder& episode,
 KeyframeReviewConfig config = {},
 KeyframeReviewOutcome outcome = {});

} // namespace lost_signal::sim::viewer
