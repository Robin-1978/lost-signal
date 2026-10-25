#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "sim/eval/episode.hpp"

namespace lost_signal::sim::viewer {

enum class ReviewKeyframeKind {
 kFirstObservation,
 kBrakeCommand,
 kStableStop,
};

struct ReviewKeyframe {
 ReviewKeyframeKind kind = ReviewKeyframeKind::kFirstObservation;
 std::size_t episode_index = 0;
 std::uint64_t step = 0;
 robot::core::MonotonicNs stamp_ns = robot::core::kInvalidTime;
 std::string title;
 std::string evidence;
};

struct KeyframeReviewConfig {
 // Use the deployed arrival policy's speed limits and settle count. The
 // review must not invent a second definition of physical standstill.
 double maximum_forward_speed_mps = 0.08;
 double maximum_yaw_rate_radps = 0.08;
 int settle_samples = 3;
};

// Extracts presentation-only bookmarks from a completed episode. It reads
// recorded observations, ROBOT estimates, commands, and evidence; it never
// mutates or republishes them and never participates in a live decision.
std::vector<ReviewKeyframe> extract_review_keyframes(
 const eval::EpisodeRecorder& episode,
 KeyframeReviewConfig config = {});

} // namespace lost_signal::sim::viewer
