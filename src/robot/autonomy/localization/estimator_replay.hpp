#pragma once

#include "robot/autonomy/planning/local/predictor.hpp"
#include "robot/autonomy/localization/state_estimator.hpp"

#include <cstddef>
#include <vector>

namespace sim {

// StateEstimator replay + latency validation (observation only, no behavior).
// The question it answers: when the planner asks "where will the robot be",
// how much of the answer's error comes from TIME (latency/dropout) as
// opposed to MODEL (the 0.3 s startup bias from 4549388, which is drive lag
// and belongs to the planner-margin/execution-model account, NOT here)?
//
// All functions are pure. The holder is the only stateful piece and it is
// caller-owned. No filter, no fusion, no release: replay reports, Phase-2
// stays frozen.

// A. Latency distribution over a stream of (publish - stamp) samples.
struct LatencyStats {
  std::size_t n{0};
  double mean_s{0.0};
  double p50_s{0.0};
  double p95_s{0.0};
  double p99_s{0.0};
  double max_s{0.0};
};
// Deterministic nearest-rank percentiles on a sorted copy. Empty -> zeros.
LatencyStats latency_stats(std::vector<double> latencies);

// B. Stale-pose model: what the planner sees when the estimate is L seconds
// late. Query at time t returns truth at (t - L), clamped to the first pose
// (before motion starts the robot is where it is — holding, not inventing).
std::vector<TimedPose> delay_trajectory(const std::vector<TimedPose>& truth, double latency_s);

struct GapLatencyRow {
  double latency_s{0.0};
  double gap_m{0.0};
  double gap_error_m{0.0}; // gap(L) - gap(0): the margin latency eats.
};
// Same trajectory, same threat, swept per latency. The table is the evidence
// a future margin decision would cite; this slice sets no margin.
std::vector<GapLatencyRow> gap_vs_latency(const std::vector<TimedPose>& ego_truth,
                                          const std::vector<std::vector<DynamicObjectState>>& frames,
                                          double frame_dt_s, double ego_radius_m,
                                          const std::vector<double>& latencies);

// C. Dropout observation: stream truth into a holder with a stamp gap
// [drop_start_s, drop_start_s + drop_duration_s), then query at query_s.
struct DropoutRow {
  double dropout_s{0.0};
  EstimateStatus status{EstimateStatus::kInvalid};
  double confidence{0.0}; // same reporting carrier as the shadow episode.
  double position_error_m{0.0}; // holder mean vs truth at query_s (inf if Invalid).
};
DropoutRow dropout_observe(const std::vector<TimedPose>& truth, double stream_latency_s,
                           double drop_start_s, double drop_duration_s, double query_s,
                           double max_staleness_s, DropoutPolicy policy);

} // namespace sim
