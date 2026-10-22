#pragma once

#include <cstddef>

#include "sim/eval/episode.hpp"

namespace lost_signal::sim::eval {

// Offline score over a finished episode. All inputs are recorded history;
// scoring never mutates ROBOT state and never feeds a running tick.
//
// Truth-vs-estimate comparison is the algorithm-tuning signal: truth comes
// from the recorded WorldTruth, the estimate comes from the recorded ROBOT
// Diagnostics. Both are history here, never live inputs.
struct EstimateError {
  bool has_estimate = false;
  double pos_err_m = 0.0;  // hypot(est.x - truth.x, est.y - truth.y)
  double yaw_err_rad = 0.0;  // wrapped |est.yaw - truth.yaw|
};

struct EpisodeScore {
  std::size_t steps = 0;
  std::size_t safe_stop_steps = 0;
  std::size_t obs_accepted = 0;
  std::size_t obs_rejected = 0;
  double distance_m = 0.0;  // truth displacement over the episode
  // Estimate quality (offline truth-vs-estimate, for optimizer reference):
  std::size_t estimate_samples = 0;
  double mean_pos_err_m = 0.0;
  double max_pos_err_m = 0.0;
  double mean_yaw_err_rad = 0.0;
  double max_yaw_err_rad = 0.0;
};

EpisodeScore score_episode(const EpisodeRecorder& episode);

// Per-step truth-vs-estimate error for one recorded step (offline only).
EstimateError estimate_error(const EvalStep& step);

// Offline RL reward for one recorded transition (truth_before -> truth_after).
// Reference only: the training loader reads exported episodes, the live
// decision path never sees this value.
double reward_step(const EvalStep& step);

}  // namespace lost_signal::sim::eval
