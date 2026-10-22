#pragma once

#include <cstddef>

#include "sim/eval/episode.hpp"

namespace lost_signal::sim::eval {

// Offline score over a finished episode. All inputs are recorded history;
// scoring never mutates ROBOT state and never feeds a running tick.
struct EpisodeScore {
  std::size_t steps = 0;
  std::size_t safe_stop_steps = 0;
  std::size_t obs_accepted = 0;
  std::size_t obs_rejected = 0;
  double distance_m = 0.0;  // truth displacement over the episode
};

EpisodeScore score_episode(const EpisodeRecorder& episode);

// Offline RL reward for one recorded transition (truth_before -> truth_after).
// Reference only: the training loader reads exported episodes, the live
// decision path never sees this value.
double reward_step(const EvalStep& step);

}  // namespace lost_signal::sim::eval
