#include "sim/eval/scoring.hpp"

#include <cmath>

namespace lost_signal::sim::eval {

EpisodeScore score_episode(const EpisodeRecorder& episode) {
  EpisodeScore s;
  s.steps = episode.size();
  if (s.steps == 0) return s;
  const auto& steps = episode.steps();
  double x0 = steps.front().truth_before.body_x_m;
  double y0 = steps.front().truth_before.body_y_m;
  double x1 = steps.back().truth_after.body_x_m;
  double y1 = steps.back().truth_after.body_y_m;
  s.distance_m = std::hypot(x1 - x0, y1 - y0);
  for (const auto& st : steps) {
    if (st.cmd.is_safe_stop()) s.safe_stop_steps += 1;
    if (st.obs_accepted) {
      s.obs_accepted += 1;
    } else {
      s.obs_rejected += 1;
    }
  }
  return s;
}

double reward_step(const EvalStep& step) {
  // Reference reward: forward progress minus stop penalty. Offline label.
  double dx = step.truth_after.body_x_m - step.truth_before.body_x_m;
  double reward = dx;  // meters of truth progress this step
  if (step.cmd.is_safe_stop()) reward -= 0.01;
  if (!step.obs_accepted) reward -= 0.05;
  return reward;
}

}  // namespace lost_signal::sim::eval
