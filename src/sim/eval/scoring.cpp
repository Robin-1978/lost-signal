#include "sim/eval/scoring.hpp"

#include <cmath>

namespace lost_signal::sim::eval {

namespace {
double wrap_pi(double a) {
  while (a > M_PI) a -= 2.0 * M_PI;
  while (a < -M_PI) a += 2.0 * M_PI;
  return a;
}
}  // namespace

EstimateError estimate_error(const EvalStep& step) {
  EstimateError e;
  if (!step.diag.has_observation || !step.diag.estimate.valid) return e;
  e.has_estimate = true;
  double dx = step.diag.estimate.x_m - step.truth_after.body_x_m;
  double dy = step.diag.estimate.y_m - step.truth_after.body_y_m;
  e.pos_err_m = std::hypot(dx, dy);
  e.yaw_err_rad =
      std::abs(wrap_pi(step.diag.estimate.yaw_rad - step.truth_after.body_yaw_rad));
  return e;
}

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
  double pos_sum = 0.0;
  double yaw_sum = 0.0;
  for (const auto& st : steps) {
    if (st.cmd.is_safe_stop()) s.safe_stop_steps += 1;
    if (st.judge.static_collision || st.judge.dynamic_collision ||
        st.judge.terrain_collision) {
      s.collision_steps += 1;
    }
    if (st.obs_accepted) {
      s.obs_accepted += 1;
    } else {
      s.obs_rejected += 1;
    }
    EstimateError e = estimate_error(st);
    if (e.has_estimate) {
      s.estimate_samples += 1;
      pos_sum += e.pos_err_m;
      yaw_sum += e.yaw_err_rad;
      if (e.pos_err_m > s.max_pos_err_m) s.max_pos_err_m = e.pos_err_m;
      if (e.yaw_err_rad > s.max_yaw_err_rad) s.max_yaw_err_rad = e.yaw_err_rad;
    }
  }
  if (s.estimate_samples > 0) {
    s.mean_pos_err_m = pos_sum / s.estimate_samples;
    s.mean_yaw_err_rad = yaw_sum / s.estimate_samples;
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
