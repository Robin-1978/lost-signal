#include "sim/eval/scoring.hpp"

#include <algorithm>
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
 EpisodeScoreAccumulator accumulator;
 for (const auto& step : episode.steps()) accumulator.record(step);
 return accumulator.score();
}

void EpisodeScoreAccumulator::reset() {
 *this = EpisodeScoreAccumulator{};
}

void EpisodeScoreAccumulator::record(const EvalStep& step) {
 if (!have_origin_) {
 have_origin_ = true;
 origin_x_m_ = step.truth_before.body_x_m;
 origin_y_m_ = step.truth_before.body_y_m;
 }
 final_x_m_ = step.truth_after.body_x_m;
 final_y_m_ = step.truth_after.body_y_m;
 ++score_.steps;
 score_.elapsed_s = static_cast<double>(score_.steps) * 0.02;
 score_.distance_m = std::hypot(
 final_x_m_ - origin_x_m_, final_y_m_ - origin_y_m_);
 score_.path_length_m += std::hypot(
 step.truth_after.body_x_m - step.truth_before.body_x_m,
 step.truth_after.body_y_m - step.truth_before.body_y_m);
 if (step.cmd.is_safe_stop()) ++score_.safe_stop_steps;
 if (step.judge.static_collision || step.judge.dynamic_collision ||
 step.judge.terrain_collision) {
 ++score_.collision_steps;
 }
 if (step.judge.terrain_collision) ++score_.terrain_collision_steps;
 if (step.cmd.is_safe_stop() && step.judge.dynamic_clearance_m < 1.0) {
 ++score_.dynamic_hold_steps;
 }
 const double clearance = std::min(
 step.judge.static_clearance_m, step.judge.dynamic_clearance_m);
 if (std::isfinite(clearance) && clearance < 999.0) {
 score_.min_clearance_m = std::min(score_.min_clearance_m, clearance);
 clearance_sum_m_ += clearance;
 ++clearance_samples_;
 }
 const double terrain_clearance = step.judge.terrain_clearance_m;
 if (std::isfinite(terrain_clearance) && terrain_clearance < 999.0) {
 score_.min_terrain_clearance_m = std::min(
 score_.min_terrain_clearance_m, terrain_clearance);
 terrain_clearance_sum_m_ += terrain_clearance;
 ++terrain_clearance_samples_;
 }
 double wheel_power = 0.0;
 for (std::size_t i = 0; i < step.truth_after.drive_count; ++i) {
 wheel_power += std::abs(step.truth_after.drive_duty[i] *
 step.truth_after.drive_radps[i]);
 }
 score_.energy_proxy += wheel_power * 0.02;
 if (step.obs_accepted) ++score_.obs_accepted;
 else ++score_.obs_rejected;
 const EstimateError error = estimate_error(step);
 if (error.has_estimate) {
 ++score_.estimate_samples;
 position_error_sum_m_ += error.pos_err_m;
 yaw_error_sum_rad_ += error.yaw_err_rad;
 score_.max_pos_err_m = std::max(score_.max_pos_err_m, error.pos_err_m);
 score_.max_yaw_err_rad = std::max(
 score_.max_yaw_err_rad, error.yaw_err_rad);
 }
}

EpisodeScore EpisodeScoreAccumulator::score() const {
 EpisodeScore result = score_;
 if (result.estimate_samples > 0) {
 result.mean_pos_err_m = position_error_sum_m_ /
 static_cast<double>(result.estimate_samples);
 result.mean_yaw_err_rad = yaw_error_sum_rad_ /
 static_cast<double>(result.estimate_samples);
 }
 if (clearance_samples_ > 0) {
 result.mean_clearance_m = clearance_sum_m_ /
 static_cast<double>(clearance_samples_);
 }
 if (terrain_clearance_samples_ > 0) {
 result.mean_terrain_clearance_m = terrain_clearance_sum_m_ /
 static_cast<double>(terrain_clearance_samples_);
 }
 return result;
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
