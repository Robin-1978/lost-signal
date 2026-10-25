#include "sim/viewer/playback_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace lost_signal::sim::viewer {

namespace {

bool positive_finite(double value) {
  return std::isfinite(value) && value > 0.0;
}

} // namespace

PlaybackController::PlaybackController() : PlaybackController(Config{}) {}

PlaybackController::PlaybackController(Config config) : config_(config) {
  if (!positive_finite(config_.step_s) ||
      !positive_finite(config_.requested_rate) ||
      !positive_finite(config_.max_debt_s) ||
      !positive_finite(config_.frame_work_budget_ms) ||
 !positive_finite(config_.turbo_frame_work_budget_ms) ||
      config_.max_steps_per_frame <= 0 ||
      !positive_finite(config_.statistics_window_s)) {
    throw std::invalid_argument("invalid playback controller configuration");
  }
}

void PlaybackController::set_requested_rate(double rate) {
  if (!positive_finite(rate)) {
    throw std::invalid_argument("playback rate must be finite and positive");
  }
  config_.requested_rate = rate;
}

void PlaybackController::set_turbo(bool enabled) {
  turbo_ = enabled;
  debt_s_ = 0.0;
}

void PlaybackController::begin_frame(double wall_dt_s) {
  if (!std::isfinite(wall_dt_s) || wall_dt_s < 0.0) wall_dt_s = 0.0;
  frame_wall_s_ = std::min(wall_dt_s, 0.25);
  wall_s_ += frame_wall_s_;
  frame_steps_ = 0;
  frame_throttled_ = false;
  if (!turbo_) {
    debt_s_ = std::min(config_.max_debt_s,
                       debt_s_ + frame_wall_s_ * config_.requested_rate);
  }
}

bool PlaybackController::step_due(double elapsed_work_ms) {
  const bool demand = turbo_ || debt_s_ + 1e-12 >= config_.step_s;
  if (!demand) return false;
 const double work_budget_ms = turbo_
 ? config_.turbo_frame_work_budget_ms : config_.frame_work_budget_ms;
 if (frame_steps_ >= config_.max_steps_per_frame ||
 elapsed_work_ms >= work_budget_ms) {
    frame_throttled_ = true;
    return false;
  }
  return true;
}

void PlaybackController::on_step(double tick_ms) {
  if (!std::isfinite(tick_ms) || tick_ms < 0.0) tick_ms = 0.0;
  ++frame_steps_;
  if (!turbo_) debt_s_ = std::max(0.0, debt_s_ - config_.step_s);
  ticks_.push_back({wall_s_, tick_ms});
}

void PlaybackController::end_frame() {
  frames_.push_back({wall_s_, frame_wall_s_, frame_steps_,
                     frame_throttled_});
  trim_window();
}

void PlaybackController::discard_debt() {
  debt_s_ = 0.0;
  frame_steps_ = 0;
  frame_throttled_ = false;
}

void PlaybackController::trim_window() {
  const double cutoff = wall_s_ - config_.statistics_window_s;
  while (!frames_.empty() && frames_.front().end_wall_s < cutoff) {
    frames_.pop_front();
  }
  while (!ticks_.empty() && ticks_.front().wall_s < cutoff) {
    ticks_.pop_front();
  }
}

PlaybackController::Statistics PlaybackController::statistics() const {
  Statistics out;
  out.requested_rate = config_.requested_rate;
  out.turbo = turbo_;
  double wall_total_s = 0.0;
  std::size_t steps = 0;
  for (const auto& frame : frames_) {
    wall_total_s += frame.wall_s;
    steps += static_cast<std::size_t>(std::max(0, frame.steps));
    out.throttled = out.throttled || frame.throttled;
  }
  if (wall_total_s > 1e-9) {
    out.steps_per_s = static_cast<double>(steps) / wall_total_s;
    out.actual_rtf = out.steps_per_s * config_.step_s;
  }
  if (!ticks_.empty()) {
    std::vector<double> sorted;
    sorted.reserve(ticks_.size());
    for (const auto& tick : ticks_) sorted.push_back(tick.tick_ms);
    std::sort(sorted.begin(), sorted.end());
    const std::size_t index = static_cast<std::size_t>(
        std::ceil(0.99 * static_cast<double>(sorted.size())) - 1.0);
    out.tick_p99_ms = sorted[std::min(index, sorted.size() - 1)];
  }
  return out;
}

} // namespace lost_signal::sim::viewer
