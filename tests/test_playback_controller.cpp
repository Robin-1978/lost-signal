#include <cassert>
#include <cmath>
#include <cstdio>
#include <vector>

#include "sim/scenario/scenario_session.hpp"
#include "sim/viewer/playback_controller.hpp"

namespace {

using lost_signal::sim::scenario::ScenarioSession;
using lost_signal::sim::viewer::PlaybackController;

struct Result {
  lost_signal::sim::world::WorldTruth truth;
  lost_signal::sim::eval::EpisodeScore score;
  std::size_t steps = 0;
};

Result run_with_schedule(double rate, bool turbo) {
  ScenarioSession session;
  session.set_startup_mode(lost_signal::robot::autonomy::StartupMode::kSlamOnly);
  session.setup_world().add_place({"goal", 100.0, 0.0, 0.0});
  session.set_objectives({"goal"}, 0.5);

  PlaybackController::Config config;
  config.requested_rate = rate;
  config.frame_work_budget_ms = 1000.0;
  config.max_steps_per_frame = 1000;
  PlaybackController playback(config);
  playback.set_turbo(turbo);

  constexpr std::size_t kSteps = 120;
  while (session.episode().steps().size() < kSteps) {
    playback.begin_frame(1.0 / 60.0);
    while (session.episode().steps().size() < kSteps &&
           playback.step_due(0.0)) {
      session.step_once();
      playback.on_step(session.last_step_timing().total_ms);
    }
    playback.end_frame();
  }
  return {session.episode().steps().back().truth_after, session.score(),
          session.episode().steps().size()};
}

Result run_headless() {
 ScenarioSession session;
 session.set_startup_mode(lost_signal::robot::autonomy::StartupMode::kSlamOnly);
 session.setup_world().add_place({"goal", 100.0, 0.0, 0.0});
 session.set_objectives({"goal"}, 0.5);
 constexpr std::size_t kSteps = 120;
 for (std::size_t step = 0; step < kSteps; ++step) session.step_once();
 return {session.episode().steps().back().truth_after, session.score(),
 session.episode().steps().size()};
}

void assert_same_result(const Result& a, const Result& b) {
  assert(a.steps == b.steps);
  assert(a.truth.step == b.truth.step);
  assert(std::abs(a.truth.body_x_m - b.truth.body_x_m) < 1e-12);
  assert(std::abs(a.truth.body_y_m - b.truth.body_y_m) < 1e-12);
  assert(std::abs(a.truth.body_yaw_rad - b.truth.body_yaw_rad) < 1e-12);
 assert(std::abs(a.score.path_length_m - b.score.path_length_m) < 1e-12);
 assert(std::abs(a.score.mean_pos_err_m - b.score.mean_pos_err_m) < 1e-12);
  assert(a.score.collision_steps == b.score.collision_steps);
}

} // namespace

int main() {
  // A 1x schedule emits 50 fixed 20 ms steps over one second.
  PlaybackController playback;
  int steps = 0;
  for (int frame = 0; frame < 60; ++frame) {
    playback.begin_frame(1.0 / 60.0);
    while (playback.step_due(0.0)) {
      playback.on_step(1.0 + 0.01 * steps);
      ++steps;
    }
    playback.end_frame();
  }
  assert(steps == 50);
  auto stats = playback.statistics();
  assert(std::abs(stats.actual_rtf - 1.0) < 1e-9);
  assert(std::abs(stats.steps_per_s - 50.0) < 1e-9);
  assert(stats.tick_p99_ms > 1.45 && stats.tick_p99_ms < 1.50);
  assert(!stats.throttled);

  // Work limits keep the UI responsive and are observable in the HUD.
  PlaybackController::Config limited_config;
  limited_config.requested_rate = 8.0;
  limited_config.max_steps_per_frame = 2;
  PlaybackController limited(limited_config);
  limited.begin_frame(1.0 / 60.0);
  assert(limited.step_due(0.0));
  limited.on_step(1.0);
  assert(limited.step_due(1.0));
  limited.on_step(1.0);
  assert(!limited.step_due(2.0));
  limited.end_frame();
  assert(limited.statistics().throttled);

 // Turbo has its own bounded work allowance so it can spend less wall time
 // rendering without making ordinary playback unresponsive.
 PlaybackController::Config turbo_budget_config;
 turbo_budget_config.frame_work_budget_ms = 2.0;
 turbo_budget_config.turbo_frame_work_budget_ms = 5.0;
 PlaybackController turbo_budget(turbo_budget_config);
 turbo_budget.set_turbo(true);
 turbo_budget.begin_frame(1.0 / 60.0);
 assert(turbo_budget.step_due(0.0));
 turbo_budget.on_step(1.0);
 assert(turbo_budget.step_due(2.0));
 turbo_budget.on_step(1.0);
 assert(!turbo_budget.step_due(5.0));

  // Playback rate and turbo alter wall-time throughput only. Fixed-step
  // physics, sensor stamps, ROBOT decisions, truth, and score stay identical.
  const Result one_x = run_with_schedule(1.0, false);
  const Result eight_x = run_with_schedule(8.0, false);
 const Result sixty_four_x = run_with_schedule(64.0, false);
  const Result turbo = run_with_schedule(1.0, true);
 const Result headless = run_headless();
  assert_same_result(one_x, eight_x);
 assert_same_result(one_x, sixty_four_x);
  assert_same_result(one_x, turbo);
 assert_same_result(one_x, headless);

  std::puts("test_playback_controller: ok");
  return 0;
}
