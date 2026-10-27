#include <cassert>
#include <cmath>
#include <cstdio>
#include <utility>
#include <vector>

#include "sim/scenario/scenario_session.hpp"
#include "sim/viewer/playback_controller.hpp"

namespace {

using lost_signal::sim::scenario::ScenarioSession;
using lost_signal::sim::viewer::PlaybackController;

struct Result {
 struct StepTrace {
 std::uint64_t step = 0;
 std::uint64_t observation_sequence = 0;
 lost_signal::robot::core::MonotonicNs observation_stamp_ns =
 lost_signal::robot::core::kInvalidTime;
 lost_signal::robot::core::MonotonicNs scan_stamp_ns =
 lost_signal::robot::core::kInvalidTime;
 lost_signal::robot::core::MonotonicNs motion_stamp_ns =
 lost_signal::robot::core::kInvalidTime;
 lost_signal::robot::core::MonotonicNs imu_stamp_ns =
 lost_signal::robot::core::kInvalidTime;
 lost_signal::robot::iface::MotionCommand command;
 bool observation_accepted = false;
 bool static_collision = false;
 bool dynamic_collision = false;
 };

  lost_signal::sim::world::WorldTruth truth;
  lost_signal::sim::eval::EpisodeScore score;
  std::size_t steps = 0;
 bool objectives_done = false;
 bool objectives_passed = false;
 std::size_t objectives_completed = 0;
 std::size_t objectives_successful = 0;
 std::size_t truth_arrivals = 0;
 std::vector<StepTrace> trace;
};

Result result_from(const ScenarioSession& session) {
 Result out;
 out.truth = session.episode().steps().back().truth_after;
 out.score = session.score();
 out.steps = session.episode().steps().size();
 out.objectives_done = session.objectives_done();
 out.objectives_passed = session.objectives_passed();
 out.objectives_completed = session.objectives().completed();
 out.objectives_successful = session.objectives().successful();
 out.truth_arrivals = session.objectives().truth_arrivals();
 out.trace.reserve(out.steps);
 for (const auto& step : session.episode().steps()) {
 Result::StepTrace trace;
 trace.step = step.step;
 trace.observation_sequence = step.obs.sequence;
 trace.observation_stamp_ns = step.obs.stamp_ns;
 trace.scan_stamp_ns = lost_signal::robot::iface::scan_stamp_ns(step.obs);
 trace.motion_stamp_ns =
 lost_signal::robot::iface::motion_stamp_ns(step.obs);
 trace.imu_stamp_ns = lost_signal::robot::iface::imu_stamp_ns(step.obs);
 trace.command = step.cmd;
 trace.observation_accepted = step.obs_accepted;
 trace.static_collision = step.judge.static_collision;
 trace.dynamic_collision = step.judge.dynamic_collision;
 out.trace.push_back(std::move(trace));
 }
 return out;
}

Result run_with_schedule(double rate, bool turbo, double goal_x_m = 100.0) {
  ScenarioSession session;
  session.set_startup_mode(lost_signal::robot::autonomy::StartupMode::kSlamOnly);
 session.setup_world().add_place({"goal", goal_x_m, 0.0, 0.0});
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
 return result_from(session);
}

Result run_headless(double goal_x_m = 100.0) {
 ScenarioSession session;
 session.set_startup_mode(lost_signal::robot::autonomy::StartupMode::kSlamOnly);
 session.setup_world().add_place({"goal", goal_x_m, 0.0, 0.0});
 session.set_objectives({"goal"}, 0.5);
 constexpr std::size_t kSteps = 120;
 for (std::size_t step = 0; step < kSteps; ++step) session.step_once();
 return result_from(session);
}

void assert_same_result(const Result& a, const Result& b) {
  assert(a.steps == b.steps);
 assert(a.objectives_done == b.objectives_done);
 assert(a.objectives_passed == b.objectives_passed);
 assert(a.objectives_completed == b.objectives_completed);
 assert(a.objectives_successful == b.objectives_successful);
 assert(a.truth_arrivals == b.truth_arrivals);
  assert(a.truth.step == b.truth.step);
  assert(std::abs(a.truth.body_x_m - b.truth.body_x_m) < 1e-12);
  assert(std::abs(a.truth.body_y_m - b.truth.body_y_m) < 1e-12);
 assert(std::abs(a.truth.body_z_m - b.truth.body_z_m) < 1e-12);
 assert(std::abs(a.truth.body_roll_rad - b.truth.body_roll_rad) < 1e-12);
 assert(std::abs(a.truth.body_pitch_rad - b.truth.body_pitch_rad) < 1e-12);
  assert(std::abs(a.truth.body_yaw_rad - b.truth.body_yaw_rad) < 1e-12);
 assert(std::abs(a.truth.forward_mps - b.truth.forward_mps) < 1e-12);
 assert(std::abs(a.truth.lateral_mps - b.truth.lateral_mps) < 1e-12);
 assert(std::abs(a.truth.yaw_rate_radps - b.truth.yaw_rate_radps) < 1e-12);
 assert(a.score.steps == b.score.steps);
 assert(a.score.elapsed_s == b.score.elapsed_s);
 assert(a.score.safe_stop_steps == b.score.safe_stop_steps);
 assert(a.score.obs_accepted == b.score.obs_accepted);
 assert(a.score.obs_rejected == b.score.obs_rejected);
  assert(a.score.collision_steps == b.score.collision_steps);
 assert(a.score.distance_m == b.score.distance_m);
 assert(a.score.path_length_m == b.score.path_length_m);
 assert(a.score.dynamic_hold_steps == b.score.dynamic_hold_steps);
 assert(a.score.terrain_collision_steps == b.score.terrain_collision_steps);
 assert(a.score.estimate_samples == b.score.estimate_samples);
 assert(a.score.mean_pos_err_m == b.score.mean_pos_err_m);
 assert(a.score.max_pos_err_m == b.score.max_pos_err_m);
 assert(a.score.mean_yaw_err_rad == b.score.mean_yaw_err_rad);
 assert(a.score.max_yaw_err_rad == b.score.max_yaw_err_rad);
 assert(a.score.min_clearance_m == b.score.min_clearance_m);
 assert(a.score.mean_clearance_m == b.score.mean_clearance_m);
 assert(a.score.min_terrain_clearance_m == b.score.min_terrain_clearance_m);
 assert(a.score.mean_terrain_clearance_m == b.score.mean_terrain_clearance_m);
 assert(a.score.energy_proxy == b.score.energy_proxy);
 assert(a.trace.size() == b.trace.size());
 for (std::size_t i = 0; i < a.trace.size(); ++i) {
 const auto& lhs = a.trace[i];
 const auto& rhs = b.trace[i];
 assert(lhs.step == rhs.step);
 assert(lhs.observation_sequence == rhs.observation_sequence);
 assert(lhs.observation_stamp_ns == rhs.observation_stamp_ns);
 assert(lhs.scan_stamp_ns == rhs.scan_stamp_ns);
 assert(lhs.motion_stamp_ns == rhs.motion_stamp_ns);
 assert(lhs.imu_stamp_ns == rhs.imu_stamp_ns);
 assert(lhs.observation_accepted == rhs.observation_accepted);
 assert(lhs.static_collision == rhs.static_collision);
 assert(lhs.dynamic_collision == rhs.dynamic_collision);
 assert(lhs.command.space == rhs.command.space);
 assert(lhs.command.stamp_ns == rhs.command.stamp_ns);
 assert(lhs.command.expiry_ns == rhs.command.expiry_ns);
 assert(lhs.command.forward_mps == rhs.command.forward_mps);
 assert(lhs.command.lateral_mps == rhs.command.lateral_mps);
 assert(lhs.command.yaw_rate_radps == rhs.command.yaw_rate_radps);
 assert(lhs.command.steer_angle_rad == rhs.command.steer_angle_rad);
 }
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
 assert(std::abs(stats.tick_max_ms - 1.49) < 1e-9);
  assert(!stats.throttled);

 // High requested rates keep the full current-frame demand. Backlog limits
 // scale in wall time and therefore cannot silently turn 64x into ~30x.
 const auto scheduled_steps = [](double rate) {
 PlaybackController::Config config;
 config.requested_rate = rate;
 config.frame_work_budget_ms = 1000.0;
 config.max_steps_per_frame = 1000;
 PlaybackController controller(config);
 int count = 0;
 for (int frame = 0; frame < 60; ++frame) {
 controller.begin_frame(1.0 / 60.0);
 while (controller.step_due(0.0)) {
 controller.on_step(0.0);
 ++count;
 }
 controller.end_frame();
 }
 return std::pair<int, PlaybackController::Statistics>{
 count, controller.statistics()};
 };
 const auto scheduled_thirty_two_x = scheduled_steps(32.0);
 const auto scheduled_sixty_four_x = scheduled_steps(64.0);
 assert(scheduled_thirty_two_x.first == 1600);
 assert(scheduled_sixty_four_x.first == 3200);
 assert(std::abs(scheduled_thirty_two_x.second.actual_rtf - 32.0) < 1e-9);
 assert(std::abs(scheduled_sixty_four_x.second.actual_rtf - 64.0) < 1e-9);

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

 // Rate-scaled work remains bounded by its explicit cap. A higher requested
 // rate can consume more deterministic steps without changing step dt.
 PlaybackController::Config scaled_config;
 scaled_config.requested_rate = 8.0;
 scaled_config.frame_work_budget_ms = 2.0;
 scaled_config.maximum_rate_scaled_work_budget_ms = 8.0;
 PlaybackController scaled(scaled_config);
 scaled.begin_frame(1.0 / 60.0);
 assert(scaled.step_due(7.9));
 assert(!scaled.step_due(8.0));

 // Dropped historical backlog and long wall stalls are visible as LIMIT.
 PlaybackController::Config backlog_config;
 backlog_config.max_backlog_wall_s = 0.05;
 backlog_config.frame_work_budget_ms = 1000.0;
 PlaybackController backlog(backlog_config);
 backlog.begin_frame(0.25);
 backlog.end_frame();
 backlog.begin_frame(0.01);
 backlog.end_frame();
 assert(backlog.statistics().throttled);

 PlaybackController long_frame;
 long_frame.begin_frame(0.75);
 assert(long_frame.step_due(0.0));
 long_frame.on_step(0.0);
 long_frame.end_frame();
 const auto long_stats = long_frame.statistics();
 assert(long_stats.throttled);
 assert(std::abs(long_stats.actual_rtf - (0.02 / 0.75)) < 1e-12);

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

 // Repeat with an objective at the initial pose so the comparison also
 // covers the settled ROBOT-done + independent SIM-arrival terminal verdict.
 // The fixed number of subsequent transitions proves that completed mission
 // state and the full accumulated score remain schedule-independent too.
 const Result completed_one_x = run_with_schedule(1.0, false, 0.0);
 const Result completed_eight_x = run_with_schedule(8.0, false, 0.0);
 const Result completed_sixty_four_x =
 run_with_schedule(64.0, false, 0.0);
 const Result completed_turbo = run_with_schedule(1.0, true, 0.0);
 const Result completed_headless = run_headless(0.0);
 assert(completed_one_x.objectives_done);
 assert(completed_one_x.objectives_passed);
 assert(completed_one_x.objectives_completed == 1);
 assert(completed_one_x.objectives_successful == 1);
 assert(completed_one_x.truth_arrivals == 1);
 assert_same_result(completed_one_x, completed_eight_x);
 assert_same_result(completed_one_x, completed_sixty_four_x);
 assert_same_result(completed_one_x, completed_turbo);
 assert_same_result(completed_one_x, completed_headless);

  std::puts("test_playback_controller: ok");
  return 0;
}
