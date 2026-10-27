#include <cassert>
#include <cmath>
#include <cstdio>

#include "sim/eval/scoring.hpp"
#include "sim/scenario/scenario_session.hpp"

int main() {
  using namespace lost_signal;

  // 1. Episode records truth + obs + cmd + estimate, robot ticks safe-stop.
  sim::scenario::ScenarioSession session;
  session.setup_world().add_place({"dock", 0.0, 0.0, 0.0});
  robot::iface::TaskRequest req;
  req.request_id = "eval";
  req.place_id = "dock";
  session.request_goal(req);
  for (int i = 0; i < 3; ++i) session.step_once();
  assert(session.episode().size() == 3);

  // 2. Offline truth-vs-estimate: estimate exists, error is measurable.
  // This is the optimizer reference signal. It is computed from recorded
  // history only, never fed back into tick().
  sim::eval::EpisodeScore score = sim::eval::score_episode(session.episode());
  assert(score.steps == 3);
  assert(score.obs_accepted == 3);
  assert(score.safe_stop_steps == 3);
  assert(score.estimate_samples == 3);
  for (const auto& st : session.episode().steps()) {
    assert(st.diag.has_observation);
    assert(st.diag.estimate.valid);
 assert(!st.obs.scan_static_valid);
 assert(st.obs.static_ranges_m.empty());
    sim::eval::EstimateError e = sim::eval::estimate_error(st);
    assert(e.has_estimate);
    assert(std::isfinite(e.pos_err_m) && std::isfinite(e.yaw_err_rad));
  }
  assert(std::isfinite(score.mean_pos_err_m));
  assert(std::isfinite(score.max_pos_err_m));
 const sim::eval::EpisodeScore online = session.score();
 assert(online.steps == score.steps);
 assert(online.elapsed_s == score.elapsed_s);
 assert(online.safe_stop_steps == score.safe_stop_steps);
 assert(online.obs_accepted == score.obs_accepted);
 assert(online.obs_rejected == score.obs_rejected);
 assert(online.collision_steps == score.collision_steps);
 assert(online.distance_m == score.distance_m);
 assert(online.path_length_m == score.path_length_m);
 assert(online.dynamic_hold_steps == score.dynamic_hold_steps);
 assert(online.terrain_collision_steps == score.terrain_collision_steps);
 assert(online.estimate_samples == score.estimate_samples);
 assert(online.mean_pos_err_m == score.mean_pos_err_m);
 assert(online.max_pos_err_m == score.max_pos_err_m);
 assert(online.mean_yaw_err_rad == score.mean_yaw_err_rad);
 assert(online.max_yaw_err_rad == score.max_yaw_err_rad);
 assert(online.min_clearance_m == score.min_clearance_m);
 assert(online.mean_clearance_m == score.mean_clearance_m);
 assert(online.min_terrain_clearance_m == score.min_terrain_clearance_m);
 assert(online.mean_terrain_clearance_m == score.mean_terrain_clearance_m);
 assert(online.energy_proxy == score.energy_proxy);

  // 3. Offline RL reward is a pure function of the recorded transition.
  double r0 = sim::eval::reward_step(session.episode().steps()[0]);
  (void)r0;  // label only; must not alter the next tick

  // 4. Truth never penetrates: no API exists to inject WorldTruth into
  // RobotApplication (enforced by the robot_boundary gate: no sim includes,
  // no truth tokens, no sim links in src/robot). Mutating sim truth without
  // publishing an observation cannot change the next command.
  sim::scenario::ScenarioSession session2;
  session2.request_goal(req);
  session2.step_once();
  assert(session2.episode().size() == 1);
  assert(session2.episode().steps()[0].cmd.is_safe_stop());

 // Summary-only mode remains constant-memory while retaining the exact
 // aggregate and latest diagnostics needed by long headless benchmarks.
 sim::scenario::ScenarioSession streaming;
 streaming.set_episode_history_enabled(false);
 for (int i = 0; i < 100; ++i) streaming.step_once();
 assert(streaming.episode().steps().empty());
 assert(streaming.score().steps == 100);
 assert(streaming.last_eval_step() != nullptr);
 // EvalStep ordinals are one-based completed transitions.  The world truth
 // clock remains zero-based at construction, but the public replay ordinal
 // starts at one so exported episodes and score completion steps agree.
 assert(streaming.last_eval_step()->step == 100);

 // Interactive recent-history mode is a bounded deque, not a repeatedly
 // reallocating vector. Dropping old presentation frames cannot change the
 // full-run aggregate score or latest diagnostic.
 sim::scenario::ScenarioSession recent;
 recent.set_episode_history_limit(8);
 for (int i = 0; i < 100; ++i) recent.step_once();
 assert(recent.episode().steps().size() == 8);
 assert(recent.episode().maximum_steps() == 8);
 assert(recent.episode().total_recorded() == 100);
 assert(recent.episode().dropped_steps() == 92);
 assert(!recent.episode().complete());
 assert(recent.episode().steps().front().step == 93);
 assert(recent.episode().steps().back().step == 100);
 assert(recent.score().steps == 100);
 assert(recent.last_eval_step() != nullptr);
 assert(recent.last_eval_step()->step == 100);

  std::printf("truth_isolation ok: steps=%zu mean_pos_err=%f reward=%f\n",
              score.steps, score.mean_pos_err_m, r0);
  return 0;
}
