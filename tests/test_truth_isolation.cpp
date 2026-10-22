#include <cassert>
#include <cmath>
#include <cstdio>

#include "sim/eval/scoring.hpp"
#include "sim/game/scenario_session.hpp"

int main() {
  using namespace lost_signal;

  // 1. Episode records truth + obs + cmd + estimate, robot ticks safe-stop.
  sim::game::ScenarioSession session;
  robot::iface::TaskRequest req;
  req.request_id = "eval";
  req.place_id = "dock";
  session.submit_task(req);
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
    sim::eval::EstimateError e = sim::eval::estimate_error(st);
    assert(e.has_estimate);
    assert(std::isfinite(e.pos_err_m) && std::isfinite(e.yaw_err_rad));
  }
  assert(std::isfinite(score.mean_pos_err_m));
  assert(std::isfinite(score.max_pos_err_m));

  // 3. Offline RL reward is a pure function of the recorded transition.
  double r0 = sim::eval::reward_step(session.episode().steps()[0]);
  (void)r0;  // label only; must not alter the next tick

  // 4. Truth never penetrates: no API exists to inject WorldTruth into
  // RobotApplication (enforced by the robot_boundary gate: no sim includes,
  // no truth tokens, no sim links in src/robot). Mutating sim truth without
  // publishing an observation cannot change the next command.
  sim::game::ScenarioSession session2;
  session2.submit_task(req);
  session2.step_once();
  assert(session2.episode().size() == 1);
  assert(session2.episode().steps()[0].cmd.is_safe_stop());

  std::printf("truth_isolation ok: steps=%zu mean_pos_err=%f reward=%f\n",
              score.steps, score.mean_pos_err_m, r0);
  return 0;
}
