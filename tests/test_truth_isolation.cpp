#include <cassert>
#include <cstdio>

#include "sim/eval/scoring.hpp"
#include "sim/game/scenario_session.hpp"

int main() {
  using namespace lost_signal;

  // 1. Episode records truth + obs + cmd, robot still ticks safe-stop on v1.
  sim::game::ScenarioSession session;
  robot::iface::TaskRequest req;
  req.request_id = "eval";
  req.place_id = "dock";
  session.submit_task(req);
  for (int i = 0; i < 3; ++i) session.step_once();
  assert(session.episode().size() == 3);

  // 2. Offline scoring sees the recorded history (never fed back to tick).
  sim::eval::EpisodeScore score = sim::eval::score_episode(session.episode());
  assert(score.steps == 3);
  assert(score.obs_accepted == 3);
  assert(score.safe_stop_steps == 3);

  // 3. Offline RL reward is a pure function of the recorded transition.
  double r0 = sim::eval::reward_step(session.episode().steps()[0]);
  (void)r0;  // label only; must not alter the next tick

  // 4. Truth mutation without a new observation must not change the robot:
  // next command is still the deterministic safe-stop from recorded state.
  // (No API exists to inject truth into RobotApplication; this is enforced by
  // the robot_boundary gate: src/robot never includes sim/.)
  sim::game::ScenarioSession session2;
  session2.submit_task(req);
  session2.step_once();
  assert(session2.episode().size() == 1);
  assert(session2.episode().steps()[0].cmd.is_safe_stop());

  std::printf("truth_isolation ok: steps=%zu reward=%f\n", score.steps, r0);
  return 0;
}
