#include <cassert>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "sim/eval/scoring.hpp"
#include "sim/scenario/scenario_session.hpp"
#include "sim/world/actor_behavior.hpp"
#include "sim/world/world.hpp"

using namespace lost_signal;

int main() {
  // Loop-closure mission: a 4x4 m square of docks through landmark boxes,
  // mapless. The open-loop stack drifts ~0.07 m per meter; closing the
  // loop must fire the pose-graph machinery and bound the final error.
  sim::scenario::ScenarioSession session;
  session.setup_world().add_place({"loopA", 4.0, 0.0, 0.0});
  session.setup_world().add_place({"loopB", 4.0, 4.0, 0.0});
  session.setup_world().add_place({"loopC", 0.0, 4.0, 0.0});
  session.setup_world().add_place({"loopD", 0.0, 0.0, 0.0});  // = start
  // Landmarks: off-path boxes, one per leg, for scan-overlap at revisit.
  // Lanes keep >= 1 m clearance for the 0.4 m footprint (single-horizon
  // DWA cannot thread tighter gaps; that is planner scope, not this test).
  session.setup_world().set_static_boxes({
      {5.2, 1.0, 6.0, 1.8, 1.0},     // east of A->B
      {2.5, 5.0, 3.3, 5.8, 1.0},     // north of B->C
      {-2.4, 2.0, -1.6, 2.8, 1.0},   // west of C->D
      {1.0, -2.4, 1.8, -1.6, 1.0},   // south of D->A
  });
  session.set_objectives({"loopA", "loopB", "loopC", "loopD"}, 1.0);
  // Mapless mission: no surveyed map to search, so skip the global-retry
  // cadence; SLAM loop closures establish the frame (same as before, the
  // vacuous survey never accepted anyway — now explicit, not accidental).
  session.set_startup_mode(
      lost_signal::robot::autonomy::StartupMode::kSlamOnly);

  std::size_t max_loops = 0;
  std::size_t max_keyframes = 0;
 for (int i = 0; i < 2000 && !session.objectives_done(); ++i) {
    session.step_once();
    const auto& diag = session.episode().steps().back().diag;
    max_loops = std::max(max_loops, diag.slam_loops_closed);
    max_keyframes = std::max(max_keyframes, diag.slam_keyframes);
  }
  std::printf("loop: objectives_done=%d loops=%zu keyframes=%zu steps=%zu\n",
              session.objectives_done(), max_loops, max_keyframes,
              session.episode().size());
  fflush(stdout);
  assert(session.objectives_done());
  const auto& results = session.objectives().results();
  assert(results.size() == 4);
  for (const auto& r : results) assert(r.robot_done);

  // The machinery engaged: keyframes along the way, at least one accepted
  // closure when the square revisited its own submaps.
  assert(max_keyframes >= 10);  // ~16 m at 0.5 m spacing, minus holds
  assert(max_loops >= 1);

  // Bounded final error after ~16 m of travel: open-loop drift at the
  // measured 0.07 m/m rate would leave > 1 m; the closure pulls it back.
  const auto& last = results.back();
  assert(last.place_id == "loopD");
  assert(last.truth_miss_m < 1.0);

  // No collisions on a landmark-lined loop; estimate stayed sane. History:
  // the M2 engine cutover grazed the east box's conservative judge circle
  // (r=0.4) for 39 sub-centimeter steps because the chassis PID integrated
  // with a fixed 0.1 s step at the 20 ms tick (5x over-integration, wide
  // arcs). Measuring dt per call healed the arc; the strict pin is back.
  const auto score = sim::eval::score_episode(session.episode());
  assert(score.collision_steps == 0);
  assert(score.max_pos_err_m < 1.0);
  // Closed loop, geometrically: truth displacement start->end is ~0 while
  // the mission traveled ~16 m through 4 docks (keyframed above).
  assert(score.distance_m < 1.0);

  std::printf("loop_closure ok\n");
  return 0;
}
