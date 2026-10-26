#include <cassert>
#include <cmath>
#include <cstdio>
#include <map>
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
 std::size_t completed_objectives_at_first_loop = 0;
 double first_loop_truth_distance_m = -1.0;
 // Pure SLAM owns the frame in this test; allow the final loop correction,
 // heading alignment, and settled arrival to finish without survey assists.
 // The 16 m route must include four physical corner alignments, a second
 // independent loop view, pose-graph correction, and the final settled
 // heading. The acceptance gates below stay unchanged; this bound only
 // avoids declaring failure while the measured chassis is still converging.
 for (int i = 0; i < 3000; ++i) {
    session.step_once();
 const auto& diag = session.episode().steps().back().diag;
 if (max_loops == 0 && diag.slam_loops_closed > 0) {
 completed_objectives_at_first_loop = session.objectives().results().size();
 const auto& truth = session.episode().steps().back().truth_after;
 first_loop_truth_distance_m = std::hypot(
 truth.body_x_m, truth.body_y_m);
 std::fprintf(stderr,
 " first loop step=%zu edge=%llu->%llu correction=%.3f/%.3f "
 "score=%.3f/%.3f overlap=%.3f margin=%.3f/%.3f\n",
 session.episode().size(),
 static_cast<unsigned long long>(diag.slam_loop_from_id),
 static_cast<unsigned long long>(diag.slam_loop_to_id),
 diag.slam_loop_correction_m, diag.slam_loop_correction_rad,
 diag.slam_loop_forward_score, diag.slam_loop_reverse_score,
 diag.slam_loop_overlap, diag.slam_loop_forward_margin,
 diag.slam_loop_reverse_margin);
 }
 max_loops = std::max(max_loops, diag.slam_loops_closed);
    max_keyframes = std::max(max_keyframes, diag.slam_keyframes);
  }
 std::printf("loop: objectives_done=%d loops=%zu keyframes=%zu steps=%zu "
 "first_loop_objectives=%zu first_loop_truth=%.3f\n",
 session.objectives_done(), max_loops, max_keyframes,
 session.episode().size(), completed_objectives_at_first_loop,
 first_loop_truth_distance_m);
  fflush(stdout);
 if (!session.objectives_done() || max_loops == 0) {
 const auto& diag = session.episode().steps().back().diag;
 const auto& truth = session.episode().steps().back().truth_after;
 std::fprintf(stderr,
 "loop final: truth=(%.3f,%.3f,%.3f) est=(%.3f,%.3f,%.3f) "
 "vel=(%.3f,%.3f) sigma=%.3f loc=%s reason=%s goal=%d/%s\n",
 truth.body_x_m, truth.body_y_m, truth.body_yaw_rad,
 diag.estimate.x_m, diag.estimate.y_m, diag.estimate.yaw_rad,
 diag.estimate.forward_mps, diag.estimate.yaw_rate_radps,
 diag.estimate.position_sigma_m, diag.localization_action.c_str(),
 diag.evidence.reason_code.c_str(), static_cast<int>(diag.goal.state),
 diag.goal.detail.c_str());
 std::map<std::string, std::pair<std::size_t, std::uint64_t>> loop_reasons;
 for (const auto& step : session.episode().steps()) {
 auto& summary = loop_reasons[step.diag.slam_loop_diagnostic];
 ++summary.first;
 summary.second = step.step;
 }
 for (const auto& [reason, summary] : loop_reasons) {
 std::fprintf(stderr, " loop diagnostic count=%zu last_step=%llu: %s\n",
 summary.first, static_cast<unsigned long long>(summary.second),
 reason.c_str());
 }
 }
  assert(session.objectives_done());
  const auto& results = session.objectives().results();
  assert(results.size() == 4);
  for (const auto& r : results) assert(r.robot_done);

  // The machinery engaged: keyframes along the way, at least one accepted
  // closure when the square revisited its own submaps.
  assert(max_keyframes >= 10);  // ~16 m at 0.5 m spacing, minus holds
 assert(max_loops >= 1);
 // The first three mission events establish that the robot traversed east,
 // north, and west before any closure. SIM truth distance is diagnostic
 // only; the ROBOT loop detector receives neither mission progress nor truth.
 assert(completed_objectives_at_first_loop >= 3);

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
