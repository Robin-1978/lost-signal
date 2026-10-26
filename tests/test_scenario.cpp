#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "sim/eval/scoring.hpp"
#include "sim/scenario/collision_judge.hpp"
#include "sim/scenario/objectives.hpp"
#include "sim/scenario/scheduler.hpp"
#include "sim/scenario/scenario_session.hpp"
#include "sim/world/actor_behavior.hpp"
#include "sim/world/world.hpp"

using namespace lost_signal;

namespace {

double min_over(const std::vector<double>& v) {
  double m = 1e9;
  for (double x : v) m = std::min(m, x);
  return m;
}

bool has_reason(const sim::scenario::ObjectiveResult& result,
 const std::string& reason) {
 return std::find(result.failure_reasons.begin(),
 result.failure_reasons.end(), reason) != result.failure_reasons.end();
}

}  // namespace

int main() {
  using sim::scenario::DeterministicScheduler;

 // Planar obstacle clearance and terrain/belly clearance are different
 // physical metrics. A nominal 5 cm belly gap must not make every obstacle
 // clearance score read as 5 cm.
 {
 sim::eval::EpisodeRecorder episode;
 sim::eval::EvalStep step;
 step.judge.static_clearance_m = 0.4;
 step.judge.dynamic_clearance_m = 0.8;
 step.judge.terrain_clearance_m = 0.05;
 episode.record(step);
 const auto score = sim::eval::score_episode(episode);
 assert(std::abs(score.min_clearance_m - 0.4) < 1e-12);
 assert(std::abs(score.mean_clearance_m - 0.4) < 1e-12);
 assert(std::abs(score.min_terrain_clearance_m - 0.05) < 1e-12);
 assert(std::abs(score.mean_terrain_clearance_m - 0.05) < 1e-12);
 }

  // Scheduler: sim-time deadlines, never wall clock.
  {
    DeterministicScheduler sched;
    sched.reset(0.1, 0.2, 0.0);  // record disabled (period 0)
    assert(sched.scan_due(0.0));   // first deadline 0: due immediately
    assert(!sched.scan_due(0.05));
    assert(sched.scan_due(0.1));
    assert(sched.decision_due(0.1));  // first deadline 0 as well
    assert(!sched.decision_due(0.15));
    assert(sched.decision_due(0.2));
    assert(!sched.record_due(1e9));  // disabled stays disabled
    bool threw = false;
    try {
      DeterministicScheduler bad;
      bad.reset(-0.1, 0.1, 0.1);
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    assert(threw);
    threw = false;
    try {
      sim::scenario::PeriodicDeadline d;
      d.reset(0.1, std::numeric_limits<double>::quiet_NaN());
    } catch (const std::invalid_argument&) {
      threw = true;
    }
    assert(threw);
 assert(sim::scenario::on_physics_grid(0.007));
 assert(!sim::scenario::on_physics_grid(0.0065));
  }

  // Judge: circle-vs-box/actor assessment, honest clearances.
  {
    using sim::scenario::assess_collision;
    std::vector<sim::world::StaticBox> boxes{{1.0, 1.0, 2.0, 2.0, 1.0}};
    std::vector<sim::world::Actor> actors;
    // Clear: nearest box corner (1,1) is sqrt(2) away, minus radius.
    auto clear = assess_collision(0.0, 0.0, 0.4, boxes, actors);
    assert(!clear.static_collision && !clear.dynamic_collision);
    assert(std::abs(clear.static_clearance_m - (std::sqrt(2.0) - 0.4)) < 1e-9);
    assert(clear.dynamic_clearance_m == 999.0);  // no actors: no evidence
    // Touching: center 0.3 from the box edge with radius 0.4.
    auto hit = assess_collision(0.7, 1.5, 0.4, boxes, actors);
    assert(hit.static_collision);
    assert(hit.static_clearance_m < 0.0);
    // Actor: radii sum decides.
    sim::world::Actor a;
    a.x = 1.0;
    a.y = 0.0;
    a.radius_m = 0.4;
    actors.push_back(a);
    auto near = assess_collision(0.0, 0.0, 0.4, {}, actors);
    assert(!near.dynamic_collision);
    assert(std::abs(near.dynamic_clearance_m - 0.2) < 1e-9);
    auto bump = assess_collision(0.5, 0.0, 0.4, {}, actors);
    assert(bump.dynamic_collision);
    // Degenerate box (inverted corners) still assesses by clamp, no NaN.
    std::vector<sim::world::StaticBox> weird{{2.0, 2.0, 1.0, 1.0, 1.0}};
    auto w = assess_collision(0.0, 0.0, 0.4, weird, {});
    assert(!w.static_collision && std::isfinite(w.static_clearance_m));
    // Flat paint (height <= 0): no volume — judge, engine, map all ignore.
    std::vector<sim::world::StaticBox> paint{{0.5, -0.5, 1.5, 0.5, 0.0}};
    auto p = assess_collision(1.0, 0.0, 0.4, paint, {});
    assert(!p.static_collision && p.static_clearance_m > 900.0);
  }

  // Terrain-contact judge: belly clearance vs the authored surface (pure
  // function, same numbers the scenario composes into the episode judge).
  {
    using sim::scenario::assess_terrain_contact;
    std::vector<sim::world::TerrainPatch> flat;
    std::vector<sim::world::SupportSlab> no_slabs;
    // Flat floor at ride height: clear with margin.
    auto clear = assess_terrain_contact(0.0, 0.0, 0.15, 0.4, flat, no_slabs,
                                        0.05);
    assert(!clear.terrain_collision);
    assert(std::abs(clear.terrain_clearance_m - 0.10) < 1e-9);
    // Center ridge under the belly: scrape.
    std::vector<sim::world::TerrainPatch> ridge = {
        {"spine", -2.0, -0.1, 8.0, 0.1, 0.12, 0.0, 0.0, 0.0, 0.0}};
    auto scrape = assess_terrain_contact(1.0, 0.0, 0.15, 0.4, ridge, no_slabs,
                                         0.05);
    assert(scrape.terrain_collision);
    assert(scrape.terrain_clearance_m < 0.0);
    // Deck slab counts as surface too.
    std::vector<sim::world::SupportSlab> deck = {
        {"deck", 1.0, 0.0, 1.0, 1.0, 0.0, 0.14, 0.0}};
    auto deck_hit = assess_terrain_contact(1.0, 0.0, 0.15, 0.4, flat, deck,
                                           0.05);
    assert(deck_hit.terrain_collision);
  }

  // Full mission: two docks, a crosser, an off-path box. Headless to
  // completion, judged every step, scored at the end.
  {
    sim::scenario::ScenarioSession session;
 // This mission deliberately has no globally localizable survey; its single
 // off-path box is live obstacle context, not an initial-pose source.
 session.set_startup_mode(robot::autonomy::StartupMode::kSlamOnly);
    session.setup_world().add_place({"dockA", 3.0, 0.0, 0.0});
    session.setup_world().add_place({"dockB", 6.0, 0.0, 0.0});
    // Off-path box: finite clearance, never a collision.
    sim::world::StaticBox box;
    box.x0 = 1.0;
    box.y0 = 1.0;
    box.x1 = 2.0;
    box.y1 = 2.0;
    box.height_m = 1.0;  // true prism: the scan must see it off-path
    session.setup_world().set_static_boxes({box});
    // Crosser cuts the corridor early, then leaves (S4 behavior).
    sim::world::Actor crosser;
    crosser.x = 2.0;
    crosser.y = -1.5;
    crosser.radius_m = 0.4;
    assert(sim::world::ActorBehaviorRegistry::global().create(
        "waypoints", crosser.behavior));
    crosser.behavior.waypoints = {{2.0, 4.0}};
    crosser.behavior.speed_mps = 1.0;
    crosser.behavior.loop = false;
    session.setup_world().add_actor(crosser);
    // Driver tolerance 1.0 m: the ROBOT arrives per its estimate (0.5 m
    // radius); truth-side counting budgets the honest mapless drift on top.
    session.set_objectives({"dockA", "dockB"}, 1.0);

    std::vector<double> dyn_clear;
 // Precise arrival now brakes, settles, and confirms each pose instead of
 // declaring Done at a footprint-sized distance. Budget the two full stops.
 for (int i = 0; i < 1000 && !session.objectives_done(); ++i) {
      session.step_once();
      dyn_clear.push_back(
          session.episode().steps().back().judge.dynamic_clearance_m);
    }
 if (!session.objectives_done()) {
 const auto& stalled = session.episode().steps().back();
 std::fprintf(stderr,
 "scenario stalled: truth=(%.3f,%.3f) est=(%.3f,%.3f) sigma=%.3f "
 "goal=%d/%s decision=%s/%s\n",
 stalled.truth_after.body_x_m, stalled.truth_after.body_y_m,
 stalled.diag.estimate.x_m, stalled.diag.estimate.y_m,
 stalled.diag.estimate.position_sigma_m,
 static_cast<int>(stalled.diag.goal.state),
 stalled.diag.goal.detail.c_str(),
 stalled.diag.evidence.constraint_code.c_str(),
 stalled.diag.evidence.reason_code.c_str());
 }
 assert(session.objectives_done());
 assert(session.objectives_passed());
    // Score: completion with a score, zero collision steps.
 const auto score = sim::eval::score_episode(session.episode());
 const auto& scenario_last = session.episode().steps().back();
 if (score.max_pos_err_m >= 0.5) {
 std::fprintf(stderr,
 "scenario final: steps=%zu done=%d pos_err=%.3f truth=(%.3f,%.3f) "
 "est=(%.3f,%.3f) sigma=%.3f loc=%s safety=%s goal=%d/%s\n",
 score.steps, session.objectives_done(), score.max_pos_err_m,
 scenario_last.truth_after.body_x_m, scenario_last.truth_after.body_y_m,
 scenario_last.diag.estimate.x_m, scenario_last.diag.estimate.y_m,
 scenario_last.diag.estimate.position_sigma_m,
 scenario_last.diag.localization_action.c_str(),
 scenario_last.diag.safety_reason.c_str(),
 static_cast<int>(scenario_last.diag.goal.state),
 scenario_last.diag.goal.detail.c_str());
 }
    // Estimator health over the mission (offline truth-vs-estimate): the
    // stack drifts decimeters on a 6 m mapless run; the driver tolerance
    // below budgets exactly this.
    assert(score.max_pos_err_m < 0.5);
    const auto& results = session.objectives().results();
    assert(results.size() == 2);
    assert(results[0].place_id == "dockA" && results[1].place_id == "dockB");
    assert(results[0].robot_done && results[1].robot_done);
    assert(results[0].truth_arrived && results[1].truth_arrived);
    assert(results[0].truth_miss_m <= 1.0 && results[1].truth_miss_m <= 1.0);

    // Judge fired every step, never a collision; the crosser dipped the
    // dynamic clearance mid-run, the box gave finite static clearance.
    std::size_t collisions = 0;
    double min_static = 1e9;
    for (const auto& st : session.episode().steps()) {
      if (st.judge.static_collision || st.judge.dynamic_collision) {
        ++collisions;
      }
      min_static = std::min(min_static, st.judge.static_clearance_m);
    }
    assert(collisions == 0);
    assert(min_static < 1.0 && min_static > 0.0);  // the box was seen
    assert(min_over(dyn_clear) < 2.0);  // the crosser passed close


    assert(score.steps == session.episode().size());
    assert(score.collision_steps == 0);
    assert(score.distance_m > 5.0);  // (0,0) -> (6,0), truth displacement
    assert(score.obs_rejected == 0);
  }

  // Cadence determinism: a slower decision tap is still pure sim time —
  // two runs with identical cadences produce identical commands.
  {
    auto run = []() {
      sim::scenario::ScenarioSession session;
      session.setup_world().add_place({"dock", 4.0, 0.0, 0.0});
 // The decision tap is an independent sim-time deadline and is applied to
 // the coordinator's planning branch; this run also proves deterministic
 // command output under the slower cadence.
      session.set_cadence(0.02, 0.1, 0.02);
      session.set_objectives({"dock"});
      std::vector<robot::iface::MotionCommand> cmds;
      for (int i = 0; i < 100; ++i) {
        session.step_once();
        cmds.push_back(session.episode().steps().back().cmd);
      }
      return cmds;
    };
    const auto a = run();
    const auto b = run();
    assert(a.size() == b.size());
    for (std::size_t i = 0; i < a.size(); ++i) {
      assert(a[i].space == b[i].space);
      assert(a[i].forward_mps == b[i].forward_mps);
      assert(a[i].yaw_rate_radps == b[i].yaw_rate_radps);
    }
  }

 // Sensor cadence selects samples from the canonical 1 ms physics grid; it
 // cannot repartition integration. With no motion task, radically different
 // legal sensor rates therefore leave every physical state bit-identical.
 {
 auto settle = [](double scan_s, double motion_s, double imu_s) {
 sim::scenario::ScenarioSession session;
 session.set_sensor_cadence(scan_s, motion_s, imu_s, 0.02, 0.02);
 for (int i = 0; i < 50; ++i) session.step_once();
 return session.setup_world().truth();
 };
 const auto regular = settle(0.02, 0.02, 0.02);
 const auto staggered = settle(0.007, 0.011, 0.013);
 assert(regular.step == 50 && staggered.step == 50);
 assert(regular.body_x_m == staggered.body_x_m);
 assert(regular.body_y_m == staggered.body_y_m);
 assert(regular.body_z_m == staggered.body_z_m);
 assert(regular.body_yaw_rad == staggered.body_yaw_rad);
 assert(regular.body_roll_rad == staggered.body_roll_rad);
 assert(regular.body_pitch_rad == staggered.body_pitch_rad);
 assert(regular.forward_mps == staggered.forward_mps);
 assert(regular.lateral_mps == staggered.lateral_mps);
 assert(regular.yaw_rate_radps == staggered.yaw_rate_radps);
 assert(regular.drive_angle_rad == staggered.drive_angle_rad);
 assert(regular.drive_radps == staggered.drive_radps);
 }

 // Real asynchronous rates: a 10 Hz scan remains valid evidence between
 // 50 Hz decisions while 50 Hz motion and 100 Hz IMU continue updating the
 // estimator. The old envelope replacement bug stopped on four of every
 // five decisions and barely moved in this open mission.
 {
 sim::scenario::ScenarioSession session;
 session.set_startup_mode(robot::autonomy::StartupMode::kSlamOnly);
 session.setup_world().add_place({"dock", 8.0, 0.0, 0.0});
 session.set_sensor_cadence(0.10, 0.02, 0.01, 0.02, 0.02);
 session.set_objectives({"dock"}, 1.0);
 for (int i = 0; i < 300; ++i) session.step_once();
 const auto score = sim::eval::score_episode(session.episode());
 if (!(score.distance_m > 3.0)) {
 std::fprintf(stderr, "async cadence distance=%.3f m steps=%zu\n",
 score.distance_m, score.steps);
 }
 assert(score.distance_m > 3.0);
 const auto& steps = session.episode().steps();
 assert(!steps.empty());
 // Two distinct IMU acquisitions per 20 ms transition prove the configured
 // 100 Hz stream is sampled rather than silently aliased to 50 Hz.
 assert(steps.back().obs.imu_header.sequence == 600);
 for (std::size_t i = 5; i < steps.size(); ++i) {
 assert(steps[i].diag.safety_reason != "no-scan-evidence");
 }
 }

 // Unknown place fails closed through the seam: recorded miss; mission
 // continues to the next objective instead of wedging.
  {
    sim::scenario::ScenarioSession session;
    session.setup_world().add_place({"dock", 3.0, 0.0, 0.0});
 session.setup_world().set_static_boxes({
 {1.0, 1.0, 2.0, 2.0, 1.0}});
 session.set_initial_pose(0.0, 0.0, 0.0, 0.02, 0.01);
    session.set_objectives({"nope", "dock"}, 1.0);
 for (int i = 0; i < 800 && !session.objectives_done(); ++i) {
      session.step_once();
    }
 if (!session.objectives_done()) {
 const auto& stalled = session.episode().steps().back();
 std::fprintf(stderr,
 "unknown-place continuation stalled: truth=(%.3f,%.3f) "
 "est=(%.3f,%.3f) sigma=%.3f goal=%d/%s decision=%s/%s\n",
 stalled.truth_after.body_x_m, stalled.truth_after.body_y_m,
 stalled.diag.estimate.x_m, stalled.diag.estimate.y_m,
 stalled.diag.estimate.position_sigma_m,
 static_cast<int>(stalled.diag.goal.state),
 stalled.diag.goal.detail.c_str(),
 stalled.diag.evidence.constraint_code.c_str(),
 stalled.diag.evidence.reason_code.c_str());
 }
    assert(session.objectives_done());
 assert(!session.objectives_passed());
    const auto& results = session.objectives().results();
    assert(results.size() == 2);
    assert(!results[0].robot_done && !results[0].truth_arrived);
 assert(has_reason(results[0], "robot_goal_rejected"));
 assert(has_reason(results[0], "truth_place_missing"));
 if (!(results[1].robot_done && results[1].truth_arrived)) {
 std::fprintf(stderr,
 "unknown-place follow-up: robot_done=%d truth_arrived=%d "
 "miss=%.3f yaw=%.3f speed=%.3f yaw_rate=%.3f stable=%u\n",
 results[1].robot_done, results[1].truth_arrived,
 results[1].truth_miss_m, results[1].truth_yaw_error_rad,
 results[1].truth_linear_speed_mps, results[1].truth_yaw_rate_radps,
 results[1].truth_stable_steps);
 for (const auto& reason : results[1].failure_reasons) {
 std::fprintf(stderr, " failure=%s\n", reason.c_str());
 }
 }
    assert(results[1].robot_done && results[1].truth_arrived);
 assert(session.objectives().successful() == 1);
 assert(session.objectives().completion_ratio() == 0.5);
  }

 // A ROBOT Done report cannot pass a SIM objective by itself. Use a truth
 // tolerance tighter than the ROBOT arrival radius so the estimate can
 // legitimately finish while the physical body remains outside the SIM
 // acceptance circle.
 {
 sim::scenario::ScenarioSession session;
 session.setup_world().add_place({"dock", 3.0, 0.0, 0.0});
 session.setup_world().set_static_boxes({
 {1.0, 1.0, 2.0, 2.0, 1.0}});
 session.set_initial_pose(0.0, 0.0, 0.0, 0.02, 0.01);
 session.set_objectives({"dock"}, 0.01);
 for (int i = 0; i < 800 && !session.objectives_done(); ++i) {
 session.step_once();
 }
 assert(session.objectives_done());
 assert(!session.objectives_passed());
 const auto& result = session.objectives().results().front();
 assert(result.robot_done);
 assert(!result.truth_arrived);
 assert(!result.successful());
 assert(has_reason(result, "truth_position_outside_tolerance"));
 assert(session.objectives().completion_ratio() == 0.0);
 }

 // SIM acceptance requires a continuous truth-side stability window. The
 // ROBOT can correctly finish from its own estimate while the independent
 // evaluator rejects an incomplete window; the verdict never re-enters the
 // ROBOT goal state or diagnostics.
 {
 sim::scenario::ScenarioSession session;
 session.setup_world().add_place({"dock", 3.0, 0.0, 0.0});
 session.setup_world().set_static_boxes({
 {1.0, 1.0, 2.0, 2.0, 1.0}});
 session.set_initial_pose(0.0, 0.0, 0.0, 0.02, 0.01);
 sim::eval::ObjectiveAcceptanceConfig acceptance;
 acceptance.position_tolerance_m = 1.0;
 acceptance.heading_tolerance_rad = 1.0;
 acceptance.maximum_linear_speed_mps = 1.0;
 acceptance.maximum_yaw_rate_radps = 1.0;
 acceptance.stable_steps = 100'000;
 assert(session.set_objective_acceptance(acceptance));
 session.set_objectives({"dock"});
 for (int i = 0; i < 800 && !session.objectives_done(); ++i) {
 session.step_once();
 }
 assert(session.objectives_done());
 const auto& result = session.objectives().results().front();
 assert(result.robot_done);
 assert(!result.truth_arrived);
 assert(result.truth_stable_steps > 0);
 assert(has_reason(result, "truth_stability_window_incomplete"));
 assert(session.current_goal().state == robot::iface::GoalState::kDone);
 }

  // Step budget abandons a stuck objective: unreachable-in-time "far" is
  // recorded (not done, truth miss kept), its goal canceled through the
  // seam, and the mission continues onto "dock" instead of wedging. Zero
  // budget is legacy (no abandon): covered by every mission above.
  {
    sim::scenario::ScenarioSession session;
    session.setup_world().add_place({"far", 40.0, 0.0, 0.0});
    session.setup_world().add_place({"dock", 3.0, 0.0, 0.0});
 session.setup_world().set_static_boxes({
 {1.0, 1.0, 2.0, 2.0, 1.0}});
 session.set_initial_pose(0.0, 0.0, 0.0, 0.02, 0.01);
    session.set_objectives({"far", "dock"}, 1.0);
 // Budget between dock's natural completion and far's
    // reach (~1500+): far abandons, dock completes normally.
 session.set_objective_step_budget(700);
 for (int i = 0; i < 2000 && !session.objectives_done(); ++i) {
      session.step_once();
    }
    assert(session.objectives_done());
 assert(!session.objectives_passed());
    const auto& results = session.objectives().results();
    assert(results.size() == 2);
    assert(results[0].abandoned && !results[0].robot_done);
    assert(results[0].truth_miss_m > 20.0);  // miss kept at abandon time
    assert(!results[1].abandoned && results[1].robot_done &&
           results[1].truth_arrived);  // mission continued cleanly
  }

  std::printf("scenario ok\n");
  return 0;
}
