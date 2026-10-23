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

}  // namespace

int main() {
  using sim::scenario::DeterministicScheduler;

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
    for (int i = 0; i < 700 && !session.objectives_done(); ++i) {
      session.step_once();
      dyn_clear.push_back(
          session.episode().steps().back().judge.dynamic_clearance_m);
    }
    assert(session.objectives_done());
    // Score: completion with a score, zero collision steps.
    const auto score = sim::eval::score_episode(session.episode());
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
      // NOTE: the 0.1 decision arg is currently inert (the coordinator
      // gates its own decision branch at 10 Hz); this run only proves
      // same-args determinism of the scan/record taps.
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

  // Unknown place fails closed through the seam: recorded miss, mission  // continues to the next objective instead of wedging.
  {
    sim::scenario::ScenarioSession session;
    session.setup_world().add_place({"dock", 3.0, 0.0, 0.0});
    session.set_objectives({"nope", "dock"}, 1.0);
    for (int i = 0; i < 400 && !session.objectives_done(); ++i) {
      session.step_once();
    }
    assert(session.objectives_done());
    const auto& results = session.objectives().results();
    assert(results.size() == 2);
    assert(!results[0].robot_done && !results[0].truth_arrived);
    assert(results[1].robot_done && results[1].truth_arrived);
  }

  // Step budget abandons a stuck objective: unreachable-in-time "far" is
  // recorded (not done, truth miss kept), its goal canceled through the
  // seam, and the mission continues onto "dock" instead of wedging. Zero
  // budget is legacy (no abandon): covered by every mission above.
  {
    sim::scenario::ScenarioSession session;
    session.setup_world().add_place({"far", 40.0, 0.0, 0.0});
    session.setup_world().add_place({"dock", 3.0, 0.0, 0.0});
    session.set_objectives({"far", "dock"}, 1.0);
    // Budget between dock's natural completion (~300 steps) and far's
    // reach (~1500+): far abandons, dock completes normally.
    session.set_objective_step_budget(400);
    for (int i = 0; i < 1500 && !session.objectives_done(); ++i) {
      session.step_once();
    }
    assert(session.objectives_done());
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
