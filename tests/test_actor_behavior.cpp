#include <cassert>
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <filesystem>
#include <string>
#include <vector>

#include "robot/app/robot_application.hpp"
#include "sim/providers/replay.hpp"
#include "sim/providers/sim_motion.hpp"
#include "sim/providers/static_map.hpp"
#include "sim/scenario/scenario_session.hpp"
#include "sim/world/actor_behavior.hpp"
#include "sim/world/world.hpp"

using namespace lost_signal;

namespace {

std::string tmp_path(const char* name) {
  return (std::filesystem::temp_directory_path() / name).string();
}

double dist(double x0, double y0, double x1, double y1) {
  return std::hypot(x1 - x0, y1 - y0);
}

}  // namespace

int main() {
  using sim::world::ActorBehavior;
  using sim::world::ActorBehaviorKind;
  using sim::world::ActorBehaviorRegistry;

  // Registry: builtins present, unknown fails closed, first wins.
  {
    ActorBehaviorRegistry& reg = ActorBehaviorRegistry::global();
    assert(reg.contains("constant-velocity"));
    assert(reg.contains("waypoints"));
    assert(reg.contains("wander"));
    assert(!reg.contains("nope"));
    ActorBehavior b;
    assert(reg.create("wander", b));
    assert(b.kind == ActorBehaviorKind::kWander);
    assert(!reg.create("nope", b));
    assert(!reg.register_kind("", nullptr));
    assert(!reg.register_kind("wander", nullptr));  // duplicate rejected
  }

  // Waypoints: square loop at 1 m/s, deterministic, revisits corners.
  {
    sim::world::World w;
    sim::world::Actor a;
    a.x = 0.0;
    a.y = 0.0;
    assert(ActorBehaviorRegistry::global().create("waypoints", a.behavior));
    a.behavior.waypoints = {{4.0, 0.0}, {4.0, 4.0}, {0.0, 4.0}, {0.0, 0.0}};
    a.behavior.speed_mps = 1.0;
    w.add_actor(a);
    double min_corner = 1e9;
    double path_length = 0.0;
    double prev_x = 0.0, prev_y = 0.0;
    double min_return = 1e9;
    for (int i = 0; i < 1600; ++i) {  // 32 s at exactly 1 m/s = 32 m of path
      w.step(0.02);
      const auto& p = w.actors()[0];
      min_corner =
          std::min({min_corner, dist(p.x, p.y, 4.0, 0.0),
                    dist(p.x, p.y, 4.0, 4.0), dist(p.x, p.y, 0.0, 4.0)});
      path_length += dist(prev_x, prev_y, p.x, p.y);
      prev_x = p.x;
      prev_y = p.y;
      if (i > 800) {  // second half: keeps coming back, not wandering off
        min_return = std::min(min_return, dist(p.x, p.y, 0.0, 0.0));
      }
      assert(std::hypot(p.vx_mps, p.vy_mps) <= 1.0 + 1e-9);
    }
    assert(min_corner < 0.3);  // every corner visited
    assert(std::abs(path_length - 32.0) < 0.5);  // constant speed throughout
    assert(min_return < 0.5);  // looping: returns near the start
  }

  // Non-loop waypoints: stops exactly at the final point.
  {
    sim::world::World w;
    sim::world::Actor a;
    ActorBehaviorRegistry::global().create("waypoints", a.behavior);
    a.behavior.waypoints = {{2.0, 0.0}, {2.0, 2.0}};
    a.behavior.speed_mps = 1.0;
    a.behavior.loop = false;
    w.add_actor(a);
    for (int i = 0; i < 600; ++i) w.step(0.02);
    const auto& p = w.actors()[0];
    assert(dist(p.x, p.y, 2.0, 2.0) < 0.3);
    assert(p.vx_mps == 0.0 && p.vy_mps == 0.0);  // latched stop
    for (int i = 0; i < 50; ++i) w.step(0.02);
    assert(w.actors()[0].vx_mps == 0.0);  // stays stopped
  }

  // Dwell: timed stop at each arrival (velocity zero, position frozen),
  // then proceeds; final point dwells once, then latches stopped.
  {
    sim::world::World w;
    sim::world::Actor a;
    ActorBehaviorRegistry::global().create("waypoints", a.behavior);
    a.behavior.waypoints = {{2.0, 0.0}, {2.0, 2.0}};
    a.behavior.speed_mps = 1.0;
    a.behavior.loop = false;
    a.behavior.dwell_s = 1.0;  // 50 steps at 0.02 s
    w.add_actor(a);
    auto step_until_near = [&](double tx, double ty, int cap) {
      int used = 0;
      while (used < cap &&
             dist(w.actors()[0].x, w.actors()[0].y, tx, ty) >= 0.3) {
        w.step(0.02);
        ++used;
      }
      return used;
    };
    assert(step_until_near(2.0, 0.0, 200) < 200);  // reached wp1
    const double dx = w.actors()[0].x, dy = w.actors()[0].y;
    for (int i = 0; i < 40; ++i) {  // inside the 50-step dwell
      w.step(0.02);
      const auto& p = w.actors()[0];
      assert(p.vx_mps == 0.0 && p.vy_mps == 0.0);
      assert(p.x == dx && p.y == dy);  // frozen, not drifting
    }
    assert(step_until_near(2.0, 2.0, 300) < 300);  // dwell over, proceeded
    for (int i = 0; i < 100; ++i) w.step(0.02);    // final latches stopped
    const auto& p = w.actors()[0];
    assert(dist(p.x, p.y, 2.0, 2.0) < 0.3);
    assert(p.vx_mps == 0.0 && p.vy_mps == 0.0);  // stays stopped
  }

  // Per-point waits: waits[i] overrides dwell_s at waypoints[i];
  // short lists fall back to dwell_s, negative entries read 0.
  {
    sim::world::World w;
    sim::world::Actor a;
    ActorBehaviorRegistry::global().create("waypoints", a.behavior);
    a.behavior.waypoints = {{2.0, 0.0}, {2.0, 2.0}};
    a.behavior.speed_mps = 1.0;
    a.behavior.loop = false;
    a.behavior.dwell_s = 1.0;            // fallback for points past waits
    a.behavior.waits_s = {0.5, -1.0};    // wp1: 25 steps; wp2: final stop
    w.add_actor(a);
    auto step_until_near = [&](double tx, double ty, int cap) {
      int used = 0;
      while (used < cap &&
             dist(w.actors()[0].x, w.actors()[0].y, tx, ty) >= 0.3) {
        w.step(0.02);
        ++used;
      }
      return used;
    };
    assert(step_until_near(2.0, 0.0, 200) < 200);
    int frozen = 0;  // wp1 wait is 0.5 s = 25 steps, not the 1.0 fallback
    for (int i = 0; i < 30; ++i) {
      w.step(0.02);
      const auto& p = w.actors()[0];
      if (p.vx_mps == 0.0 && p.vy_mps == 0.0) ++frozen;
      else break;
    }
    assert(frozen >= 20 && frozen <= 26);
    assert(step_until_near(2.0, 2.0, 300) < 300);  // proceeded onward
    // Short-list fallback: waits[1] is negative (reads 0), but the final
    // point latches stopped either way; prove fallback with a loop below.
    for (int i = 0; i < 60; ++i) w.step(0.02);
    const auto& p = w.actors()[0];
    assert(dist(p.x, p.y, 2.0, 2.0) < 0.3);
    assert(p.vx_mps == 0.0 && p.vy_mps == 0.0);
  }

  // Fallback: waits shorter than the list reads dwell_s per index.
  {
    sim::world::World w;
    sim::world::Actor a;
    ActorBehaviorRegistry::global().create("waypoints", a.behavior);
    a.behavior.waypoints = {{2.0, 0.0}, {4.0, 0.0}};
    a.behavior.speed_mps = 1.0;
    a.behavior.loop = true;
    a.behavior.dwell_s = 1.0;      // wp2 (past waits) dwells the full 1 s
    a.behavior.waits_s = {0.5};    // wp1 dwells 0.5 s
    w.add_actor(a);
    auto step_until_near = [&](double tx, double ty, int cap) {
      int used = 0;
      while (used < cap &&
             dist(w.actors()[0].x, w.actors()[0].y, tx, ty) >= 0.3) {
        w.step(0.02);
        ++used;
      }
      return used;
    };
    assert(step_until_near(2.0, 0.0, 200) < 200);
    int frozen1 = 0;  // per-point 0.5 s
    for (int i = 0; i < 60; ++i) {
      w.step(0.02);
      const auto& p = w.actors()[0];
      if (p.vx_mps == 0.0 && p.vy_mps == 0.0 &&
          dist(p.x, p.y, 2.0, 0.0) < 0.3) {
        ++frozen1;
      } else {
        break;
      }
    }
    assert(frozen1 >= 20 && frozen1 <= 26);
    assert(step_until_near(4.0, 0.0, 300) < 300);
    int frozen2 = 0;  // fallback 1.0 s = 50 steps
    for (int i = 0; i < 60; ++i) {
      w.step(0.02);
      const auto& p = w.actors()[0];
      if (p.vx_mps == 0.0 && p.vy_mps == 0.0) {
        ++frozen2;
      } else {
        break;
      }
    }
    assert(frozen2 >= 45 && frozen2 <= 51);
  }

  // Dwell on a loop: pauses eat distance but every corner is still visited.
  {
    sim::world::World w;
    sim::world::Actor a;
    a.x = 0.0;
    a.y = 0.0;
    assert(ActorBehaviorRegistry::global().create("waypoints", a.behavior));
    a.behavior.waypoints = {{4.0, 0.0}, {4.0, 4.0}, {0.0, 4.0}, {0.0, 0.0}};
    a.behavior.speed_mps = 1.0;
    a.behavior.dwell_s = 0.5;
    w.add_actor(a);
    double min_corner = 1e9;
    double path_length = 0.0;
    double prev_x = 0.0, prev_y = 0.0;
    int stopped_steps = 0;
    for (int i = 0; i < 1600; ++i) {  // 32 s wall clock, less motion
      w.step(0.02);
      const auto& p = w.actors()[0];
      min_corner =
          std::min({min_corner, dist(p.x, p.y, 4.0, 0.0),
                    dist(p.x, p.y, 4.0, 4.0), dist(p.x, p.y, 0.0, 4.0)});
      path_length += dist(prev_x, prev_y, p.x, p.y);
      prev_x = p.x;
      prev_y = p.y;
      if (p.vx_mps == 0.0 && p.vy_mps == 0.0) ++stopped_steps;
    }
    assert(min_corner < 0.3);  // every corner still visited
    assert(path_length < 31.0);  // pauses ate distance (no-dwell: 32 +- .5)
    assert(stopped_steps > 100);  // timed stops actually served
  }

  // Misconfigured waypoints (empty list): stops, never invents motion.
  {
    sim::world::World w;
    sim::world::Actor a{1.0, 1.0, 0.5, 0.5, 0.4};
    ActorBehaviorRegistry::global().create("waypoints", a.behavior);
    w.add_actor(a);
    for (int i = 0; i < 10; ++i) w.step(0.02);
    assert(w.actors()[0].vx_mps == 0.0 && w.actors()[0].vy_mps == 0.0);
    assert(w.actors()[0].x == 1.0 && w.actors()[0].y == 1.0);
  }

  // Wander: seeded determinism + bounded turn rate.
  {
    auto run = [](std::uint64_t seed) {
      sim::world::World w;
      sim::world::Actor a;
      ActorBehaviorRegistry::global().create("wander", a.behavior);
      a.behavior.seed = seed;
      a.behavior.speed_mps = 1.0;
      a.behavior.turn_rate_radps = 0.6;
      w.add_actor(a);
      std::vector<std::pair<double, double>> path;
      for (int i = 0; i < 200; ++i) {
        w.step(0.02);
        path.emplace_back(w.actors()[0].x, w.actors()[0].y);
      }
      return path;
    };
    const auto p1 = run(42);
    const auto p2 = run(42);
    assert(p1.size() == p2.size());
    for (std::size_t i = 0; i < p1.size(); ++i) {
      assert(p1[i] == p2[i]);  // same seed: bit-identical
    }
    const auto p3 = run(43);
    bool diverged = false;
    for (std::size_t i = 0; i < p1.size(); ++i) {
      if (p1[i] != p3[i]) {
        diverged = true;
        break;
      }
    }
    assert(diverged);  // different seed: different trajectory
    // Speed exact, heading rate bounded by the policy.
    sim::world::World w;
    sim::world::Actor a;
    ActorBehaviorRegistry::global().create("wander", a.behavior);
    a.behavior.seed = 7;
    a.behavior.speed_mps = 1.5;
    w.add_actor(a);
    double prev_heading = 0.0;
    for (int i = 0; i < 200; ++i) {
      w.step(0.02);
      const auto& p = w.actors()[0];
      assert(std::abs(std::hypot(p.vx_mps, p.vy_mps) - 1.5) < 1e-9);
      const double h = std::atan2(p.vy_mps, p.vx_mps);
      if (i > 0) {
        double dh = std::abs(h - prev_heading);
        if (dh > M_PI) dh = 2.0 * M_PI - dh;
        assert(dh <= 0.6 * 0.02 + 1e-9);
      }
      prev_heading = h;
    }
  }

  // Constant velocity unchanged: authored straight lines still fly straight.
  {
    sim::world::World w;
    w.add_actor({0.0, 0.0, 1.0, 0.5, 0.4});
    for (int i = 0; i < 100; ++i) w.step(0.02);
    assert(std::abs(w.actors()[0].x - 2.0) < 1e-9);
    assert(std::abs(w.actors()[0].y - 1.0) < 1e-9);
  }

 // A yielding actor uses continuous relative-motion prediction and its
 // declared braking capability. It stops outside the combined envelopes,
 // then accelerates along the authored path after the robot leaves.
 {
 sim::world::Actor actor;
 actor.x = 0.0;
 actor.y = -3.0;
 actor.radius_m = 0.3;
 actor.behavior.yield_to_robot = true;
 actor.behavior.reaction_time_s = 0.2;
 actor.behavior.acceleration_mps2 = 1.0;
 actor.behavior.brake_deceleration_mps2 = 2.0;
 actor.behavior.clearance_m = 0.2;
 double nearest = 1e9;
 for (int step = 0; step < 250; ++step) {
 actor.vx_mps = 0.0;
 actor.vy_mps = 1.0; // nominal scripted velocity
 sim::world::apply_actor_robot_yield(
 actor, actor.behavior, 0.0, 0.0, 0.0, 0.0, 0.4, 0.02);
 actor.x += actor.vx_mps * 0.02;
 actor.y += actor.vy_mps * 0.02;
 nearest = std::min(nearest, std::hypot(actor.x, actor.y));
 }
 assert(nearest >= 0.3 + 0.4 + 0.2 - 0.03);
 assert(actor.behavior.yield_active_);
 const double stopped_y = actor.y;
 // A single clear quantum cannot cancel an established right-of-way
 // commitment and restart the actor into the crossing.
 actor.vx_mps = 0.0;
 actor.vy_mps = 1.0;
 sim::world::apply_actor_robot_yield(
 actor, actor.behavior, 10.0, 0.0, 0.0, 0.0, 0.4, 0.02);
 assert(actor.vx_mps == 0.0 && actor.vy_mps == 0.0);
 assert(actor.behavior.yield_active_);
 for (int step = 0; step < 100; ++step) {
 actor.vx_mps = 0.0;
 actor.vy_mps = 1.0;
 sim::world::apply_actor_robot_yield(
 actor, actor.behavior, 10.0, 0.0, 0.0, 0.0, 0.4, 0.02);
 actor.y += actor.vy_mps * 0.02;
 }
 assert(actor.y > stopped_y + 0.5);
 assert(!actor.behavior.yield_active_);
 }

  // Scenario acceptance: waypoint crosser cuts the corridor; the chain
  // holds, resumes, and replay reproduces the exact decision sequence.
  {
    sim::scenario::ScenarioSession session;
    session.setup_world().add_place({"dock", 6.0, 0.0, 0.0});
    sim::world::Actor crosser;
    crosser.x = 2.0;
    crosser.y = -1.5;
    crosser.radius_m = 0.4;
    assert(
        ActorBehaviorRegistry::global().create("waypoints", crosser.behavior));
    crosser.behavior.waypoints = {{2.0, 4.0}};  // crosses, then leaves
    crosser.behavior.speed_mps = 1.0;
    crosser.behavior.loop = false;
    session.setup_world().add_actor(crosser);
    robot::iface::TaskRequest req;
    req.request_id = "crossing";
    req.place_id = "dock";
    assert(session.request_goal(req).valid());
    std::vector<double> xs;
    for (int i = 0; i < 280; ++i) {
      session.step_once();
      // RobotApplication is hidden inside the session; read progress from
      // the recorded truth instead (offline labels, never ROBOT input).
      xs.push_back(session.episode()
                       .steps()
                       .back()
                       .truth_after.body_x_m);
    }
    assert(xs[8] > 0.005);  // drove toward the goal initially
    bool held = false;
    for (int i = 20; i < 120; ++i) {
      if (xs[i] < 0.5) held = true;  // wandered less than half a meter in
    }
    assert(held);  // held while the crosser cut across
    assert(xs[279] > 2.0);  // resumed after the crosser passed

    // Replay: same episode through a fresh stack + fresh plant gives the
    // identical MotionCommand sequence (decision determinism).
    const std::string path = tmp_path("test_actor_replay.jsonl");
    std::string err;
    assert(sim::providers::export_episode(path, "sim-world", 2,
                                          session.episode(), &err));
    sim::providers::ReplayProvider replay;
    assert(replay.load(path, &err));
    sim::world::World map_world;
    map_world.add_place({"dock", 6.0, 0.0, 0.0});
    auto maps = std::make_unique<sim::providers::StaticMapProvider>(
        &map_world, -10.0, -10.0, 20.0, 20.0, 0.25);
    robot::app::RobotApplication app(std::move(maps));
    assert(app.configure().ok && app.activate().ok);
 for (const auto& descriptor : replay.descriptors()) {
 assert(app.register_provider(descriptor));
 }
    robot::iface::TaskRequest replay_req;
    replay_req.request_id = "crossing";
    replay_req.place_id = "dock";
    assert(app.request_goal(replay_req).valid());
    sim::world::World plant;
    sim::providers::SimMotionEndpoint motion(&plant);
    std::vector<robot::iface::MotionCommand> live_cmds;
    for (const auto& st : session.episode().steps())
      live_cmds.push_back(st.cmd);
    std::vector<robot::iface::MotionCommand> replayed_cmds;
    robot::iface::ObservationFrame obs;
    while (replay.poll(obs)) {
      assert(app.publish_observation(obs));
      replayed_cmds.push_back(app.tick(obs.stamp_ns));
      motion.consume_command(app.take_actuator(obs.stamp_ns));
      plant.step(0.02);
    }
    assert(replayed_cmds.size() == live_cmds.size());
    for (std::size_t i = 0; i < live_cmds.size(); ++i) {
      assert(replayed_cmds[i].space == live_cmds[i].space);
      assert(replayed_cmds[i].forward_mps == live_cmds[i].forward_mps);
      assert(replayed_cmds[i].yaw_rate_radps == live_cmds[i].yaw_rate_radps);
    }
    assert(!replay.poll(obs));
    std::filesystem::remove(path);
  }

  std::printf("actor_behavior ok\n");
  return 0;
}
