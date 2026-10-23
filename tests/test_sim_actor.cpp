#include <cassert>
#include <cstdio>
#include <fstream>
#include <vector>

#include "robot/iface/task.hpp"
#include "sim/scenario/actor_sidecar.hpp"
#include "sim/scenario/scenario_session.hpp"
#include "sim/robot/sim_robot.hpp"

int main() {
  using namespace lost_signal;

  // Actor crosses the robot's path: drive -> hold while near -> resume.
  sim::robot::SimRobot sim;
  assert(sim.configure().ok && sim.activate().ok);
  sim::world::Actor crosser;
  crosser.x = 2.0;
  crosser.y = -1.5;
  crosser.vx_mps = 0.0;
  crosser.vy_mps = 1.0;
  crosser.radius_m = 0.4;
  // Actors are sim-owned test setup: configured before the run.
  sim.setup_world().add_actor(crosser);
  sim.setup_world().add_place({"dock", 0.0, 0.0, 0.0});

  robot::iface::TaskRequest req;
  req.request_id = "crossing";
  req.place_id = "dock";
  assert(sim.request_goal(req).valid());
  sim.set_goal({6.0, 0.0, 0.0});

  std::vector<double> xs;
  for (int i = 0; i < 280; ++i) {
    sim.tick(static_cast<robot::core::MonotonicNs>(i * 20'000'000));
    xs.push_back(sim.world().truth().body_x_m);
  }
  // Rigid-coupling plant accelerates at ΣF/M (4.5 m/s^2 full duty), so
  // Engine line is lazier off the line; still strictly toward the goal,
  // holds for the actor, resumes after it passes.
  assert(xs[8] > 0.005);              // drove toward the goal initially
  assert(xs[60] < 0.5);               // held while the actor was near
  assert(xs[279] > 2.0);              // resumed after the actor passed
  assert(sim.current_goal().state == robot::iface::GoalState::kActive);

 // Headless sidecar: the same loader used by lost_signal_sim.
 {
 std::ofstream out("/tmp/test_actors_sidecar.json");
 out << R"({"actors":[{"x":1.5,"y":2.0,"radius_m":0.3,
 "behavior":"wander","speed_mps":0.7,"seed":4}]})";
 out.close();
 sim::scenario::ScenarioSession session;
 std::string err;
 const int n = sim::scenario::load_actors_sidecar(
 "/tmp/test_map.json", session, "/tmp/test_actors_sidecar.json", &err);
 assert(n == 1);
 assert(session.setup_world().actors().size() == 1);
 assert(session.setup_world().actors()[0].radius_m == 0.3);
 assert(session.setup_world().actors()[0].behavior.speed_mps == 0.7);
 std::remove("/tmp/test_actors_sidecar.json");
 }

  std::printf("sim_actor ok: x8=%.2f x60=%.2f x_end=%.2f\n", xs[8], xs[60],
              xs[279]);
  return 0;
}
