#include <cassert>
#include <cstdio>
#include <vector>

#include "robot/iface/task.hpp"
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
  // the creep-out is lazier than the wheel-mass days; still strictly
  // positive motion toward the goal.
  assert(xs[8] > 0.01);               // drove toward the goal initially
  assert(xs[60] < 0.5);               // held while the actor was near
  assert(xs[279] > 2.0);              // resumed after the actor passed
  assert(sim.current_goal().state == robot::iface::GoalState::kActive);

  std::printf("sim_actor ok: x8=%.2f x60=%.2f x_end=%.2f\n", xs[8], xs[60],
              xs[279]);
  return 0;
}
