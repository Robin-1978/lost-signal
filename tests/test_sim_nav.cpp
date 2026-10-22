#include <cassert>
#include <cstdio>
#include <vector>

#include "robot/iface/task.hpp"
#include "sim/robot/sim_robot.hpp"

// Static wall with a gap + goal beyond: A* routes through the gap (static
// map), DWA drives it (rolling map), validator + safety guard raw scans.
int main() {
  using namespace lost_signal;

  sim::robot::SimRobot sim;
  assert(sim.configure().ok && sim.activate().ok);
  sim.setup_world().set_static_boxes({
      {4.0, -10.0, 5.0, -1.0},
      {4.0, 1.0, 5.0, 10.0},
  });
  sim.setup_world().add_place({"far", 8.0, 0.0, 0.0});

  robot::iface::TaskRequest req;
  req.request_id = "nav";
  req.place_id = "far";
  assert(sim.submit_task(req));
  sim.set_goal({8.0, 0.0, 0.0});

  std::vector<double> xs;
  for (int i = 0; i < 600; ++i) {
    sim.tick(static_cast<robot::core::MonotonicNs>(i * 20'000'000));
    xs.push_back(sim.world().truth().body_x_m);
    if (xs.back() > 7.0) break;
  }
  assert(xs.back() > 7.0);  // reached beyond the wall through the gap
  assert(sim.task_status().state == robot::iface::TaskState::kActive);

  std::printf("sim_nav ok: steps=%zu x_end=%.2f\n", xs.size(), xs.back());
  return 0;
}
