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
      {4.0, -10.0, 5.0, -1.0, 1.5},
      {4.0, 1.0, 5.0, 10.0, 1.5},
  });
  sim.setup_world().add_place({"far", 8.0, 0.0, 0.0});
  sim.setup_world().add_place({"near", 2.0, 0.0, 0.0});

  robot::iface::TaskRequest req;
  req.request_id = "nav";
  req.place_id = "far";
  assert(sim.request_goal(req).valid());
  sim.set_goal({8.0, 0.0, 0.0});

  std::vector<double> xs;
 // Whole-map localization is cooperatively sliced across control ticks.
 // Allow the mission to finish its ordinary arrival-settling interval after
 // that bounded startup work instead of pinning completion to one trajectory.
 for (int i = 0; i < 800; ++i) {
    sim.tick(static_cast<robot::core::MonotonicNs>(i * 20'000'000));
    xs.push_back(sim.world().truth().body_x_m);
    if (sim.current_goal().state == robot::iface::GoalState::kDone) break;
  }
  assert(xs.back() > 7.0);  // reached beyond the wall through the gap
  // Arrival completes the goal asynchronously: arrived inside 0.5 m of far.
 const auto nav_goal = sim.current_goal();
 const auto nav_evidence = sim.last_evidence();
 const auto nav_diag = sim.diagnostics();
 if (nav_goal.state != robot::iface::GoalState::kDone) {
 std::fprintf(stderr,
 "sim_nav final: steps=%zu truth=(%.3f,%.3f,%.3f) "
 "est=(%.3f,%.3f,%.3f) vel=(%.3f,%.3f) sigma=%.3f "
 "loc=%s state=%d detail=%s "
 "constraint=%s reason=%s cmd=(%.3f,%.3f)\n",
 xs.size(), sim.world().truth().body_x_m, sim.world().truth().body_y_m,
 sim.world().truth().body_yaw_rad, nav_diag.estimate.x_m,
 nav_diag.estimate.y_m, nav_diag.estimate.yaw_rad,
 nav_diag.estimate.forward_mps, nav_diag.estimate.yaw_rate_radps,
 nav_diag.estimate.position_sigma_m, nav_diag.localization_action.c_str(),
 static_cast<int>(nav_goal.state),
 nav_goal.detail.c_str(), nav_evidence.constraint_code.c_str(),
 nav_evidence.reason_code.c_str(), nav_evidence.target_speed_mps,
 nav_evidence.target_yaw_rate_radps);
 }
 assert(sim.current_goal().state == robot::iface::GoalState::kDone);
  assert(sim.current_goal().detail == "arrived");

  // Preempt + cancel: a new request cancels the running goal; canceling an
  // active goal withdraws it; terminal handles stay queryable.
  robot::iface::TaskRequest again;
  again.request_id = "nav2";
  again.place_id = "far";
  robot::iface::GoalHandle h2 = sim.request_goal(again);
  assert(h2.valid());
  assert(sim.goal_status(h2).state == robot::iface::GoalState::kActive);
  robot::iface::TaskRequest side;
  side.request_id = "nav3";
  side.place_id = "near";
  robot::iface::GoalHandle h3 = sim.request_goal(side);
  assert(h3.valid());
  assert(sim.goal_status(h2).state == robot::iface::GoalState::kCanceled);
  assert(sim.goal_status(h2).detail == "preempted");
  assert(sim.goal_status(h3).state == robot::iface::GoalState::kActive);
  assert(sim.cancel_goal(h3));
  assert(sim.goal_status(h3).state == robot::iface::GoalState::kCanceled);
  assert(!sim.cancel_goal(h3));  // terminal: nothing to withdraw

  std::printf("sim_nav ok: steps=%zu x_end=%.2f\n", xs.size(), xs.back());
  return 0;
}
