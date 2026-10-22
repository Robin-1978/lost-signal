#include <cassert>
#include <cstdio>
#include <memory>

#include "robot/app/robot_application.hpp"
#include "robot/robots/differential_robot.hpp"
#include "sim/robot/sim_robot.hpp"

int main() {
  using namespace lost_signal;

  // Both deployments behind one IRobot* facade.
  auto native = std::make_unique<robot::app::RobotApplication>(nullptr);
  native->register_provider({"t.lidar", robot::iface::EndpointRole::kLidar,
                             robot::iface::ProviderBackend::kNative,
                             "lidar_link"});
  robot::platform::DifferentialChassis chassis(
      0.1, 0.5,
      {.max_forward_mps = 2.0,
       .max_yaw_rate_radps = 1.5,
       .footprint_radius_m = 0.4});
  robot::robots::DifferentialRobot diff(std::move(native), chassis);

  sim::robot::SimRobot sim;
  robot::app::IRobot* robots[2] = {&diff, &sim};

  for (auto* r : robots) {
    assert(r->configure());
    assert(r->activate());
    assert(r->bringup_state() ==
           robot::lifecycle::ModuleState::kActive);
    robot::iface::TaskRequest req;
    req.request_id = "t";
    req.place_id = "dock";
    assert(r->submit_task(req));
    robot::iface::MotionCommand cmd = r->tick(1'000'000);
    // v1 skeleton holds position through the chassis envelope.
    assert(cmd.is_safe_stop());
    assert(r->task_status().state == robot::iface::TaskState::kActive);
  }
  // SimRobot actually stepped its world while ticking.
  assert(sim.world().truth().step >= 1);

  std::printf("sim_robot ok\n");
  return 0;
}
