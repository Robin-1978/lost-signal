#pragma once

#include "robot/app/robot_application.hpp"
#include "sim/providers/sim_platform.hpp"
#include "sim/world/world.hpp"

namespace lost_signal::sim::game {

// SIM composition root. Owns world truth and presentation state; drives the
// robot only through RobotApplication + LSRI. Must never call autonomy,
// estimator, planner, or safety code directly.
class ScenarioSession {
 public:
  ScenarioSession();

  void submit_task(const robot::iface::TaskRequest& req);
  void step_once();

 private:
  world::World world_;
  providers::SimPlatformProvider provider_;
  robot::app::RobotApplication app_;
};

}  // namespace lost_signal::sim::game
