#pragma once

#include "robot/app/robot_application.hpp"
#include "sim/eval/episode.hpp"
#include "sim/providers/sim_platform.hpp"
#include "sim/world/world.hpp"

namespace lost_signal::sim::game {

// SIM composition root. Owns world truth and presentation state; drives the
// robot only through RobotApplication + LSRI. Must never call autonomy,
// estimator, planner, or safety code directly.
//
// Truth isolation: WorldTruth is recorded into EpisodeRecorder for offline
// comparison / scoring / RL labels. It is never passed to publish_observation,
// tick, or any ROBOT input.
class ScenarioSession {
 public:
  ScenarioSession();

  void submit_task(const robot::iface::TaskRequest& req);
  void step_once();

  const eval::EpisodeRecorder& episode() const { return episode_; }
  void clear_episode() { episode_.clear(); }

 private:
  world::World world_;
  providers::SimPlatformProvider provider_;
  robot::app::RobotApplication app_;
  eval::EpisodeRecorder episode_;
};

}  // namespace lost_signal::sim::game
