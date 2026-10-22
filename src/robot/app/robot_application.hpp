#pragma once

#include <memory>
#include <string>

#include "robot/autonomy/coordinator.hpp"
#include "robot/core/status.hpp"
#include "robot/executive/executive.hpp"
#include "robot/iface/command.hpp"
#include "robot/iface/map.hpp"
#include "robot/iface/observation.hpp"
#include "robot/iface/task.hpp"

namespace lost_signal::robot::app {

// Composition root and single tick boundary for one robot process.
// SIM, replay, UI, or native providers are adapters around this class;
// none of them may reach AutonomyCoordinator or RobotExecutive directly.
class RobotApplication {
 public:
  explicit RobotApplication(std::unique_ptr<iface::IMapProvider> maps);

  core::Status submit_task(const iface::TaskRequest& req);
  core::Status request_map(const std::string& map_id);

  // Provider-neutral ingestion: the only path from LSRI observations into
  // estimator/perception state.
  void publish_observation(const iface::ObservationFrame& obs);

  // One deterministic cycle: executive observation -> autonomy -> validated
  // command. Always returns a valid command (safe stop on failure).
  iface::MotionCommand tick(core::MonotonicNs now_ns);

  iface::TaskStatus task_status() const;
  iface::MapSnapshot active_map() const;

 private:
  std::unique_ptr<iface::IMapProvider> maps_;
  autonomy::AutonomyCoordinator coordinator_;
  executive::RobotExecutive executive_;
};

}  // namespace lost_signal::robot::app
