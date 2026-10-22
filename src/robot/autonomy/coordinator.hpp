#pragma once

#include "robot/iface/command.hpp"
#include "robot/iface/map.hpp"
#include "robot/iface/observation.hpp"

namespace lost_signal::robot::autonomy {

// ROBOT-owned autonomy transaction. One entry point for estimation,
// perception, planning, validation, and control state. SIM must never call
// these algorithms directly; it goes through RobotApplication.
class AutonomyCoordinator {
 public:
  void ingest_observation(const iface::ObservationFrame& obs);
  iface::MotionCommand update(iface::MapSnapshot map, core::MonotonicNs now_ns);

 private:
  bool has_observation_ = false;
  iface::ObservationFrame last_obs_{};
};

}  // namespace lost_signal::robot::autonomy
