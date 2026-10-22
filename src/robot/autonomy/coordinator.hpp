#pragma once

#include "robot/core/time.hpp"
#include "robot/iface/command.hpp"
#include "robot/iface/map.hpp"
#include "robot/iface/observation.hpp"

namespace lost_signal::robot::autonomy {

// ROBOT-owned state estimate. Produced only from LSRI observations.
// Never seeded or corrected by simulator truth.
struct EstimatedState {
  bool valid = false;
  core::MonotonicNs stamp_ns = core::kInvalidTime;
  double x_m = 0.0;
  double y_m = 0.0;
  double yaw_rad = 0.0;
  double forward_mps = 0.0;
  double yaw_rate_radps = 0.0;
};

// ROBOT-owned autonomy transaction. One entry point for estimation,
// perception, planning, validation, and control state. SIM must never call
// these algorithms directly; it goes through RobotApplication.
class AutonomyCoordinator {
 public:
  void ingest_observation(const iface::ObservationFrame& obs);
  iface::MotionCommand update(iface::MapSnapshot map, core::MonotonicNs now_ns);
  bool has_observation() const { return has_observation_; }
  const iface::ObservationFrame& last_observation() const { return last_obs_; }
  const EstimatedState& estimate() const { return estimate_; }

 private:
  bool has_observation_ = false;
  iface::ObservationFrame last_obs_{};
  EstimatedState estimate_{};
};

}  // namespace lost_signal::robot::autonomy
