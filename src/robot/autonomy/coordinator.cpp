#include "robot/autonomy/coordinator.hpp"

namespace lost_signal::robot::autonomy {

void AutonomyCoordinator::ingest_observation(const iface::ObservationFrame& obs) {
  if (!iface::valid_observation_frame(obs)) return;
  // Fail-closed: stale or repeated input never advances state.
  if (has_observation_ && obs.sequence <= last_obs_.sequence) return;
  if (has_observation_ && obs.stamp_ns <= last_obs_.stamp_ns) return;
  last_obs_ = obs;
  has_observation_ = true;
}

iface::MotionCommand AutonomyCoordinator::update(iface::MapSnapshot /*map*/,
                                                 core::MonotonicNs now_ns) {
  // v1 skeleton: hold position until real modules land here.
  // Missing observation is unknown evidence -> safe stop, never free space.
  if (!has_observation_) return iface::MotionCommand::SafeStop(now_ns);
  return iface::MotionCommand::SafeStop(now_ns);
}

}  // namespace lost_signal::robot::autonomy
