#include "robot/autonomy/coordinator.hpp"

#include <cmath>

namespace lost_signal::robot::autonomy {

void AutonomyCoordinator::ingest_observation(const iface::ObservationFrame& obs) {
  if (!iface::valid_observation_frame(obs)) return;
  // Fail-closed: stale or repeated input never advances state.
  if (has_observation_ && obs.sequence <= last_obs_.sequence) return;
  if (has_observation_ && obs.stamp_ns <= last_obs_.stamp_ns) return;
  // v1 dead-reckoning placeholder: integrates ONLY the measured odometry twist.
  // No truth, no map correction here; real estimator modules replace this body.
  if (!has_observation_) {
    estimate_.valid = true;
    estimate_.stamp_ns = obs.stamp_ns;
    estimate_.x_m = 0.0;
    estimate_.y_m = 0.0;
    estimate_.yaw_rad = 0.0;
  } else if (obs.odometry_valid && last_obs_.odometry_valid) {
    double dt_s =
        static_cast<double>(obs.stamp_ns - last_obs_.stamp_ns) * 1e-9;
    if (dt_s > 0.0 && dt_s < 1.0) {
      double yaw_mid = estimate_.yaw_rad +
                       0.5 * last_obs_.yaw_rate_radps * dt_s;
      estimate_.x_m += last_obs_.forward_mps * std::cos(yaw_mid) * dt_s;
      estimate_.y_m += last_obs_.forward_mps * std::sin(yaw_mid) * dt_s;
      estimate_.yaw_rad += last_obs_.yaw_rate_radps * dt_s;
      estimate_.stamp_ns = obs.stamp_ns;
    }
  } else {
    estimate_.stamp_ns = obs.stamp_ns;
  }
  if (obs.odometry_valid) {
    estimate_.forward_mps = obs.forward_mps;
    estimate_.yaw_rate_radps = obs.yaw_rate_radps;
  }
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
