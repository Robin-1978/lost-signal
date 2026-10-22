#include "sim/game/scenario_session.hpp"

namespace lost_signal::sim::game {

ScenarioSession::ScenarioSession() : provider_(&world_), app_(nullptr) {
  // SIM bring-up: claim the sim endpoints before any observation flows.
  // Same seam a native driver uses, only the backend flag differs.
  app_.register_provider({"sim.platform",
                          robot::iface::EndpointRole::kLidar,
                          robot::iface::ProviderBackend::kSim, "lidar_link"});
  app_.register_provider({"sim.platform",
                          robot::iface::EndpointRole::kImu,
                          robot::iface::ProviderBackend::kSim, "imu_link"});
  app_.register_provider({"sim.platform",
                          robot::iface::EndpointRole::kEncoder,
                          robot::iface::ProviderBackend::kSim, "base_link"});
}

void ScenarioSession::submit_task(const robot::iface::TaskRequest& req) {
  (void)app_.submit_task(req);
}

void ScenarioSession::step_once() {
  world_.step(0.02);
  robot::iface::ObservationFrame obs;
  if (!provider_.poll_observation(obs)) return;
  if (!app_.publish_observation(obs)) return;  // rejected: never ticks on it
  robot::iface::MotionCommand cmd = app_.tick(obs.stamp_ns);
  provider_.consume_command(cmd);
}

}  // namespace lost_signal::sim::game
