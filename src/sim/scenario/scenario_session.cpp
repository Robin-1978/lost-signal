#include "sim/scenario/scenario_session.hpp"

namespace lost_signal::sim::scenario {

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
  // Truth snapshot BEFORE the step: recorded offline only.
  eval::EvalStep rec;
  rec.truth_before = world_.truth();

  world_.step(0.02);

  robot::iface::ObservationFrame obs;
  if (!provider_.poll_observation(obs)) return;
  rec.obs = obs;
  rec.obs_accepted = app_.publish_observation(obs);
  if (!rec.obs_accepted) {
    // Rejected observation still recorded with truth for offline analysis,
    // but never ticks the robot on it.
    rec.cmd = robot::iface::MotionCommand::SafeStop(obs.stamp_ns);
    rec.diag = app_.diagnostics();
    rec.truth_after = world_.truth();
    rec.step = episode_.size();
    episode_.record(rec);
    return;
  }
  robot::iface::MotionCommand cmd = app_.tick(obs.stamp_ns);
  provider_.consume_command(cmd);

  rec.cmd = cmd;
  rec.diag = app_.diagnostics();
  rec.truth_after = world_.truth();
  rec.step = episode_.size();
  episode_.record(rec);
}

}  // namespace lost_signal::sim::scenario
