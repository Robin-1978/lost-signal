#include "sim/game/scenario_session.hpp"

namespace lost_signal::sim::game {

ScenarioSession::ScenarioSession() : provider_(&world_), app_(nullptr) {}

void ScenarioSession::submit_task(const robot::iface::TaskRequest& req) {
  (void)app_.submit_task(req);
}

void ScenarioSession::step_once() {
  world_.step(0.02);
  robot::iface::ObservationFrame obs;
  if (!provider_.poll_observation(obs)) return;
  app_.publish_observation(obs);
  robot::iface::MotionCommand cmd = app_.tick(obs.stamp_ns);
  provider_.consume_command(cmd);
}

}  // namespace lost_signal::sim::game
