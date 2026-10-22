#include "sim/scenario/scenario_session.hpp"

namespace lost_signal::sim::scenario {

ScenarioSession::ScenarioSession() : suite_(&world_), app_(nullptr) {
  // SIM bring-up: the suite claims motion + lidar + imu descriptors.
  // Same seam a native driver uses, only the backend flag differs.
  suite_.register_all(app_);
  // Harness bringup: ordered configure -> activate; a failed module blocks.
  (void)app_.configure();
  (void)app_.activate();
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
  if (!suite_.poll_envelope(obs)) return;
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
  suite_.consume(cmd);

  rec.cmd = cmd;
  rec.diag = app_.diagnostics();
  rec.truth_after = world_.truth();
  rec.step = episode_.size();
  episode_.record(rec);
}

}  // namespace lost_signal::sim::scenario
