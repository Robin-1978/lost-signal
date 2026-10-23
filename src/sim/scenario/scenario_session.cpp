#include "sim/scenario/scenario_session.hpp"

#include <cmath>
#include <cstdint>

namespace lost_signal::sim::scenario {

ScenarioSession::ScenarioSession()
    : world_(),
      suite_(&world_),
      app_(std::make_unique<providers::StaticMapProvider>(
          &world_, -10.0, -10.0, 20.0, 20.0, 0.25)) {
  // SIM bring-up: the suite claims motion + lidar + imu descriptors.
  // Same seam a native driver uses.
  suite_.register_all(app_);
  // Harness bringup: ordered configure -> activate; a failed module blocks.
  (void)app_.configure();
  (void)app_.activate();
  // Default cadence runs the scan/record taps each 20 ms plant step:
  // identical to the pre-scheduler session. The ROBOT decision branch
  // still runs at the coordinator's own 10 Hz (decision_period_s is
  // accepted but currently inert). Scenarios opt into slower taps
  // explicitly.
  scheduler_.reset(0.02, 0.02, 0.02);
}

ScenarioSession::ScenarioSession(
    robot::platform::RobotDescription desc, world::MotorParams motor)
    : world_(),
      suite_(&world_),
      app_(std::make_unique<providers::StaticMapProvider>(
               &world_, -10.0, -10.0, 20.0, 20.0, 0.25),
           std::move(desc)) {
  world_.set_motor_params(motor);
  suite_.register_all(app_);
  (void)app_.configure();
  (void)app_.activate();
  scheduler_.reset(0.02, 0.02, 0.02);
}

void ScenarioSession::set_cadence(double scan_period_s,
                                  double decision_period_s,
                                  double record_period_s) {
  auto valid = [](double p) {
    return std::isfinite(p) && p >= 0.0;
  };
  if (!valid(scan_period_s) || !valid(decision_period_s) ||
      !valid(record_period_s)) {
    return;  // fail closed: keep the previous cadence, never half-apply
  }
  scheduler_.reset(scan_period_s, decision_period_s, record_period_s);
}

void ScenarioSession::set_objectives(std::vector<std::string> place_ids,
                                     double arrive_tol_m) {
  driver_.set_objectives(std::move(place_ids), arrive_tol_m);
}

robot::iface::GoalHandle ScenarioSession::request_goal(
    const robot::iface::TaskRequest& req) {
  return app_.request_goal(req);
}

robot::iface::GoalStatus ScenarioSession::goal_status(
    robot::iface::GoalHandle h) const {
  return app_.goal_status(h);
}

bool ScenarioSession::cancel_goal(robot::iface::GoalHandle h) {
  return app_.cancel_goal(h);
}

void ScenarioSession::step_once() {
  constexpr double kDtS = 0.02;  // plant base rate; deadlines read sim time
  sim_time_s_ += kDtS;
  const auto sim_stamp_ns = static_cast<robot::core::MonotonicNs>(
      static_cast<std::int64_t>(sim_time_s_ * 1e9));
  // Truth snapshot BEFORE the step: recorded offline only.
  eval::EvalStep rec;
  rec.truth_before = world_.truth();

  world_.step(kDtS);
  // Truth-side judge AFTER the plant steps: mission failure, scoring, and
  // replay diagnostics. Never a ROBOT input (see collision_judge.hpp).
  rec.judge = assess_collision(world_.truth().body_x_m,
                               world_.truth().body_y_m, judge_radius_m_,
                               world_.static_boxes(), world_.actors());

  bool have_obs = false;
  robot::iface::ObservationFrame obs;
  if (scheduler_.scan_due(sim_time_s_)) {
    have_obs = suite_.poll_envelope(obs) && app_.publish_observation(obs);
  }
  rec.obs = obs;
  rec.obs_accepted = have_obs;
  if (!have_obs) {
    // Rejected (or skipped) observation still records truth + judge for
    // offline analysis, but never ticks the robot on it.
    rec.cmd = robot::iface::MotionCommand::SafeStop(sim_stamp_ns);
    rec.diag = app_.diagnostics();
    rec.truth_after = world_.truth();
    rec.step = episode_.size();
    if (scheduler_.record_due(sim_time_s_)) episode_.record(rec);
    driver_.update(app_, world_);
    return;
  }
  robot::iface::MotionCommand cmd = app_.tick(obs.stamp_ns);
  robot::iface::ActuatorCommand act = app_.take_actuator(obs.stamp_ns);
  suite_.consume(act);
  driver_.update(app_, world_);

  rec.cmd = cmd;
  rec.diag = app_.diagnostics();
  rec.truth_after = world_.truth();
  rec.step = episode_.size();
  if (scheduler_.record_due(sim_time_s_)) episode_.record(rec);
}

}  // namespace lost_signal::sim::scenario
