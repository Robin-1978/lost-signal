#include "sim/scenario/scenario_session.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace lost_signal::sim::scenario {

ScenarioSession::ScenarioSession()
    : world_(),
      suite_(&world_),
      app_([this]() {
        auto maps = std::make_unique<providers::StaticMapProvider>(
            &world_, -10.0, -10.0, 20.0, 20.0, 0.25);
        map_provider_ = maps.get();
        return robot::app::RobotApplication(std::move(maps));
      }()) {
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
  // Mounts from the same description the app runs: SIM generation and the
  // ROBOT-side TF rotate by identical extrinsics (never double/single).
  suite_.set_mounts(app_.description().sensors);
}

ScenarioSession::ScenarioSession(
    robot::platform::RobotDescription desc, world::MotorParams motor)
    : world_(),
      suite_(&world_),
      app_([this, &desc]() {
        auto maps = std::make_unique<providers::StaticMapProvider>(
            &world_, -10.0, -10.0, 20.0, 20.0, 0.25);
        map_provider_ = maps.get();
        return robot::app::RobotApplication(std::move(maps), std::move(desc));
      }()) {
  world_.set_motor_params(motor);
  suite_.register_all(app_);
  (void)app_.configure();
  (void)app_.activate();
  scheduler_.reset(0.02, 0.02, 0.02);
  suite_.set_mounts(app_.description().sensors);
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
  // Belly / high-center contact against the authored surface (same
  // truth-side discipline; the ROBOT never sees this outcome). Needs a live
  // engine height: an uncommanded plant (drive_count == 0) reports frozen
  // zero truth, and z = 0 is "no data", never "body at ground level".
  if (world_.truth().drive_count > 0) {
    eval::CollisionOutcome terrain = assess_terrain_contact(
        world_.truth().body_x_m, world_.truth().body_y_m,
        world_.truth().body_z_m, judge_radius_m_, world_.terrain_patches(),
        world_.support_slabs(), terrain_clearance_m_);
    rec.judge.terrain_collision = terrain.terrain_collision;
    rec.judge.terrain_clearance_m = terrain.terrain_clearance_m;
  }

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

void ScenarioSession::fit_map_window_to_world(double margin_m) {
  if (map_provider_ == nullptr) return;
  const double m = (std::isfinite(margin_m) && margin_m > 0.0) ? margin_m : 4.0;
  double x0 = 0.0, y0 = 0.0, x1 = 0.0, y1 = 0.0;  // robot starts at origin
  auto eat = [&](double x, double y) {
    x0 = std::min(x0, x);
    y0 = std::min(y0, y);
    x1 = std::max(x1, x);
    y1 = std::max(y1, y);
  };
  for (const auto& b : world_.static_boxes()) {
    eat(b.x0, b.y0);
    eat(b.x1, b.y1);
  }
  for (const auto& p : world_.places()) eat(p.x_m, p.y_m);
  for (const auto& z : world_.material_zones()) {
    eat(z.x0, z.y0);
    eat(z.x1, z.y1);
  }
  for (const auto& p : world_.terrain_patches()) {
    eat(p.x0, p.y0);
    eat(p.x1, p.y1);
  }
  map_provider_->set_window(x0 - m, y0 - m, (x1 - x0) + 2.0 * m,
                            (y1 - y0) + 2.0 * m);
}

}  // namespace lost_signal::sim::scenario
