#include "sim/scenario/scenario_session.hpp"

#include <algorithm>
#include <chrono>
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
 // Default cadence runs all taps at the 20 ms plant step. The scheduler
 // still keeps scan, decision, and record deadlines independent so slower
 // sensor streams exercise the same stale-data path as deployment.
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
 set_sensor_cadence(scan_period_s, scan_period_s, scan_period_s,
 decision_period_s, record_period_s);
}

void ScenarioSession::set_sensor_cadence(double scan_period_s,
 double motion_period_s, double imu_period_s, double decision_period_s,
 double record_period_s) {
  auto valid = [](double p) {
    return std::isfinite(p) && p >= 0.0;
  };
 if (!valid(scan_period_s) || !valid(motion_period_s) ||
 !valid(imu_period_s) || !valid(decision_period_s) ||
      !valid(record_period_s)) {
    return;  // fail closed: keep the previous cadence, never half-apply
  }
 scheduler_.reset(scan_period_s, motion_period_s, imu_period_s,
 decision_period_s, record_period_s);
 // Make the configured decision cadence real: the coordinator gates its
 // planning branch on this period. Zero keeps the deployment default.
 if (decision_period_s > 0.0) {
 app_.set_decision_period(static_cast<robot::core::MonotonicNs>(
 decision_period_s * 1e9));
 }
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
 using Clock = std::chrono::steady_clock;
 const auto total_begin = Clock::now();
 const auto elapsed_ms = [](Clock::time_point begin, Clock::time_point end) {
 return std::chrono::duration<double, std::milli>(end - begin).count();
 };
  constexpr double kDtS = 0.02;  // plant base rate; deadlines read sim time
  sim_time_s_ += kDtS;
  // Truth snapshot BEFORE the step: recorded offline only.
  eval::EvalStep rec;
  rec.truth_before = world_.truth();

  world_.step(kDtS);
 // Decision time follows the plant clock exactly (no accumulated-double
 // jitter): the observation keeps its acquisition stamp, so latency is
 // visible as a stamp older than now, while a fresh scan ticks at now.
 const auto sim_stamp_ns = static_cast<robot::core::MonotonicNs>(
 static_cast<std::int64_t>(world_.truth().step) * 20'000'000);
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
 const auto physics_end = Clock::now();

 const bool decision_due = scheduler_.decision_due(sim_time_s_);
 const bool motion_due = scheduler_.motion_due(sim_time_s_);
 const bool imu_due = scheduler_.imu_due(sim_time_s_);
 bool have_obs = false;
  robot::iface::ObservationFrame obs;
 const bool scan_due = scheduler_.scan_due(sim_time_s_);
 if (scan_due || motion_due || imu_due) {
 have_obs = suite_.poll_envelope(scan_due, motion_due, imu_due, obs) &&
 app_.publish_observation(obs);
  }
  rec.obs = obs;
  rec.obs_accepted = have_obs;
 const auto sensors_end = Clock::now();
 robot::iface::MotionCommand cmd = last_cmd_;
 if (decision_due) {
 // A decision tick is allowed to run without a newly acquired scan. The
 // channel retains the latest accepted frame and the ROBOT freshness gate
 // decides whether it is still safe to use; no-data remains fail-closed.
 cmd = app_.tick(sim_stamp_ns);
 last_cmd_ = cmd;
 }
 const auto robot_end = Clock::now();
  robot::iface::ActuatorCommand act = app_.take_actuator(sim_stamp_ns);
  suite_.consume(act);
  driver_.update(app_, world_);
 const auto actuation_end = Clock::now();

 rec.cmd = cmd;
  rec.diag = app_.diagnostics();
  rec.truth_after = world_.truth();
  rec.step = episode_.size();
  if (scheduler_.record_due(sim_time_s_)) episode_.record(rec);
 const auto record_end = Clock::now();
 last_step_timing_.physics_ms = elapsed_ms(total_begin, physics_end);
 last_step_timing_.sensors_ms = elapsed_ms(physics_end, sensors_end);
 last_step_timing_.robot_tick_ms = elapsed_ms(sensors_end, robot_end);
 last_step_timing_.actuation_ms = elapsed_ms(robot_end, actuation_end);
 last_step_timing_.record_ms = elapsed_ms(actuation_end, record_end);
 last_step_timing_.total_ms = elapsed_ms(total_begin, record_end);
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
 // Oriented support slabs are authored world geometry too. Include all four
 // corners so the ROBOT survey window cannot clip a bridge/deck out of the
 // map before sensing begins.
 for (const auto& s : world_.support_slabs()) {
 const double c = std::cos(s.yaw_rad);
 const double sn = std::sin(s.yaw_rad);
 for (const auto& corner : {std::pair<double, double>{-s.half_length_m,
 -s.half_width_m},
 std::pair<double, double>{-s.half_length_m, s.half_width_m},
 std::pair<double, double>{s.half_length_m, -s.half_width_m},
 std::pair<double, double>{s.half_length_m, s.half_width_m}}) {
 eat(s.center_x_m + c * corner.first - sn * corner.second,
 s.center_y_m + sn * corner.first + c * corner.second);
 }
 }
  map_provider_->set_window(x0 - m, y0 - m, (x1 - x0) + 2.0 * m,
                            (y1 - y0) + 2.0 * m);
}

}  // namespace lost_signal::sim::scenario
