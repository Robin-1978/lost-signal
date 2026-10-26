#include "sim/scenario/scenario_session.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>

namespace lost_signal::sim::scenario {
namespace {

robot::iface::ActuatorMode actuator_mode_for(const std::string& kind) {
 if (kind == "ackermann") return robot::iface::ActuatorMode::kAckermannDuty;
 if (kind == "swerve") return robot::iface::ActuatorMode::kSwerveDuty;
 if (kind == "mecanum") return robot::iface::ActuatorMode::kMecanumDuty;
 // Differential and tracked chassis both expose two differential duties.
 return robot::iface::ActuatorMode::kDiffDuty;
}

void capture_presentation(const world::World& world,
 eval::PresentationSnapshot& snapshot) {
 world::WheelViz wheels[6];
 const int wheel_count = world.wheels(wheels, 6);
 snapshot.wheels.assign(wheels, wheels + std::max(0, wheel_count));
 world::CasterViz casters[world::MotorParams::kMaxSupports];
 const int caster_count = world.casters(
 casters, world::MotorParams::kMaxSupports);
 snapshot.casters.assign(casters, casters + std::max(0, caster_count));
 snapshot.actors.reserve(world.actors().size());
 for (const auto& actor : world.actors()) {
 snapshot.actors.push_back({actor.x, actor.y, actor.vx_mps, actor.vy_mps,
 actor.radius_m, actor.height_m,
 actor.visual_kind == "pedestrian"});
 }
 snapshot.impact = world.impact();
 snapshot.captured = true;
}

} // namespace

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
 scheduler_.reset_after_ns(0, kOuterStepNs, kOuterStepNs, kOuterStepNs,
 kOuterStepNs, kOuterStepNs);
  // Mounts from the same description the app runs: SIM generation and the
  // ROBOT-side TF rotate by identical extrinsics (never double/single).
  suite_.set_mounts(app_.description().sensors);
}

ScenarioSession::ScenarioSession(
    robot::platform::RobotDescription desc, world::MotorParams motor)
    : world_(),
      suite_(&world_),
 idle_actuator_mode_(actuator_mode_for(desc.chassis_kind)),
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
 scheduler_.reset_after_ns(0, kOuterStepNs, kOuterStepNs, kOuterStepNs,
 kOuterStepNs, kOuterStepNs);
  suite_.set_mounts(app_.description().sensors);
}

void ScenarioSession::prepare_world() {
 world_.apply(robot::iface::ActuatorCommand::SafeZero(
 world_.sim_time_ns(), idle_actuator_mode_));
 world_.prepare();
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
 auto valid = [](double p) { return on_physics_grid(p); };
 if (!valid(scan_period_s) || !valid(motion_period_s) ||
 !valid(imu_period_s) || !valid(decision_period_s) ||
      !valid(record_period_s)) {
    return;  // fail closed: keep the previous cadence, never half-apply
  }
 // Recording is the public transition cadence. It cannot run faster than
 // the 20 ms ScenarioSession step; sensors and decisions use exact sub-step
 // deadlines and may run faster.
 constexpr double kOuterStepS =
 static_cast<double>(kOuterStepNs) / static_cast<double>(kNsPerSecond);
 if (record_period_s > 0.0 && record_period_s < kOuterStepS) {
 return;
 }
 if (record_period_s > 0.0) {
 const double outer_steps = record_period_s / kOuterStepS;
 if (std::abs(outer_steps - std::round(outer_steps)) > 1e-12) return;
 }
 const SimTimeNs scan_period_ns = seconds_to_sim_ns(scan_period_s);
 const SimTimeNs motion_period_ns = seconds_to_sim_ns(motion_period_s);
 const SimTimeNs imu_period_ns = seconds_to_sim_ns(imu_period_s);
 const SimTimeNs decision_period_ns = seconds_to_sim_ns(decision_period_s);
 const SimTimeNs record_period_ns = seconds_to_sim_ns(record_period_s);
 scheduler_.reset_after_ns(sim_time_ns_, scan_period_ns, motion_period_ns,
 imu_period_ns, decision_period_ns, record_period_ns);
 suite_.lidar().set_scan_period(scan_period_s);
 if (scan_period_s > 0.0) {
 suite_.lidar3d().set_scan_period(static_cast<float>(scan_period_s));
 }
 // Make the configured decision cadence real: the coordinator gates its
 // planning branch on this period. Zero keeps the deployment default.
 if (decision_period_s > 0.0) {
 app_.set_decision_period(decision_period_ns);
 }
}

void ScenarioSession::set_objectives(std::vector<std::string> place_ids,
 std::optional<double> arrive_tol_m) {
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
 const SimTimeNs outer_end_ns = sim_time_ns_ + kOuterStepNs;
 eval::EvalStep rec;
 rec.truth_before = world_.truth();
 double physics_ms = 0.0;
 double sensors_ms = 0.0;
 double robot_ms = 0.0;
 double actuation_ms = 0.0;
 bool accepted_observation = false;
 robot::iface::ObservationFrame last_observation;
 robot::iface::MotionCommand cmd = last_cmd_;

 // The plant always advances on the canonical 1 ms grid. Sensor/control
 // cadence changes which completed ticks are sampled; it never repartitions
 // MuJoCo integration, actor motion, contact hysteresis, or material lookup.
 // A 100 Hz IMU therefore samples ticks 10 and 20 inside this transition.
 world_.begin_fixed_transition();
 while (sim_time_ns_ < outer_end_ns) {
 const auto physics_begin = Clock::now();
 world_.step(sim_ns_to_seconds(kPhysicsQuantumNs));
 sim_time_ns_ += kPhysicsQuantumNs;
 physics_ms += elapsed_ms(physics_begin, Clock::now());

 const bool scan_due = scheduler_.scan_due_ns(sim_time_ns_);
 const bool motion_due = scheduler_.motion_due_ns(sim_time_ns_);
 const bool imu_due = scheduler_.imu_due_ns(sim_time_ns_);
 if (scan_due || motion_due || imu_due) {
 const auto sensors_begin = Clock::now();
 robot::iface::ObservationFrame observation;
 const bool accepted =
 suite_.poll_envelope(scan_due, motion_due, imu_due, observation) &&
 app_.publish_observation(observation);
 sensors_ms += elapsed_ms(sensors_begin, Clock::now());
 if (!observation.empty()) last_observation = std::move(observation);
 accepted_observation = accepted_observation || accepted;
 }

 if (scheduler_.decision_due_ns(sim_time_ns_)) {
 const robot::core::MonotonicNs sim_stamp_ns = world_.sim_time_ns();
 const auto robot_begin = Clock::now();
 cmd = app_.tick(sim_stamp_ns);
 last_cmd_ = cmd;
 robot_ms += elapsed_ms(robot_begin, Clock::now());
 const auto actuation_begin = Clock::now();
 suite_.consume(app_.take_actuator(sim_stamp_ns));
 actuation_ms += elapsed_ms(actuation_begin, Clock::now());
 }
 }
 world_.end_fixed_transition();
 // The World clock is canonical for sensor stamps. Integer clocks must
 // remain identical after every public transition.
 sim_time_ns_ = world_.sim_time_ns();
 rec.obs = std::move(last_observation);
 rec.obs_accepted = accepted_observation;

 // Truth-side judge AFTER the plant steps: mission failure, scoring, and
 // replay diagnostics. Never a ROBOT input (see collision_judge.hpp).
 rec.judge = assess_collision(world_.truth().body_x_m,
 world_.truth().body_y_m, judge_radius_m_,
 world_.static_boxes(), world_.actors());
 // Belly / high-center contact against the authored surface (same
 // truth-side discipline; the ROBOT never sees this outcome). Needs a live
 // engine height: an uncommanded plant (drive_count == 0) reports frozen
 // zero truth, and z = 0 is "no data", never "body at ground level".
 if (world_.truth().drive_count > 0 &&
 world_.truth().body_z_m > 0.0 &&
 std::isfinite(world_.truth().body_z_m)) {
 eval::CollisionOutcome terrain = assess_terrain_contact(
 world_.truth().body_x_m, world_.truth().body_y_m,
 world_.truth().body_z_m, judge_radius_m_, world_.terrain_patches(),
 world_.support_slabs(), terrain_clearance_m_);
 rec.judge.terrain_collision = terrain.terrain_collision;
 rec.judge.terrain_clearance_m = terrain.terrain_clearance_m;
 }

 const auto actuation_begin = Clock::now();
 driver_.update(app_, world_);
 actuation_ms += elapsed_ms(actuation_begin, Clock::now());
 rec.cmd = cmd;
 rec.diag = app_.diagnostics();
 rec.truth_after = world_.truth();
 if (scheduler_.record_due_ns(sim_time_ns_)) {
 if (retain_episode_history_) capture_presentation(world_, rec.presentation);
 rec.step = score_accumulator_.score().steps;
 score_accumulator_.record(rec);
 last_eval_step_ = rec;
 if (retain_episode_history_) episode_.record(std::move(rec));
 }
 const auto record_end = Clock::now();
 last_step_timing_.physics_ms = physics_ms;
 last_step_timing_.sensors_ms = sensors_ms;
 last_step_timing_.robot_tick_ms = robot_ms;
 last_step_timing_.actuation_ms = actuation_ms;
 last_step_timing_.record_ms = std::max(
 0.0, elapsed_ms(total_begin, record_end) - physics_ms - sensors_ms -
 robot_ms - actuation_ms);
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
