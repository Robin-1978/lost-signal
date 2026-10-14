#include "mission_simulation.hpp"

#include "algo/astar.hpp"
#include "hw/chassis/ackermann.hpp"  // yaw_for_steer only (kinematics pair, interface use)
#include "hw/chassis/chassis_factory.hpp"
#include "algo/slip_detect.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

namespace game {
namespace {

constexpr double kBuiltInDecisionHz = 10.0;
constexpr double kDynamicTrackMaxAgeS = 1.25;

sim::Vec2 as_vec(const Pose& pose) {
 return {pose.x, pose.y};
}

double distance(sim::Vec2 a, sim::Vec2 b) {
 return std::hypot(a.x - b.x, a.y - b.y);
}

bool inside(sim::Vec2 point, const SurfaceDefinition& box) {
 return point.x >= box.min.x && point.x < box.max.x && point.y >= box.min.y && point.y < box.max.y;
}

} // namespace

const char* mission_status_name(MissionStatus status) {
 switch (status) {
 case MissionStatus::Ready:
  return "ready";
 case MissionStatus::Running:
  return "running";
 case MissionStatus::Succeeded:
  return "succeeded";
 case MissionStatus::Failed:
  return "failed";
 }
 return "unknown";
}

const std::string& MissionSimulation::level_robot_id(const ContentCatalog& catalog,
                                                     const std::string& level_id,
                                                     const BuildSelection& build) {
 // Validated player choice wins (validate_build enforces allow-list + unlock);
 // empty request keeps the presentation default / companion path bit-identical.
 if (!build.chassis_robot.empty()) {
  return build.chassis_robot;
 }
 const std::string& presentation_robot = catalog.presentation_for_level(level_id).robot_id;
 return presentation_robot.empty() ? catalog.campaign().companion_robot : presentation_robot;
}

MissionSimulation::MissionSimulation(const ContentCatalog& catalog, std::string level_id,
                                     BuildSelection build, std::optional<uint64_t> seed_override,
                                     const std::vector<std::string>& owned_abilities)
    : catalog_(catalog), level_id_(std::move(level_id)), build_(std::move(build)),
      seed_override_(seed_override), level_(catalog_.level(level_id_)),
      map_definition_(catalog_.map(level_.map)),
      robot_definition_(catalog_.robot(level_robot_id(catalog_, level_id_, build_))),
      behavior_(catalog_.style_behavior(build_.style)), rng_(effective_seed()),
      perception_rng_(effective_seed() ^ 0x9e3779b9U) {
 catalog_.validate_build(level_id_, build_, owned_abilities);
 for (const auto& [slot, id] : build_.parts) {
  if (!id.empty()) {
   parts_.emplace(slot, &catalog_.component(id));
  }
 }
 reset();
}

void MissionSimulation::reset() {
 rng_ = sim::Rng(effective_seed());
 perception_rng_ = sim::Rng(effective_seed() ^ 0x9e3779b9U);
 imu_rng_ = sim::Rng(effective_seed() ^ 0x51ed2707U);
 result_ = {};
 snapshot_ = {};
 events_.clear();
 replay_.clear();
 held_command_ = {};
 manual_command_ = {};
 manual_control_active_ = false;
 guide_state_ = {};
 next_scan_s_ = 0.0;
 next_decision_s_ = 0.0;
 next_replay_s_ = 0.0;
 last_event_time_s_ = -1000.0;
 last_event_code_.clear();
 dynamic_obstacle_map_.clear();
 // Gyro bias is drawn once per run: constant within the run, new every run.
 gyro_bias_ = has("imu") ? imu_rng_.gauss(spec("imu", "gyro_bias_radps")) : 0.0;
 was_saturated_ = false;
 build_world();
 plan_route();

 const auto& spawn = map_definition_.markers.at(level_.objective.spawn);
 snapshot_.robot = {spawn.x, spawn.y, spawn.theta, 0.0, 0.0};
 snapshot_.estimated_position = {spawn.x, spawn.y};
 snapshot_.estimated_heading = spawn.theta;
 snapshot_.status = MissionStatus::Ready;
 odometry_.reset(spawn.x, spawn.y, spawn.theta);
 update_actors();
 slip_detector_.reset();
 slip_episode_times_.clear();
 slip_prev_tread_v_ = 0.0;
 slip_prev_body_v_ = 0.0;
 slip_have_prev_ = false;
 record_replay();
}

void MissionSimulation::build_world() {
 const int width = static_cast<int>(std::ceil(map_definition_.size_m.x / map_definition_.grid_resolution_m));
 const int height = static_cast<int>(std::ceil(map_definition_.size_m.y / map_definition_.grid_resolution_m));
 truth_map_ = std::make_unique<sim::GridMap>(width, height, map_definition_.grid_resolution_m);
 for (const auto& obstacle : map_definition_.obstacles) {
  if (active_in_level(obstacle.active_in, level_id_)) {
   truth_map_->addBox(obstacle.min.x, obstacle.min.y, obstacle.max.x, obstacle.max.y);
  }
 }
 lidar_ = std::make_unique<sim::SimLidar>(*truth_map_);
 collision_map_ = std::make_unique<sim::GridMap>(truth_map_->inflated(robot_definition_.collision_radius_m));
 const double requested_clearance = property(behavior_, "safe_clearance_m");
 const bool has_dynamic_actors =
   std::any_of(map_definition_.actors.begin(), map_definition_.actors.end(),
               [&](const auto& actor) { return active_in_level(actor.active_in, level_id_); });
 const double planning_clearance =
   robot_definition_.collision_radius_m +
   std::clamp(requested_clearance * (has_dynamic_actors ? 0.80 : 0.45), 0.40, 0.75);
 planning_map_ = std::make_unique<sim::GridMap>(truth_map_->inflated(planning_clearance));

 sim::DiffParams params;
 params.v_max = std::max(1.5, property(behavior_, "cruise_mps") * 1.25);
 params.accel_max = spec("tires", "brake_decel_mps2");
 params.wheel_base = robot_definition_.wheel_base_m;
 drive_ = std::make_unique<sim::DiffDrive>(params);
 // Chassis dispatch: drive.type selects the actuator model. Unknown types
 // fail loudly in content, never silently fall back to another chassis.
 // The equipped motor component overrides the robot default; an empty motor
 // slot (early levels) runs the robot stock motor. New motors stay plug-and-play
 // because PID gains are derived from these same numbers.
 const double motor_vmax =
   has("motor") ? spec("motor", "max_wheel_speed_mps") : robot_definition_.motor.max_wheel_speed_mps;
 const double motor_stall =
   has("motor") ? spec("motor", "stall_force_N") : robot_definition_.motor.stall_force_N;
 const double motor_free =
   has("motor") ? spec("motor", "free_speed_mps") : robot_definition_.motor.free_speed_mps;
 const double wheel_mass = std::max(0.5, robot_definition_.mass_kg / 2.0);
 const sim::WheelPidGains wheel_gains =
   sim::tune_wheel_pid(wheel_mass, motor_stall, motor_free, 0.5 * motor_vmax);
 // Chassis construction goes through the factory registry (hw/chassis):
 // new drive.type implementations register themselves; mission only
 // translates robot JSON + equipped specs into neutral build params.
 sim::ChassisBuildParams chassis_params;
 chassis_params.wheel_base_m = robot_definition_.wheel_base_m;
 chassis_params.wheel_radius_m = robot_definition_.wheel_radius_m;
 chassis_params.track_width_m = robot_definition_.track_width_m;
 chassis_params.track_length_m = robot_definition_.track_length_m;
 chassis_params.max_wheel_speed_mps = motor_vmax;
 chassis_params.max_wheel_accel_mps2 = robot_definition_.motor.max_wheel_accel_mps2;
 chassis_params.stall_force_N = motor_stall;
 chassis_params.free_speed_mps = motor_free;
 chassis_params.pwm_limit = robot_definition_.motor.pwm_limit;
 chassis_params.integral_limit = robot_definition_.motor.integral_limit;
 chassis_params.wheel_equiv_mass_kg = robot_definition_.motor.wheel_equiv_mass_kg;
 chassis_params.encoder_counts_per_revolution =
   robot_definition_.encoder.counts_per_revolution;
 chassis_params.mass_kg = robot_definition_.mass_kg;
 chassis_params.yaw_inertia_kgm2 = robot_definition_.yaw_inertia_kgm2;
 chassis_params.max_steer_rad = robot_definition_.steering.max_steer_rad;
 chassis_params.max_steer_rate_radps = robot_definition_.steering.max_steer_rate_radps;
 chassis_params.servo_response_s = robot_definition_.steering.servo_response_s;
 chassis_params.steering_integral_limit = robot_definition_.steering.integral_limit;
 chassis_params.tracked_max_yaw_rate_rads = robot_definition_.tracked_max_yaw_rate_rads;
 chassis_params.behavior_w_max = params.w_max;
 chassis_params.behavior_accel_max = params.accel_max;
 // Builtin registration is explicit at the composition root (idempotent):
 // archive linkers drop unreferenced self-registering objects, so static
 // registrars are platform luck. Custom types: register_creator + JSON type.
 sim::ChassisFactory::register_builtin_chassis();
 sim::BuiltChassis built =
   sim::ChassisFactory::create(robot_definition_.drive_type, chassis_params, wheel_gains);
 chassis_ = std::move(built.model);
 yaw_cap_rads_ = built.yaw_cap_rads;
 // Heading loop auto-tune: gain normalized by speed and wheelbase so the same
 // rule serves every DIY assembly. Kp = w_max / (2 * cruise * base) reproduces
 // the validated RO-1 feel; the integral/derivative ratios are method constants
 // (slow integrator kills steady-state error, short derivative damps).
 const double cruise = property(behavior_, "cruise_mps");
 const double heading_kp =
   yaw_cap_rads_ / (2.0 * std::max(0.2, cruise) * std::max(0.1, robot_definition_.wheel_base_m));
 // PID output units follow the chassis (follow.hpp contract): yaw rate for
 // differential, steer angle for ackermann. Same derivation, capped by what
 // the actuator takes.
 // Differential and tracked share yaw-rate output units; their caps differ
 // (geometry vs vendor spin spec) and both already sit in yaw_cap_rads_.
 const double heading_out_lim =
   robot_definition_.drive_type == "ackermann" ? robot_definition_.steering.max_steer_rad : yaw_cap_rads_;
 heading_pid_ = std::make_unique<sim::PID>(heading_kp, heading_kp / 125.0, heading_kp * 0.05, heading_out_lim,
                                           robot_definition_.steering.integral_limit);
 heading_kp_ = heading_kp;

 patrols_.clear();
 patrol_ids_.clear();
 patrol_t0_.clear();
 dormant_patrols_.clear();
 dormant_ids_.clear();
 for (const auto& actor : map_definition_.actors) {
  if (!active_in_level(actor.active_in, level_id_) || actor.path.size() < 2) {
   continue;
  }
  const sim::Patrol patrol{actor.path.front(), actor.path.back(), actor.speed_mps, actor.radius_m};
  if (actor.dormant_until_summoned) {
   dormant_patrols_.push_back(patrol);
   dormant_ids_.push_back(actor.id);
   continue;
  }
  patrols_.push_back(patrol);
  patrol_ids_.push_back(actor.id);
  patrol_t0_.push_back(0.0);
 }
 actual_actors_.assign(patrols_.size(), {});
}

void MissionSimulation::plan_route() {
 planned_path_.clear();
 result_.astar_expanded = 0;
 const auto& start = map_definition_.markers.at(level_.objective.spawn);
 const auto& goal = map_definition_.markers.at(level_.objective.goal);
 auto append_segment = [&](sim::Vec2 from, sim::Vec2 to) {
  auto plan = sim::astar_plan(*planning_map_, from, to, 1.0);
  result_.astar_expanded += plan.expanded;
  if (!plan.ok) {
   throw ContentError("built-in A* found no route for level '" + level_id_ + "'");
  }
  if (!planned_path_.empty() && !plan.path.empty()) {
   plan.path.erase(plan.path.begin());
  }
  planned_path_.insert(planned_path_.end(), plan.path.begin(), plan.path.end());
 };
 if (!level_.objective.via.empty()) {
  const auto& via = map_definition_.markers.at(level_.objective.via);
  append_segment(as_vec(start), as_vec(via));
  append_segment(as_vec(via), as_vec(goal));
 } else {
  append_segment(as_vec(start), as_vec(goal));
 }
}

void MissionSimulation::start() {
 if (snapshot_.status != MissionStatus::Ready) {
  return;
 }
 snapshot_.status = MissionStatus::Running;
 result_.status = MissionStatus::Running;
 record_event("mission-start", catalog_.tr("event.mission_start"));
}

void MissionSimulation::run_to_completion() {
 start();
 while (snapshot_.status == MissionStatus::Running) {
  step();
 }
}

void MissionSimulation::set_manual_command(double forward, double turn) {
 if (level_.control_mode != "manual_calibration")
  return;
 manual_control_active_ = true;
 const double cruise = property(behavior_, "cruise_mps");
 manual_command_.target_v = std::clamp(forward, -0.65, 1.0) * cruise;
 manual_command_.target_w = std::clamp(turn, -1.0, 1.0) * yaw_cap_rads_;
}

void MissionSimulation::step() {
 if (snapshot_.status != MissionStatus::Running) {
  return;
 }
 update_actors();
 if (has("sensor") && snapshot_.time_s + 1e-9 >= next_scan_s_) {
  scan_environment();
  next_scan_s_ += 1.0 / spec("sensor", "scan_hz");
 }
 if (manual_control_active_) {
  held_command_ = manual_command_;
 } else if (snapshot_.time_s + 1e-9 >= next_decision_s_) {
  decide_control();
  next_decision_s_ += 1.0 / decision_hz();
 }
 apply_motion();
 snapshot_.time_s += fixed_step_s;
 result_.time_s = snapshot_.time_s;

 const auto& goals = level_.objective.goals;
 if (snapshot_.goals_reached < goals.size()) {
  const auto& step = goals[snapshot_.goals_reached];
  const auto target = as_vec(map_definition_.markers.at(step.marker));
  if (distance({snapshot_.robot.x, snapshot_.robot.y}, target) <= step.radius_m) {
   if (step.action == "complete" &&
       (level_.objective.type != "observe_and_stop" || snapshot_.static_obstacle_detected)) {
    ++snapshot_.goals_reached;
    finish_success();
   } else if (step.action == "pickup") {
    ++snapshot_.goals_reached;
    snapshot_.via_reached = true;
    result_.via_reached = true;
    const std::string next_label = snapshot_.goals_reached < goals.size()
                                       ? goals[snapshot_.goals_reached].label
                                       : level_.objective.goal_label;
    record_event("via-reached", catalog_.trf("event.via_reached", {step.label, next_label}));
   } else if (step.action == "recharge") {
    // Pass-through goal: top up the battery account, keep driving. Net energy
    // never goes negative; the top-up amount is content, the accounting here.
    ++snapshot_.goals_reached;
    result_.energy_wh = std::max(0.0, result_.energy_wh - step.recharge_wh);
    record_event("recharged", catalog_.trf("event.recharged", {step.label}));
   } else if (step.action == "summon") {
    // Pass-through goal: wake a dormant actor; its clock starts now, so the
    // encounter reproduces from this moment on every same-seed run.
    ++snapshot_.goals_reached;
    bool woke = false;
    for (std::size_t index = 0; index < dormant_ids_.size(); ++index) {
     if (dormant_ids_[index] == step.summon_actor) {
      patrols_.push_back(dormant_patrols_[index]);
      patrol_ids_.push_back(dormant_ids_[index]);
      patrol_t0_.push_back(snapshot_.time_s);
      actual_actors_.push_back({});
      dormant_patrols_.erase(dormant_patrols_.begin() + (std::ptrdiff_t)index);
      dormant_ids_.erase(dormant_ids_.begin() + (std::ptrdiff_t)index);
      woke = true;
      break;
     }
    }
    // Full-literal trf keys (no dynamic key building): the content gate greps
    // string literals, constructed keys would slip past it.
    if (woke) {
     record_event("summoned", catalog_.trf("event.summoned_ok", {step.label}));
    } else {
     record_event("summoned", catalog_.trf("event.summoned_miss", {step.label}));
    }
   }
  }
 }
 if (snapshot_.status == MissionStatus::Running && snapshot_.time_s >= level_.objective.time_limit_s) {
  finish_failure("timeout", catalog_.tr("event.timeout_failure"));
 }
 if (snapshot_.time_s + 1e-9 >= next_replay_s_) {
  record_replay();
  next_replay_s_ += 0.05;
 }
}

void MissionSimulation::update_actors() {
 snapshot_.actors.clear();
 for (std::size_t index = 0; index < patrols_.size(); ++index) {
  actual_actors_[index] =
      sim::patrol_state(patrols_[index], snapshot_.time_s - patrol_t0_[index]);
  snapshot_.actors.push_back({
    patrol_ids_[index],
    actual_actors_[index].p,
    actual_actors_[index].v,
    actual_actors_[index].radius,
    false,
  });
 }
 refresh_dynamic_obstacles();
}

void MissionSimulation::scan_environment() {
 if (!has("sensor"))
  return;
 const sim::Vec2 position{snapshot_.robot.x, snapshot_.robot.y};
 const int rays = static_cast<int>(spec("sensor", "rays"));
 const double range = spec("sensor", "range_m");
 const auto scan = lidar_->scan(position, snapshot_.robot.theta, rays, range, spec("sensor", "noise_sigma_m"),
                                rng_, actual_actors_);
 snapshot_.lidar_hits.clear();
 std::vector<int> dynamic_hit_count(actual_actors_.size(), 0);
 dynamic_centroid_.assign(actual_actors_.size(), {0.0, 0.0});
 const bool first_static_detection = !snapshot_.static_obstacle_detected;
 for (int index = 0; index < scan.n_rays; ++index) {
  if (!scan.hit[index]) {
   continue;
  }
  if (level_.objective.type == "observe_and_stop" && scan.ranges[index] <= std::min(range, 2.5)) {
   snapshot_.static_obstacle_detected = true;
  }
  const double angle = snapshot_.robot.theta + (2.0 * sim::kPi * index / scan.n_rays);
  const sim::Vec2 endpoint{
    position.x + std::cos(angle) * scan.ranges[index],
    position.y + std::sin(angle) * scan.ranges[index],
  };
  snapshot_.lidar_hits.push_back(endpoint);
  const int dynamic_index = scan.dynamic_hit[index];
  if (dynamic_index >= 0 && static_cast<std::size_t>(dynamic_index) < actual_actors_.size()) {
   ++dynamic_hit_count[dynamic_index];
   dynamic_centroid_[dynamic_index].x += endpoint.x;
   dynamic_centroid_[dynamic_index].y += endpoint.y;
  }
 }

 if (first_static_detection && snapshot_.static_obstacle_detected &&
     level_.objective.type == "observe_and_stop") {
  record_event("obstacle-detected", catalog_.tr("event.obstacle_detected"));
 }

 // Observation confirmation: a single scan of hits is only a suspect.
 // A track is born after two consecutive scans see the same actor, and dies
 // after track_max_age_s without confirmation. First sight is noisy by nature;
 // persistence is the cheapest honest filter.
 bool first_detection = !snapshot_.dynamic_object_detected;
 if (confirm_count_.size() != actual_actors_.size()) {
  confirm_count_.assign(actual_actors_.size(), 0);
 }
 for (std::size_t index = 0; index < actual_actors_.size(); ++index) {
  if (dynamic_hit_count[index] > 0) {
   ++confirm_count_[index];
   if (confirm_count_[index] < 2) {
    continue;
   }
  } else {
   confirm_count_[index] = 0;
   continue;
  }
  {
   // Honest detector: the center is completed from this scan's own hit
   // endpoints, never the actor's true position. Only the facing arc is
   // visible, so its centroid sits ~one radius toward the robot; radial
   // completion by the (known) radius restores the center to centimeter
   // accuracy at range. Ray-to-actor association and the radius still come
   // from the simulator (documented simplifications); route and velocity
   // are never exposed.
   const double hits = static_cast<double>(dynamic_hit_count[index]);
   sim::Vec2 arc{dynamic_centroid_[index].x / hits, dynamic_centroid_[index].y / hits};
   sim::Vec2 sight{arc.x - position.x, arc.y - position.y};
   const double sight_len = std::max(1e-6, std::hypot(sight.x, sight.y));
   const double residual = spec("sensor", "noise_sigma_m") / (2.0 * std::sqrt(hits));
   const sim::Vec2 measured_center{
     arc.x + sight.x / sight_len * actual_actors_[index].radius + perception_rng_.gauss(residual),
     arc.y + sight.y / sight_len * actual_actors_[index].radius + perception_rng_.gauss(residual),
   };
   dynamic_obstacle_map_.observe(
     {
       index,
       measured_center,
       actual_actors_[index].radius,
     },
     snapshot_.time_s);
   snapshot_.dynamic_object_detected = true;
  }
 }
 refresh_dynamic_obstacles();
 if (first_detection && snapshot_.dynamic_object_detected) {
  double first_range = 0.0;
  for (const auto& obstacle : snapshot_.dynamic_obstacles) {
   const double range = std::hypot(obstacle.estimated_position.x - snapshot_.robot.x,
                                   obstacle.estimated_position.y - snapshot_.robot.y);
   if (first_range <= 0.0 || range < first_range) {
    first_range = range;
   }
  }
  std::ostringstream detected;
  detected << std::fixed << std::setprecision(1) << first_range;
  record_event("dynamic-detected",
               catalog_.trf("event.dynamic_detected", {detected.str()}));
 }
}

void MissionSimulation::refresh_dynamic_obstacles() {
 snapshot_.dynamic_obstacles.clear();
 const auto tracks = dynamic_obstacle_map_.active_tracks(snapshot_.time_s, track_max_age_s());
 for (const auto& track : tracks) {
  if (track.object_id >= patrol_ids_.size())
   continue;
  snapshot_.dynamic_obstacles.push_back({
    patrol_ids_[track.object_id],
    track.state.p,
    track.state.v,
    track.state.radius,
    track.last_observed_s,
  });
  if (track.object_id < snapshot_.actors.size()) {
   snapshot_.actors[track.object_id].detected = true;
  }
 }
}

sim::Vec2 MissionSimulation::current_navigation_target() {
 const double lookahead = property(behavior_, "lookahead_m");
 const bool fused_localization = level_.localization_mode == "fused";
 const double nav_x = fused_localization ? snapshot_.robot.x : odometry_.p.x;
 const double nav_y = fused_localization ? snapshot_.robot.y : odometry_.p.y;
 const double nav_heading = fused_localization ? snapshot_.robot.theta : odometry_.th;
 sim::advance_wp(planned_path_, guide_state_.idx, nav_x, nav_y, nav_heading, 0.55, lookahead);
 if (guide_state_.idx >= planned_path_.size()) {
  return as_vec(map_definition_.markers.at(level_.objective.goal));
 }
 return sim::lookahead_pt(planned_path_, guide_state_.idx, nav_x, nav_y, lookahead, *planning_map_);
}

void MissionSimulation::decide_control() {
 ++result_.local_decisions;
 snapshot_.control = {};
 if (guide_state_.idx >= planned_path_.size()) {
  if (planned_path_.empty()) {
   held_command_ = {0.0, 0.0};
   return;
  }
  // Docking hold: waypoint bookkeeping exhausted (resync/accept ran ahead of
  // a lagging chassis) but arrival tolerance unmet — success is measured by
  // distance, not by idx. Hold the last waypoint so normal tracking (reverse
  // hysteresis, watchdog, DWA arbitration below) docks the robot instead of
  // parking short with tv=0 until timeout.
  guide_state_.idx = planned_path_.size() - 1;
 }
 const auto target = current_navigation_target();
 const bool fused_localization = level_.localization_mode == "fused";
 const double nav_x = fused_localization ? snapshot_.robot.x : odometry_.p.x;
 const double nav_y = fused_localization ? snapshot_.robot.y : odometry_.p.y;
 const double nav_heading = fused_localization ? snapshot_.robot.theta : odometry_.th;
 auto route_control = sim::guide_step(planned_path_, guide_state_, nav_x, nav_y, nav_heading, *heading_pid_,
                                      property(behavior_, "lookahead_m"), property(behavior_, "cruise_mps"),
                                      1.0 / decision_hz(), *planning_map_, chassis_->caps());
 snapshot_.control.local_target = target;
 snapshot_.control.heading_error_rad = route_control.err;
 snapshot_.control.heading_pid_p = heading_pid_->p();
 snapshot_.control.heading_pid_i = heading_pid_->i();
 snapshot_.control.heading_pid_d = heading_pid_->d();
 // Docking controller: guide_step reports reached when waypoint bookkeeping
 // exhausts (accept/resync ran ahead of a lagging chassis), but mission
 // success is distance-to-goal. If the goal is still beyond tolerance yet
 // close, dock with the same PID instead of parking short until timeout.
 // Structure mirrors guide_step (reverse hysteresis, spin capability branch)
 // so behavior stays consistent across the tracking/docking seam.
 {
  const auto goal = as_vec(map_definition_.markers.at(level_.objective.goal));
  const double dist_goal = distance({nav_x, nav_y}, goal);
  // Bounded: far exhaustion means the follower lost the path, not that the
  // goal is near; beelining across the map without waypoints risks walls.
  // (An unbounded trial stranded swift tracks in a reversing cycle instead
  // of docking — evidence for curation, not for a longer leash.)
  if (route_control.reached && dist_goal > level_.objective.arrive_tolerance_m && dist_goal < 3.0) {
   const double dock_err = sim::heading_err(goal, nav_x, nav_y, nav_heading);
   if (!guide_state_.reversing && std::abs(dock_err) > 1.4)
    guide_state_.reversing = true;
   else if (guide_state_.reversing && std::abs(dock_err) < 0.7)
    guide_state_.reversing = false;
   const double dock_dt = 1.0 / decision_hz();
   if (!guide_state_.reversing) {
    route_control.tv = std::min(0.3, dist_goal);
    route_control.steer = heading_pid_->update(dock_err, dock_dt);
   } else if (chassis_->caps().can_spin) {
    guide_state_.reversing = false;
    route_control.tv = 0.0;
    route_control.steer = heading_pid_->update(dock_err, dock_dt);
   } else {
    route_control.tv = -0.3;
    route_control.steer = -heading_pid_->update(dock_err, dock_dt);
   }
   route_control.err = dock_err;
   route_control.reached = false;
  }
 }
 if (actual_actors_.empty()) {
  const double preview_cap = corner_speed_cap(nav_x, nav_y);
  held_command_ = {route_control.tv > 0.0 ? std::min(route_control.tv, preview_cap) : route_control.tv,
                   route_control.steer};
  snapshot_.waiting_for_clearance = false;
  update_shadow_episode();
  snapshot_.control.shadow_assessed = false;
  snapshot_.control.shadow_pocket_verdict = -1;
  snapshot_.control.shadow_pocket_feasible = 0;
  snapshot_.control.shadow_pocket_total = 0;
  for (int& bin : snapshot_.control.shadow_pocket_reasons) {
   bin = 0;
  }
  return;
 }

 const auto known_tracks = dynamic_obstacle_map_.active_tracks(snapshot_.time_s, track_max_age_s());

 double nearest_actor = std::numeric_limits<double>::infinity();
 for (const auto& track : known_tracks) {
  const auto& actor = track.state;
  nearest_actor = std::min(nearest_actor, distance({snapshot_.robot.x, snapshot_.robot.y}, actor.p) -
                                            actor.radius - robot_definition_.collision_radius_m);
 }
 const double prediction_horizon = has("computer") ? spec("computer", "prediction_horizon_s") : 1.2;
 const double requested_clearance = property(behavior_, "safe_clearance_m");
 const double risk_distance =
   requested_clearance * requested_clearance * 2.5 + property(behavior_, "cruise_mps") * 0.6 + 0.1;
 if (known_tracks.empty() || nearest_actor > risk_distance) {
  const double preview_cap = corner_speed_cap(nav_x, nav_y);
  held_command_ = {route_control.tv > 0.0 ? std::min(route_control.tv, preview_cap) : route_control.tv,
                   route_control.steer};
  snapshot_.waiting_for_clearance = false;
  update_shadow_episode();
  snapshot_.control.shadow_assessed = false;
  snapshot_.control.shadow_pocket_verdict = -1;
  snapshot_.control.shadow_pocket_feasible = 0;
  snapshot_.control.shadow_pocket_total = 0;
  for (int& bin : snapshot_.control.shadow_pocket_reasons) {
   bin = 0;
  }
  return;
 }

 const double heading_error =
   sim::heading_err(target, snapshot_.robot.x, snapshot_.robot.y, snapshot_.robot.theta);
 const double speed_cap =
   std::min(std::max(0.20, sim::sched_v(heading_error, property(behavior_, "cruise_mps"))),
            corner_speed_cap(nav_x, nav_y));
 const sim::DwaWeights weights{1.0, 1.3 + requested_clearance * 1.8, 0.25};
 constexpr double prediction_step_s = 0.1;
 // Predictor module (src/algo/predictor.*) owns future frames; the tracker
 // owns observations. known_tracks (age-advanced, for display) must not
 // feed prediction: map.predict() snapshots raw tracker state internally.
 const auto predicted_frames =
  dynamic_obstacle_map_.predict(snapshot_.time_s, prediction_horizon, prediction_step_s, track_max_age_s());
 // Shadow departure assessment (gate 3): observe only. This block reads
 // tracker state and writes snapshot shadow fields; held_command_ (the actual
 // control output) is untouched by everything in this block.
 std::vector<sim::Vec2> shadow_remaining{{snapshot_.robot.x, snapshot_.robot.y}};
 for (std::size_t i = guide_state_.idx; i < planned_path_.size(); ++i) {
  shadow_remaining.push_back(planned_path_[i]);
 }
 const double shadow_cruise = std::max(1e-6, property(behavior_, "cruise_mps"));
 {
  double shadow_length = 0.0;
  for (std::size_t i = 0; i + 1 < shadow_remaining.size(); ++i) {
   shadow_length += distance(shadow_remaining[i + 1], shadow_remaining[i]);
  }
  // Wait slack = the computer's own prediction horizon (existing content, not
  // a new knob): waits beyond what the computer predicts are unassessable by
  // construction (kHorizonShort) — which is itself shadow data.
  const double shadow_horizon = shadow_length / shadow_cruise + prediction_horizon;
  const auto shadow_frames = dynamic_obstacle_map_.predict(snapshot_.time_s, shadow_horizon,
                                                           prediction_step_s, track_max_age_s());
  const auto assessment = sim::assess_departure(shadow_remaining, 0, shadow_cruise, shadow_frames,
                                                prediction_step_s,
                                                robot_definition_.collision_radius_m,
                                                requested_clearance);
  snapshot_.control.shadow_assessed = true;
  snapshot_.control.shadow_depart_s = assessment.wait_s;
  snapshot_.control.shadow_reason = static_cast<int>(assessment.reason);
 }
 // Spatial pocket shadow (Gate B observation): same read-only contract, only
 // while waiting (last cycle's flag) — bounded cost, and waits are exactly
 // what Gate B must accumulate. Observation budget (window/offsets/cap), not
 // tuned: it cannot affect behavior by construction.
 snapshot_.control.shadow_pocket_verdict = -1;
 snapshot_.control.shadow_pocket_feasible = 0;
 snapshot_.control.shadow_pocket_total = 0;
 for (int& bin : snapshot_.control.shadow_pocket_reasons) {
  bin = 0;
 }
 if (snapshot_.waiting_for_clearance) {
  sim::PocketSampleParams pocket_params;
  pocket_params.lateral_offsets_m = {1.5, -1.5};
  pocket_params.window_m = 6.0;
  pocket_params.vertex_stride_m = 1.5;
  pocket_params.min_downstream_m = 3.0;
  pocket_params.min_pocket_dist_m = 0.3;
  pocket_params.max_candidates = 12;
  double shadow_length = 0.0;
  for (std::size_t i = 0; i + 1 < shadow_remaining.size(); ++i) {
   shadow_length += distance(shadow_remaining[i + 1], shadow_remaining[i]);
  }
  const double pocket_horizon = shadow_length / shadow_cruise + prediction_horizon;
  const auto pocket_frames = dynamic_obstacle_map_.predict(snapshot_.time_s, pocket_horizon,
                                                           prediction_step_s, track_max_age_s());
  const auto search = sim::sample_pockets(
    *planning_map_, shadow_remaining, {snapshot_.robot.x, snapshot_.robot.y}, snapshot_.robot.theta,
    shadow_cruise, sim::ChassisExecCaps{chassis_->caps().can_spin, yaw_cap_rads_,
                                        robot_definition_.wheel_base_m,
                                        robot_definition_.steering.max_steer_rad,
                                        robot_definition_.steering.max_steer_rate_radps,
                                        robot_definition_.steering.servo_response_s},
    prediction_horizon, pocket_params, pocket_frames, prediction_step_s,
    robot_definition_.collision_radius_m, requested_clearance);
  int pocket_feasible = 0;
  for (const auto& candidate : search.candidates) {
   if (candidate.assessment.overall == sim::Feasibility::Verdict::kFeasible) {
    ++pocket_feasible;
   }
   ++snapshot_.control.shadow_pocket_reasons[static_cast<int>(candidate.assessment.transit.reason)];
   ++snapshot_.control.shadow_pocket_reasons[static_cast<int>(candidate.assessment.hold.reason)];
   ++snapshot_.control.shadow_pocket_reasons[static_cast<int>(candidate.assessment.rejoin.reason)];
  }
  snapshot_.control.shadow_pocket_verdict = static_cast<int>(search.verdict);
  snapshot_.control.shadow_pocket_feasible = pocket_feasible;
  snapshot_.control.shadow_pocket_total = static_cast<int>(search.candidates.size());
 }
 const auto dwa = sim::dwa_compute_trajectory(
   {snapshot_.robot.x, snapshot_.robot.y}, snapshot_.robot.theta, snapshot_.robot.v, snapshot_.robot.w,
   target, *planning_map_, weights, predicted_frames, speed_cap, yaw_cap_rads_,
   spec("tires", "brake_decel_mps2"), prediction_horizon, prediction_step_s, prediction_step_s,
   requested_clearance, robot_definition_.collision_radius_m, robot_definition_.max_yaw_accel_radps2);
 snapshot_.control.dwa_active = true;
 snapshot_.control.dwa_selected = dwa.best_path;
 constexpr std::size_t kVisibleDwaCandidates = 24;
 const std::size_t candidate_stride =
   std::max<std::size_t>(1, (dwa.rollouts.size() + kVisibleDwaCandidates - 1) / kVisibleDwaCandidates);
 for (std::size_t index = 0; index < dwa.rollouts.size(); index += candidate_stride) {
  const auto& rollout = dwa.rollouts[index];
  snapshot_.control.dwa_candidates.push_back({
    rollout.points,
    rollout.collision,
    rollout.command.score,
  });
 }
 if (!dwa.best.valid) {
  held_command_ = {0.0, 0.0};
  if (!snapshot_.waiting_for_clearance) {
   std::ostringstream waiting;
   waiting << std::fixed << std::setprecision(1) << nearest_actor;
   record_event("waiting", catalog_.trf("event.waiting", {waiting.str()}));
  }
  snapshot_.waiting_for_clearance = true;
  shadow_open_speed_ = 0.0;
  for (const auto& track : known_tracks) {
   shadow_open_speed_ =
     std::max(shadow_open_speed_, std::hypot(track.state.v.x, track.state.v.y));
  }
  shadow_open_nearest_ = nearest_actor;
  update_shadow_episode();
 } else {
  // Arbitration without magic gains: while the avoidance speed is healthy, DWA
  // owns both speed and turn; at creep speed the robot holds the route heading
  // with the same derived proportional gain (P-only: integrator and derivative
  // states are meaningless at standstill and would wind up or kick).
  constexpr double kCreepSpeedMps = 0.15;
  const double arbitrated_turn = std::abs(dwa.best.v) < kCreepSpeedMps
                                   ? std::clamp(heading_kp_ * heading_error, -yaw_cap_rads_, yaw_cap_rads_)
                                   : dwa.best.w;
  held_command_ = {dwa.best.v < 0.0 ? dwa.best.v
                                    : std::min({std::max(0.0, route_control.tv), dwa.best.v,
                                                corner_speed_cap(nav_x, nav_y)}),
                   arbitrated_turn};
  snapshot_.waiting_for_clearance = std::abs(dwa.best.v) < kCreepSpeedMps;
  if (snapshot_.waiting_for_clearance) {
   std::ostringstream waiting;
   waiting << std::fixed << std::setprecision(1) << nearest_actor;
   record_event("waiting", catalog_.trf("event.waiting", {waiting.str()}), 2.0);
  }
  shadow_open_speed_ = 0.0;
  for (const auto& track : known_tracks) {
   shadow_open_speed_ =
     std::max(shadow_open_speed_, std::hypot(track.state.v.x, track.state.v.y));
  }
  shadow_open_nearest_ = nearest_actor;
  update_shadow_episode();
 }
}

void MissionSimulation::apply_motion() {
 snapshot_.control.target_v_mps = held_command_.target_v;
 snapshot_.control.target_w_radps = held_command_.target_w;
 // NOTE (FF verdict): a shadow feedforward computation lived here in stage 3
 // and was removed when stage 4 measured net-negative tracking benefit with
 // saturation up. The pure mapping (src/hw/feedforward.*) and its evidence
 // (test_feedforward, test_ff_sensitivity) stay parked with the verdict; no
 // per-step shadow compute remains in this hot path.
 const auto before = snapshot_.robot;
 auto candidate = chassis_->step(before, held_command_, fixed_step_s, wheel_ground({before.x, before.y}));
 const auto& motors = chassis_->telemetry();
 // Telemetry shape follows the chassis (telemetry_.differential selects the
 // active side). Ackermann mirrors the single drive unit into the left_*
 // fields and reports the servo below; the right_* fields stay zero rather
 // than carrying fiction.
 const auto& drive_unit = motors.differential ? motors.left : motors.drive;
 snapshot_.control.left_target_mps = drive_unit.target_mps;
 snapshot_.control.right_target_mps = motors.differential ? motors.right.target_mps : 0.0;
 snapshot_.control.left_measured_mps = drive_unit.measured_mps;
 snapshot_.control.right_measured_mps = motors.differential ? motors.right.measured_mps : 0.0;
 snapshot_.control.left_pwm = drive_unit.pwm;
 snapshot_.control.right_pwm = motors.differential ? motors.right.pwm : 0.0;
 snapshot_.control.left_speed_pid_p = drive_unit.pid_p;
 snapshot_.control.left_speed_pid_i = drive_unit.pid_i;
 snapshot_.control.left_speed_pid_d = drive_unit.pid_d;
 snapshot_.control.right_speed_pid_p = motors.differential ? motors.right.pid_p : 0.0;
 snapshot_.control.right_speed_pid_i = motors.differential ? motors.right.pid_i : 0.0;
 snapshot_.control.right_speed_pid_d = motors.differential ? motors.right.pid_d : 0.0;
 // Shadow slip detection (upper-bound study): TRACKED ONLY. Differential
 // encoders cannot observe tread (lesson §5: they read body-twist-derived
 // wheel speeds), so the detector stays off there by design, not omission.
 // True body accel stands in for the IMU: perfect-IMU upper bound, labeled
 // as such. Encoder side is real quantized tread telemetry.
 if (robot_definition_.drive_type == "tracked") {
  const double tread_v =
    (snapshot_.control.left_measured_mps + snapshot_.control.right_measured_mps) / 2.0;
  if (slip_have_prev_) {
   const double encoder_accel = (tread_v - slip_prev_tread_v_) / fixed_step_s;
   const double true_accel = (snapshot_.robot.v - slip_prev_body_v_) / fixed_step_s;
   slip_detector_.step(encoder_accel, true_accel, fixed_step_s);
   if (slip_detector_.episodes() > static_cast<int>(slip_episode_times_.size())) {
    slip_episode_times_.push_back(snapshot_.time_s);
   }
  }
  slip_prev_tread_v_ = tread_v;
  slip_prev_body_v_ = snapshot_.robot.v;
  slip_have_prev_ = true;
 }
 snapshot_.control.steer_target_rad = motors.differential ? 0.0 : motors.steer.target_rad;
 snapshot_.control.steer_angle_rad = motors.differential ? 0.0 : motors.steer.angle_rad;
 snapshot_.control.steer_pwm = motors.differential ? 0.0 : motors.steer.pwm;
 snapshot_.control.steer_left_rad = motors.differential ? 0.0 : motors.front_left_steer_rad;
 snapshot_.control.steer_right_rad = motors.differential ? 0.0 : motors.front_right_steer_rad;
 const double meters_per_revolution = 2.0 * sim::kPi * robot_definition_.wheel_radius_m;
 snapshot_.control.left_target_rpm = snapshot_.control.left_target_mps * 60.0 / meters_per_revolution;
 snapshot_.control.right_target_rpm = snapshot_.control.right_target_mps * 60.0 / meters_per_revolution;
 snapshot_.control.left_measured_rpm = snapshot_.control.left_measured_mps * 60.0 / meters_per_revolution;
 snapshot_.control.right_measured_rpm = snapshot_.control.right_measured_mps * 60.0 / meters_per_revolution;
 const sim::Vec2 candidate_position{candidate.x, candidate.y};

 if (!collision_map_->isFree(candidate.x, candidate.y)) {
  ++result_.collisions;
  finish_failure("collision-static", "洛一与固定障碍发生碰撞。");
  return;
 }
 for (const auto& actor : actual_actors_) {
  const double clearance =
    distance(candidate_position, actor.p) - actor.radius - robot_definition_.collision_radius_m;
  result_.min_clearance_m = std::min(result_.min_clearance_m, clearance);
  if (clearance < result_.min_dynamic_clearance_m) {
   result_.min_dynamic_clearance_m = clearance;
   result_.min_dynamic_clearance_time_s = snapshot_.time_s;
   result_.min_dynamic_robot_position = candidate_position;
   result_.min_dynamic_actor_position = actor.p;
  }
  if (clearance < 0.0) {
   ++result_.collisions;
   finish_failure("collision-dynamic", "洛一与巡逻叉车发生碰撞。");
   return;
  }
 }

 snapshot_.robot = candidate;
 result_.path_length_m += distance({before.x, before.y}, candidate_position);
 const double motor_power = robot_definition_.energy_base_w +
                            robot_definition_.energy_per_v_w * std::abs(candidate.v) +
                            robot_definition_.energy_per_w_w * std::abs(candidate.w);
 const double motor_scale = has("motor") ? spec("motor", "power_scale") : 1.0;
 const double total_power = motor_power * spec("tires", "power_scale") * motor_scale +
                            (has("sensor") ? spec("sensor", "power_w") : 0.0) +
                            (has("computer") ? spec("computer", "power_w") : 0.0) +
                            (has("imu") ? spec("imu", "power_w") : 0.0);
 result_.energy_wh += total_power * fixed_step_s / 3600.0;

 double slip = spec("tires", "slip_scale");
 const double traction = surface_traction(candidate_position);
 if (traction < 1.0) {
  slip /= traction;
 }
 // Traction saturation (model-side contact state, like motor force: used for
 // odometry degradation, never displayed as a sensed reading). Rising edge
 // counts a slip episode for the debrief.
 const bool saturated =
   motors.differential ? (motors.left.slipping || motors.right.slipping) : motors.drive.slipping;
 if (saturated && !was_saturated_) {
  ++result_.slip_events;
 }
 was_saturated_ = saturated;
 if (saturated) {
  slip *= 3.0;
 }
 double encoder_v = 0.0;
 double encoder_w = 0.0;
 if (motors.differential) {
  sim::DiffDrive::wheelsToTwist(motors.left.measured_mps, motors.right.measured_mps,
                                robot_definition_.wheel_base_m, encoder_v, encoder_w);
 } else {
  // Ackermann odometry: drive-wheel speed plus steer geometry. At standstill
  // the geometry gives zero yaw — a car cannot turn without rolling.
  encoder_v = motors.drive.measured_mps;
  encoder_w = sim::yaw_for_steer(encoder_v, motors.steer.angle_rad, robot_definition_.wheel_base_m);
 }
 const double gyro_rate =
   has("imu") ? candidate.w + gyro_bias_ + imu_rng_.gauss(spec("imu", "gyro_noise_radps")) : 0.0;
 const double gyro_sigma = has("imu") ? spec("imu", "gyro_noise_radps") : 0.0;
 odometry_.update(encoder_v, encoder_w, fixed_step_s, rng_, slip, 0.003 * slip, has("imu"), gyro_rate,
                  gyro_sigma);
 snapshot_.estimated_position = odometry_.p;
 snapshot_.estimated_heading = odometry_.th;
 result_.odometry_error_m = odometry_.driftErr(candidate_position);
 const double fixed_clearance = static_clearance(candidate_position);
 if (fixed_clearance < result_.min_static_clearance_m) {
  result_.min_static_clearance_m = fixed_clearance;
  result_.min_static_clearance_time_s = snapshot_.time_s;
  result_.min_static_robot_position = candidate_position;
 }
 result_.min_clearance_m = std::min(result_.min_clearance_m, fixed_clearance);
}

double MissionSimulation::surface_traction(sim::Vec2 position) const {
 for (const auto& surface : map_definition_.surfaces) {
  if (active_in_level(surface.active_in, level_id_) && inside(position, surface)) {
   return surface.traction_scale;
  }
 }
 return 1.0;
}

double MissionSimulation::static_clearance(sim::Vec2 position) const {
 double minimum = std::min({position.x, position.y, map_definition_.size_m.x - position.x,
                            map_definition_.size_m.y - position.y}) -
                  robot_definition_.collision_radius_m;
 for (const auto& obstacle : map_definition_.obstacles) {
  if (!active_in_level(obstacle.active_in, level_id_)) {
   continue;
  }
  const double dx = std::max({obstacle.min.x - position.x, 0.0, position.x - obstacle.max.x});
  const double dy = std::max({obstacle.min.y - position.y, 0.0, position.y - obstacle.max.y});
  minimum = std::min(minimum, std::hypot(dx, dy) - robot_definition_.collision_radius_m);
 }
 return std::max(0.0, minimum);
}

void MissionSimulation::finish_success() {
 held_command_ = {};
 snapshot_.status = MissionStatus::Succeeded;
 snapshot_.robot.v = 0.0;
 snapshot_.robot.w = 0.0;
 snapshot_.control.target_v_mps = 0.0;
 snapshot_.control.target_w_radps = 0.0;
 snapshot_.control.left_target_mps = 0.0;
 snapshot_.control.right_target_mps = 0.0;
 snapshot_.control.left_target_rpm = 0.0;
 snapshot_.control.right_target_rpm = 0.0;
 snapshot_.control.left_measured_mps = 0.0;
 snapshot_.control.right_measured_mps = 0.0;
 snapshot_.control.left_measured_rpm = 0.0;
 snapshot_.control.right_measured_rpm = 0.0;
 snapshot_.control.left_pwm = 0.0;
 snapshot_.control.right_pwm = 0.0;
 result_.status = MissionStatus::Succeeded;
 result_.safe_score = result_.collisions == 0 && result_.min_clearance_m >= level_.safe_min_clearance_m &&
                      result_.odometry_error_m <= level_.max_localization_error_m;
 result_.fast_score = result_.time_s <= level_.fast_time_s;
 result_.efficient_score = result_.energy_wh <= level_.efficient_energy_wh;
 record_event("mission-complete", "已抵达" + level_.objective.goal_label + "，任务完成。");
 record_replay();
}

void MissionSimulation::finish_failure(std::string reason, std::string message) {
 held_command_ = {};
 snapshot_.status = MissionStatus::Failed;
 snapshot_.robot.v = 0.0;
 snapshot_.robot.w = 0.0;
 snapshot_.control.target_v_mps = 0.0;
 snapshot_.control.target_w_radps = 0.0;
 snapshot_.control.left_target_mps = 0.0;
 snapshot_.control.right_target_mps = 0.0;
 snapshot_.control.left_target_rpm = 0.0;
 snapshot_.control.right_target_rpm = 0.0;
 snapshot_.control.left_measured_mps = 0.0;
 snapshot_.control.right_measured_mps = 0.0;
 snapshot_.control.left_measured_rpm = 0.0;
 snapshot_.control.right_measured_rpm = 0.0;
 snapshot_.control.left_pwm = 0.0;
 snapshot_.control.right_pwm = 0.0;
 result_.status = MissionStatus::Failed;
 result_.failure_reason = std::move(reason);
 result_.safe_score = false;
 result_.fast_score = false;
 result_.efficient_score = false;
 record_event(result_.failure_reason, std::move(message));
 record_replay();
}

void MissionSimulation::record_event(std::string code, std::string message, double throttle_s) {
 if (code == last_event_code_ && snapshot_.time_s - last_event_time_s_ < throttle_s) {
  return;
 }
 last_event_code_ = code;
 last_event_time_s_ = snapshot_.time_s;
 events_.push_back({snapshot_.time_s, std::move(code), std::move(message)});
}

void MissionSimulation::update_shadow_episode() {
 if (snapshot_.waiting_for_clearance && !shadow_episode_open_) {
  shadow_episode_open_ = true;
  shadow_episode_start_s_ = snapshot_.time_s;
  shadow_episode_first_s_ = snapshot_.control.shadow_depart_s;
  shadow_episode_first_reason_ = snapshot_.control.shadow_reason;
  shadow_episode_entry_speed_ = shadow_open_speed_;
  shadow_episode_entry_nearest_ = shadow_open_nearest_;
 } else if (!snapshot_.waiting_for_clearance && shadow_episode_open_) {
  shadow_episode_open_ = false;
  ShadowEpisode episode;
  episode.start_s = shadow_episode_start_s_;
  episode.first_pred_s = shadow_episode_first_s_;
  episode.first_reason = shadow_episode_first_reason_;
  episode.entry_threat_speed_mps = shadow_episode_entry_speed_;
  episode.entry_nearest_m = shadow_episode_entry_nearest_;
  episode.end_s = snapshot_.time_s;
  episode.last_pred_s = snapshot_.control.shadow_depart_s;
  episode.last_reason = snapshot_.control.shadow_reason;
  episode.waited_s = snapshot_.time_s - shadow_episode_start_s_;
  shadow_episodes_.push_back(episode);
  std::ostringstream shadow;
  shadow << std::fixed << std::setprecision(1) << "影子评估：预测初值" << shadow_episode_first_s_
         << "秒[" << sim::depart_reason_name(static_cast<sim::DepartReason>(shadow_episode_first_reason_))
         << "]→末值" << snapshot_.control.shadow_depart_s << "秒["
         << sim::depart_reason_name(static_cast<sim::DepartReason>(snapshot_.control.shadow_reason))
         << "]，实际等待" << (snapshot_.time_s - shadow_episode_start_s_) << "秒。";
  record_event("shadow_wait", shadow.str());
 }
}

void MissionSimulation::record_replay() {
 MissionSnapshot frame = snapshot_;
 // Candidate fans are useful live but would dominate a 20 Hz replay. The chosen
 // local path and all control values remain available for deterministic review.
 frame.control.dwa_candidates.clear();
 replay_.push_back(std::move(frame));
}

bool MissionSimulation::has(const std::string& slot) const {
 return parts_.find(slot) != parts_.end();
}

const ComponentDefinition& MissionSimulation::req(const std::string& slot) const {
 const auto found = parts_.find(slot);
 if (found == parts_.end()) {
  throw ContentError("missing required part '" + slot + "'");
 }
 return *found->second;
}

double MissionSimulation::spec(const std::string& slot, const std::string& key) const {
 return property(req(slot).behavior, key);
}

// Corner preview cap: the tightest bend within braking + reaction preview
// sets the speed ceiling via lateral physics (see follow.hpp). Grip is the
// minimum tire×surface mu over the preview — wet pavement ahead slows the
// car before it arrives, not after it slides.
double MissionSimulation::corner_speed_cap(double nav_x, double nav_y) const {
 const double cruise = property(behavior_, "cruise_mps");
 const double lookahead = property(behavior_, "lookahead_m");
 const double speed = std::abs(snapshot_.robot.v);
 const double decel = spec("tires", "brake_decel_mps2");
 const double preview =
   std::max(lookahead, speed * speed / (2.0 * std::max(0.5, decel)) + speed * 2.0 / decision_hz());
 double grip = wheel_ground({nav_x, nav_y}).mu_eff;
 double along = 0.0;
 for (size_t i = guide_state_.idx + 1; i < planned_path_.size(); ++i) {
  along += distance(planned_path_[i], planned_path_[i - 1]);
  if (along > preview) {
   break;
  }
  grip = std::min(grip, wheel_ground({planned_path_[i].x, planned_path_[i].y}).mu_eff);
 }
 return sim::corner_speed_cap(planned_path_, guide_state_.idx, cruise, 0.5 * grip * 9.81, preview);
}

double MissionSimulation::decision_hz() const {
 return has("computer") ? spec("computer", "decision_hz") : kBuiltInDecisionHz;
}

double MissionSimulation::track_max_age_s() const {
 // Dynamic-track memory lives on the computer; computer-less levels use the
 // documented engine default.
 return has("computer") ? spec("computer", "track_max_age_s") : kDynamicTrackMaxAgeS;
}

sim::GroundContact MissionSimulation::wheel_ground(sim::Vec2 position) const {
 // Traction is always tire tread combined with ground material: neither alone
 // decides grip. mu_ref anchors the product so concrete + standard behaves as
 // the long-validated reference.
 constexpr double kMuReference = 0.8;
 const double tire_mu = spec("tires", "friction_mu");
 const double tire_rr = spec("tires", "rolling_resistance");
 double ground_mu = 1.0;
 double ground_rr = 0.0;
 double traction = 1.0;
 for (const auto& surface : map_definition_.surfaces) {
  if (active_in_level(surface.active_in, level_id_) && inside(position, surface)) {
   ground_mu = surface.friction_mu;
   ground_rr = surface.rolling_drag;
   traction = surface.traction_scale;
   break;
  }
 }
 sim::GroundContact contact;
 contact.mu_eff = tire_mu * ground_mu / kMuReference;
 contact.rolling_resist = tire_rr + ground_rr;
 contact.accel_limit_mps2 = std::min(robot_definition_.motor.max_wheel_accel_mps2,
                                     spec("tires", "brake_decel_mps2") * (0.5 + 0.5 * traction));
 return contact;
}

} // namespace game
