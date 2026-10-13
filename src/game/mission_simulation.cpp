#include "mission_simulation.hpp"

#include "sim/astar.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
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

MissionSimulation::MissionSimulation(const ContentCatalog& catalog, std::string level_id,
                                     BuildSelection build)
    : catalog_(catalog), level_id_(std::move(level_id)), build_(std::move(build)),
      level_(catalog_.level(level_id_)), map_definition_(catalog_.map(level_.map)),
      robot_definition_(catalog_.robot(catalog_.campaign().companion_robot)),
      tires_(catalog_.component(build_.tires)),
      sensor_(build_.sensor.empty() ? nullptr : &catalog_.component(build_.sensor)),
      computer_(build_.computer.empty() ? nullptr : &catalog_.component(build_.computer)),
      behavior_(catalog_.style_behavior(build_.style)), rng_(level_.seed),
      perception_rng_(level_.seed ^ 0x9e3779b9U) {
 catalog_.validate_build(level_id_, build_);
 reset();
}

void MissionSimulation::reset() {
 rng_ = sim::Rng(level_.seed);
 perception_rng_ = sim::Rng(level_.seed ^ 0x9e3779b9U);
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
 build_world();
 plan_route();

 const auto& spawn = map_definition_.markers.at(level_.objective.spawn);
 snapshot_.robot = {spawn.x, spawn.y, spawn.theta, 0.0, 0.0};
 snapshot_.estimated_position = {spawn.x, spawn.y};
 snapshot_.estimated_heading = spawn.theta;
 snapshot_.status = MissionStatus::Ready;
 odometry_.reset(spawn.x, spawn.y, spawn.theta);
 update_actors();
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
 collision_map_ = std::make_unique<sim::GridMap>(truth_map_->inflated(sim::kRobotRadius));
 const double requested_clearance = property(behavior_, "safe_clearance_m");
 const bool has_dynamic_actors =
   std::any_of(map_definition_.actors.begin(), map_definition_.actors.end(),
               [&](const auto& actor) { return active_in_level(actor.active_in, level_id_); });
 const double planning_clearance =
   sim::kRobotRadius + std::clamp(requested_clearance * (has_dynamic_actors ? 0.80 : 0.45), 0.40, 0.75);
 planning_map_ = std::make_unique<sim::GridMap>(truth_map_->inflated(planning_clearance));

 sim::DiffParams params;
 params.v_max = std::max(1.5, property(behavior_, "cruise_mps") * 1.25);
 params.accel_max = property(tires_.behavior, "brake_decel_mps2");
 params.wheel_base = robot_definition_.wheel_base_m;
 drive_ = std::make_unique<sim::DiffDrive>(params);
 // Chassis dispatch: drive.type selects the actuator model. Differential drive
 // is the first implementation; unknown types fail loudly in content, never
 // silently fall back to another chassis.
 if (robot_definition_.drive_type != "differential") {
  throw ContentError("unsupported drive type '" + robot_definition_.drive_type + "'");
 }
 const double wheel_mass = std::max(0.5, robot_definition_.mass_kg / 2.0);
 const sim::WheelPidGains wheel_gains = sim::tune_wheel_pid(
   wheel_mass, robot_definition_.motor.stall_force_N, robot_definition_.motor.free_speed_mps,
   0.5 * robot_definition_.motor.max_wheel_speed_mps);
 chassis_ = std::make_unique<sim::DifferentialMotorDrive>(
   sim::MotorDriveParams{
     robot_definition_.wheel_base_m,
     robot_definition_.wheel_radius_m,
     robot_definition_.motor.max_wheel_speed_mps,
     std::min(robot_definition_.motor.max_wheel_accel_mps2, params.accel_max),
     robot_definition_.motor.stall_force_N,
     robot_definition_.motor.free_speed_mps,
     robot_definition_.mass_kg,
     robot_definition_.yaw_inertia_kgm2,
     robot_definition_.encoder.counts_per_revolution,
     robot_definition_.motor.pwm_limit,
     robot_definition_.motor.integral_limit,
   },
   wheel_gains);
 // Heading loop auto-tune: gain normalized by speed and wheelbase so the same
 // rule serves every DIY assembly. Kp = w_max / (2 * cruise * base) reproduces
 // the validated RO-1 feel; the integral/derivative ratios are method constants
 // (slow integrator kills steady-state error, short derivative damps).
 const double cruise = property(behavior_, "cruise_mps");
 const double heading_kp =
   params.w_max / (2.0 * std::max(0.2, cruise) * std::max(0.1, robot_definition_.wheel_base_m));
 heading_pid_ = std::make_unique<sim::PID>(heading_kp, heading_kp / 125.0, heading_kp * 0.05, params.w_max,
                                           robot_definition_.steering_integral_limit);

 patrols_.clear();
 patrol_ids_.clear();
 for (const auto& actor : map_definition_.actors) {
  if (!active_in_level(actor.active_in, level_id_) || actor.path.size() < 2) {
   continue;
  }
  patrols_.push_back({actor.path.front(), actor.path.back(), actor.speed_mps, actor.radius_m});
  patrol_ids_.push_back(actor.id);
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
 record_event("mission-start", "洛一开始自主执行任务。");
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
 manual_command_.target_w = std::clamp(turn, -1.0, 1.0) * drive_->params().w_max;
}

void MissionSimulation::step() {
 if (snapshot_.status != MissionStatus::Running) {
  return;
 }
 update_actors();
 if (sensor_ && snapshot_.time_s + 1e-9 >= next_scan_s_) {
  scan_environment();
  next_scan_s_ += 1.0 / property(sensor_->behavior, "scan_hz");
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

 const auto goal = as_vec(map_definition_.markers.at(level_.objective.goal));
 if (!level_.objective.via.empty() && !snapshot_.via_reached) {
  const auto via = as_vec(map_definition_.markers.at(level_.objective.via));
  if (distance({snapshot_.robot.x, snapshot_.robot.y}, via) <= level_.objective.via_radius_m) {
   snapshot_.via_reached = true;
   result_.via_reached = true;
   record_event("via-reached",
                level_.objective.via_label + "已取得，继续前往" + level_.objective.goal_label + "。");
  }
 }
 const double goal_tolerance = level_.objective.arrive_tolerance_m;
 if (distance({snapshot_.robot.x, snapshot_.robot.y}, goal) <= goal_tolerance &&
     (level_.objective.type != "observe_and_stop" || snapshot_.static_obstacle_detected) &&
     (level_.objective.via.empty() || snapshot_.via_reached)) {
  finish_success();
 }
 if (snapshot_.status == MissionStatus::Running && snapshot_.time_s >= level_.objective.time_limit_s) {
  finish_failure("timeout", "洛一未能在时限内抵达目标。");
 }
 if (snapshot_.time_s + 1e-9 >= next_replay_s_) {
  record_replay();
  next_replay_s_ += 0.05;
 }
}

void MissionSimulation::update_actors() {
 snapshot_.actors.clear();
 for (std::size_t index = 0; index < patrols_.size(); ++index) {
  actual_actors_[index] = sim::patrol_state(patrols_[index], snapshot_.time_s);
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
 if (!sensor_)
  return;
 const sim::Vec2 position{snapshot_.robot.x, snapshot_.robot.y};
 const int rays = static_cast<int>(property(sensor_->behavior, "rays"));
 const double range = property(sensor_->behavior, "range_m");
 const auto scan = sim::lidar_scan(*truth_map_, position, snapshot_.robot.theta, rays, range,
                                   property(sensor_->behavior, "noise_sigma_m"), rng_, actual_actors_);
 snapshot_.lidar_hits.clear();
 std::vector<int> dynamic_hit_count(actual_actors_.size(), 0);
 const bool first_static_detection = !snapshot_.static_obstacle_detected;
 for (int index = 0; index < scan.n_rays; ++index) {
  if (!scan.hit[index]) {
   continue;
  }
  if (level_.objective.type == "observe_and_stop" && scan.ranges[index] <= std::min(range, 2.5)) {
   snapshot_.static_obstacle_detected = true;
  }
  const double angle = snapshot_.robot.theta + (2.0 * sim::kPi * index / scan.n_rays);
  snapshot_.lidar_hits.push_back({
    position.x + std::cos(angle) * scan.ranges[index],
    position.y + std::sin(angle) * scan.ranges[index],
  });
  const int dynamic_index = scan.dynamic_hit[index];
  if (dynamic_index >= 0 && static_cast<std::size_t>(dynamic_index) < actual_actors_.size()) {
   ++dynamic_hit_count[dynamic_index];
  }
 }

 if (first_static_detection && snapshot_.static_obstacle_detected &&
     level_.objective.type == "observe_and_stop") {
  record_event("obstacle-detected", "距离雷达发现玻璃门，洛一开始控制停车距离。");
 }

 bool first_detection = !snapshot_.dynamic_object_detected;
 for (std::size_t index = 0; index < actual_actors_.size(); ++index) {
  if (dynamic_hit_count[index] > 0) {
   // The ray association above represents clustering. The object detector emits a
   // noisy center estimate; planners never receive the actor's route or velocity.
   const double center_noise = property(sensor_->behavior, "noise_sigma_m") /
                               (3.0 * std::sqrt(static_cast<double>(dynamic_hit_count[index])));
   const sim::Vec2 measured_center{
     actual_actors_[index].p.x + perception_rng_.gauss(center_noise),
     actual_actors_[index].p.y + perception_rng_.gauss(center_noise),
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
  record_event("dynamic-detected", "距离雷达发现巡逻叉车，洛一开始预测它的运动。");
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
  held_command_ = {0.0, 0.0};
  return;
 }
 const auto target = current_navigation_target();
 const bool fused_localization = level_.localization_mode == "fused";
 const double nav_x = fused_localization ? snapshot_.robot.x : odometry_.p.x;
 const double nav_y = fused_localization ? snapshot_.robot.y : odometry_.p.y;
 const double nav_heading = fused_localization ? snapshot_.robot.theta : odometry_.th;
 const auto route_control = sim::guide_step(
   planned_path_, guide_state_, nav_x, nav_y, nav_heading, *heading_pid_, property(behavior_, "lookahead_m"),
   property(behavior_, "cruise_mps"), 1.0 / decision_hz(), *planning_map_);
 snapshot_.control.local_target = target;
 snapshot_.control.heading_error_rad = route_control.err;
 snapshot_.control.heading_pid_p = heading_pid_->p();
 snapshot_.control.heading_pid_i = heading_pid_->i();
 snapshot_.control.heading_pid_d = heading_pid_->d();
 if (actual_actors_.empty()) {
  held_command_ = {route_control.tv, route_control.steer};
  snapshot_.waiting_for_clearance = false;
  return;
 }

 const auto known_tracks = dynamic_obstacle_map_.active_tracks(snapshot_.time_s, track_max_age_s());

 double nearest_actor = std::numeric_limits<double>::infinity();
 for (const auto& track : known_tracks) {
  const auto& actor = track.state;
  nearest_actor = std::min(nearest_actor, distance({snapshot_.robot.x, snapshot_.robot.y}, actor.p) -
                                            actor.radius - sim::kRobotRadius);
 }
 const double prediction_horizon = computer_ ? property(computer_->behavior, "prediction_horizon_s") : 1.2;
 const double requested_clearance = property(behavior_, "safe_clearance_m");
 const double risk_distance =
   requested_clearance * requested_clearance * 2.5 + property(behavior_, "cruise_mps") * 0.6 + 0.1;
 if (known_tracks.empty() || nearest_actor > risk_distance) {
  held_command_ = {route_control.tv, route_control.steer};
  snapshot_.waiting_for_clearance = false;
  return;
 }

 const double heading_error =
   sim::heading_err(target, snapshot_.robot.x, snapshot_.robot.y, snapshot_.robot.theta);
 const double speed_cap = std::max(0.20, sim::sched_v(heading_error, property(behavior_, "cruise_mps")));
 const sim::DwaWeights weights{1.0, 1.3 + requested_clearance * 1.8, 0.25};
 constexpr double prediction_step_s = 0.1;
 const auto predicted_frames =
   dynamic_obstacle_map_.predict(snapshot_.time_s, prediction_horizon, prediction_step_s, track_max_age_s());
 const auto dwa = sim::dwa_compute_trajectory(
   {snapshot_.robot.x, snapshot_.robot.y}, snapshot_.robot.theta, snapshot_.robot.v, snapshot_.robot.w,
   target, *planning_map_, weights, predicted_frames, speed_cap, drive_->params().w_max,
   property(tires_.behavior, "brake_decel_mps2"), prediction_horizon, prediction_step_s, prediction_step_s,
   requested_clearance);
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
   record_event("waiting", "预测路径发生冲突，洛一停车等待安全窗口。");
  }
  snapshot_.waiting_for_clearance = true;
 } else {
  // The global route remains responsible for cornering around shelves. DWA
  // contributes the safe speed and a local avoidance correction; allowing it
  // to replace the route heading entirely can leave a differential drive
  // spinning at a concave corner while a patrol is nearby.
  const double blended_turn =
    std::abs(dwa.best.v) < 0.15
      ? std::clamp(2.0 * heading_error, -drive_->params().w_max, drive_->params().w_max)
      : dwa.best.w;
  held_command_ = {dwa.best.v < 0.0 ? dwa.best.v : std::min(std::max(0.0, route_control.tv), dwa.best.v),
                   blended_turn};
  snapshot_.waiting_for_clearance = std::abs(dwa.best.v) < 0.15;
  if (snapshot_.waiting_for_clearance) {
   record_event("waiting", "预测路径发生冲突，洛一停车等待安全窗口。", 2.0);
  }
 }
}

void MissionSimulation::apply_motion() {
 snapshot_.control.target_v_mps = held_command_.target_v;
 snapshot_.control.target_w_radps = held_command_.target_w;
 const auto before = snapshot_.robot;
 auto candidate = chassis_->step(before, held_command_, fixed_step_s, wheel_ground({before.x, before.y}));
 const auto& motors = chassis_->telemetry();
 snapshot_.control.left_target_mps = motors.left.target_mps;
 snapshot_.control.right_target_mps = motors.right.target_mps;
 snapshot_.control.left_measured_mps = motors.left.measured_mps;
 snapshot_.control.right_measured_mps = motors.right.measured_mps;
 snapshot_.control.left_pwm = motors.left.pwm;
 snapshot_.control.right_pwm = motors.right.pwm;
 snapshot_.control.left_speed_pid_p = motors.left.pid_p;
 snapshot_.control.left_speed_pid_i = motors.left.pid_i;
 snapshot_.control.left_speed_pid_d = motors.left.pid_d;
 snapshot_.control.right_speed_pid_p = motors.right.pid_p;
 snapshot_.control.right_speed_pid_i = motors.right.pid_i;
 snapshot_.control.right_speed_pid_d = motors.right.pid_d;
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
  const double clearance = distance(candidate_position, actor.p) - actor.radius - sim::kRobotRadius;
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
 const double total_power = motor_power * property(tires_.behavior, "power_scale") +
                            (sensor_ ? property(sensor_->behavior, "power_w") : 0.0) +
                            (computer_ ? property(computer_->behavior, "power_w") : 0.0);
 result_.energy_wh += total_power * fixed_step_s / 3600.0;

 double slip = property(tires_.behavior, "slip_scale");
 const double traction = surface_traction(candidate_position);
 if (traction < 1.0) {
  slip /= traction;
 }
 if (motors.left.slipping || motors.right.slipping) {
  slip *= 3.0;
 }
 double encoder_v = 0.0;
 double encoder_w = 0.0;
 sim::DiffDrive::wheelsToTwist(motors.left.measured_mps, motors.right.measured_mps,
                               robot_definition_.wheel_base_m, encoder_v, encoder_w);
 odometry_.update(encoder_v, encoder_w, fixed_step_s, rng_, slip, 0.003 * slip);
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
  if (inside(position, surface)) {
   return surface.traction_scale;
  }
 }
 return 1.0;
}

double MissionSimulation::static_clearance(sim::Vec2 position) const {
 double minimum = std::min({position.x, position.y, map_definition_.size_m.x - position.x,
                            map_definition_.size_m.y - position.y}) -
                  sim::kRobotRadius;
 for (const auto& obstacle : map_definition_.obstacles) {
  if (!active_in_level(obstacle.active_in, level_id_)) {
   continue;
  }
  const double dx = std::max({obstacle.min.x - position.x, 0.0, position.x - obstacle.max.x});
  const double dy = std::max({obstacle.min.y - position.y, 0.0, position.y - obstacle.max.y});
  minimum = std::min(minimum, std::hypot(dx, dy) - sim::kRobotRadius);
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

void MissionSimulation::record_replay() {
 MissionSnapshot frame = snapshot_;
 // Candidate fans are useful live but would dominate a 20 Hz replay. The chosen
 // local path and all control values remain available for deterministic review.
 frame.control.dwa_candidates.clear();
 replay_.push_back(std::move(frame));
}

double MissionSimulation::decision_hz() const {
 return computer_ ? property(computer_->behavior, "decision_hz") : kBuiltInDecisionHz;
}

double MissionSimulation::track_max_age_s() const {
 // Dynamic-track memory lives on the computer; computer-less levels use the
 // documented engine default.
 return computer_ ? property(computer_->behavior, "track_max_age_s") : kDynamicTrackMaxAgeS;
}

sim::GroundContact MissionSimulation::wheel_ground(sim::Vec2 position) const {
 // Traction is always tire tread combined with ground material: neither alone
 // decides grip. mu_ref anchors the product so concrete + standard behaves as
 // the long-validated reference.
 constexpr double kMuReference = 0.8;
 const double tire_mu = property(tires_.behavior, "friction_mu");
 const double tire_rr = property(tires_.behavior, "rolling_resistance");
 double ground_mu = 1.0;
 double ground_rr = 0.0;
 double traction = 1.0;
 for (const auto& surface : map_definition_.surfaces) {
  if (inside(position, surface)) {
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
                                     property(tires_.behavior, "brake_decel_mps2") * (0.5 + 0.5 * traction));
 return contact;
}

} // namespace game
