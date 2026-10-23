#include "robot/autonomy/coordinator.hpp"

#include <cmath>
#include <limits>
#include <string>

namespace lost_signal::robot::autonomy {

namespace {

// Planar yaw from a quaternion (REP-103 R = Rz*Ry*Rx): the TF mount may
// carry roll/pitch, but planar consumers use the yaw projection only.
double yaw_from_quaternion(const core::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

}  // namespace

AutonomyCoordinator::AutonomyCoordinator(platform::ChassisLimits limits,
                                         core::MonotonicNs decision_period_ns,
                                         LidarPerceptionConfig perception_config,
                                         GripAdapterConfig grip_config)
    : limits_(limits),
      grip_(grip_config),
      decision_period_ns_(decision_period_ns),
      estimator_(std::make_unique<FusedOdometryEstimator>()),
      perception_module_(std::make_unique<LidarPerception>(perception_config)),
      mapper_(std::make_unique<RayClearingMapper>()),
      behavior_(std::make_unique<WaitOnIntrusionBehavior>()),
      global_planner_(std::make_unique<AStarPlanner>()),
      local_planner_(std::make_unique<DwaPlanner>()),
      validator_(std::make_unique<ClearanceValidator>(
          ClearanceValidatorConfig{limits.footprint_radius_m})),
      controller_(std::make_unique<DirectController>()),
      safety_(std::make_unique<SafetyMonitor>()) {
  // The traversal belief prices roughness for route choice for the life of
  // the coordinator (the model object is a member: rebinds never dangle).
  global_planner_->set_traversal(&traversal_);
}

core::Status AutonomyCoordinator::configure() {
  estimator_->reset();
  perception_module_->reset();
  mapper_->reset();
  behavior_->reset();
  local_planner_->reset();
  controller_->reset();
  safety_->reset();
  has_observation_ = false;
  last_obs_ = iface::ObservationFrame{};
  perception_ = PerceptionResult{};
  decision_ = BehaviorDecision{};
  active_route_ = Route{};
  supervisor_.reset(false);
  slam_.reset();
  slam_origin_seeded_ = false;
  last_localization_action_ = "init";
  last_decision_ns_ = core::kInvalidTime;
  last_map_revision_ = 0;
  replan_requested_ = false;
  last_cmd_ = iface::MotionCommand::SafeStop(0);
  last_constraint_ = "none";
  last_reason_ = "init";
  latch_.reset();
  last_published_stop_ = false;
  grip_.reset();
  traversal_ = TraversalModel{};
  last_traverse_ns_ = core::kInvalidTime;
  lidar_mount_z_m_ = 0.35;
  evidence_.reset();
  state_ = lifecycle::ModuleState::kInactive;
  return core::Status::Ok();
}

core::Status AutonomyCoordinator::activate() {
  if (state_ != lifecycle::ModuleState::kInactive) {
    return core::Status::Fail("autonomy: activate requires Inactive");
  }
  state_ = lifecycle::ModuleState::kActive;
  return core::Status::Ok();
}

core::Status AutonomyCoordinator::deactivate() {
  if (state_ == lifecycle::ModuleState::kActive) {
    state_ = lifecycle::ModuleState::kInactive;
  }
  return core::Status::Ok();
}

core::Status AutonomyCoordinator::cleanup() {
  estimator_->reset();
  perception_module_->reset();
  mapper_->reset();
  behavior_->reset();
  local_planner_->reset();
  controller_->reset();
  safety_->reset();
  has_observation_ = false;
  last_obs_ = iface::ObservationFrame{};
  perception_ = PerceptionResult{};
  decision_ = BehaviorDecision{};
  active_route_ = Route{};
  supervisor_.reset(false);
  slam_.reset();
  slam_origin_seeded_ = false;
  last_localization_action_ = "init";
  last_decision_ns_ = core::kInvalidTime;
  last_map_revision_ = 0;
  replan_requested_ = false;
  last_cmd_ = iface::MotionCommand::SafeStop(0);
  last_constraint_ = "none";
  last_reason_ = "init";
  latch_.reset();
  last_published_stop_ = false;
  grip_.reset();
  traversal_ = TraversalModel{};
  last_traverse_ns_ = core::kInvalidTime;
  lidar_mount_z_m_ = 0.35;
  evidence_.reset();
  state_ = lifecycle::ModuleState::kUnconfigured;
  return core::Status::Ok();
}

void AutonomyCoordinator::set_static_frames(
    const core::FrameTransformTree& tree) {
  tf_ = tree;
  // Fan out the full lidar mount to every consumer. Unknown mount =
  // identity (aligned), never a stale copy of a previous description.
  // Validator/safety keep the yaw projection (forward-sector azimuth);
  // the deproject path takes the whole transform so slopes tilt the
  // scan plane instead of silently staying flat.
  core::RigidTransform base_from_lidar{{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}};
  if (const auto hit = tf_.lookup("base_link", "lidar_link")) {
    base_from_lidar = *hit;
  }
  if (perception_module_ != nullptr) {
    perception_module_->set_lidar_mount(base_from_lidar);
  }
  if (mapper_ != nullptr) mapper_->set_lidar_mount(base_from_lidar);
  lidar_mount_z_m_ = base_from_lidar.translation_m.z;
  scan_matcher_.set_lidar_mount(base_from_lidar);
  global_search_.set_lidar_mount(base_from_lidar);
  slam_.set_lidar_mount(base_from_lidar);
  const double yaw = yaw_from_quaternion(base_from_lidar.rotation);
  if (validator_ != nullptr) validator_->set_lidar_yaw(yaw);
  if (safety_ != nullptr) safety_->set_lidar_yaw(yaw);
}

void AutonomyCoordinator::set_goal(const RoutePoint& goal) {
  goal_ = goal;
  has_goal_ = true;
  active_route_ = Route{};  // force refresh on the next update
  replan_requested_ = true;
}

void AutonomyCoordinator::clear_goal() {
  goal_ = RoutePoint{};
  has_goal_ = false;
  active_route_ = Route{};
  replan_requested_ = false;
}

void AutonomyCoordinator::seed_pose(double x_m, double y_m, double yaw_rad) {
  const core::MonotonicNs stamp =
      has_observation_ ? last_obs_.stamp_ns : estimate().stamp_ns;
  estimator_->seed_pose(x_m, y_m, yaw_rad, 0.1, 0.05, stamp);
  supervisor_.reset(true);
}

bool AutonomyCoordinator::route_complete(double arrival_radius_m) const {
  if (!has_goal_ || !active_route_.valid || active_route_.points.empty()) {
    return false;
  }
  const EstimatedState& est = estimator_->estimate();
  if (!est.valid) return false;
  const double dx = est.x_m - goal_.x_m;
  const double dy = est.y_m - goal_.y_m;
  return std::hypot(dx, dy) <= arrival_radius_m;
}

void AutonomyCoordinator::ingest_observation(
    const iface::ObservationFrame& obs) {
  if (state_ != lifecycle::ModuleState::kActive) return;
  if (!iface::valid_observation_frame(obs)) return;
  // Fail-closed: stale or repeated input never advances state.
  if (has_observation_ && obs.sequence <= last_obs_.sequence) return;
  if (has_observation_ && obs.stamp_ns <= last_obs_.stamp_ns) return;
  // IMU arrives in its device frame (imu_link by message contract): rotate
  // into base_link through the static TF before the estimator integrates
  // it as body rates. Unknown mount = identity, i.e. aligned behavior.
  iface::ObservationFrame shaped = obs;
  if (shaped.imu_valid) {
    if (const auto base_from_imu = tf_.lookup("base_link", "imu_link")) {
      const core::Vec3 gyro =
          core::rotate_vector(base_from_imu->rotation,
                              {shaped.imu_gyro_x_radps, shaped.imu_gyro_y_radps,
                               shaped.imu_gyro_z_radps});
      const core::Vec3 accel =
          core::rotate_vector(base_from_imu->rotation,
                              {shaped.imu_ax_mps2, shaped.imu_ay_mps2,
                               shaped.imu_az_mps2});
      shaped.imu_gyro_x_radps = gyro.x;
      shaped.imu_gyro_y_radps = gyro.y;
      shaped.imu_gyro_z_radps = gyro.z;
      shaped.imu_ax_mps2 = accel.x;
      shaped.imu_ay_mps2 = accel.y;
      shaped.imu_az_mps2 = accel.z;
    }
  }
  // Estimator runs on every observation (prediction rate = sensor rate).
  estimator_->predict(shaped);
  estimator_->correct(shaped);
  // Perception + mapping run at scan rate, not on every odometry frame.
  // They see the shaped frame (base-frame IMU); scans pass through
  // untouched.
  if (shaped.scan_valid) {
    perception_ = perception_module_->update(shaped, estimator_->estimate());
    mapper_->update(shaped, estimator_->estimate());
  }
  last_obs_ = shaped;
  has_observation_ = true;
}

iface::MotionCommand AutonomyCoordinator::update(
    iface::MapSnapshot map, const iface::ChannelHealth& channel,
    core::MonotonicNs now_ns) {
  // Missing observation is unknown evidence -> safe stop, never free space.
  if (state_ != lifecycle::ModuleState::kActive || !has_observation_) {
    return iface::MotionCommand::SafeStop(now_ns);
  }
  // Decision branch runs at the decision period, or on event wakeup:
  // explicit replan request, or a new map revision. Everything else holds
  // the last validated command while safety keeps gating every tick.
  bool due = last_decision_ns_ == core::kInvalidTime ||
             now_ns - last_decision_ns_ >= decision_period_ns_;
  bool map_changed = map.revision != last_map_revision_;
  // Localization runs on every tick; the supervisor cadence-gates the
  // matcher. Corrections feed the estimator only when accepted, so a hard
  // scan never drags the pose.
  const EstimatedState pre = estimator_->estimate();
  run_localization(map, now_ns);
  // SLAM session runs on the post-localization estimate: keyframes chain
  // odometry, accepted loop closures correct the estimate and rebuild the
  // rolling map from keyframes at their optimized poses.
  run_slam(now_ns);
  // TF publisher: odom->base is the continuous pre-correction prediction
  // in full 3D (x/y/z + roll/pitch/yaw — slopes ride here); map->odom is
  // the per-tick SE(2) correction delta (identity when nothing corrected).
  // Skipped until the estimate is valid.
  const EstimatedState post = estimator_->estimate();
  if (post.valid) {
    const EstimatedState& odom = pre.valid ? pre : post;
    tf_.set_transform(
        "odom", "base_link",
        {{odom.x_m, odom.y_m, odom.z_m},
         core::quaternion_from_rpy(odom.roll_rad, odom.pitch_rad,
                                    odom.yaw_rad)});
    tf_.set_transform("map", "odom",
                      core::map_from_odom_delta(odom.x_m, odom.y_m,
                                               odom.yaw_rad, post.x_m,
                                               post.y_m, post.yaw_rad));
  }
  // Grip governor (control layer) runs every tick, not only on
  // decisions: the speed envelope must tighten within a second of slip
  // evidence, not at the next 10 Hz boundary. The coordinator only wires
  // measured evidence in and consumes the tightened envelope below.
  // Survey grade along the estimated heading feeds the governor's gravity
  // compensation (pure transform over the published prior, owned by the
  // mapping layer — the coordinator holds no adaptation policy).
  double survey_pitch = std::numeric_limits<double>::quiet_NaN();
  const EstimatedState& est_for_grade = estimator_->estimate();
  if (est_for_grade.valid) {
    survey_pitch = survey_pitch_along(map, est_for_grade.x_m,
                                      est_for_grade.y_m,
                                      est_for_grade.yaw_rad);
  }
  grip_.update(now_ns, last_obs_, estimator_->estimate(),
               estimator_->last_slip(), survey_pitch);
  // Traversal belief (mapping layer): roughness from IMU motion evidence
  // over the surveyed prior. Cadence from the tick clock; the observer
  // skips internally without IMU or a valid estimate.
  if (last_obs_.imu_valid) {
    if (last_traverse_ns_ != core::kInvalidTime && now_ns > last_traverse_ns_) {
      const double dt_s = std::min(
          1.0, static_cast<double>(now_ns - last_traverse_ns_) * 1e-9);
      traversal_.observe_motion(estimator_->estimate(),
                                last_obs_.imu_az_mps2,
                                last_obs_.stamp_ns * 1e-9, dt_s);
    }
    last_traverse_ns_ = now_ns;
  }
  // Surveyed-elevation context for the terrain-aware layers (prior, never
  // live physics): validator ground-classification + grade gates. The map
  // outlives the decision branch (by-value parameter), so the pointer is
  // valid for both validate() calls below.
  SurveyContext survey_ctx;
  survey_ctx.map = &map;
  survey_ctx.sensor_height_m = lidar_mount_z_m_;
  if (due || replan_requested_ || map_changed) {
    // The decision branch ran: stamp it first so cadence, map tracking, and
    // event flags stay consistent even when a stage below yields no plan.
    last_decision_ns_ = now_ns;
    last_map_revision_ = map.revision;
    replan_requested_ = false;
    // Survey revision: rebind the traversal belief and push the prior to
    // the mapper (ground classification for occupancy painting).
    if (map_changed) {
      traversal_.bind(map);
      if (mapper_ != nullptr) mapper_->set_survey(map);
    }
    // Route refresh first: a new goal, a new map revision, or no route yet
    // re-runs the global planner from the current estimate. The local
    // planner only ever consumes this active route, never a raw goal.
    if (has_goal_ &&
        (!active_route_.valid || map_changed)) {
      active_route_ =
          global_planner_->plan(estimator_->estimate(), goal_, map);
    }
    // Dynamic decision first: behavior turns tracks + estimate into one
    // constraint. A hold short-circuits sampling but still passes the
    // independent validator + controller, so the evidence chain is uniform.
    // Every path below funnels into `decided`; nothing returns early, so
    // the safety gate at the end runs on every decision without exception.
    decision_ =
        behavior_->decide(estimator_->estimate(), perception_, limits_);
    iface::MotionCommand decided =
        iface::MotionCommand::SafeStop(now_ns);
    std::string reason = "no-plan";
    std::string constraint =
        decision_.must_hold ? "hold" : "speed_cap";
    if (decision_.must_hold) {
      LocalCandidate hold;
      hold.feasible = true;
      ValidationResult verdict =
          validator_->validate(hold, estimator_->estimate(), perception_,
                               last_obs_, survey_ctx);
      reason = verdict.reason;
      if (verdict.accepted) {
        decided = controller_->track(hold, estimator_->estimate(), now_ns);
      }
    } else {
      // Grip-tightened envelope from the control-layer governor: the
      // planner samples within the adapted speed cap. Behavior still
      // decides hold/cap on full limits; grip can only tighten further,
      // never override a hold or widen anything.
      platform::ChassisLimits effective =
          grip_.effective_limits(limits_, decision_.max_forward_mps);
      LocalPlan plan = local_planner_->plan(
          estimator_->estimate(), perception_, mapper_->local_map(),
          active_route_, effective);
      if (plan.has_plan) {
        ValidationResult verdict = validator_->validate(
            plan.best, estimator_->estimate(), perception_, last_obs_,
            survey_ctx);
        reason = verdict.reason;
        if (verdict.accepted) {
          decided =
              controller_->track(plan.best, estimator_->estimate(), now_ns);
        }
      }
    }
    if (!iface::valid_motion_command(decided, now_ns)) {
      decided = iface::MotionCommand::SafeStop(now_ns);
    }
    last_cmd_ = decided;
    last_constraint_ = constraint;
    last_reason_ = reason;
  }
  // Controlled-stop latch: once stopping, withhold motion until the measured
  // tread settles, so a safety rejection on a grade cannot chatter into a
  // brake/accelerate limit cycle. Independent of planner yaw intent.
  const double tread_mps = std::abs(estimator_->estimate().forward_mps);
  iface::MotionCommand gated = last_cmd_;
  if (latch_.update(last_published_stop_, tread_mps,
                    ControlledStopLatch::Config{})) {
    gated = iface::MotionCommand::SafeStop(now_ns);
  }
  // Final independent gate, every tick: safety re-checks RAW evidence
  // (freshness, near-field scan, channel health) and can only downgrade to
  // safe stop. Critical faults latch until deliberate recovery.
  SafetyVerdict verdict = safety_->evaluate(estimator_->estimate(), perception_,
                                            last_obs_, channel, now_ns);
  iface::MotionCommand final_cmd = safety_->gate(gated, verdict, now_ns);
  last_published_stop_ = final_cmd.is_safe_stop();
  // One evidence record per distinct decision; repeats update in place.
  diagnostics::DecisionEvidence evidence;
  evidence.observation_time_s = last_obs_.stamp_ns * 1e-9;
  evidence.observation_refs = {last_obs_.source + "#" +
                               std::to_string(last_obs_.sequence)};
  evidence.prediction_horizon_s = decision_period_ns_ * 1e-9;
  evidence.constraint_code = last_constraint_;
  evidence.reason_code = last_reason_;
  if (verdict.state != SafetyState::kNominal) {
    evidence.reason_code += "+safety:" + verdict.reason;
  }
  evidence.target_speed_mps = final_cmd.forward_mps;
  evidence.target_yaw_rate_radps = final_cmd.yaw_rate_radps;
  evidence.action_code = final_cmd.is_safe_stop() ? "stop" : "drive";
  evidence_.publish(std::move(evidence), now_ns * 1e-9);
  return final_cmd;
}

const EstimatedState& AutonomyCoordinator::estimate() const {
  return estimator_->estimate();
}

void AutonomyCoordinator::run_slam(core::MonotonicNs now_ns) {
  (void)now_ns;
  if (!has_observation_ || !last_obs_.scan_valid) return;
  slam_.ingest(estimator_->estimate(), last_obs_);
  SlamLoopResult loop =
      slam_.try_close_loops(estimator_->estimate(), last_obs_);
  if (!loop.closed) return;
  // Accepted closure: snap the estimate to the optimized pose and rebuild
  // the rolling map from keyframes at their optimized poses. Past cells
  // fed at drifted poses are stale; replaying keyframe scans re-registers
  // them. The decision branch refreshes on the corrected estimate.
  estimator_->seed_pose(loop.corrected_pose.position.x,
                        loop.corrected_pose.position.y,
                        loop.corrected_pose.heading_rad, 0.05, 0.02,
                        last_obs_.stamp_ns);
  mapper_->reset();
  for (const auto& kf : slam_.keyframes()) {
    iface::ObservationFrame obs;
    obs.sequence = 1;  // rebuild order is keyframe order, not live sequence
    obs.stamp_ns = static_cast<core::MonotonicNs>(kf.stamp_s * 1e9);
    obs.source = last_obs_.source;
    obs.frame = core::Frame::kLidarLink;
    obs.scan_valid = true;
    obs.ranges_m = kf.ranges_m;
    obs.static_ranges_m = kf.static_ranges_m;
    obs.scan_static_valid = !kf.static_ranges_m.empty();
    obs.scan_angle_min_rad = kf.angle_min_rad;
    obs.scan_angle_increment_rad = kf.angle_increment_rad;
    EstimatedState est;
    est.valid = true;
    est.stamp_ns = obs.stamp_ns;
    est.x_m = kf.pose.position.x;
    est.y_m = kf.pose.position.y;
    est.yaw_rad = kf.pose.heading_rad;
    mapper_->update(obs, est);
  }
  replan_requested_ = true;
}

void AutonomyCoordinator::run_localization(const iface::MapSnapshot& map,
                                          core::MonotonicNs now_ns) {
  const double now_s = now_ns * 1e-9;
  const EstimatedState& est = estimator_->estimate();
  LocalizationHealth health;
  health.valid = est.valid;
  health.position_sigma_m = est.position_sigma_m;
  health.heading_sigma_rad = est.heading_sigma_rad;
  health.confidence = est.confidence;
  health.age_s = est.valid ? now_s - est.stamp_ns * 1e-9 : 1e9;
  std::string reason;
  LocalizationAction action = supervisor_.next(
      now_s, est.x_m, est.y_m, est.yaw_rad, health, &reason);
  last_localization_action_ = localization_action_name(action);
  if (action == LocalizationAction::kDefer) return;
  const bool attempted_global = action == LocalizationAction::kGlobalRelocalization;
  // Mapless startup: no static raster exists, so neither local matching nor
  // the whole-map search can constrain the pose. Seed the odometry origin
  // once (no teleport: the current estimate IS the origin) and let the SLAM
  // session own the drift it can observe. Latched for the session.
  if (attempted_global && !map.has_raster() && !slam_origin_seeded_) {
    const StartupDecision startup =
        select_startup_mode(false, /*has_prior_pose=*/false);
    estimator_->seed_pose(est.x_m, est.y_m, est.yaw_rad, 0.1, 0.05,
                          last_obs_.stamp_ns);
    supervisor_.reset(true);
    slam_origin_seeded_ = true;
    last_localization_action_ = startup_mode_name(startup.mode);
    supervisor_.record_global(true, ScanMatchResult{});
    return;
  }
  if (attempted_global) {
    // Whole-map search: kidnap recovery and cold start. An accepted global
    // fix establishes the estimate and marks the supervisor localized; a
    // rejection is evidence only and never moves the estimate.
    GlobalSearchResult global = global_search_.search(
        last_obs_.ranges_m, last_obs_.scan_angle_min_rad,
        last_obs_.scan_angle_increment_rad, map, now_ns, est.roll_rad,
        est.pitch_rad, est.z_m);
    if (global.accepted) {
      if (!supervisor_.localized()) {
        // No trustworthy prior exists (unknown start / kidnapped): snap
        // like a deliberate initial pose instead of blending toward a
        // confident-but-wrong dead-reckoned origin.
        estimator_->seed_pose(global.x_m, global.y_m, global.yaw_rad,
                              global.position_sigma_m,
                              global.heading_sigma_rad, last_obs_.stamp_ns);
      } else {
        estimator_->correct_pose(global.x_m, global.y_m, global.yaw_rad,
                                 global.position_sigma_m,
                                 global.heading_sigma_rad,
                                 last_obs_.stamp_ns);
      }
    }
    supervisor_.record_global(global.accepted, ScanMatchResult{});
    return;
  }
  ScanMatchResult match = scan_matcher_.localize(
      last_obs_.ranges_m, last_obs_.scan_angle_min_rad,
      last_obs_.scan_angle_increment_rad, est.x_m, est.y_m, est.yaw_rad, map,
      now_ns, est.roll_rad, est.pitch_rad, est.z_m);
  if (match.accepted) {
    estimator_->correct_pose(match.x_m, match.y_m, match.yaw_rad,
                             match.position_sigma_m, match.heading_sigma_rad,
                             last_obs_.stamp_ns);
  }
  supervisor_.record_local(match.accepted, match);
}

LocalMap AutonomyCoordinator::local_map() const {
  return mapper_->local_map();
}

}  // namespace lost_signal::robot::autonomy
