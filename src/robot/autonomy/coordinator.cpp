#include "robot/autonomy/coordinator.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <string>

#include "robot/autonomy/decision/autonomy_pipeline.hpp"
#include "robot/autonomy/decision/terrain_pipeline.hpp"

namespace lost_signal::robot::autonomy {

namespace {

SafetyConfig safety_config_for(
 const platform::ChassisLimits& limits,
 const platform::NavigationSafetyPolicy& navigation,
 const platform::MotionDynamicsPolicy& motion) {
 SafetyConfig config;
 if (std::isfinite(limits.footprint_radius_m) &&
 limits.footprint_radius_m > 0.0) {
 config.footprint_radius_m = limits.footprint_radius_m;
 }
 config.static_margin_m = navigation.clearance_margin_m;
 config.brake_decel_mps2 = motion.brake_deceleration_mps2;
 config.command_reaction_time_s = motion.safety_reaction_time_s;
 return config;
}

ClearanceValidatorConfig validator_config_for(
 const platform::ChassisLimits& limits,
 const platform::NavigationSafetyPolicy& navigation,
 const platform::MotionDynamicsPolicy& motion) {
 ClearanceValidatorConfig config;
 config.footprint_radius_m = limits.footprint_radius_m;
 config.extra_margin_m = navigation.clearance_margin_m;
 config.brake_decel_mps2 = motion.brake_deceleration_mps2;
 return config;
}

RouteConflictBehaviorConfig behavior_config_for(
 const platform::NavigationSafetyPolicy& navigation,
 const platform::MotionDynamicsPolicy& motion) {
 RouteConflictBehaviorConfig config;
 config.extra_clearance_m = navigation.clearance_margin_m;
 config.brake_deceleration_mps2 = motion.brake_deceleration_mps2;
 return config;
}

} // namespace

AutonomyCoordinator::AutonomyCoordinator(platform::ChassisLimits limits,
                                         core::MonotonicNs decision_period_ns,
                                         LidarPerceptionConfig perception_config,
 GripAdapterConfig grip_config,
 double wheel_base_m,
 GoalArrivalConfig arrival_config,
 std::string local_planner_kind,
 platform::NavigationSafetyPolicy navigation_policy,
 platform::MotionDynamicsPolicy motion_policy)
    : limits_(limits),
      grip_(grip_config),
 goal_arrival_(arrival_config),
      decision_period_ns_(decision_period_ns),
      estimator_(std::make_unique<FusedOdometryEstimator>()),
      perception_module_(std::make_unique<LidarPerception>(perception_config)),
      mapper_(std::make_unique<RayClearingMapper>()),
 behavior_(std::make_unique<WaitOnIntrusionBehavior>(
 behavior_config_for(navigation_policy, motion_policy))),
 global_planner_(make_global_planner(limits, wheel_base_m,
 navigation_policy)),
 local_planner_(make_local_planner(local_planner_kind,
 navigation_policy, motion_policy)),
      validator_(std::make_unique<ClearanceValidator>(
 validator_config_for(limits, navigation_policy, motion_policy))),
      controller_(std::make_unique<DirectController>()),
 safety_(std::make_unique<SafetyMonitor>(
 safety_config_for(limits, navigation_policy, motion_policy))) {
 const double decision_period_s =
 std::max(1e-9, static_cast<double>(decision_period_ns_) * 1e-9);
 const double tracking_margin_m =
 std::max(0.0, navigation_policy.clearance_margin_m);
 trajectory_tracking_limits_.position_m = tracking_margin_m;
 trajectory_tracking_limits_.heading_rad = std::atan2(
 tracking_margin_m, std::max(1e-9, limits_.footprint_radius_m));
 trajectory_tracking_limits_.forward_speed_mps =
 tracking_margin_m / decision_period_s;
 trajectory_tracking_limits_.yaw_rate_radps =
 trajectory_tracking_limits_.heading_rad / decision_period_s;
  // The traversal belief prices roughness for route choice for the life of
  // the coordinator (the model object is a member: rebinds never dangle).
  global_planner_->set_traversal(&traversal_);
  // The same belief gates DWA rollout sampling on the surveyed grade
  // (master's StaticFeasibility doctrine): untraversable segments read as
  // collisions at plan time, the validator still owns the veto. Unknown
  // elevation reads feasible (optimistic sampling); the gate opens fully
  // before the first survey bind, legacy behavior exactly.
    if (local_planner_) local_planner_->set_terrain_gate(
        [this](double x0, double y0, double x1,
                                          double y1, double yaw) {
 if (!traversal_.terrain_gate_resolved(estimator_->estimate(),
 limits_.footprint_radius_m)) {
 return true;
 }
    return traversal_.feasible(x0, y0, x1, y1, yaw);
  });
}

core::Status AutonomyCoordinator::configure() {
    if (!local_planner_) {
        return core::Status::Fail("autonomy: unknown local planner");
    }
  estimator_->reset();
  perception_module_->reset();
  mapper_->reset();
  behavior_->reset();
  local_planner_->reset();
  controller_->reset();
  safety_->reset();
 decision_pipeline_.reset();
  has_observation_ = false;
  last_obs_ = iface::ObservationFrame{};
  perception_ = PerceptionResult{};
  decision_ = BehaviorDecision{};
  active_route_ = Route{};
 goal_arrival_.reset();
 goal_arrival_decision_ = GoalArrivalDecision{};
  localization_.reset();
  last_localization_action_ = "init";
  last_decision_ns_ = core::kInvalidTime;
  last_map_revision_ = 0;
  replan_requested_ = false;
  last_cmd_ = iface::MotionCommand::SafeStop(0);
  last_constraint_ = "none";
  last_reason_ = "init";
 last_prediction_horizon_s_ = 0.0;
 last_prediction_hypotheses_ = 0;
  latch_.reset();
  last_published_stop_ = false;
  grip_.reset();
  traversal_ = TraversalModel{};
  last_traverse_ns_ = core::kInvalidTime;
  lidar_mount_z_m_ = 0.35;
  evidence_.reset();
 trajectory_tracking_.reset();
 trajectory_tracking_report_ = {};
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
 decision_pipeline_.reset();
  has_observation_ = false;
  last_obs_ = iface::ObservationFrame{};
  perception_ = PerceptionResult{};
  decision_ = BehaviorDecision{};
  active_route_ = Route{};
  localization_.reset();
  last_localization_action_ = "init";
  last_decision_ns_ = core::kInvalidTime;
  last_map_revision_ = 0;
  replan_requested_ = false;
  last_cmd_ = iface::MotionCommand::SafeStop(0);
  last_constraint_ = "none";
  last_reason_ = "init";
 last_prediction_horizon_s_ = 0.0;
 last_prediction_hypotheses_ = 0;
  latch_.reset();
  last_published_stop_ = false;
  grip_.reset();
  traversal_ = TraversalModel{};
  last_traverse_ns_ = core::kInvalidTime;
  lidar_mount_z_m_ = 0.35;
  evidence_.reset();
 trajectory_tracking_.reset();
 trajectory_tracking_report_ = {};
  state_ = lifecycle::ModuleState::kUnconfigured;
  return core::Status::Ok();
}

void AutonomyCoordinator::set_plant_geometry(
    const platform::PlantGeometry& plant) {
  // Declared loadout only (machine file, never measured truth): support
  // width + COM re-resolve the validator's rollover envelope; the ride
  // height latches for the survey context below. Non-positive inputs are
  // ignored field-wise (validator-side rule), so the envelope stays
  // internally consistent — never half-applied.
  if (validator_ != nullptr) {
    validator_->set_plant_geometry(plant.track_width_m, plant.com_height_m);
  }
 if (plant.wheel_radius_m > 0.0) ride_height_m_ = plant.wheel_radius_m;
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
  localization_.set_lidar_mount(base_from_lidar);
 // Forward-sector gates use the same full rotation as endpoint
 // deprojection; roll/pitch-mounted lidars no longer take a yaw-only path.
 if (validator_ != nullptr) validator_->set_lidar_mount(base_from_lidar);
 if (safety_ != nullptr) safety_->set_lidar_mount(base_from_lidar);
}

void AutonomyCoordinator::set_goal(const RoutePoint& goal) {
 goal_arrival_.reset();
 goal_arrival_decision_ = GoalArrivalDecision{};
  goal_ = goal;
  has_goal_ = true;
  active_route_ = Route{};  // force refresh on the next update
  replan_requested_ = true;
}

void AutonomyCoordinator::clear_goal() {
 goal_arrival_.reset();
 goal_arrival_decision_ = GoalArrivalDecision{};
  goal_ = RoutePoint{};
  has_goal_ = false;
  active_route_ = Route{};
  replan_requested_ = false;
}

void AutonomyCoordinator::seed_pose(double x_m, double y_m, double yaw_rad,
 double position_sigma_m, double heading_sigma_rad) {
 if (!std::isfinite(x_m) || !std::isfinite(y_m) ||
 !std::isfinite(yaw_rad) || !(position_sigma_m > 0.0) ||
 !std::isfinite(position_sigma_m) || !(heading_sigma_rad > 0.0) ||
 !std::isfinite(heading_sigma_rad)) {
 return;
 }
  const core::MonotonicNs stamp =
      has_observation_ ? last_obs_.stamp_ns : estimate().stamp_ns;
 estimator_->seed_pose(x_m, y_m, yaw_rad, position_sigma_m,
 heading_sigma_rad, stamp);
  localization_.mark_localized();
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
 if (shaped.scan_valid) {
 // Normalize the legacy zero-increment form once at the ROBOT boundary so
 // scan matching/SLAM and direct beam consumers share one layout contract.
 shaped.scan_angle_increment_rad =
 iface::scan_angle_increment_rad(shaped);
 }
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
 mapper_->set_dynamic_tracks(perception_.tracks);
    mapper_->update(shaped, estimator_->estimate());
  }
  last_obs_ = shaped;
  has_observation_ = true;
}

iface::MotionCommand AutonomyCoordinator::update(
 const iface::MapSnapshot& map, const iface::ChannelHealth& channel,
    core::MonotonicNs now_ns) {
  // Missing observation is unknown evidence -> safe stop, never free space.
  if (state_ != lifecycle::ModuleState::kActive || !has_observation_) {
    return iface::MotionCommand::SafeStop(now_ns);
  }
 // Publish the latest ROBOT map prior to perception for the next scan. The
 // perception module raycasts only known occupancy; unknown cells remain a
 // scan-history problem. This is map/SLAM evidence, never simulator truth.
 perception_module_->set_static_map(map);
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
  // Localization subsystem (supervisor-sequenced corrections + SLAM):
  // estimator/mapper/observations in, action name + replan flag out.
  // Corrections feed the estimator only when accepted, so a hard scan
  // never drags the pose.
  LocalizationSystem::TickResult loc = localization_.update(
      map, last_obs_, now_ns, has_observation_, *estimator_, *mapper_);
  last_localization_action_ = loc.action;
  if (loc.replan_requested) replan_requested_ = true;
  // TF publisher: odom->base is the continuous pre-correction prediction
  // in full 3D (x/y/z + roll/pitch/yaw — slopes ride here); map->odom is
  // the per-tick SE(2) correction delta (identity when nothing corrected).
  // Skipped until the estimate is valid.
  const EstimatedState post = estimator_->estimate();
 trajectory_tracking_report_ = trajectory_tracking_.observe(post);
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
 TerrainAdaptationPipeline::update(
 now_ns, last_obs_, map, estimator_->estimate(), estimator_->last_slip(),
 last_traverse_ns_, grip_, traversal_);
 // Feed the measured-only traction envelope back into odometry prediction
 // for the next observation. This bounds wheel-spin integration without
 // exposing plant truth to ROBOT; nominal grip leaves odometry unchanged.
 // Grip adaptation constrains the command envelope. It must not silently
 // rescale the pose integration: wheel slip is a measurement uncertainty,
 // not a known travelled-distance ratio, and applying the factor to odometry
 // made mapless goals permanently lag behind the physical chassis. The
 // estimator owns covariance/slip inflation and scan corrections; it only
 // consumes the measured encoder twist here.
 // Surveyed-elevation context for the terrain-aware layers (prior, never
  // live physics): validator ground-classification + grade gates. The map
 // is an immutable provider-owned revision and outlives this update, so the
 // pointer is valid for both validate() calls below. The live overlay rides from the
 // mapper's published layer (measurement heights where confident):
 // the provider owns the immutable map revision for the whole update;
 // the live layer is refreshed here and owned by the coordinator.
  live_map_ = mapper_ != nullptr ? mapper_->local_map() : LocalMap{};
  SurveyContext survey_ctx;
  survey_ctx.map = &map;
  survey_ctx.live.height_m = live_map_.has_elevation()
                                 ? live_map_.elevation_m.data()
                                 : nullptr;
  survey_ctx.live.width = live_map_.width;
  survey_ctx.live.height = live_map_.height;
  survey_ctx.live.resolution_m = live_map_.resolution_m;
  survey_ctx.live.origin_x_m = live_map_.origin_x_m;
  survey_ctx.live.origin_y_m = live_map_.origin_y_m;
  survey_ctx.sensor_height_m = lidar_mount_z_m_;
  survey_ctx.ride_height_m = ride_height_m_;
 survey_ctx.footprint_radius_m = limits_.footprint_radius_m;
  if (due || replan_requested_ || map_changed) {
    // The decision branch ran: stamp it first so cadence, map tracking, and
    // event flags stay consistent even when a stage below yields no plan.
    last_decision_ns_ = now_ns;
    last_map_revision_ = map.revision;
 const bool force_route_refresh = replan_requested_ || map_changed;
 replan_requested_ = false;
    // Survey revision: rebind the traversal belief and push the prior to
    // the mapper (ground classification for occupancy painting).
    if (map_changed) {
      traversal_.bind(map);
      if (mapper_ != nullptr) mapper_->set_survey(map);
    }
 const LocalMap robot_local_map = mapper_->local_map();
 const std::string& safety_reason = safety_->last_verdict().reason;
 const bool forward_blocked =
 safety_reason == "forward-proximity" ||
 safety_reason == "forward-recovery" ||
 safety_reason == "static-proximity" ||
 safety_reason == "static-swept-proximity";
 MotionDecisionResult motion = decision_pipeline_.run(
 *global_planner_, *local_planner_, *behavior_, *validator_, *controller_,
 estimator_->estimate(), perception_, robot_local_map, active_route_, goal_,
 has_goal_, map, force_route_refresh,
 grip_.effective_limits(limits_, limits_.max_forward_mps), limits_, last_obs_,
 survey_ctx, now_ns, forward_blocked);
 decision_ = motion.behavior;
 iface::MotionCommand decided = motion.command;
 std::string reason = motion.reason;
 std::string constraint = motion.constraint;
 last_prediction_horizon_s_ = motion.prediction_horizon_s;
 last_prediction_hypotheses_ = motion.prediction_hypotheses;
 if (!iface::valid_motion_command(decided, now_ns)) {
 decided = iface::MotionCommand::SafeStop(now_ns);
 }
 last_cmd_ = decided;
 last_constraint_ = constraint;
 last_reason_ = reason;
 }
 // Goal completion is an autonomy policy, not coordinator arithmetic. The
 // coordinator supplies the latest estimated state and active route, then
 // applies the policy's hold while the measured chassis settles.
 goal_arrival_decision_ = goal_arrival_.evaluate(
 estimator_->estimate(), goal_,
 has_goal_ && active_route_.valid && !active_route_.points.empty());
 if (goal_arrival_decision_.hold) {
 last_cmd_ = iface::MotionCommand::SafeStop(now_ns);
 last_constraint_ = "goal_hold";
 last_reason_ = goal_arrival_decision_.reason;
 }
 // The localization subsystem owns map-frame authority. Until a surveyed
 // startup establishes that frame, a route expressed in map coordinates
 // cannot safely command motion. The coordinator only applies its verdict.
 if (!loc.motion_authorized) {
 last_cmd_ = iface::MotionCommand::SafeStop(now_ns);
 last_constraint_ = "localization_hold";
 last_reason_ = "map-frame-unresolved";
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
 verdict = safety_->last_verdict();
  last_published_stop_ = final_cmd.is_safe_stop();
  // One evidence record per distinct decision; repeats update in place.
  diagnostics::DecisionEvidence evidence;
  evidence.observation_time_s = last_obs_.stamp_ns * 1e-9;
  evidence.observation_refs = {last_obs_.source + "#" +
                               std::to_string(last_obs_.sequence)};
 evidence.prediction_horizon_s = last_prediction_horizon_s_;
    evidence.prediction_hypotheses = last_prediction_hypotheses_;
  evidence.constraint_code = last_constraint_;
  evidence.reason_code = last_reason_;
  if (verdict.state != SafetyState::kNominal) {
    evidence.reason_code += "+safety:" + verdict.reason;
  }
  evidence.target_speed_mps = final_cmd.forward_mps;
  evidence.target_yaw_rate_radps = final_cmd.yaw_rate_radps;
  evidence.action_code = final_cmd.is_safe_stop() ? "stop" : "drive";
  evidence_.publish(std::move(evidence), now_ns * 1e-9);
 trajectory_tracking_.accept(
 final_cmd, estimator_->estimate(), trajectory_tracking_limits_);
  return final_cmd;
}

const EstimatedState& AutonomyCoordinator::estimate() const {
  return estimator_->estimate();
}

LocalMap AutonomyCoordinator::local_map() const {
  return mapper_->local_map();
}

}  // namespace lost_signal::robot::autonomy
