#include "robot/autonomy/coordinator.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <string>

#include "robot/autonomy/decision/autonomy_pipeline.hpp"

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
 config.acceleration_mps2 = motion.acceleration_mps2;
 config.brake_decel_mps2 = motion.brake_deceleration_mps2;
 config.lateral_acceleration_mps2 = motion.lateral_acceleration_mps2;
 config.yaw_acceleration_radps2 = motion.yaw_acceleration_radps2;
 config.actuator_response_delay_s = motion.actuator_response_delay_s;
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
 decision_pipeline_(MotionDecisionPipeline::Config{
 .motion = motion_policy,
 .decision_period_s = std::max(
 1e-9, static_cast<double>(decision_period_ns) * 1e-9)}),
      decision_period_ns_(decision_period_ns),
      estimator_(std::make_unique<FusedOdometryEstimator>()),
      perception_module_(std::make_unique<LidarPerception>(perception_config)),
      mapper_(std::make_unique<RayClearingMapper>()),
 behavior_(std::make_unique<WaitOnIntrusionBehavior>(
 behavior_config_for(navigation_policy, motion_policy))),
 global_planner_(make_global_planner(limits, wheel_base_m,
 navigation_policy)),
 local_planner_(make_local_planner(local_planner_kind,
 navigation_policy, motion_policy,
 arrival_config.position_tolerance_m,
 arrival_config.heading_tolerance_rad,
 arrival_config.confidence_probability,
 arrival_config.maximum_position_sigma_m,
 arrival_config.maximum_heading_sigma_rad,
 arrival_config.settle_cycles)),
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

void AutonomyCoordinator::reset_runtime_state() {
  estimator_->reset();
  perception_module_->reset();
  mapper_->reset();
  behavior_->reset();
  local_planner_->reset();
  controller_->reset();
  safety_->reset();
 decision_pipeline_.reset();
 observations_.reset();
  decision_ = BehaviorDecision{};
 goal_ = RoutePoint{};
 has_goal_ = false;
  active_route_ = Route{};
 live_map_ = {};
 goal_arrival_.reset();
 goal_arrival_decision_ = GoalArrivalDecision{};
  last_decision_ns_ = core::kInvalidTime;
  last_map_revision_ = 0;
  replan_requested_ = false;
 local_plan_reset_requested_ = false;
 clear_operator_command();
 operator_command_generation_ = 0;
 operator_command_consumed_generation_ = 0;
  last_cmd_ = iface::MotionCommand::SafeStop(0);
  last_constraint_ = "none";
  last_reason_ = "init";
 last_prediction_horizon_s_ = 0.0;
 last_prediction_hypotheses_ = 0;
 last_validated_static_clearance_m_ =
 std::numeric_limits<double>::infinity();
 last_validated_dynamic_clearance_m_ =
 std::numeric_limits<double>::infinity();
 last_local_costmap_ = {};
 last_validated_swept_envelope_radius_m_ = 0.0;
 last_validated_swept_centerline_.clear();
 last_route_point_count_ = 0;
 last_route_segment_ = 0;
 last_moving_candidate_count_ = 0;
 last_requested_gear_ = 0;
 last_route_error_m_ = std::numeric_limits<double>::infinity();
 last_gear_change_pending_ = false;
 last_gear_change_braking_armed_ = false;
 last_gear_change_endpoint_distance_m_ =
 std::numeric_limits<double>::infinity();
  latch_.reset();
  last_published_stop_ = false;
  grip_.reset();
  traversal_ = TraversalModel{};
  evidence_.reset();
 trajectory_tracking_.reset();
 trajectory_tracking_report_ = {};
}

core::Status AutonomyCoordinator::configure() {
 if (!local_planner_) {
 return core::Status::Fail("autonomy: unknown local planner");
 }
 reset_runtime_state();
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
 reset_runtime_state();
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
 observations_.set_lidar_mount(base_from_lidar);
 // Forward-sector gates use the same full rotation as endpoint
 // deprojection; roll/pitch-mounted lidars no longer take a yaw-only path.
 if (validator_ != nullptr) validator_->set_lidar_mount(base_from_lidar);
 if (safety_ != nullptr) safety_->set_lidar_mount(base_from_lidar);
}

void AutonomyCoordinator::set_goal(const RoutePoint& goal) {
 local_planner_->reset();
 goal_arrival_.reset();
 goal_arrival_decision_ = GoalArrivalDecision{};
  goal_ = goal;
  has_goal_ = true;
  active_route_ = Route{};  // force refresh on the next update
  replan_requested_ = true;
}

void AutonomyCoordinator::set_operator_command(double forward_mps,
 double yaw_rate_radps) {
 if (!std::isfinite(forward_mps) || !std::isfinite(yaw_rate_radps)) return;
 operator_command_active_ = true;
 ++operator_command_generation_;
 operator_forward_mps_ = std::clamp(
 forward_mps, -limits_.max_forward_mps, limits_.max_forward_mps);
 operator_yaw_rate_radps_ = std::clamp(
 yaw_rate_radps, -limits_.max_yaw_rate_radps,
 limits_.max_yaw_rate_radps);
}

void AutonomyCoordinator::clear_operator_command() {
 operator_command_active_ = false;
 operator_forward_mps_ = 0.0;
 operator_yaw_rate_radps_ = 0.0;
 operator_command_last_update_ns_ = core::kInvalidTime;
}

void AutonomyCoordinator::clear_goal() {
 local_planner_->reset();
 goal_arrival_.reset();
 goal_arrival_decision_ = GoalArrivalDecision{};
  goal_ = RoutePoint{};
  has_goal_ = false;
  active_route_ = Route{};
  replan_requested_ = false;
 local_plan_reset_requested_ = false;
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
 observations_.has_observation() ? observations_.latest().stamp_ns
 : estimate().stamp_ns;
 estimator_->seed_pose(x_m, y_m, yaw_rad, position_sigma_m,
 heading_sigma_rad, stamp);
 observations_.mark_localized();
}

void AutonomyCoordinator::ingest_observation(
    const iface::ObservationFrame& obs) {
  if (state_ != lifecycle::ModuleState::kActive) return;
 observations_.ingest(
 obs, tf_, *estimator_, *perception_module_, *mapper_);
}

iface::MotionCommand AutonomyCoordinator::update(
 const iface::MapSnapshot& map, const iface::ChannelHealth& channel,
    core::MonotonicNs now_ns) {
  // Missing observation is unknown evidence -> safe stop, never free space.
 if (state_ != lifecycle::ModuleState::kActive ||
 !observations_.has_observation()) {
    return iface::MotionCommand::SafeStop(now_ns);
  }
 if (operator_command_active_) {
 if (operator_command_generation_ != operator_command_consumed_generation_) {
 operator_command_consumed_generation_ = operator_command_generation_;
 operator_command_last_update_ns_ = now_ns;
 }
 if (operator_command_last_update_ns_ == core::kInvalidTime ||
 now_ns - operator_command_last_update_ns_ >
 iface::kDefaultCommandWatchdogNs) {
 clear_operator_command();
 }
 }
 const iface::ObservationFrame& latest_observation = observations_.latest();
  // Decision branch runs at the decision period, or on event wakeup:
  // explicit replan request, or a new map revision. Everything else holds
  // the last validated command while safety keeps gating every tick.
  bool due = last_decision_ns_ == core::kInvalidTime ||
             now_ns - last_decision_ns_ >= decision_period_ns_;
  bool map_changed = map.revision != last_map_revision_;
 const ObservationPipeline::TickResult observation_tick = observations_.update(
 map, now_ns, *estimator_, *perception_module_, *mapper_, grip_, traversal_);
 const LocalizationSystem::TickResult& loc = observation_tick.localization;
  if (loc.replan_requested) replan_requested_ = true;
 if (loc.local_plan_reset_requested) local_plan_reset_requested_ = true;
 // TF publisher follows REP-105. The estimator owns a continuous odom pose
 // that localization never rewrites; absolute corrections update only the
 // map pose. map->odom is their persistent SE(2) relationship, so a map
 // jump cannot become a controller-visible odom jump on the next tick.
  const EstimatedState post = estimator_->estimate();
 const EstimatedState odom = estimator_->odometry_estimate();
 trajectory_tracking_report_ = trajectory_tracking_.observe(odom);
 if (post.valid && odom.valid) {
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
 live_map_ = mapper_->local_map();
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
if (due || replan_requested_ || local_plan_reset_requested_ || map_changed) {
    // The decision branch ran: stamp it first so cadence, map tracking, and
    // event flags stay consistent even when a stage below yields no plan.
    last_decision_ns_ = now_ns;
    last_map_revision_ = map.revision;
 const bool force_route_refresh = replan_requested_ || map_changed;
 replan_requested_ = false;
 const bool reset_local_plan = local_plan_reset_requested_;
 local_plan_reset_requested_ = false;
    // Survey revision: rebind the traversal belief and push the prior to
    // the mapper (ground classification for occupancy painting).
 if (map_changed) {
      traversal_.bind(map);
 mapper_->set_survey(map);
    }
 const LocalMap robot_local_map = mapper_->local_map();
 const std::string& safety_reason = safety_->last_verdict().reason;
 const bool forward_blocked =
 safety_reason == "forward-proximity" ||
 safety_reason == "forward-recovery" ||
 safety_reason == "static-proximity" ||
 safety_reason == "static-swept-proximity";
 const bool observation_motion_requested =
 loc.observation_motion_requested ||
 goal_arrival_decision_.observation_motion_required;
 MotionDecisionResult motion = decision_pipeline_.run(
 *global_planner_, *local_planner_, *behavior_, *validator_, *controller_,
 estimator_->estimate(), observations_.perception(), robot_local_map,
 active_route_, goal_,
has_goal_, map, force_route_refresh, reset_local_plan,
 grip_.effective_limits(limits_, limits_.max_forward_mps), limits_,
 latest_observation,
 survey_ctx, now_ns, forward_blocked,
 observation_motion_requested);
 decision_ = motion.behavior;
 iface::MotionCommand decided = motion.command;
 std::string reason = motion.reason;
 std::string constraint = motion.constraint;
 if (operator_command_active_) {
 decided.space = iface::CommandSpace::kYawRate;
 decided.forward_mps = operator_forward_mps_;
 decided.yaw_rate_radps = operator_yaw_rate_radps_;
 decided.steer_angle_rad = 0.0;
 decided.lateral_mps = 0.0;
 reason = "operator-command";
 constraint = "manual";
 }
 last_prediction_horizon_s_ = motion.prediction_horizon_s;
 last_prediction_hypotheses_ = motion.prediction_hypotheses;
 if (!iface::valid_motion_command(decided, now_ns)) {
 decided = iface::MotionCommand::SafeStop(now_ns);
 }
 last_cmd_ = decided;
 last_constraint_ = constraint;
 last_reason_ = reason;
 last_validated_static_clearance_m_ = motion.validated_static_clearance_m;
 last_validated_dynamic_clearance_m_ = motion.validated_dynamic_clearance_m;
 last_local_costmap_ = std::move(motion.local_costmap);
 last_validated_swept_envelope_radius_m_ =
 motion.validated_swept_envelope_radius_m;
 last_validated_swept_centerline_ =
 std::move(motion.validated_swept_centerline);
 last_route_point_count_ = motion.route_point_count;
 last_route_segment_ = motion.route_segment;
 last_moving_candidate_count_ = motion.moving_candidate_count;
 last_requested_gear_ = motion.requested_gear;
 last_route_error_m_ = motion.route_error_m;
 last_gear_change_pending_ = motion.gear_change_pending;
 last_gear_change_braking_armed_ = motion.gear_change_braking_armed;
 last_gear_change_endpoint_distance_m_ =
 motion.gear_change_endpoint_distance_m;
 }
 if (operator_command_active_) {
 last_cmd_.space = iface::CommandSpace::kYawRate;
 last_cmd_.stamp_ns = now_ns;
 last_cmd_.expiry_ns = now_ns + iface::kDefaultCommandWatchdogNs;
 last_cmd_.forward_mps = operator_forward_mps_;
 last_cmd_.yaw_rate_radps = operator_yaw_rate_radps_;
 last_cmd_.steer_angle_rad = 0.0;
 last_cmd_.lateral_mps = 0.0;
 last_constraint_ = "manual";
 last_reason_ = "operator-command";
 }
 // Goal completion is an autonomy policy, not coordinator arithmetic. The
 // coordinator supplies the latest estimated state and active route, then
 // applies the policy's hold while the measured chassis settles.
 goal_arrival_decision_ = goal_arrival_.evaluate(
 estimator_->estimate(), goal_,
 has_goal_ && active_route_.valid && !active_route_.points.empty(),
 loc.observation_motion_requested);
 if (goal_arrival_decision_.hold) {
 last_cmd_ = iface::MotionCommand::SafeStop(now_ns);
 last_constraint_ = "goal_hold";
 last_reason_ = goal_arrival_decision_.reason;
 }
 // The localization subsystem owns map-frame authority. Until a surveyed
 // startup establishes that frame, a route expressed in map coordinates
 // cannot safely command motion. The coordinator only applies its verdict.
 if (!loc.motion_authorized && !operator_command_active_) {
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
 SafetyVerdict verdict = safety_->evaluate(
 estimator_->estimate(), observations_.perception(), latest_observation,
 channel, now_ns);
  iface::MotionCommand final_cmd = safety_->gate(gated, verdict, now_ns);
 verdict = safety_->last_verdict();
  last_published_stop_ = final_cmd.is_safe_stop();
  // One evidence record per distinct decision; repeats update in place.
  diagnostics::DecisionEvidence evidence;
 evidence.observation_time_s = latest_observation.stamp_ns * 1e-9;
 auto add_observation_ref = [&](const iface::MeasurementHeader& header) {
 if (header.valid()) {
 evidence.observation_refs.push_back(
 header.source + "#" + std::to_string(header.sequence));
 }
 };
 if (observations_.has_scan()) {
 add_observation_ref(latest_observation.scan_header);
 }
 if (observations_.has_motion()) {
 add_observation_ref(latest_observation.motion_header);
 }
 if (observations_.has_imu()) {
 add_observation_ref(latest_observation.imu_header);
 }
 evidence.prediction_horizon_s = last_prediction_horizon_s_;
    evidence.prediction_hypotheses = last_prediction_hypotheses_;
  evidence.constraint_code = last_constraint_;
  evidence.reason_code = last_reason_;
  if (verdict.state != SafetyState::kNominal) {
    evidence.reason_code += "+safety:" + verdict.reason;
  }
  evidence.target_speed_mps = final_cmd.forward_mps;
  evidence.target_yaw_rate_radps = final_cmd.yaw_rate_radps;
 evidence.proposed_speed_mps = gated.forward_mps;
 evidence.proposed_yaw_rate_radps = gated.yaw_rate_radps;
 evidence.route_point_count = last_route_point_count_;
 evidence.route_segment = last_route_segment_;
 evidence.moving_candidate_count = last_moving_candidate_count_;
 evidence.requested_gear = last_requested_gear_;
 evidence.route_error_m = last_route_error_m_;
 evidence.gear_change_pending = last_gear_change_pending_;
 evidence.gear_change_braking_armed = last_gear_change_braking_armed_;
 evidence.gear_change_endpoint_distance_m =
 last_gear_change_endpoint_distance_m_;
 evidence.validator_static_clearance_m =
 last_validated_static_clearance_m_;
 evidence.local_costmap = last_local_costmap_;
 evidence.validator_swept_envelope_radius_m =
 last_validated_swept_envelope_radius_m_;
 evidence.validator_swept_centerline =
 last_validated_swept_centerline_;
 evidence.predicted_separation_m = last_validated_dynamic_clearance_m_;
 evidence.safety_hold_clearance_m = verdict.static_hold_clearance_m;
 evidence.measured_separation_m = verdict.static_hold_clearance_m;
 evidence.safety_command_clearance_m =
 verdict.static_command_min_clearance_m;
 evidence.safety_terminal_clearance_m =
 verdict.static_command_terminal_clearance_m;
 evidence.safety_dynamic_hold_clearance_m =
 verdict.dynamic_hold_clearance_m;
 evidence.safety_dynamic_command_clearance_m =
 verdict.dynamic_command_clearance_m;
 evidence.safety_static_envelope_radius_m =
 verdict.static_command_envelope_radius_m;
 evidence.safety_static_sweep = verdict.static_command_sweep;
  evidence.action_code = final_cmd.is_safe_stop() ? "stop" : "drive";
  evidence_.publish(std::move(evidence), now_ns * 1e-9);
 trajectory_tracking_.accept(
 final_cmd, estimator_->odometry_estimate(), trajectory_tracking_limits_);
  return final_cmd;
}

const EstimatedState& AutonomyCoordinator::estimate() const {
  return estimator_->estimate();
}

LocalMap AutonomyCoordinator::local_map() const {
  return mapper_->local_map();
}

}  // namespace lost_signal::robot::autonomy
