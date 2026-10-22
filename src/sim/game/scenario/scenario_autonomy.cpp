#include "sim/game/scenario/scenario_session.hpp"
#include "robot/autonomy/planning/local/dynamic_clearance.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iterator>
#include <limits>
#include <numeric>
#include <optional>
#include <sstream>
#include <utility>

namespace mission {
namespace {

double distance(sim::Vec2 a, sim::Vec2 b) {
 return std::hypot(a.x - b.x, a.y - b.y);
}

sim::Vec2 as_vec(const Pose& pose) {
 return {pose.x, pose.y};
}

bool dispatch_has_priority(const FieldDispatchState& dispatch,
 const WorldIncidentState& incident) {
 if (!dispatch.active)
 return false;
 return !incident.active || dispatch.opened_at_s <= incident.opened_at_s;
}

} // namespace

void ScenarioSession::decide_control() {
 ++result_.local_decisions;
 // Actuator feedback belongs to the observation that preceded this decision.
 // Preserve it before clearing presentation/output fields for the new cycle;
 // otherwise differential reachability silently receives zero wheel speeds
 // on every update even though LSRI delivered valid encoder feedback.
 const double measured_left_mps = snapshot_.control.left_measured_mps;
 const double measured_right_mps = snapshot_.control.right_measured_mps;
 snapshot_.control = {};
 if (robot_application_->route_path_index() >= planned_route_.size()) {
  if (planned_route_.empty()) {
   held_command() = {0.0, 0.0};
   DecisionEvidence decision;
   decision.observation_time_s = snapshot_.time_s;
   decision.constraint_code = "route";
   decision.reason_code = "no-route";
   decision.action_code = "wait";
   publish_decision(std::move(decision));
   return;
  }
  // Docking hold: waypoint bookkeeping exhausted (resync/accept ran ahead of
  // a lagging chassis) but arrival tolerance unmet — success is measured by
  // distance, not by idx. Hold the last waypoint so normal tracking (reverse
  // hysteresis, watchdog, DWA arbitration below) docks the robot instead of
  // parking short with tv=0 until timeout.
  robot_application_->set_route_path_index(planned_route_.size() - 1);
 }
 const auto& estimate = robot_application_->estimator().state();
 const double nav_x = estimate.position.x;
 const double nav_y = estimate.position.y;
 const double nav_heading = estimate.heading_rad;
 // Grade compensation is a body-motion safety input, not a map-frame truth
 // lookup. When wheel slip has already made odometric yaw unreliable, using
 // the drifting estimate to project the slope can reverse the gravity term
 // and actively drive downhill. The route heading is the deployable motion
 // intent and remains stable under that failure; fall back to the estimate
 // only when no route heading is available.
 const double grade_heading = [&] {
 if (!planned_route_.empty()) {
 const auto index = std::min(robot_application_->route_path_index(),
 planned_route_.size() - 1);
 if (std::isfinite(planned_route_[index].heading_rad))
 return planned_route_[index].heading_rad;
 }
 return nav_heading;
 }();
 // The declared localization error is the physical uncertainty reserve. The
 // preferred static envelope is the normal planning margin; recovery is
 // allowed to consume that envelope, but never this hard stand-off. A small
 // non-zero stand-off is required even when a level declares zero localization
 // error: tracker lag, grade creep and an offset axle are physical motion, not
 // localization noise. Keep the preferred envelope (robot radius plus policy
 // margin) as the normal planning contract and use this floor only to stop a
 // recovery from spending the last centimetres before contact.
 const auto& execution_policy = motion_policy_.execution;
 const double physical_execution_margin_m = std::max(
 level_.planning_localization_margin_m, execution_policy.physical_standoff_reserve_m);
 // The local planning view composes immutable authored occupancy with the
 // transient LiDAR obstacle layer. Dynamic target prediction remains a
 // separate time-indexed input below; an observed obstacle therefore cannot
 // become a permanent global wall or disappear merely because tracking lost
 // its identity.
 const sim::GridMap local_planning_map = robot_application_->compose_local_map(
 *planning_map_, snapshot_.time_s, planning_clearance_m_);
 const auto transient_obstacles =
 robot_application_->active_observed_obstacles(
 snapshot_.time_s);
 snapshot_.control.transient_obstacle_cells = transient_obstacles.size();
 snapshot_.control.nearest_transient_obstacle_m =
 std::numeric_limits<double>::infinity();
 for (const auto obstacle : transient_obstacles) {
 snapshot_.control.nearest_transient_obstacle_m = std::min(
 snapshot_.control.nearest_transient_obstacle_m,
 distance(estimate.position, obstacle));
 }
 snapshot_.control.local_map_pose_free =
 local_planning_map.isFree(nav_x, nav_y);
 snapshot_.control.local_map_pose_clearance_m =
 local_planning_map.clearance(nav_x, nav_y);
 const auto active_dynamic_tracks =
 robot_application_->active_dynamic_tracks(
 snapshot_.time_s, track_max_age_s());
 if (robot_application_->route_should_replan(
 planned_route_, estimate, local_planning_map, active_dynamic_tracks,
 1.0 / decision_hz())) {
 // Persistent unmodelled static occupancy invalidates the global route.
 // Replan from the estimated pose on the same layered costmap consumed by
 // local planning. Live tracked actors are excluded by the ROBOT-side route
 // monitor, so a passing forklift remains a prediction/behavior problem
 // rather than becoming a permanent detour in mission code.
 plan_route(&local_planning_map);
 if (planned_route_.empty()) {
 held_command() = {};
 return;
 }
 }
 // Static planning owns two severities. The preferred map includes the
 // normal numerical/policy envelope. Recovery keeps footprint-inflated
 // transient observations in a raster while the analytic collision monitor
 // checks authored geometry. Reusing the coarse static occupancy cells as a
 // physical oracle can add almost one cell around a box corner and trap a
 // footprint that still has real clearance.
 // A recovery may relax policy inflation, but it still has to reserve the
 // declared localization error around the physical footprint. Otherwise an
 // estimated pose a centimetre inside the map can authorize a trajectory
 // whose true body crosses a wall or map boundary. This margin is deployable
 // configuration, not simulator ground truth.
 const sim::GridMap local_physical_collision_map = robot_application_->compose_local_map(
 *recovery_base_map_, snapshot_.time_s,
 robot_definition_.collision_radius_m + level_.planning_localization_margin_m);
 const auto tracking = robot_application_->observe_trajectory(
 snapshot_.time_s, estimate.position, estimate.heading_rad, estimate.linear_velocity_mps);
 snapshot_.control.trajectory_sequence = tracking.sequence;
 snapshot_.control.trajectory_tracking_active = tracking.active;
 snapshot_.control.trajectory_within_envelope = tracking.within_envelope;
 snapshot_.control.trajectory_position_error_m = tracking.position_error_m;
 snapshot_.control.trajectory_heading_error_rad = tracking.heading_error_rad;
 snapshot_.control.trajectory_speed_error_mps = tracking.speed_error_mps;
 const double chassis_cruise = cruise_speed_mps();
 auto route_control =
 robot_application_->follow_route(planned_route_, estimate,
 property(behavior_, "lookahead_m"), chassis_cruise,
 1.0 / decision_hz(), local_planning_map, chassis_caps_);
 // The target and control request must come from the same stateful route
 // update. Re-running waypoint advancement here used to select a different
 // target at layered-map corners and could lock a valid forward route into
 // an in-place spin with a zero-velocity DWA window.
 auto target = route_control.target_valid
 ? route_control.target
 : navigation_fallback_target();
 const auto publish = [&](std::string constraint_code, std::string reason_code, std::string action_code,
                          double observation_time_s, std::vector<std::string> observation_refs,
                          double prediction_horizon_s, double measured_separation_m,
                          double predicted_separation_m) {
  DecisionEvidence decision;
  decision.observation_time_s = observation_time_s;
  decision.observation_refs = std::move(observation_refs);
  decision.prediction_horizon_s = prediction_horizon_s;
  decision.measured_separation_m = measured_separation_m;
  decision.predicted_separation_m = predicted_separation_m;
  decision.constraint_code = std::move(constraint_code);
  decision.reason_code = std::move(reason_code);
  decision.action_code = std::move(action_code);
  decision.target_speed_mps = held_command().target_v;
  decision.target_yaw_rate_radps = held_command().target_w;
  publish_decision(std::move(decision));
 };
 sim::StrategyObservation strategy_observation;
 strategy_observation.stamp_s = estimate.timestamp_s;
 strategy_observation.ego = estimate;
 strategy_observation.route_target = target;
 const GoalStep* strategy_goal = active_goal();
 strategy_observation.goal = strategy_goal ? as_vec(map_definition_.markers.at(strategy_goal->marker))
                                           : as_vec(map_definition_.markers.at(level_.objective.goal));
 // ROBOT policy consumes only the attitude estimated from delivered IMU
 // observations. MuJoCo's post-step roll and the mission rollover score are
 // SIM truth and must not become a policy input.
 const double estimated_rollover_angle = robot_application_->static_rollover_angle(
 chassis_caps_.geometry.support_width_m, combined_center_height_m());
 strategy_observation.rollover_risk = estimate.attitude_valid
 ? std::abs(estimate.roll_rad) / std::max(0.05, estimated_rollover_angle)
 : 0.0;
 strategy_observation.traction_slip_observed = snapshot_.traction_slip_observed;
 strategy_observation.goals_reached = static_cast<int>(snapshot_.goals_reached);
 for (const auto& hit : snapshot_.lidar_hits) {
  strategy_observation.visible_clearance_m =
    std::min(strategy_observation.visible_clearance_m,
             distance(estimate.position, hit) - robot_definition_.collision_radius_m);
 }
 sim::StrategySafetyContext strategy_context;
 strategy_context.fallback_target = target;
 strategy_context.map_size = map_definition_.size_m;
 strategy_context.clearance_hold_enabled = custom_high_level_policy_;
 const auto strategy_terrain = robot_application_->estimated_terrain({nav_x, nav_y});
 const bool material_terrain_load =
   std::abs(robot_application_->longitudinal_grade(strategy_terrain, nav_heading)) >= 0.12 ||
   std::abs(robot_application_->lateral_grade(strategy_terrain, nav_heading)) >= 0.12;
 // A custom upper policy gets the complete safety shield. The validated
 // route baseline already owns flat-ground speed and obstacle arbitration;
 // its residual-based relief is reserved for a mapped slope, where reducing
 // translation preserves differential steering authority under motor load.
 strategy_context.traction_relief_enabled = custom_high_level_policy_ || material_terrain_load;
 strategy_context.subgoal_is_safe = [this, nav_x, nav_y, &local_planning_map](sim::Vec2 subgoal) {
  const double length = distance({nav_x, nav_y}, subgoal);
  const int samples = std::max(1, static_cast<int>(std::ceil(length / 0.25)));
  sim::Vec2 previous{nav_x, nav_y};
  for (int index = 1; index <= samples; ++index) {
   const double alpha = static_cast<double>(index) / samples;
   const sim::Vec2 point{
     nav_x + (subgoal.x - nav_x) * alpha,
     nav_y + (subgoal.y - nav_y) * alpha,
   };
 if (!local_planning_map.isFree(point.x, point.y) ||
       !std::isfinite(navigation_traversal_cost(previous, point))) {
    return false;
   }
   previous = point;
  }
  return true;
 };
 // The policy and its common safety shield are ROBOT-owned. SIM supplies
 // only the timestamped estimate, task target and provider-neutral map
 // feasibility callback through the application cycle; it does not invoke
 // the coordinator policy directly.
 if (!robot_application_)
 throw std::logic_error("strategy evaluation requires the ROBOT application");
 sim::robot::RobotCycleInput strategy_cycle;
 strategy_cycle.autonomy = {strategy_observation, strategy_context};
 const auto strategy_cycle_output = robot_application_->tick(strategy_cycle);
 snapshot_.strategy = strategy_cycle_output.autonomy.decision;
 snapshot_.strategy_policy = strategy_cycle_output.autonomy.policy_name;
 if (snapshot_.strategy.applied.has_subgoal)
  target = snapshot_.strategy.applied.subgoal;
 if (snapshot_.strategy.applied.has_subgoal) {
  const double target_distance = distance({nav_x, nav_y}, target);
  const double subgoal_error = robot_application_->heading_error(target, {nav_x, nav_y}, nav_heading);
  route_control.tv = std::min(chassis_cruise, target_distance);
  route_control.steer = robot_application_->route_heading_command(subgoal_error, 1.0 / decision_hz());
  route_control.err = subgoal_error;
  route_control.reached = target_distance < 0.25;
 }
 snapshot_.control.local_target = target;
 snapshot_.control.heading_error_rad = route_control.err;
 snapshot_.control.heading_pid_p = robot_application_->route_pid_p();
 snapshot_.control.heading_pid_i = robot_application_->route_pid_i();
 snapshot_.control.heading_pid_d = robot_application_->route_pid_d();
 // Docking controller: guide_step reports reached when waypoint bookkeeping
 // exhausts (accept/resync ran ahead of a lagging chassis), but mission
 // success is distance-to-goal. If the goal is still beyond tolerance yet
 // close, dock with the same PID instead of parking short until timeout.
 // Structure mirrors guide_step (reverse hysteresis, spin capability branch)
 // so behavior stays consistent across the tracking/docking seam.
 {
  const GoalStep* active = active_goal();
  sim::Vec2 goal = active ? as_vec(map_definition_.markers.at(active->marker))
                          : as_vec(map_definition_.markers.at(level_.objective.goal));
  double arrival_radius = active ? active->radius_m : level_.objective.arrive_tolerance_m;
  if (endless_mode_) {
   const auto& intent = field_operator_system_.intent();
   if (intent.active) {
    goal = intent.target;
    arrival_radius = 0.25;
   } else if (dispatch_has_priority(field_dispatch_system_.state(), snapshot_.world_activity.incident)) {
    goal = field_dispatch_system_.state().target;
    arrival_radius = 0.85;
   } else if (snapshot_.world_activity.incident.active) {
    goal = snapshot_.world_activity.incident.position;
    arrival_radius = std::max(0.25, snapshot_.world_activity.incident.radius_m);
   }
  }
  const double dist_goal = distance({nav_x, nav_y}, goal);
  const double tracking_arrival_radius = std::min(arrival_radius, 0.25);
  // Bounded: far exhaustion means the follower lost the path, not that the
  // goal is near; beelining across the map without waypoints risks walls.
  // (An unbounded trial stranded swift tracks in a reversing cycle instead
  // of docking — evidence for curation, not for a longer leash.)
  if (route_control.reached && dist_goal > tracking_arrival_radius && dist_goal < 3.0) {
   const double dock_err = robot_application_->heading_error(goal, {nav_x, nav_y}, nav_heading);
   if (!robot_application_->route_reversing() && std::abs(dock_err) > 1.4)
    robot_application_->set_route_reversing(true);
   else if (robot_application_->route_reversing() && std::abs(dock_err) < 0.7)
    robot_application_->set_route_reversing(false);
   const double dock_dt = 1.0 / decision_hz();
   if (!robot_application_->route_reversing()) {
    route_control.tv = std::min(0.3, dist_goal);
    route_control.steer = robot_application_->route_heading_command(dock_err, dock_dt);
   } else if (chassis_->caps().can_rotate_about_base_link()) {
    robot_application_->set_route_reversing(false);
    route_control.tv = 0.0;
    route_control.steer = robot_application_->route_heading_command(dock_err, dock_dt);
   } else {
    route_control.tv = -0.3;
    route_control.steer = -robot_application_->route_heading_command(dock_err, dock_dt);
   }
   route_control.err = dock_err;
   route_control.reached = false;
  }
 }
 route_control.tv *= snapshot_.strategy.applied.speed_scale;
 const double unavailable = std::numeric_limits<double>::quiet_NaN();
 if (snapshot_.strategy.applied.mode == sim::StrategyMode::Hold) {
  held_command() = {};
  snapshot_.waiting_for_clearance = true;
  snapshot_.control.local_target = target;
  snapshot_.control.heading_error_rad = route_control.err;
  publish("high-level-intent", "strategy-hold", "wait", estimate.timestamp_s, {}, unavailable, unavailable,
          unavailable);
  update_shadow_episode();
  return;
 }
 const auto all_tracks =
 robot_application_->active_dynamic_tracks(snapshot_.time_s, track_max_age_s());
 std::vector<sim::DynamicTrack> known_tracks;
 std::copy_if(all_tracks.begin(), all_tracks.end(), std::back_inserter(known_tracks), [](const auto& track) {
 return track.motion_class != sim::TrackMotionClass::Unconfirmed;
 });

 std::vector<std::string> observation_refs;
 double observation_time_s = std::numeric_limits<double>::quiet_NaN();
 observation_refs.reserve(known_tracks.size());
 for (const auto& track : known_tracks) {
  observation_refs.push_back("track." + std::to_string(track.object_id));
  if (!std::isfinite(observation_time_s) || track.last_observed_s > observation_time_s)
   observation_time_s = track.last_observed_s;
 }

 double nearest_actor = std::numeric_limits<double>::infinity();
 for (const auto& track : known_tracks) {
  const auto& actor = track.state;
  nearest_actor = std::min(nearest_actor, distance({nav_x, nav_y}, actor.p) - actor.radius -
                                            robot_definition_.collision_radius_m);
 }
 snapshot_.control.nearest_dynamic_track_m = nearest_actor;

 const auto* computer = resolved_build_.computer();
 const double prediction_horizon = computer ? computer->prediction_horizon_s : 1.2;
 const auto* active_lidar = resolved_build_.lidar();
 const double scan_motion_margin =
   active_lidar
 ? 2.0 * (cruise_speed_mps() + yaw_cap_rads_ * robot_definition_.collision_radius_m) /
         active_lidar->scan_hz
     : 0.0;
 // A rotating scan is up to one frame old and the tracker consumes it on the
 // next decision. Reserve the distance the robot footprint can sweep across
 // those two periods. This belongs to dynamic avoidance; inflating the static
 // global map with it would incorrectly close narrow but stationary routes.
 const double preferred_clearance =
 property(behavior_, "safe_clearance_m") + scan_motion_margin;
 // Separate three quantities: physical footprint overlap is the absolute
 // collision floor; sensing/control uncertainty is the hard normal-motion
 // reserve; policy and score clearances are soft preferences. The level's
 // safe_min_clearance_m lives under `score` and must never become an
 // autonomy permission threshold. Doing so makes legitimate narrow or
 // time-critical maneuvers impossible and couples ROBOT behavior to GAME.
 const double hard_crossing_clearance = scan_motion_margin;
 const double heading_error = robot_application_->heading_error(target, {nav_x, nav_y}, nav_heading);
 // Ride limit. Relief the chassis cannot tilt over lifts a contact instead
 // of steering the body, and the caster only retains the pitch restraint
 // while it carries weight. Hold the demanded vertical excitation below
 // half of g so a bump cannot unload the support that keeps the body level.
 const double ride_speed_limit_mps = robot_application_->ride_speed_limit(
 robot_application_->estimated_terrain({nav_x, nav_y}).roughness,
 chassis_caps_.geometry.support_length_m, 0.5);
 const double route_independent_speed_cap =
 std::min({std::max(0.20, robot_application_->scheduled_speed(heading_error, cruise_speed_mps())),
 corner_speed_cap(nav_x, nav_y), ride_speed_limit_mps});
 double speed_cap = route_independent_speed_cap * snapshot_.strategy.applied.speed_scale;
 // The selected rollout is the command contract. Feed route and corner caps
 // into candidate generation so mission code never rewrites its speed later.
 if (chassis_caps_.command_space != sim::ChassisCommandSpace::HolonomicTwist)
 speed_cap = std::min(speed_cap, std::abs(route_control.tv));
 const auto weights = robot_application_->local_planner_weights(preferred_clearance);
 // Decision-cycle reachability step. Deployment-owned: a native computer may
 // run a different control period than the simulation baseline.
 const double prediction_step_s = execution_policy.prediction_step_s;
 // Predictor module (src/robot/autonomy/predictor.*) owns future frames; the tracker
 // owns observations. known_tracks (age-advanced, for display) must not
 // feed prediction: map.predict() snapshots raw tracker state internally.
 const auto predicted_batch = robot_application_->predict_dynamic_objects(
 snapshot_.time_s, prediction_horizon, prediction_step_s, track_max_age_s());
 const auto predicted_frames = robot_application_->flatten_prediction(predicted_batch);
 const auto stationary_forecast_clearance = robot_application_->minimum_stationary_clearance(
 {nav_x, nav_y}, predicted_frames, prediction_step_s,
 robot_definition_.collision_radius_m);
 // Shadow departure assessment (gate 3): observe only. This block reads
 // tracker state and writes snapshot shadow fields; held_command() (the actual
 // control output) is untouched by everything in this block.
 std::vector<sim::Vec2> shadow_remaining{{nav_x, nav_y}};
 for (std::size_t i = robot_application_->route_path_index(); i < planned_route_.size(); ++i) {
  shadow_remaining.push_back(planned_route_[i].position);
 }
 const double shadow_cruise = std::max(1e-6, cruise_speed_mps());
 const double crossing_preferred_speed_mps =
 shadow_cruise * snapshot_.strategy.applied.speed_scale;
 const double crossing_platform_speed_mps = std::max(
 crossing_preferred_speed_mps,
 std::min(max_forward_speed_mps_, robot_definition_.rated_speed_mps) *
 chassis_caps_.rigid_body_cruise_scale *
 snapshot_.strategy.applied.speed_scale);
 // Crossing, route-departure shadow, and pocket evaluation share one
 // time-indexed prediction. Recomputing each consumer independently could
 // make a single decision explain itself with mutually different futures.
 // This is a local receding-horizon contract: extending a short-horizon
 // motion model to the final mission goal makes uncertainty grow for tens of
 // seconds and can push a safe crossing hold pose outside sensor range. A
 // crossing whose complete execution does not fit this declared computer
 // horizon is explicitly HorizonShort and remains closed.
 const double assessment_horizon = prediction_horizon;
 const auto assessment_batch = robot_application_->predict_dynamic_objects(
 snapshot_.time_s, assessment_horizon, prediction_step_s, track_max_age_s());
 const auto assessment_frames = robot_application_->flatten_prediction(assessment_batch);

 // A map-semantic conflict zone is the high-level contract for crossing
 // traffic. It yields a stable route-derived stop point and a bounded path
 // through the zone. Actor truth and game collision flags are deliberately
 // absent: only tracked predictions participate in departure assessment.
 bool mapped_crossing_relevant = false;
 std::optional<sim::GovernorOutput> mapped_governor;
 // A differential chassis whose driven axle is offset from base_link sweeps
 // base_link while yawing at a stationary axle. Behavior backs the stop pose
 // up by this same bound; the local stop-plane validator may then admit that
 // physically necessary pose correction without spending safety clearance.
 const double stop_plane_tracking_tolerance_m =
 std::abs(chassis_caps_.geometry.drive_axle_x_m);
 const auto traffic_decision = robot_application_->assess_crossing(
 planned_route_, robot_application_->route_path_index(), {nav_x, nav_y},
 estimate.linear_velocity_mps, nav_heading, assessment_frames,
 prediction_step_s,
 {robot_definition_.collision_radius_m, hard_crossing_clearance,
 // Route vertices are world-space poses even when the global search used a
 // raster. Grid resolution affects which poses exist; it is not physical
 // stop-controller error. Reserve only the declared localization margin here.
 level_.planning_localization_margin_m, 0.35,
 sim::kCrossingPositionTrackingToleranceM +
 level_.planning_localization_margin_m,
 stop_plane_tracking_tolerance_m},
 // Admission may prove only a speed that the receding-horizon controller
 // will actually execute. Searching up to motor free speed would prove one
 // timed crossing and then hand a different high-speed steering rollout to
 // DWA. The executor below consumes the selected transit speed explicitly;
 // behavior may choose any speed within the assembled platform limit while
 // every candidate remains subject to the shared validator.
 {crossing_preferred_speed_mps, crossing_platform_speed_mps,
 std::min(robot_definition_.motor.max_wheel_accel_mps2,
 resolved_build_.traction().brake_decel_mps2),
 resolved_build_.traction().brake_decel_mps2, yaw_cap_rads_,
 robot_definition_.max_yaw_accel_radps2,
 1.0 / decision_hz() + 1.0 / std::max(
 0.1, resolved_build_.motor()
 ? resolved_build_.motor()->controller_bandwidth_radps
 : robot_definition_.motor.controller_bandwidth_radps),
 prediction_horizon,
 // The current DWA/RO-1 plant has validated stop-turn-go execution. Its
 // continuous-curvature yaw response is still a roadmap capability and may
 // not be assumed by crossing admission merely because the base can spin.
 false},
 1.0 / decision_hz(), snapshot_.time_s,
 [this, &local_planning_map, physical_execution_margin_m](sim::Vec2 from, sim::Vec2 to) {
 return std::isfinite(navigation_traversal_cost(from, to)) &&
 robot_application_->static_motion_free(
 from, to, physical_execution_margin_m) &&
 local_planning_map.segmentIsFree(from, to);
 });
 const auto& crossing = traffic_decision.assessment;
 // Behavior may grant permission to enter a controlled crossing, but it is
 // never a collision waiver. The local planner is the final motion-safety
 // authority and therefore checks every maintained intent hypothesis. The
 // nominal forecast remains useful as behavior evidence; it must not replace
 // the conservative forecast used to authorize an actuator command.
 const auto& local_predicted_frames = predicted_frames;
 snapshot_.control.crossing_assessed = crossing.relevant;
 snapshot_.control.crossing_control_authority = traffic_decision.control_authority;
 snapshot_.control.crossing_zone_id = crossing.zone_id;
 snapshot_.control.crossing_stop = crossing.stop_point;
 snapshot_.control.crossing_wait_s =
   std::isfinite(crossing.departure.wait_s) ? crossing.departure.wait_s : -1.0;
 snapshot_.control.crossing_reason = crossing.relevant ? static_cast<int>(crossing.departure.reason) : -1;
 snapshot_.control.crossing_phase = static_cast<int>(traffic_decision.control.phase);
 snapshot_.control.crossing_committed = traffic_decision.control.committed;
 snapshot_.control.crossing_body_in_conflict = crossing.body_in_conflict;
 snapshot_.control.crossing_execution_blocked =
 robot_application_->crossing_execution_feedback().blocked;
 snapshot_.control.crossing_recovery_to_stop =
 traffic_decision.control.recovery_to_stop;
 snapshot_.control.crossing_recovery_reverse =
 traffic_decision.control.recovery_reverse;
 snapshot_.control.crossing_no_return_started =
 traffic_decision.control.no_return_started;
 snapshot_.control.crossing_alignment_required =
 traffic_decision.control.execution_alignment_required;
 snapshot_.control.crossing_alignment_complete =
 traffic_decision.control.execution_alignment_complete;
 snapshot_.control.crossing_execution_initial_heading_rad =
 traffic_decision.control.execution_initial_heading_rad;
 snapshot_.control.crossing_ready_to_commit = traffic_decision.ready_to_commit;
 snapshot_.control.crossing_safe_to_commit = traffic_decision.safe_to_commit;
 snapshot_.control.crossing_position_ready = traffic_decision.readiness.position_ready;
 snapshot_.control.crossing_speed_ready = traffic_decision.readiness.speed_ready;
 snapshot_.control.crossing_heading_ready = traffic_decision.readiness.heading_ready;
 snapshot_.control.crossing_stop_longitudinal_m = traffic_decision.readiness.stop_longitudinal_m;
 snapshot_.control.crossing_stop_lateral_m = traffic_decision.readiness.stop_lateral_m;
 snapshot_.control.crossing_stop_distance_m = crossing.stop_distance_m;
 snapshot_.control.crossing_stop_heading_error_rad = traffic_decision.readiness.heading_error_rad;
 snapshot_.control.crossing_prediction_evidence_available = crossing.prediction_evidence_available;
 snapshot_.control.crossing_prediction_models_ready =
 crossing.prediction_models_ready;
 snapshot_.control.crossing_free_space_evidence_available =
 crossing.free_space_evidence_available;
 snapshot_.control.crossing_observed_approaches_clear =
 crossing.observed_approaches_clear;
 snapshot_.control.crossing_observation_coverage = crossing.observation_coverage;
 snapshot_.control.crossing_unseen_arrival_lower_bound_s =
 crossing.unseen_arrival_lower_bound_s;
 snapshot_.control.crossing_observation_stamp_s =
 std::isfinite(crossing.observation_stamp_s) ? crossing.observation_stamp_s : -1.0;
 snapshot_.control.crossing_threats_moving_away = crossing.relevant_threats_moving_away;
 snapshot_.control.crossing_exit = traffic_decision.control.exit_point;
 snapshot_.control.crossing_transit_speed_mps = crossing.transit_speed_mps;
 snapshot_.control.crossing_execution_duration_s = crossing.execution_duration_s;
 snapshot_.control.crossing_execution_min_gap_m = crossing.execution_min_gap_m;
 snapshot_.control.crossing_execution_required_gap_m = crossing.execution_required_gap_m;
 // Commitment locks a minimum-progress contract, while the ordinary route
 // and corner caps remain authoritative. The crossing assessment above uses
 // that same cruise limit, so it cannot grant admission for a faster motion
 // than the controller will execute.
 if (traffic_decision.control_authority && crossing.relevant) {
  mapped_crossing_relevant = true;
  mapped_governor = sim::GovernorOutput{
    traffic_decision.control.hold_active,
    traffic_decision.control.stop_point,
 traffic_decision.control.stop_direction,
    traffic_decision.control.stop_valid,
  };
 }

 {
  // Wait slack = the computer's own prediction horizon (existing content, not
  // a new knob): waits beyond what the computer predicts are unassessable by
  // construction (kHorizonShort) — which is itself shadow data.
  const auto assessment =
    robot_application_->assess_departure(shadow_remaining, 0, shadow_cruise, assessment_frames, prediction_step_s,
 robot_definition_.collision_radius_m, preferred_clearance);
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
  const auto search = robot_application_->sample_pockets(
 local_planning_map, shadow_remaining, {nav_x, nav_y}, nav_heading, shadow_cruise,
    sim::ChassisExecCaps{chassis_caps_.can_spin, chassis_caps_.max_yaw_rate_radps,
                         chassis_caps_.geometry.steering_wheel_base_m, chassis_caps_.geometry.max_steer_rad,
                         chassis_caps_.geometry.max_steer_rate_radps,
                         chassis_caps_.geometry.steering_response_s},
    prediction_horizon, pocket_params, assessment_frames, prediction_step_s,
 robot_definition_.collision_radius_m, preferred_clearance);
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
 // Entry governor fallback (Phase-2, Behavior one-sided brake): fresh perceived tracks
 // feed the window shadow; Occupied engages HOLD_AT now, sustained Clear or
 // prolonged silence releases. Every perceived track is safety-relevant here:
 // tracker object IDs are sensor-owned identities, not indices into simulation
 // actors, and game collision policy must not leak into autonomy. Stopped ego
 // is assessed at a 0.5 m/s departure floor
 // (conservative: slower than any cruise, so the window is pessimistic).
 // Stop geometry uses PREDICTED threat occupancy (all forecast frames), not
 // current positions: parking next to the incoming threat's future path is
 // how the first wiring collided on signal_hold (39.4s hold -> rammed while
 // stopped). The stop point must clear the whole swept band.
 sim::GovernorOutput governor_out;
 sim::GovernorOutput stop_plane_governor;
 bool stop_plane_governor_available = false;
 bool governor_uses_mapped_geometry = false;
 std::optional<double> local_target_heading;
 bool local_stop_plane_active = false;
 const bool crossing_reverse_recovery =
 traffic_decision.control_authority &&
 traffic_decision.control.recovery_to_stop &&
 traffic_decision.control.recovery_reverse;
 const bool crossing_committed =
 traffic_decision.control_authority && traffic_decision.control.committed;
 bool crossing_stop_pose_reverse = false;
 // Always evaluate the map-independent conflict path. A mapped behavior may
 // own the current stop/commit contract, but its presence must not disable
 // ROBOT's generic perception-to-trajectory conflict detector elsewhere.
 const auto entry_batch = robot_application_->predict_dynamic_objects(
 snapshot_.time_s, prediction_horizon, prediction_step_s,
 sim::kEntryStaleBudgetS);
 const auto entry_frames = robot_application_->flatten_prediction(entry_batch);
 sim::EntryWindowRequest entry_request;
 entry_request.route = shadow_remaining;
 entry_request.cruise_speed_mps = shadow_cruise;
 entry_request.predicted_frames = entry_frames;
 entry_request.frame_dt_s = prediction_step_s;
 entry_request.ego_radius_m = robot_definition_.collision_radius_m;
 entry_request.clearance_m = hard_crossing_clearance;
 entry_request.braking_deceleration_mps2 =
 resolved_build_.traction().brake_decel_mps2;
 const bool generic_was_hold = robot_application_->entry_governor_holding();
 const auto generic_entry =
 robot_application_->assess_entry_window(entry_request);
 if (mapped_governor && !crossing_committed &&
 !traffic_decision.control.no_return_started) {
 if (mapped_governor->hold_active && mapped_governor->stop_valid) {
 // The mapped behavior wall remains a no-entry boundary even when the
 // generic dynamic assessment proves that waiting at the current pose is
 // unsafe. In that case the local planner may search for an upstream or
 // lateral minimum-risk escape, but it may not turn that emergency into an
 // unadmitted crossing. One merged GovernorOutput cannot represent both
 // facts, so retain the behavior wall as an independent validator input.
 stop_plane_governor = *mapped_governor;
 stop_plane_governor_available = true;
 }
 // Before the no-return boundary, mapped rules and map-independent online
 // conflicts are an intersection of one-sided constraints. A semantic zone
 // cannot suppress a temporary object outside that zone, and the generic
 // detector cannot grant permission over a mapped stop/yield rule.
 governor_out = robot_application_->merge_entry_constraints(
 *mapped_governor, generic_entry.control, shadow_remaining);
 governor_uses_mapped_geometry =
 mapped_governor->hold_active && governor_out.stop_valid &&
 std::hypot(governor_out.stop_point.x - mapped_governor->stop_point.x,
 governor_out.stop_point.y - mapped_governor->stop_point.y) <= 1e-9;
  snapshot_.control.governor_hold = governor_out.hold_active;
  snapshot_.control.governor_stop = governor_out.stop_point;
 if (governor_out.hold_active &&
 ((governor_uses_mapped_geometry && traffic_decision.phase_changed) ||
 (!governor_uses_mapped_geometry && !generic_was_hold)))
   record_event("governor_hold", catalog_.tr("event.governor_hold"));
 } else if (mapped_governor) {
 // A committed maneuver is already governed by its timed execution proof.
 // Reintroducing the generic entry wall here creates contradictory authority
 // (Committed and HOLD in the same cycle). Fresh dynamic forecasts still pass
 // through the independent local validator; before no-return, rejection feeds
 // a controlled stop back to the crossing state machine, and after no-return
 // only a fully validated continuation or minimum-risk contingency may run.
 governor_out = *mapped_governor;
 if (mapped_governor->hold_active && mapped_governor->stop_valid) {
 stop_plane_governor = *mapped_governor;
 stop_plane_governor_available = true;
 }
 governor_uses_mapped_geometry = true;
 snapshot_.control.governor_hold = governor_out.hold_active;
 snapshot_.control.governor_stop = governor_out.stop_point;
 } else {
 // Map-independent dynamic interaction. ROBOT compares the executable route
 // prefix with perception-owned time occupancy and derives a temporary hold
 // point when they conflict. Authored crossing semantics are optional context,
 // not a prerequisite for avoiding a temporary participant.
 governor_out = generic_entry.control;
 snapshot_.control.governor_hold = governor_out.hold_active;
 snapshot_.control.governor_stop = governor_out.stop_point;
 if (governor_out.hold_active && !generic_was_hold)
 record_event("governor_hold", catalog_.tr("event.governor_hold"));
 }
 if (!stop_plane_governor_available && governor_out.hold_active &&
 governor_out.stop_valid) {
 stop_plane_governor = governor_out;
 stop_plane_governor_available = true;
 }
 // Virtual wall (Phase-2 consumption): while the governor holds, cap speed
 // by stopping physics to the stop point. The local planner also receives
 // the point and plane below, so steering converges to the proven hold pose
 // instead of continuing to aim beyond a speed-only wall.
 if (governor_out.hold_active && governor_out.stop_valid) {
 const double brake = resolved_build_.traction().brake_decel_mps2;
 double dist = std::hypot(nav_x - governor_out.stop_point.x, nav_y - governor_out.stop_point.y);
 if (governor_uses_mapped_geometry) {
 dist = crossing.stop_distance_m;
 const double local_stop_activation_m = std::max(
 2.0, speed_cap * prediction_horizon + property(behavior_, "lookahead_m"));
 local_stop_plane_active = dist <= local_stop_activation_m;
 if (local_stop_plane_active) {
 target = governor_out.stop_point;
 snapshot_.control.local_target = target;
 // A differential chassis cannot correct lateral stop-line error while it
 // Differential kinematics are referenced at the driven axle. RO-1's axle
 // is offset from base_link, so controlling the body origin as if it were the
 // instantaneous rotation centre creates a circular chase during pure yaw.
 const auto pose_approach = robot_application_->crossing_pose_approach(
 {nav_x, nav_y}, nav_heading, governor_out.stop_point,
 governor_out.stop_direction, crossing_reverse_recovery,
 chassis_caps_.geometry.drive_axle_x_m,
 sim::kCrossingPositionTrackingToleranceM);
 if (pose_approach.control_position_acquired) {
 local_target_heading = std::atan2(
 governor_out.stop_direction.y, governor_out.stop_direction.x);
 // Position readiness closes the translational part of the hold contract.
 // Keep only the in-place heading correction active until behavior grants a
 // new crossing commitment. Reapplying the route speed cap here makes the
 // robot drive through and orbit the stop pose while correctly waiting for
 // a safe traffic window.
 speed_cap = 0.0;
 route_control.tv = 0.0;
 } else {
 // The behavior stop pose temporarily supersedes the ordinary route
 // lookahead. Its approach speed must therefore be derived from that pose,
 // not from a route follower that may currently be in a spin/reverse recovery
 // aimed at a waypoint beyond the virtual wall. Collision and stop-plane
 // validation below remain authoritative.
 crossing_stop_pose_reverse = pose_approach.reverse;
 const double stop_heading_error = std::remainder(
 pose_approach.body_heading_rad - nav_heading, 2.0 * sim::kPi);
 const double stop_distance = pose_approach.control_distance_m;
 route_control.err = stop_heading_error;
 route_control.tv = (crossing_stop_pose_reverse ? -1.0 : 1.0) *
 std::min(chassis_cruise, stop_distance);
 // A spin-capable differential platform reaches a pose robustly in three
 // stages: face the point, translate to it, then align with the crossing.
 // Allowing translation through a large bearing error makes constant-twist
 // rollouts orbit the stop point and can move laterally toward traffic even
 // though their stop-plane projection remains behind the virtual wall.
 local_target_heading = pose_approach.body_heading_rad;
 // Stop-pose recovery is a rotate-then-translate controller. Near the
 // virtual plane, a loose heading gate lets every translating rollout cross
 // the plane before it can correct lateral error, so the validator correctly
 // rejects all motion. Use the same tight initial-heading contract that the
 // crossing trajectory itself proves.
 constexpr double kStopApproachHeadingToleranceRad =
 sim::kCrossingInitialHeadingToleranceRad;
 const bool align_before_approach = chassis_caps_.can_rotate_about_base_link() &&
 std::abs(stop_heading_error) > kStopApproachHeadingToleranceRad;
 const double stop_approach_cap = std::min(
 std::max(0.20, robot_application_->scheduled_speed(stop_heading_error, chassis_cruise)),
 corner_speed_cap(nav_x, nav_y)) * snapshot_.strategy.applied.speed_scale;
 const bool stationary_hold_safe =
 stationary_forecast_clearance.conservative_m + 1e-9 >=
 hard_crossing_clearance;
 speed_cap = align_before_approach && stationary_hold_safe
 ? 0.0
 : std::min(stop_approach_cap, std::abs(route_control.tv));
 // Rotate-then-translate is the preferred pose controller while holding is
 // safe. If a predicted actor will invade the stationary footprint, expose
 // bounded translation as well and let the shared trajectory validator pick
 // a same-time minimum-risk arc. A hard zero-speed alignment in that case
 // creates an avoidable collision/deadlock.
 if (align_before_approach && stationary_hold_safe)
 route_control.tv = 0.0;
 snapshot_.control.heading_error_rad = stop_heading_error;
 }
 }
 } else {
 // An inferred conflict has no authored stop pose. Its ROBOT-derived wall
 // still enters the same swept stop-plane validator as mapped behavior; a
 // scalar speed cap alone can be crossed by response lag or steering motion.
 const double local_stop_activation_m = std::max(
 2.0, speed_cap * prediction_horizon + property(behavior_, "lookahead_m"));
 local_stop_plane_active = dist <= local_stop_activation_m;
 if (local_stop_plane_active) {
 target = governor_out.stop_point;
 snapshot_.control.local_target = target;
 local_target_heading = std::atan2(
 governor_out.stop_direction.y, governor_out.stop_direction.x);
 }
 }
 // Include decision and actuator response before ideal constant braking.
 // Solving v*t_delay + v^2/(2a) <= distance prevents a correct geometric
 // stop point from being crossed by a lagging speed loop.
 const auto* motor = resolved_build_.motor();
 const double bandwidth = motor
 ? motor->controller_bandwidth_radps
 : robot_definition_.motor.controller_bandwidth_radps;
 const double response_delay_s = 1.0 / decision_hz() +
 1.0 / std::max(0.1, bandwidth);
 // Route progress can be zero while the body is still laterally displaced
 // from the stop pose. Preserve a bounded corrective approach; the swept
 // stop-plane validator below still prevents entry into the conflict zone.
 if (governor_uses_mapped_geometry &&
 !traffic_decision.readiness.position_ready) {
 dist = std::max(dist, distance({nav_x, nav_y}, governor_out.stop_point));
 }
 const double v_allow = dist <= 1e-9 || brake <= 1e-9
 ? 0.0
 : brake * (std::sqrt(response_delay_s * response_delay_s +
 2.0 * dist / brake) - response_delay_s);
  speed_cap = std::min(speed_cap, v_allow);
 }
 const auto route_direction = robot_application_->route_travel_direction(planned_route_);
 // Gear is owned by the global route for every chassis. The follower's
 // signed command is only a fallback once the route has no direction metadata
 // (for example after the terminal index); it may not silently reverse a
 // forward section to improve a local score.
 const bool reverse_route = crossing_reverse_recovery ||
 crossing_stop_pose_reverse ||
 route_direction == sim::TravelDirection::Reverse ||
 (route_direction == sim::TravelDirection::Unspecified && route_control.tv < 0.0);
 if (crossing_committed && traffic_decision.control.exit_valid) {
 // The committed crossing owns a complete route to a proved refuge. The
 // ordinary route follower may still expose a short lookahead waypoint near
 // the stop line; keeping that stale point as DWA's goal makes the local
 // planner brake or spin immediately after admission. Drive toward the
 // latched exit while the spatial corridor and dynamic validator constrain
 // every published command.
 // The same ownership applies to longitudinal intent. A grid follower can
 // report zero at a raster corner that behavior deliberately collapsed out
 // of the crossing maneuver. Rebuild the cap from the admitted transit speed;
 // actuator reachability and every independent validator remain downstream.
 speed_cap = std::min(
 traffic_decision.control.transit_speed_mps,
 crossing_platform_speed_mps);
 const auto& execution = traffic_decision.control.execution_trajectory;
 const bool continuous_execution =
 traffic_decision.control.execution_continuous;
 target = traffic_decision.control.exit_point;
 std::vector<sim::TimedPose> execution_path;
 for (const auto& pose : execution) {
 if (!execution_path.empty() &&
 distance(execution_path.back().p, pose.p) <= 1e-6) {
 // A repeated position is a planned dwell (typically stop-turn-go).
 // Retain its departure time so elapsed-time advancement cannot leave the
 // corner before the admitted yaw maneuver has completed.
 execution_path.back().t_s = std::max(execution_path.back().t_s, pose.t_s);
 continue;
 }
 while (execution_path.size() >= 2) {
 const auto a = execution_path[execution_path.size() - 2].p;
 const auto b = execution_path.back().p;
 const double first = std::atan2(b.y - a.y, b.x - a.x);
 const double second = std::atan2(pose.p.y - b.y, pose.p.x - b.x);
 if (std::abs(std::remainder(second - first, 2.0 * sim::kPi)) > 1e-3)
 break;
 execution_path.pop_back();
 }
 execution_path.push_back(pose);
 }
 if (traffic_decision.phase_changed) {
 crossing_execution_segment() = 0;
 crossing_alignment_segment() = 0;
 crossing_segment_alignment_complete() = false;
 }
 if (execution_path.size() >= 2) {
 crossing_execution_segment() = std::min(
 crossing_execution_segment(), execution_path.size() - 2);
 const double waypoint_tolerance_m = std::max(
 sim::kCrossingPositionTrackingToleranceM,
 traffic_decision.control.tracking_corridor_m);
 const bool initial_alignment_pending =
 traffic_decision.control.execution_alignment_required &&
 !traffic_decision.control.execution_alignment_complete;
 const auto execution_waypoint_reached = [&](std::size_t segment) {
 const auto from = execution_path[segment].p;
 const auto to = execution_path[segment + 1].p;
 if (distance({nav_x, nav_y}, to) <= waypoint_tolerance_m)
 return true;
 if (!continuous_execution)
 return false;
 const sim::Vec2 delta{to.x - from.x, to.y - from.y};
 const double length_m = std::hypot(delta.x, delta.y);
 if (length_m <= 1e-9)
 return true;
 const double progress_m =
 ((nav_x - from.x) * delta.x + (nav_y - from.y) * delta.y) /
 length_m;
 return progress_m >= length_m;
 };
 // A grid route may put its first vertex inside the ordinary waypoint
 // tolerance. Do not skip that leg before completing the heading contract
 // that behavior proved for it; otherwise the executor turns toward the
 // second leg while behavior waits forever for the first heading.
 while (!initial_alignment_pending &&
 crossing_execution_segment() + 2 < execution_path.size() &&
 execution_waypoint_reached(crossing_execution_segment()) &&
 (continuous_execution ||
 std::abs(estimate.linear_velocity_mps) <= 0.10)) {
 // Planned time is a safety schedule, never proof that the physical body
 // reached a waypoint. Advancing solely because its timestamp elapsed made
 // a lagging base accelerate toward the next leg before completing the
 // required stop, then leave the admitted tracking tube. The timed
 // validator handles early/late safety; spatial execution advances only on
 // measured arrival and settling.
 ++crossing_execution_segment();
 if (!continuous_execution) {
 crossing_alignment_segment() = crossing_execution_segment();
 crossing_segment_alignment_complete() = false;
 }
 }
 const auto segment_start = execution_path[crossing_execution_segment()].p;
 target = execution_path[crossing_execution_segment() + 1].p;
 const double segment_heading = std::atan2(
 target.y - segment_start.y, target.x - segment_start.x);
 // A crossing leg is a pose-constrained stop-turn-go maneuver, not merely a
 // point target. Preserve its body-heading contract throughout translation.
 // Near a point goal the distance critic is otherwise indifferent to yaw;
 // DWA can then select a sharp turn just before the corner even though the
 // behavior proof requires arriving at rest along the current leg.
 local_target_heading = segment_heading;
 const double heading_error = std::abs(std::remainder(
 segment_heading - nav_heading, 2.0 * sim::kPi));
 // Re-align whenever a safety stop or minimum-risk escape has rotated the
 // differential body away from the admitted segment. Restricting this stage
 // to the exact segment start deadlocks a displaced body: every forced
 // forward rollout leaves the proved corridor, while the needed in-place
 // turn is never generated. The local validator still checks the stationary
 // footprint and dynamic forecast before publishing the turn.
 if (!continuous_execution &&
 crossing_alignment_segment() != crossing_execution_segment()) {
 crossing_alignment_segment() = crossing_execution_segment();
 crossing_segment_alignment_complete() = false;
 }
 if (!crossing_segment_alignment_complete() &&
 heading_error <= sim::kCrossingInitialHeadingToleranceRad) {
 crossing_segment_alignment_complete() = true;
 }
 // Segment alignment is a one-shot execution stage. Encoder lag just after
 // translation starts must not re-arm an in-place turn and alternate the
 // speed cap between zero and cruise on successive replans.
 const bool align_segment =
 (traffic_decision.control.execution_alignment_required &&
 !traffic_decision.control.execution_alignment_complete) ||
 (!continuous_execution && !crossing_segment_alignment_complete());
 if (align_segment) {
 speed_cap = 0.0;
 local_target_heading = segment_heading;
 } else {
 const auto* crossing_motor = resolved_build_.motor();
 const double crossing_bandwidth = crossing_motor
 ? crossing_motor->controller_bandwidth_radps
 : robot_definition_.motor.controller_bandwidth_radps;
 const double response_delay_s = 1.0 / decision_hz() +
 1.0 / std::max(0.1, crossing_bandwidth);
 const double brake = resolved_build_.traction().brake_decel_mps2;
 const double segment_length_m = std::max(
 1e-9, distance(segment_start, target));
 const sim::Vec2 segment_direction{
 (target.x - segment_start.x) / segment_length_m,
 (target.y - segment_start.y) / segment_length_m,
 };
 const double remaining_m = std::max(
 0.0,
 (target.x - nav_x) * segment_direction.x +
 (target.y - nav_y) * segment_direction.y);
 const bool final_crossing_segment =
 continuous_execution ||
 crossing_execution_segment() + 2 == execution_path.size();
 // Intermediate corners are explicit stop-turn-go poses. The final point is
 // a conflict-clear boundary: crossing control ends there and the canonical
 // route continues, so braking to zero would implement a different maneuver
 // from the behavior trajectory used for admission.
 if (!final_crossing_segment) {
 const double v_allow = remaining_m <= 1e-9
 ? 0.0
 : brake * (std::sqrt(response_delay_s * response_delay_s +
 2.0 * remaining_m / brake) - response_delay_s);
 speed_cap = std::min(speed_cap, v_allow);
 }
 if (continuous_execution) {
 const double planned_dt_s =
 execution_path[crossing_execution_segment() + 1].t_s -
 execution_path[crossing_execution_segment()].t_s;
 if (planned_dt_s > 1e-6) {
 speed_cap = std::min(speed_cap,
 segment_length_m / planned_dt_s);
 }
 // Follow a point ahead on the complete curve rather than aiming at the
 // next dense arc sample. A next-sample target can lie only centimetres
 // away, causing DWA to delay steering until it has overshot the curve.
 double lookahead_m = std::clamp(speed_cap * 0.8, 0.40, 0.80);
 const double projected_progress_m = std::clamp(
 (nav_x - segment_start.x) * segment_direction.x +
 (nav_y - segment_start.y) * segment_direction.y,
 0.0, segment_length_m);
 sim::Vec2 cursor{
 segment_start.x + projected_progress_m * segment_direction.x,
 segment_start.y + projected_progress_m * segment_direction.y,
 };
 std::size_t look_segment = crossing_execution_segment();
 while (look_segment + 1 < execution_path.size()) {
 const auto end = execution_path[look_segment + 1].p;
 const double available_m = distance(cursor, end);
 if (available_m >= lookahead_m - 1e-9) {
 const double blend = available_m <= 1e-9 ? 0.0 :
 lookahead_m / available_m;
 target = {
 cursor.x + (end.x - cursor.x) * blend,
 cursor.y + (end.y - cursor.y) * blend,
 };
 local_target_heading = std::atan2(end.y - cursor.y,
 end.x - cursor.x);
 break;
 }
 lookahead_m -= available_m;
 cursor = end;
 ++look_segment;
 target = cursor;
 if (look_segment + 1 < execution_path.size()) {
 const auto next = execution_path[look_segment + 1].p;
 local_target_heading = std::atan2(next.y - cursor.y,
 next.x - cursor.x);
 }

 // A rolling curve tangent is path guidance, not a terminal body-pose
 // constraint. Passing it as target_heading_rad makes a constant-command
 // DWA rollout try to finish its whole horizon at the near-field tangent;
 // that suppresses the yaw-rate increase needed by the upcoming curve.
 // The reference-path critic already evaluates the rollout's projected
 // tangent at its endpoint and is the correct receding-horizon contract.
 local_target_heading.reset();
 }
 }
 // DWA is a receding-horizon tracker. Giving it only the distant refuge
 // lets a constant-yaw rollout match the final heading with a broad arc that
 // slowly leaves a narrow proved tube. Prune the current straight leg to a
 // rolling carrot while retaining the leg endpoint for braking and waypoint
 // completion above.
if (!continuous_execution && remaining_m > 0.5) {
 const double projected_progress_m = std::clamp(
 (nav_x - segment_start.x) * segment_direction.x +
 (nav_y - segment_start.y) * segment_direction.y,
 0.0, segment_length_m);
 const double local_lookahead_m = std::clamp(speed_cap, 0.5, 1.0);
 const double carrot_progress_m = std::min(
 segment_length_m, projected_progress_m + local_lookahead_m);
 target = {
 segment_start.x + carrot_progress_m * segment_direction.x,
 segment_start.y + carrot_progress_m * segment_direction.y,
 };
 }
 }
 }
 // A merged invalid hold means stationary occupancy is unsafe; it does not
 // erase a separately valid crossing boundary. Activate that boundary for
 // the same local preview distance used by ordinary virtual walls so every
 // recovery candidate remains on the uncommitted side.
 if (!crossing_committed && stop_plane_governor_available &&
 !traffic_decision.control.relocated_hold) {
 const double stop_plane_distance_m = std::hypot(
 nav_x - stop_plane_governor.stop_point.x,
 nav_y - stop_plane_governor.stop_point.y);
 const double local_stop_activation_m = std::max(
 2.0, speed_cap * prediction_horizon + property(behavior_, "lookahead_m"));
 local_stop_plane_active = local_stop_plane_active ||
 stop_plane_distance_m <= local_stop_activation_m;
 }
 snapshot_.control.local_target = target;
 }
 const double kCommittedProgressFloorMps = execution_policy.committed_progress_floor_mps;
 // The admitted trajectory may begin with an in-place alignment while the
 // body is still at the safe stop pose. Keep the floor disabled during that
 // stage, then require actuator-reachable progress as soon as alignment is
 // complete. Waiting until physical conflict entry lets a zero-speed DWA
 // candidate consume the complete admitted traffic window before moving.
 const double committed_floor = crossing_committed &&
 traffic_decision.control.execution_alignment_complete
 ? std::min(speed_cap, kCommittedProgressFloorMps)
 : 0.0;
 // A zero-speed recovery on a spin-capable chassis is an explicit pose
 // maneuver. Give the local planner that body-heading goal so the path-
 // tangent critic cannot reward holding the old yaw forever after waypoint
 // progress has stopped. Ackermann remains governed by its route/gear model.
 if (!local_target_heading && !local_stop_plane_active &&
 chassis_caps_.can_rotate_about_base_link() &&
 std::abs(route_control.tv) <= 1e-9 && std::abs(route_control.err) > 0.05) {
 local_target_heading = std::atan2(
 target.y - nav_y, target.x - nav_x);
 }
 sim::LocalPlanningRequest local_request;
 switch (chassis_caps_.command_space) {
 case sim::ChassisCommandSpace::YawRate:
 local_request.command_space = sim::LocalCommandSpace::DifferentialTwist;
 break;
 case sim::ChassisCommandSpace::SteeringAngle:
 local_request.command_space = sim::LocalCommandSpace::SteeringAngle;
 break;
 case sim::ChassisCommandSpace::HolonomicTwist:
 local_request.command_space = sim::LocalCommandSpace::HolonomicTwist;
 break;
 }
 local_request.position = {nav_x, nav_y};
 local_request.heading_rad = nav_heading;
 local_request.time_s = snapshot_.time_s;
 const bool has_map_localization = level_.localization_mode == "known_map";
 // `localization_confidence` describes map-frame correction health. An
 // odometry-only level intentionally has no such stream; use the estimator's
 // wheel/IMU confidence there instead of treating the absent map correction as
 // a lost pose and freezing the teaching drive.
 local_request.localization_confidence = has_map_localization
 ? estimate.localization_confidence : estimate.confidence;
 local_request.localization_valid = has_map_localization
 ? estimate.localization_valid : std::isfinite(estimate.confidence) && estimate.confidence > 0.0;
 // In odometry-only teaching levels the estimator intentionally has no map
 // correction stream. Its map-fix age is therefore not a stale-sensor signal;
 // the local planner must use the wheel/IMU estimate rather than freezing the
 // vehicle after two seconds. Known-map levels pass the correction age.
 local_request.localization_age_s = has_map_localization
 ? estimate.localization_age_s : 0.0;
 // Encoder and fused-pose filters retain millimetres-per-second residuals
 // after the physical wheels have stopped. Feeding that noise into the
 // rollout makes every in-place correction drift microscopically through an
 // exact behavior stop plane, so the safety validator can never authorize a
 // recovery. Apply a narrow planning deadband only when the encoder common
 // mode reports no translation; differential tread motion remains available
 // to the yaw-reachability model.
 const double kPlanningRestDeadbandMps = execution_policy.planning_rest_deadband_mps;
 const double measured_encoder_forward_mps =
 0.5 * (measured_left_mps + measured_right_mps);
 // Opposite tread motion is an in-place turn, so its common mode is the
 // longitudinal observation. Requiring both treads to stop leaves fused-pose
 // centimetre noise looking like real forward/reverse travel during a pivot.
 const double kPlanningPoseVelocityDeadbandMps =
 execution_policy.planning_pose_velocity_deadband_mps;
 const bool encoder_translation_stopped =
 std::abs(measured_encoder_forward_mps) < kPlanningRestDeadbandMps;
 local_request.forward_mps = encoder_translation_stopped &&
 std::abs(estimate.linear_velocity_mps) < kPlanningPoseVelocityDeadbandMps
 ? 0.0
 : estimate.linear_velocity_mps;
 local_request.lateral_mps = estimate.lateral_velocity_mps;
 local_request.yaw_rate_radps = estimate.angular_velocity_radps;
 local_request.steering_angle_rad = chassis_->telemetry().steer.angle_rad;
 local_request.steering_rate_radps = chassis_->telemetry().steer.rate_radps;
 local_request.target = target;
 local_request.target_heading_rad = local_target_heading;
 // A route lookahead and a final destination are different contracts. A
 // point-and-go operator command requires a physical stop at its selected
 // point; low-speed authored goals also expose their arrival requirement.
 // Keeping this explicit lets the deployable holonomic planner perform fine
 // signed corrections without learning simulator markers or game actions.
 const auto& field_intent = field_operator_system_.intent();
 if ((endless_mode_ || level_.control_mode == "operator_drive") &&
 field_intent.active && field_intent.kind != FieldIntentKind::Follow &&
 field_intent.kind != FieldIntentKind::Wait) {
 local_request.terminal_position = field_intent.target;
 local_request.terminal_position_tolerance_m =
 kFieldIntentArrivalPositionToleranceM;
 local_request.terminal_max_speed_mps = kFieldIntentArrivalSpeedMps;
 local_request.terminal_heading_tolerance_rad =
 kFieldIntentArrivalYawToleranceRad;
 local_request.terminal_max_yaw_rate_radps =
 kFieldIntentArrivalYawRateRadps;
 if (!planned_route_.empty())
 local_request.target_heading_rad = planned_route_.back().heading_rad;
 } else if (const GoalStep* goal = active_goal();
 goal && (goal->action == "inspect" ||
 level_.objective.max_arrival_speed_mps < chassis_cruise)) {
 const auto& marker = map_definition_.markers.at(goal->marker);
 local_request.terminal_position = as_vec(marker);
 local_request.terminal_position_tolerance_m = goal->radius_m;
 local_request.terminal_max_speed_mps = goal->action == "inspect"
 ? 0.18
 : level_.objective.max_arrival_speed_mps;
 if (local_request.command_space == sim::LocalCommandSpace::HolonomicTwist)
 local_request.target_heading_rad = marker.theta;
 }
 local_request.weights = weights;
 local_request.max_speed_mps = speed_cap;
 // Preserve a behavior-independent motion budget for the safety planner.
 // A stop line or yield decision can set max_speed_mps to zero, but waiting
 // is not safe when a predicted participant will enter the stationary
 // footprint. This cap comes from the ordinary chassis/terrain speed limit;
 // it is not actor truth or a scenario-specific escape command.
 local_request.collision_escape_max_speed_mps =
 route_independent_speed_cap * snapshot_.strategy.applied.speed_scale;
 local_request.max_yaw_rate_radps = yaw_cap_rads_;
 sim::SafeStopLimits current_stop_limits;
 current_stop_limits.brake_decel_mps2 =
 resolved_build_.traction().brake_decel_mps2;
 current_stop_limits.longitudinal_grade =
 robot_application_->longitudinal_grade(strategy_terrain, grade_heading);
 current_stop_limits.center_of_mass_x_m = combined_center_x_m();
 current_stop_limits.center_of_mass_height_m = combined_center_height_m();
 const double safe_stop_decel_mps2 = robot_application_->safe_stop_deceleration_limit(
 chassis_caps_, current_stop_limits);
 // Candidate generation uses the normal drive/brake actuator envelope. The
 // separately modelled emergency tail uses the stricter stability-aware stop
 // limit below; conflating the two would unnecessarily cap ordinary forward
 // acceleration by the smaller of the front and rear support margins.
 local_request.max_accel_mps2 = std::min(
 robot_definition_.motor.max_wheel_accel_mps2,
 resolved_build_.traction().brake_decel_mps2);
 local_request.max_decel_mps2 = resolved_build_.traction().brake_decel_mps2;
 local_request.max_yaw_accel_radps2 = robot_definition_.max_yaw_accel_radps2;
 local_request.prediction_horizon_s = prediction_horizon;
 local_request.step_s = prediction_step_s;
 // Validation enforces the authored safety threshold plus sensing motion.
 // The driving style's larger value remains a comfort preference and may
 // influence scoring, but it cannot turn an otherwise safe periodic corridor
 // into a permanent hard stop.
 local_request.required_clearance_m = hard_crossing_clearance;
 local_request.robot_radius_m = robot_definition_.collision_radius_m;
 local_request.differential_track_m = robot_definition_.drive_type == "tracked"        ? robot_definition_.effective_track_width_m
   : robot_definition_.drive_type == "differential"
 ? robot_definition_.wheel_base_m
 : 0.0;
 local_request.drive_axle_x_m = chassis_caps_.geometry.drive_axle_x_m;
 local_request.max_wheel_speed_mps = max_forward_speed_mps_;
 if (local_request.differential_track_m > 0.0) {
 local_request.measured_wheel_speeds = sim::DifferentialWheelSpeeds{
 measured_left_mps,
 measured_right_mps,
 };
 }
 local_request.steering_wheel_base_m = chassis_caps_.geometry.steering_wheel_base_m;
 local_request.max_steer_rad = chassis_caps_.geometry.max_steer_rad;
 local_request.max_steer_rate_radps = chassis_caps_.geometry.max_steer_rate_radps;
 local_request.steering_response_s = chassis_caps_.geometry.steering_response_s;
 local_request.support_width_m = chassis_caps_.geometry.support_width_m;
 local_request.support_polygon = chassis_caps_.geometry.support_polygon;
 local_request.center_of_mass_x_m = combined_center_x_m();
 local_request.center_of_mass_height_m = combined_center_height_m();
 local_request.max_dynamic_rollover_risk = execution_policy.dynamic_rollover_risk_limit;
 local_request.preferred_direction = route_direction;
 local_request.route_is_reverse = reverse_route;
 local_request.crossing_relevant = mapped_crossing_relevant;
 local_request.crossing_committed = crossing_committed;
 local_request.committed_no_return =
 traffic_decision.control.no_return_started;
 local_request.stop_plane_active = local_stop_plane_active &&
 stop_plane_governor_available &&
 !crossing_committed && !traffic_decision.control.relocated_hold;
 local_request.stop_point = stop_plane_governor_available
 ? stop_plane_governor.stop_point
 : governor_out.stop_point;
 local_request.stop_direction = stop_plane_governor_available
 ? stop_plane_governor.stop_direction
 : governor_out.stop_direction;
 local_request.stop_tolerance_m = stop_plane_tracking_tolerance_m;
 local_request.committed_progress_floor_mps = committed_floor;
 if (crossing_committed) {
 local_request.committed_tracking_corridor_m =
 traffic_decision.control.tracking_corridor_m;
 // Admission explicitly swept the behavior schedule over
 // +/- kCrossingTimeMarginS. Match measured progress to a phase inside that
 // already-proved band, then keep that phase for the candidate interval and
 // remaining-trajectory recheck. Always choosing the slow edge incorrectly
 // rejected a platform that accelerated ahead of that edge; always choosing
 // nominal time incorrectly rejected ordinary actuator lag.
 const double matched_execution_phase_s =
 robot_application_->match_timed_trajectory_phase(
 traffic_decision.control.execution_trajectory, {nav_x, nav_y},
 traffic_decision.control.execution_elapsed_s,
 sim::kCrossingTimeMarginS);
 local_request.committed_reference_trajectory =
 robot_application_->remaining_timed_trajectory(
 traffic_decision.control.execution_trajectory,
 matched_execution_phase_s);
 local_request.committed_reference_path.reserve(
 traffic_decision.control.execution_trajectory.size());
 for (const auto& pose : traffic_decision.control.execution_trajectory)
 local_request.committed_reference_path.push_back(pose.p);
 }
 // The stopping tail is based on the delivered controller bandwidth and the
 // tire/contact deceleration contract. It applies to actuator-level wheeled
 // and tracked plants; the trajectory validator then checks the resulting
 // swept path in the chassis' own command space.
 if (chassis_->accepts_actuator_duty()) {
 const auto* local_motor = resolved_build_.motor();
 const double local_controller_bandwidth = local_motor
 ? local_motor->controller_bandwidth_radps
 : robot_definition_.motor.controller_bandwidth_radps;
 local_request.emergency_stop_required = true;
 local_request.actuator_response_delay_s =
 1.0 / std::max(0.1, local_controller_bandwidth);
 local_request.emergency_stop_delay_s = 1.0 / decision_hz() +
 local_request.actuator_response_delay_s;
 local_request.emergency_decel_mps2 = safe_stop_decel_mps2;
 const double brake_decel_mps2 = resolved_build_.traction().brake_decel_mps2;
 const double stop_center_x_m = combined_center_x_m();
 const double stop_center_height_m = combined_center_height_m();
 local_request.safe_stop_decel =
 [this, brake_decel_mps2, stop_center_x_m, stop_center_height_m](
 sim::Vec2 position, double heading_rad) {
 sim::SafeStopLimits limits;
 limits.brake_decel_mps2 = brake_decel_mps2;
 limits.longitudinal_grade = robot_application_->longitudinal_grade(
 robot_application_->estimated_terrain(position), heading_rad);
 limits.center_of_mass_x_m = stop_center_x_m;
 limits.center_of_mass_height_m = stop_center_height_m;
 return robot_application_->safe_stop_deceleration_limit(chassis_caps_, limits);
 };
 }
 // Path/progress/oscillation critics are shared guidance for twist-space
 // planners, including tracked and holonomic chassis. A tracked plant still
 // includes scrub, while a holonomic plant may make progress laterally;
 // both need the global path to distinguish useful motion from a rollout
 // that merely points toward the goal through an obstacle.
 if (local_request.command_space != sim::LocalCommandSpace::SteeringAngle) {
 local_request.reference.previous_angular_command_radps = held_command().target_w;
 local_request.reference.path.push_back({nav_x, nav_y});
 if (crossing_committed && !local_request.committed_reference_path.empty()) {
 for (const auto& point : local_request.committed_reference_path) {
 if (distance(local_request.reference.path.back(), point) > 1e-6)
 local_request.reference.path.push_back(point);
 }
 } else if (local_request.stop_plane_active) {
 // A behavior hold inserts a temporary terminal pose into the local plan.
 // Referencing route vertices beyond the closed virtual wall asks the path
 // critic to continue through the crossing while the validator forbids it.
 // Truncate the local reference at the latched stop pose so all critics and
 // the behavior constraint describe the same maneuver.
 if (distance({nav_x, nav_y}, local_request.stop_point) > 1e-6)
 local_request.reference.path.push_back(local_request.stop_point);
 } else if (!planned_route_.empty()) {
 const std::size_t guide_index = std::min(
 robot_application_->route_path_index(), planned_route_.size() - 1);
 const double reference_length_m =
 speed_cap * prediction_horizon + property(behavior_, "lookahead_m");
 local_request.reference.path = robot_application_->forward_route_reference(
 planned_route_, guide_index, {nav_x, nav_y}, reference_length_m);
 }
 }
 const sim::autonomy::RobotLocalPlanningInput local_planning_input{
 &local_request, &local_planning_map, &local_predicted_frames,
 [this, physical_execution_margin_m](sim::Vec2 from, sim::Vec2 to) {
 const bool feasible = std::isfinite(navigation_traversal_cost(from, to)) &&
 robot_application_->static_motion_free(
 from, to, physical_execution_margin_m);
 return feasible;
 }, &local_physical_collision_map,
 [this, physical_execution_margin_m](sim::Vec2 from, sim::Vec2 to) {
 // The physical map owns observed obstacles, but its static base is authored
 // occupancy rather than a footprint-inflated raster. Keep the analytic
 // world/box collision check here as well; otherwise a recovery may leave
 // soft inflation by driving through a wall or across the world boundary.
 // The localization reserve is physical uncertainty, not the comfort/policy
 // envelope that this recovery is allowed to relax.
 const bool feasible = std::isfinite(navigation_traversal_cost(from, to)) &&
 (robot_application_->static_motion_free(
 from, to, physical_execution_margin_m) ||
 // A physical-map escape is entered only after the preferred inflation
 // envelope has stranded the measured pose. Permit a bounded, non-worsening
 // sweep through that reserve, while retaining exact footprint collision and
 // terrain checks; this is not a general planning relaxation.
 (robot_application_->static_motion_free(from, to, 0.0) &&
 robot_application_->static_clearance(to) + 0.01 >=
 robot_application_->static_clearance(from)));
 return feasible;
 }, [this](sim::Vec2 position) {
 return robot_application_->estimated_terrain(position);
 }};
 // The scenario owns only provider-neutral request assembly. ROBOT owns the
 // local planner invocation and trajectory validation through the common
 // application cycle; run_autonomy=false avoids updating the already applied
 // high-level strategy a second time in this decision tick.
 sim::robot::RobotCycleInput local_cycle;
 local_cycle.local_planning = &local_planning_input;
 local_cycle.run_autonomy = false;
 const auto local_cycle_output = robot_application_->tick(local_cycle);
 if (!local_cycle_output.local_planning)
 throw std::runtime_error("ROBOT application did not produce local planning output");
 const auto& local_plan = *local_cycle_output.local_planning;
 snapshot_.control.local_motion_mode = static_cast<int>(local_plan.status.mode);
 snapshot_.control.local_motion_authorized = local_plan.status.motion_authorized;
 snapshot_.control.local_motion_decision_time_s = local_plan.status.decision_time_s;
 snapshot_.control.local_motion_speed_cap_mps = local_plan.status.speed_cap_mps;
 snapshot_.control.local_motion_reason = local_plan.status.reason;
 snapshot_.control.local_motion_recovery_condition = local_plan.status.recovery_condition;
 const auto& dwa = local_plan.proposals;
 const auto& validated_selection = local_plan.selection;
 snapshot_.control.static_margin_recovery_active =
 local_plan.static_margin_recovery_active;
 snapshot_.control.static_margin_recovery_gear =
 local_plan.static_margin_recovery_gear;
 snapshot_.control.static_margin_escape_origin =
 local_plan.static_margin_escape_origin;
 snapshot_.control.static_margin_escape_heading_rad =
 local_plan.static_margin_escape_heading_rad;
 snapshot_.control.static_margin_escape_required_progress_m =
 local_plan.static_margin_escape_required_progress_m;
 snapshot_.control.static_margin_escape_progress_m =
 local_plan.static_margin_escape_progress_m;
 snapshot_.control.near_boundary_clearance_recovery_active =
 local_plan.near_boundary_clearance_recovery_active;
 snapshot_.control.dwa_active =
 local_request.command_space == sim::LocalCommandSpace::DifferentialTwist;
 snapshot_.control.dwa_proposed_v_mps = dwa.best.command.v;
 snapshot_.control.dwa_proposed_w_radps = dwa.best.command.w;
 snapshot_.control.dwa_accepted_candidates = validated_selection.accepted_candidates;
 const auto& selected_trajectory = validated_selection.trajectory;
 for (const auto& sample : selected_trajectory.samples)
 snapshot_.control.dwa_selected.push_back(sample.position);
 const auto& trajectory_validation = validated_selection.validation;
 // Local collision freedom is weaker than the time-indexed crossing proof.
 // Reassess an admitted schedule whenever its validated continuation is lost.
 robot_application_->report_crossing_execution(
 crossing_committed, trajectory_validation,
 snapshot_.control.trajectory_sequence, snapshot_.time_s);
 snapshot_.control.crossing_execution_blocked =
 robot_application_->crossing_execution_feedback().blocked;
 snapshot_.control.trajectory_validated = trajectory_validation.accepted();
 snapshot_.control.trajectory_verdict = static_cast<int>(trajectory_validation.verdict);
 snapshot_.control.trajectory_diagnostic = trajectory_validation.diagnostic;
 snapshot_.control.trajectory_min_dynamic_clearance_m = trajectory_validation.min_dynamic_clearance_m;
 snapshot_.control.trajectory_max_dynamic_rollover_risk =
 trajectory_validation.max_dynamic_rollover_risk;
 snapshot_.control.trajectory_preserves_committed_schedule =
 trajectory_validation.preserves_committed_schedule;
 snapshot_.control.trajectory_committed_position_error_m =
 trajectory_validation.committed_position_error_m;
 snapshot_.control.trajectory_committed_heading_error_rad =
 trajectory_validation.committed_heading_error_rad;
 snapshot_.control.trajectory_committed_speed_error_mps =
 trajectory_validation.committed_speed_error_mps;
 snapshot_.control.trajectory_committed_tracking_cost =
 trajectory_validation.committed_tracking_cost;
 double predicted_separation_m = std::isfinite(trajectory_validation.min_dynamic_clearance_m)
 ? trajectory_validation.min_dynamic_clearance_m
 : std::numeric_limits<double>::quiet_NaN();
 const std::size_t kVisibleDwaCandidates = execution_policy.visible_dwa_candidates;
 const std::size_t candidate_stride =
   std::max<std::size_t>(1, (dwa.rollouts.size() + kVisibleDwaCandidates - 1) / kVisibleDwaCandidates);
 for (std::size_t index = 0; index < dwa.rollouts.size(); index += candidate_stride) {
  const auto& rollout = dwa.rollouts[index];
  std::vector<sim::Vec2> rollout_points;
  rollout_points.reserve(rollout.trajectory.samples.size());
  for (const auto& sample : rollout.trajectory.samples)
   rollout_points.push_back(sample.position);
  snapshot_.control.dwa_candidates.push_back({
    std::move(rollout_points),
    rollout.collision,
    rollout.trajectory.command.score,
 rollout.trajectory.command.v,
 rollout.trajectory.command.w,
  });
 }
 // Once a safety stop has started, finish the physically reachable stop
 // before transferring authority back to a newly valid moving rollout. A
 // planner can become nominally feasible one cycle after rejection while
 // the plant is still carrying the rejected maneuver's momentum; alternating
 // those two commands produces a brake/accelerate limit cycle on slopes.
 const double kControlledStopLinearReleaseMps = execution_policy.controlled_stop_release_mps;
 // A stop latch belongs to the plant state, not to the filtered pose used by
 // the planner. The fused estimate may already be near zero while the wheel
 // joints still carry substantial speed, especially after a short safety
 // rejection on a slope. Releasing on that estimate immediately republishes
 // a high target and turns the stop into a brake/accelerate limit cycle.
 // Peak wheel speed also rises with a differential, so an in-place rotation
 // keeps the latch closed without consulting the planner's requested yaw. A
 // request is evidence of intent, never of motion, and gating release on it
 // deadlocks the turn the planner needs in order to leave the stop.
 const double measured_stop_wheel_mps = local_request.measured_wheel_speeds
 ? std::max(std::abs(local_request.measured_wheel_speeds->left_mps),
 std::abs(local_request.measured_wheel_speeds->right_mps))
 : std::abs(local_request.forward_mps);
 // The release contract itself is ROBOT code; the deployment policy supplies
 // only the calibrated thresholds.
 sim::ControlledStopLatch::Config latch_config;
 latch_config.release_speed_mps = execution_policy.controlled_stop_release_mps;
 latch_config.settle_cycles =
 static_cast<int>(execution_policy.controlled_stop_settle_cycles);
 const bool controlled_stop_in_progress = robot_application_->update_controlled_stop(
 held_command().controlled_stop, measured_stop_wheel_mps, latch_config);
 const bool trajectory_accepted =
 trajectory_validation.accepted() && !controlled_stop_in_progress;
 const bool stability_rejection =
 trajectory_validation.verdict == sim::TrajectoryVerdict::DynamicStabilityViolation ||
 (trajectory_validation.verdict == sim::TrajectoryVerdict::UnsafeEmergencyStop &&
 trajectory_validation.diagnostic.find("support-polygon") != std::string::npos);
 if (!trajectory_accepted) {
 // A rejected rollout requests a stop, but zero is not an instantaneous
 // actuator state. Stage the emergency command through the assembled support
 // polygon and the same wheel-space rate contract used by DWA. This prevents
 // an otherwise harmless soft-margin failure from pitching a high-COM load
 // over its short front support while leaving MuJoCo free to realize the
 // commanded torque, slip, and stopping distance.
 const auto stop_terrain = robot_application_->estimated_terrain({nav_x, nav_y});
 const double stop_grade = robot_application_->longitudinal_grade(stop_terrain, grade_heading);
 const bool steering_command =
 local_request.command_space == sim::LocalCommandSpace::SteeringAngle;
 // A safety stop must start from the motion that the plant is actually
 // carrying, not from the last requested twist.  On a downhill grade the
 // request can already be zero while encoder feedback continues to show a
 // substantial rollback.  Seeding the ramp from the request would publish
 // zero again and leave the controller with no reachable braking target.
 const double measured_stop_forward_mps = local_request.measured_wheel_speeds
 ? 0.5 * (local_request.measured_wheel_speeds->left_mps +
 local_request.measured_wheel_speeds->right_mps)
 : local_request.forward_mps;
 // A stop trajectory is initialized from feedback once, then remains a
 // monotonic command ramp. Re-seeding it from measured speed every cycle
 // makes a downhill disturbance raise the next target and prevents the
 // speed loop from developing braking error.
 const bool stop_target_initialized =
 std::abs(held_command().target_v) > 1e-4 ||
 std::abs(held_command().target_lateral_v) > 1e-4 ||
 std::abs(held_command().target_w) > 1e-4;
 // A zero stop target is not enough to prove that the ramp was initialized:
 // gravity can move the plant after the target has already reached zero.
 // Re-seed once from feedback when that happens, then continue the monotonic
 // ramp from the newly staged target.
 const bool continuing_controlled_stop =
 held_command().controlled_stop && stop_target_initialized;
 const sim::SafeStopMotion stop_motion{
 continuing_controlled_stop ? held_command().target_v : measured_stop_forward_mps,
 continuing_controlled_stop ? held_command().target_lateral_v : local_request.lateral_mps,
 continuing_controlled_stop
 ? held_command().target_w
 : (steering_command ? local_request.steering_angle_rad : local_request.yaw_rate_radps),
 };
 sim::SafeStopLimits stop_limits;
 stop_limits.update_interval_s = prediction_step_s;
 stop_limits.brake_decel_mps2 = resolved_build_.traction().brake_decel_mps2;
 stop_limits.max_speed_mps = local_request.max_speed_mps;
 stop_limits.max_turn = steering_command
 ? local_request.max_steer_rad : local_request.max_yaw_rate_radps;
 stop_limits.max_turn_rate = steering_command
 ? local_request.max_steer_rate_radps : local_request.max_yaw_accel_radps2;
 stop_limits.differential_track_m = local_request.differential_track_m;
 stop_limits.max_wheel_speed_mps = local_request.max_wheel_speed_mps;
 stop_limits.longitudinal_grade = stop_grade;
 stop_limits.center_of_mass_x_m = combined_center_x_m();
 stop_limits.center_of_mass_height_m = combined_center_height_m();
 const auto stop_command = robot_application_->stage_safe_stop(
 chassis_caps_, stop_motion, stop_limits);
 // A negative measured velocity is already motion away from the requested
 // route. A less-negative velocity target still asks the motor loop to track
 // downhill motion for another interval, which can amplify rollback before
 // the grade is arrested. Command the zero target directly; the controlled
 // stop flag makes the low-level controller reset stale effort and apply its
 // normal opposing feedback torque.
 held_command().target_v = measured_stop_forward_mps < -kControlledStopLinearReleaseMps
 ? 0.0 : stop_command.forward_mps;
 held_command().target_lateral_v = stop_command.lateral_mps;
 held_command().target_w = stop_command.turn;
 held_command().controlled_stop = true;
  // A local-planner failure enters the zero-speed safety state. A recovery
  // planner needs its own dynamically validated trajectory and termination
  // contract; an unverified steering heuristic must never bypass DWA.
  if (!snapshot_.waiting_for_clearance) {
 if (stability_rejection) {
 record_event("stability_hold", catalog_.tr("event.stability_hold"));
 } else if (std::isfinite(nearest_actor)) {
 std::ostringstream waiting;
 waiting << std::fixed << std::setprecision(1) << nearest_actor;
 record_event("waiting", catalog_.trf("event.waiting", {waiting.str()}));
 }
  }
  snapshot_.waiting_for_clearance = true;
 shadow_open_speed_ = known_tracks.empty() ? -1.0 : 0.0;
 for (const auto& track : known_tracks) {
 shadow_open_speed_ = std::max(shadow_open_speed_, std::hypot(track.state.v.x, track.state.v.y));
 }
 shadow_open_nearest_ = std::isfinite(nearest_actor) ? nearest_actor : -1.0;
 if (!known_tracks.empty() || governor_out.hold_active)
 update_shadow_episode();
 } else {
  const auto& command = selected_trajectory.command;
  // This is the only command publication point for a validated local
  // trajectory. Every speed/turn cap was applied before rollout generation.
 const double kCreepSpeedMps = execution_policy.creep_speed_mps;
  held_command() = {command.v, command.w};
 robot_application_->reset_controlled_stop();
  held_command().target_lateral_v = command.lateral_v;
 const double commanded_translation_mps =
 std::hypot(command.v, command.lateral_v);
 // Slow terminal settling is not a dynamic-obstacle wait. Clearance waiting
 // requires both a near-stationary command and actual dynamic/governor
 // evidence; this also avoids classifying pure lateral motion as stopped.
 snapshot_.waiting_for_clearance = commanded_translation_mps < kCreepSpeedMps &&
 (governor_out.hold_active || !known_tracks.empty());
 const sim::TrajectoryTrackingLimits tracking_limits{
    std::max(2.0 * map_definition_.grid_resolution_m, level_.planning_localization_margin_m),
    std::max(0.10, std::atan2(map_definition_.grid_resolution_m,
                              std::max(0.05, robot_definition_.collision_radius_m))),
 std::max(0.10, 2.0 * resolved_build_.traction().brake_decel_mps2 / decision_hz()),
  };
  snapshot_.control.trajectory_sequence =
 robot_application_->accept_trajectory(snapshot_.time_s, selected_trajectory, tracking_limits);
  if (snapshot_.waiting_for_clearance) {
   std::ostringstream waiting;
   waiting << std::fixed << std::setprecision(1) << nearest_actor;
   record_event("waiting", catalog_.trf("event.waiting", {waiting.str()}), 2.0);
  }
 shadow_open_speed_ = known_tracks.empty() ? -1.0 : 0.0;
 for (const auto& track : known_tracks) {
 shadow_open_speed_ = std::max(shadow_open_speed_, std::hypot(track.state.v.x, track.state.v.y));
 }
 shadow_open_nearest_ = std::isfinite(nearest_actor) ? nearest_actor : -1.0;
  update_shadow_episode();
 }
 // Compensate the gravity component using deployable state only. A calibrated
 // IMU attitude owns the live estimate when available; otherwise the deployed
 // 2.5D terrain map supplies the local grade. The low-level controller maps
 // this SI request through finite motor authority, while tire/contact physics
 // remains free to slip when the surface cannot carry the requested force.
 const auto current_terrain = robot_application_->estimated_terrain({nav_x, nav_y});
 const double current_grade = robot_application_->longitudinal_grade(current_terrain, grade_heading);
 const auto& control_estimate = robot_application_->estimator().state();
 held_command().longitudinal_acceleration_feedforward_mps2 =
 control_estimate.attitude_valid
 ? -execution_policy.gravity_mps2 * std::sin(control_estimate.pitch_rad)
 : execution_policy.gravity_mps2 * current_grade /
 std::sqrt(1.0 + current_grade * current_grade);
 // Gravity compensation is a bounded actuator request, not an unbounded
 // estimate. An attitude estimate that lags or overshoots on a slide can
 // otherwise consume the whole duty range and cancel the velocity loop's
 // braking effort, driving the wheels further in the direction of travel.
 // The support-polygon limit is the same physical bound used to stage a
 // controlled stop, so the request stays inside what the chassis can apply
 // without pitching over.
 // The stability-aware emergency deceleration is a bound on a moving stop
 // trajectory. It is not a bound on static grade-hold effort: applying it to
 // gravity compensation can leave a capable motor below the force required
 // to balance the slope, after which the real plant rolls back. Use the
 // declared motor force and tire braking envelope for this actuator request;
 // MuJoCo still decides whether contact can realize it.
 const double motor_stall_force_N = resolved_build_.motor()
 ? resolved_build_.motor()->stall_force_N
 : robot_definition_.motor.stall_force_N;
 const double motor_accel_limit_mps2 = motor_stall_force_N *
 std::max(0.1, chassis_caps_.drive_force_units) /
 std::max(0.5, total_mass_kg());
 const double feedforward_limit_mps2 = std::max(
 resolved_build_.traction().brake_decel_mps2, motor_accel_limit_mps2);
 held_command().longitudinal_acceleration_feedforward_mps2 = std::clamp(
 held_command().longitudinal_acceleration_feedforward_mps2,
 -feedforward_limit_mps2, feedforward_limit_mps2);
 const double measured_control_forward_mps = local_request.measured_wheel_speeds
 ? 0.5 * (local_request.measured_wheel_speeds->left_mps +
 local_request.measured_wheel_speeds->right_mps)
 : local_request.forward_mps;
 // During a controlled stop, gravity compensation must not become a drive
 // command in the same direction as the residual motion. Let the velocity
 // loop dissipate that motion; retain feedforward only when it assists the
 // stop or when the chassis is not in the stop state.
 if (held_command().controlled_stop &&
 held_command().longitudinal_acceleration_feedforward_mps2 *
 measured_control_forward_mps > 0.0) {
 held_command().longitudinal_acceleration_feedforward_mps2 = 0.0;
 }
 const bool minimum_risk_escape = trajectory_validation.minimum_risk_escape();
 const bool committed_transit = trajectory_validation.committed_transit();
 const std::string action_code =
 committed_transit
 ? "committed-transit"
 : minimum_risk_escape
     ? "minimum-risk-escape"
     : (snapshot_.waiting_for_clearance ? "wait" : (trajectory_accepted ? "avoid" : "wait"));
 const std::string reason_code =
 !trajectory_accepted
 ? (controlled_stop_in_progress
 ? "controlled-stop-in-progress"
 : (stability_rejection ? "stability-envelope" : "no-safe-local-trajectory"))
 : (committed_transit
 ? "crossing-committed"
     : (governor_out.hold_active
          ? "entry-window-occupied"
 : (snapshot_.waiting_for_clearance ? "predicted-path-conflict" : "safe-local-trajectory")));
 const std::string constraint_code = reason_code == "safe-local-trajectory"
 ? "none"
 : (controlled_stop_in_progress
 ? "physical-stop"
 : (stability_rejection ? "dynamic-stability" : "dynamic-clearance"));
 publish(constraint_code, reason_code, action_code,
         observation_time_s, std::move(observation_refs), prediction_horizon,
         std::isfinite(nearest_actor) ? nearest_actor : unavailable, predicted_separation_m);
}

} // namespace mission
