#include "sim/game/scenario/scenario_session.hpp"
#include "robot/autonomy/control/safe_stop_limiter.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iterator>
#include <limits>
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
 snapshot_.control = {};
 if (route_tracking_system_->path_index() >= planned_route_.size()) {
  if (planned_route_.empty()) {
   held_command_ = {0.0, 0.0};
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
  route_tracking_system_->set_path_index(planned_route_.size() - 1);
 }
 auto target = current_navigation_target();
 const auto& estimate = estimation_system_.state();
 const double nav_x = estimate.position.x;
 const double nav_y = estimate.position.y;
 const double nav_heading = estimate.heading_rad;
 // The local planning view composes immutable authored occupancy with the
 // transient LiDAR obstacle layer. Dynamic target prediction remains a
 // separate time-indexed input below; an observed obstacle therefore cannot
 // become a permanent global wall or disappear merely because tracking lost
 // its identity.
 const sim::GridMap local_planning_map = autonomy_pipeline_->compose_local_map(
 *planning_map_, snapshot_.time_s, planning_clearance_m_);
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
 const sim::GridMap local_physical_collision_map = autonomy_pipeline_->compose_local_map(
 *recovery_base_map_, snapshot_.time_s,
 robot_definition_.collision_radius_m + level_.planning_localization_margin_m);
 const auto tracking = autonomy_pipeline_->observe_trajectory(
 snapshot_.time_s, estimate.position, estimate.heading_rad, estimate.linear_velocity_mps);
 snapshot_.control.trajectory_sequence = tracking.sequence;
 snapshot_.control.trajectory_tracking_active = tracking.active;
 snapshot_.control.trajectory_within_envelope = tracking.within_envelope;
 snapshot_.control.trajectory_position_error_m = tracking.position_error_m;
 snapshot_.control.trajectory_heading_error_rad = tracking.heading_error_rad;
 snapshot_.control.trajectory_speed_error_mps = tracking.speed_error_mps;
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
  decision.target_speed_mps = held_command_.target_v;
  decision.target_yaw_rate_radps = held_command_.target_w;
  publish_decision(std::move(decision));
 };
 sim::StrategyObservation strategy_observation;
 strategy_observation.stamp_s = estimate.timestamp_s;
 strategy_observation.ego = estimate;
 strategy_observation.route_target = target;
 const GoalStep* strategy_goal = active_goal();
 strategy_observation.goal = strategy_goal ? as_vec(map_definition_.markers.at(strategy_goal->marker))
                                           : as_vec(map_definition_.markers.at(level_.objective.goal));
 strategy_observation.rollover_risk = snapshot_.rollover_risk;
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
 const auto strategy_terrain = terrain_.sample({nav_x, nav_y});
 const bool material_terrain_load =
   std::abs(sim::longitudinal_grade(strategy_terrain, nav_heading)) >= 0.12 ||
   std::abs(sim::lateral_grade(strategy_terrain, nav_heading)) >= 0.12;
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
 const auto strategy_request = high_level_policy_->decide(strategy_observation);
 // Speed and stability constraints belong to the strategy boundary,
 // regardless of who produced the request. Context flags avoid duplicating
 // obstacle arbitration already owned by the validated local planner.
 snapshot_.strategy = sim::apply_strategy_safety(strategy_request, strategy_observation, strategy_context);
 snapshot_.strategy_policy = high_level_policy_->name();
 if (snapshot_.strategy.applied.has_subgoal)
  target = snapshot_.strategy.applied.subgoal;
 const double chassis_cruise = cruise_speed_mps();
 auto route_control =
   route_tracking_system_->follow_route(planned_route_, estimate, property(behavior_, "lookahead_m"), chassis_cruise,
 1.0 / decision_hz(), local_planning_map, chassis_caps_);
 if (snapshot_.strategy.applied.has_subgoal) {
  const double target_distance = distance({nav_x, nav_y}, target);
  const double subgoal_error = sim::heading_err(target, nav_x, nav_y, nav_heading);
  route_control.tv = std::min(chassis_cruise, target_distance);
  route_control.steer = route_tracking_system_->heading_command(subgoal_error, 1.0 / decision_hz());
  route_control.err = subgoal_error;
  route_control.reached = target_distance < 0.25;
 }
 snapshot_.control.local_target = target;
 snapshot_.control.heading_error_rad = route_control.err;
 snapshot_.control.heading_pid_p = route_tracking_system_->pid_p();
 snapshot_.control.heading_pid_i = route_tracking_system_->pid_i();
 snapshot_.control.heading_pid_d = route_tracking_system_->pid_d();
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
   const double dock_err = sim::heading_err(goal, nav_x, nav_y, nav_heading);
   if (!route_tracking_system_->reversing() && std::abs(dock_err) > 1.4)
    route_tracking_system_->set_reversing(true);
   else if (route_tracking_system_->reversing() && std::abs(dock_err) < 0.7)
    route_tracking_system_->set_reversing(false);
   const double dock_dt = 1.0 / decision_hz();
   if (!route_tracking_system_->reversing()) {
    route_control.tv = std::min(0.3, dist_goal);
    route_control.steer = route_tracking_system_->heading_command(dock_err, dock_dt);
   } else if (chassis_->caps().can_spin) {
    route_tracking_system_->set_reversing(false);
    route_control.tv = 0.0;
    route_control.steer = route_tracking_system_->heading_command(dock_err, dock_dt);
   } else {
    route_control.tv = -0.3;
    route_control.steer = -route_tracking_system_->heading_command(dock_err, dock_dt);
   }
   route_control.err = dock_err;
   route_control.reached = false;
  }
 }
 route_control.tv *= snapshot_.strategy.applied.speed_scale;
 const double unavailable = std::numeric_limits<double>::quiet_NaN();
 if (snapshot_.strategy.applied.mode == sim::StrategyMode::Hold) {
  held_command_ = {};
  snapshot_.waiting_for_clearance = true;
  snapshot_.control.local_target = target;
  snapshot_.control.heading_error_rad = route_control.err;
  publish("high-level-intent", "strategy-hold", "wait", estimate.timestamp_s, {}, unavailable, unavailable,
          unavailable);
  update_shadow_episode();
  return;
 }
 const auto all_tracks =
 autonomy_pipeline_->dynamic_obstacles().active_tracks(snapshot_.time_s, track_max_age_s());
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
 const double requested_clearance = property(behavior_, "safe_clearance_m") + scan_motion_margin;
 // Separate the hard crossing admission envelope from the driving policy's
 // preferred comfort margin. A cautious style may request more clearance
 // than a periodic corridor can ever provide; treating that preference as a
 // hard behavior invariant creates permanent waits. The authored safety
 // threshold plus sensing-motion uncertainty is the non-negotiable gate.
 const double hard_crossing_clearance =
 std::max(0.0, level_.safe_min_clearance_m) + scan_motion_margin;
 const double behavior_crossing_clearance =
 std::max(0.0, level_.safe_min_clearance_m);
 const double heading_error = sim::heading_err(target, nav_x, nav_y, nav_heading);
 const double route_independent_speed_cap =
 std::min(std::max(0.20, sim::sched_v(heading_error, cruise_speed_mps())),
                             corner_speed_cap(nav_x, nav_y));
 double speed_cap = route_independent_speed_cap * snapshot_.strategy.applied.speed_scale;
 // The selected rollout is the command contract. Feed route and corner caps
 // into candidate generation so mission code never rewrites its speed later.
 if (chassis_caps_.command_space != sim::ChassisCommandSpace::HolonomicTwist)
 speed_cap = std::min(speed_cap, std::abs(route_control.tv));
 const sim::DwaWeights weights{1.0, 1.3 + requested_clearance * 1.8, 0.25};
 constexpr double prediction_step_s = 0.1;
 // Predictor module (src/robot/autonomy/predictor.*) owns future frames; the tracker
 // owns observations. known_tracks (age-advanced, for display) must not
 // feed prediction: map.predict() snapshots raw tracker state internally.
 const auto predicted_frames = autonomy_pipeline_->dynamic_obstacles().predict(
   snapshot_.time_s, prediction_horizon, prediction_step_s, track_max_age_s());
 // Shadow departure assessment (gate 3): observe only. This block reads
 // tracker state and writes snapshot shadow fields; held_command_ (the actual
 // control output) is untouched by everything in this block.
 std::vector<sim::Vec2> shadow_remaining{{nav_x, nav_y}};
 for (std::size_t i = route_tracking_system_->path_index(); i < planned_route_.size(); ++i) {
  shadow_remaining.push_back(planned_route_[i].position);
 }
 const double shadow_cruise = std::max(1e-6, cruise_speed_mps());
 double shadow_length = 0.0;
 for (std::size_t i = 0; i + 1 < shadow_remaining.size(); ++i) {
  shadow_length += distance(shadow_remaining[i + 1], shadow_remaining[i]);
 }
 // Crossing, route-departure shadow, and pocket evaluation share one
 // time-indexed prediction. Recomputing each consumer independently could
 // make a single decision explain itself with mutually different futures.
 const double assessment_horizon = shadow_length / shadow_cruise + prediction_horizon;
 const auto assessment_frames = autonomy_pipeline_->dynamic_obstacles().predict(
   snapshot_.time_s, assessment_horizon, prediction_step_s, track_max_age_s());

 // A map-semantic conflict zone is the high-level contract for crossing
 // traffic. It yields a stable route-derived stop point and a bounded path
 // through the zone. Actor truth and game collision flags are deliberately
 // absent: only tracked predictions participate in departure assessment.
 bool mapped_crossing_relevant = false;
 std::optional<sim::GovernorOutput> mapped_governor;
 const auto active_crossing_zones = dynamic_traffic_system_.active_zones(
 map_definition_.conflict_zones, level_id_);
 constexpr double kStopControllerToleranceM = 0.0;
 const auto traffic_decision = crossing_behavior_system_.assess(
 active_crossing_zones, planned_route_, route_tracking_system_->path_index(), {nav_x, nav_y},
 estimate.linear_velocity_mps, nav_heading, assessment_frames,
 prediction_step_s,
 {robot_definition_.collision_radius_m, behavior_crossing_clearance,
 // Route vertices are world-space poses even when the global search used a
 // raster. Grid resolution affects which poses exist; it is not physical
 // stop-controller error. Reserve only the declared localization margin here.
 kStopControllerToleranceM + level_.planning_localization_margin_m, 0.35},
 // Admission may prove only a speed that the receding-horizon controller
 // will actually execute. Searching up to motor free speed would prove one
 // timed crossing and then hand a different high-speed steering rollout to
 // DWA. Faster crossings require an explicit timed-trajectory tracker with a
 // validated tracking envelope; the current reference controller commits at
 // its route cruise speed.
 {shadow_cruise, shadow_cruise,
 std::min(robot_definition_.motor.max_wheel_accel_mps2,
 resolved_build_.traction().brake_decel_mps2),
 resolved_build_.traction().brake_decel_mps2, yaw_cap_rads_},
 1.0 / decision_hz());
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
    sim::assess_departure(shadow_remaining, 0, shadow_cruise, assessment_frames, prediction_step_s,
                          robot_definition_.collision_radius_m, requested_clearance);
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
  const auto search = sim::sample_pockets(
 local_planning_map, shadow_remaining, {nav_x, nav_y}, nav_heading, shadow_cruise,
    sim::ChassisExecCaps{chassis_caps_.can_spin, chassis_caps_.max_yaw_rate_radps,
                         chassis_caps_.geometry.steering_wheel_base_m, chassis_caps_.geometry.max_steer_rad,
                         chassis_caps_.geometry.max_steer_rate_radps,
                         chassis_caps_.geometry.steering_response_s},
    prediction_horizon, pocket_params, assessment_frames, prediction_step_s,
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
 std::optional<double> local_target_heading;
 bool local_stop_plane_active = false;
 if (mapped_governor) {
  governor_out = *mapped_governor;
  snapshot_.control.governor_hold = governor_out.hold_active;
  snapshot_.control.governor_stop = governor_out.stop_point;
 if (governor_out.hold_active && traffic_decision.phase_changed)
   record_event("governor_hold", catalog_.tr("event.governor_hold"));
 } else if (!traffic_decision.has_active_zone) {
  const sim::Vec2 ego{nav_x, nav_y};
  const sim::Vec2 dir{std::cos(nav_heading), std::sin(nav_heading)};
 const double ego_speed = std::abs(estimate.linear_velocity_mps);
  const double assess_speed = ego_speed >= 0.15 ? ego_speed : 0.5;
  sim::GovernorInput governor_input;
  for (const auto& track : known_tracks) {
   const double age = snapshot_.time_s - track.last_observed_s;
   if (age < 0.0 || age > sim::kEntryStaleBudgetS) {
    continue;
   }
   const sim::Vec2 tp{track.state.p.x, track.state.p.y};
   const sim::Vec2 tv{track.state.v.x, track.state.v.y};
   const auto center = sim::governor_conflict_center(ego, dir, tp, tv);
   if (std::hypot(center.x - ego.x, center.y - ego.y) > 10.0) {
    continue;
   }
   const double half = 0.45 + track.state.radius + 0.2;
   sim::WindowThreat threat;
   threat.valid = true;
   threat.dist_to_zone_m = std::hypot(tp.x - center.x, tp.y - center.y) - half;
   const double to_center = std::max(1e-6, std::hypot(center.x - tp.x, center.y - tp.y));
   const double toward = (tv.x * (center.x - tp.x) + tv.y * (center.y - tp.y)) / to_center;
   threat.speed_mps = (toward > 0.0) ? std::hypot(tv.x, tv.y) : 0.0;
   threat.age_s = age;
   const double ego_dist = std::max(0.0, std::hypot(center.x - ego.x, center.y - ego.y) - half);
   const auto assessment =
     sim::assess_entry_window(ego_dist, assess_speed, half, threat, sim::kEntryStaleBudgetS);
   if (assessment.verdict == sim::EntryVerdict::kObservedOccupied) {
    governor_input.occupied_fresh = true;
   } else if (assessment.verdict == sim::EntryVerdict::kObservedClear) {
    governor_input.clear_fresh = true;
   }
  }
  if (governor_input.occupied_fresh) {
   governor_input.clear_fresh = false; // occupied wins mixed frames.
  }
  // Corridor entry against every forecast frame: the first remaining-route
  // point inside any predicted swept disc, walked back brake + margin.
  if (governor_input.occupied_fresh || governor_input.clear_fresh) {
   auto near_predicted = [&](sim::Vec2 p) {
    for (const auto& frame : predicted_frames) {
     for (const auto& future : frame) {
      if (std::hypot(p.x - future.p.x, p.y - future.p.y) < 0.9) {
       return true;
      }
     }
    }
    return false;
   };
   std::size_t entry = shadow_remaining.size();
   for (std::size_t i = 0; i < shadow_remaining.size(); ++i) {
    if (near_predicted(shadow_remaining[i])) {
     entry = i;
     break;
    }
   }
   if (entry < shadow_remaining.size()) {
 const double cruise = cruise_speed_mps();
 const double brake = resolved_build_.traction().brake_decel_mps2;
    const double need = cruise * cruise / (2.0 * brake) + 1.0;
    double walked = 0.0;
    std::size_t stop = entry;
    while (stop > 0 && walked < need) {
     walked += std::hypot(shadow_remaining[stop].x - shadow_remaining[stop - 1].x,
                          shadow_remaining[stop].y - shadow_remaining[stop - 1].y);
     --stop;
    }
    governor_input.stop_point = shadow_remaining[stop];
    governor_input.stop_valid = true;
   }
  }
 const bool was_hold = entry_governor_system_.holding();
 governor_out = entry_governor_system_.update(governor_input);
  snapshot_.control.governor_hold = governor_out.hold_active;
  snapshot_.control.governor_stop = governor_out.stop_point;
  if (governor_out.hold_active && !was_hold) {
   record_event("governor_hold", catalog_.tr("event.governor_hold"));
  }
 } else {
  // Authored conflict-zone semantics supersede the legacy geometric
  // intersection heuristic. If the current route does not enter a declared
  // zone, the behavior layer has no crossing constraint to impose; ordinary
  // local collision avoidance still runs below when a track is actually near.
 governor_out = entry_governor_system_.update({});
  snapshot_.control.governor_hold = governor_out.hold_active;
  snapshot_.control.governor_stop = governor_out.stop_point;
 }
 // Virtual wall (Phase-2 consumption): while the governor holds, cap speed
 // by stopping physics to the stop point. The local planner also receives
 // the point and plane below, so steering converges to the proven hold pose
 // instead of continuing to aim beyond a speed-only wall.
 if (governor_out.hold_active && governor_out.stop_valid) {
 const double brake = resolved_build_.traction().brake_decel_mps2;
 double dist = std::hypot(nav_x - governor_out.stop_point.x, nav_y - governor_out.stop_point.y);
 if (mapped_crossing_relevant) {
 dist = crossing.stop_distance_m;
 const double local_stop_activation_m = std::max(
 2.0, speed_cap * prediction_horizon + property(behavior_, "lookahead_m"));
 local_stop_plane_active = dist <= local_stop_activation_m;
 if (local_stop_plane_active) {
 target = governor_out.stop_point;
 snapshot_.control.local_target = target;
 // A differential chassis cannot correct lateral stop-line error while it
 // is simultaneously scored against the final crossing heading. Approach
 // the stop pose first, then align in place once the position contract is
 // satisfied. Otherwise a safe rollout can settle abeam of the line with
 // zero translation forever.
 if (traffic_decision.readiness.position_ready) {
 local_target_heading = std::atan2(
 governor_out.stop_direction.y, governor_out.stop_direction.x);
 } else {
 // The behavior stop pose temporarily supersedes the ordinary route
 // lookahead. Its approach speed must therefore be derived from that pose,
 // not from a route follower that may currently be in a spin/reverse recovery
 // aimed at a waypoint beyond the virtual wall. Collision and stop-plane
 // validation below remain authoritative.
 const double stop_heading_error = sim::heading_err(
 governor_out.stop_point, nav_x, nav_y, nav_heading);
 const double stop_distance = distance(
 {nav_x, nav_y}, governor_out.stop_point);
 route_control.err = stop_heading_error;
 route_control.tv = std::min(chassis_cruise, stop_distance);
 const double stop_approach_cap = std::min(
 std::max(0.20, sim::sched_v(stop_heading_error, chassis_cruise)),
 corner_speed_cap(nav_x, nav_y)) * snapshot_.strategy.applied.speed_scale;
 speed_cap = std::min(
 stop_approach_cap,
 std::abs(route_control.tv));
 snapshot_.control.heading_error_rad = stop_heading_error;
 }
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
 if (mapped_crossing_relevant && !traffic_decision.readiness.position_ready) {
 dist = std::max(dist, distance({nav_x, nav_y}, governor_out.stop_point));
 }
 const double v_allow = dist <= 1e-9
 ? 0.0
 : brake * (std::sqrt(response_delay_s * response_delay_s +
 2.0 * dist / brake) - response_delay_s);
  speed_cap = std::min(speed_cap, v_allow);
 }
 const auto route_direction = route_tracking_system_->travel_direction(planned_route_);
 // Gear is owned by the global route for every chassis. The follower's
 // signed command is only a fallback once the route has no direction metadata
 // (for example after the terminal index); it may not silently reverse a
 // forward section to improve a local score.
 const bool reverse_route = route_direction == sim::TravelDirection::Reverse ||
 (route_direction == sim::TravelDirection::Unspecified && route_control.tv < 0.0);
 const bool crossing_committed = traffic_decision.control_authority && traffic_decision.control.committed;
 constexpr double kCommittedProgressFloorMps = 0.15;
 const double committed_floor = crossing_committed
 ? std::min(speed_cap, std::max(kCommittedProgressFloorMps,
 traffic_decision.control.transit_speed_mps))
 : 0.0;
 // A zero-speed recovery on a spin-capable chassis is an explicit pose
 // maneuver. Give the local planner that body-heading goal so the path-
 // tangent critic cannot reward holding the old yaw forever after waypoint
 // progress has stopped. Ackermann remains governed by its route/gear model.
 if (!local_target_heading && !mapped_crossing_relevant &&
 !governor_out.hold_active && chassis_caps_.can_spin &&
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
 // Encoder and fused-pose filters retain millimetres-per-second residuals
 // after the physical wheels have stopped. Feeding that noise into the
 // rollout makes every in-place correction drift microscopically through an
 // exact behavior stop plane, so the safety validator can never authorize a
 // recovery. Apply a narrow planning deadband only when both drive encoders
 // independently report rest; real body motion above this sensor floor stays
 // visible to the planner.
 constexpr double kPlanningRestDeadbandMps = 0.005;
 const bool drive_encoders_stopped =
 std::abs(snapshot_.control.left_measured_mps) < kPlanningRestDeadbandMps &&
 std::abs(snapshot_.control.right_measured_mps) < kPlanningRestDeadbandMps;
 local_request.forward_mps = drive_encoders_stopped &&
 std::abs(estimate.linear_velocity_mps) < kPlanningRestDeadbandMps
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
 local_request.max_yaw_rate_radps = yaw_cap_rads_;
 sim::SafeStopLimits current_stop_limits;
 current_stop_limits.brake_decel_mps2 =
 resolved_build_.traction().brake_decel_mps2;
 current_stop_limits.longitudinal_grade =
 sim::longitudinal_grade(strategy_terrain, nav_heading);
 current_stop_limits.center_of_mass_x_m = combined_center_x_m();
 current_stop_limits.center_of_mass_height_m = combined_center_height_m();
 const double safe_stop_decel_mps2 = sim::safe_stop_linear_decel_limit(
 chassis_caps_, current_stop_limits);
 // Candidate generation uses the normal drive/brake actuator envelope. The
 // separately modelled emergency tail uses the stricter stability-aware stop
 // limit below; conflating the two would unnecessarily cap ordinary forward
 // acceleration by the smaller of the front and rear support margins.
 local_request.max_accel_mps2 = resolved_build_.traction().brake_decel_mps2;
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
 local_request.max_wheel_speed_mps = max_forward_speed_mps_;
 local_request.steering_wheel_base_m = chassis_caps_.geometry.steering_wheel_base_m;
 local_request.max_steer_rad = chassis_caps_.geometry.max_steer_rad;
 local_request.max_steer_rate_radps = chassis_caps_.geometry.max_steer_rate_radps;
 local_request.steering_response_s = chassis_caps_.geometry.steering_response_s;
 local_request.preferred_direction = route_direction;
 local_request.route_is_reverse = reverse_route;
 local_request.crossing_relevant = mapped_crossing_relevant;
 local_request.crossing_committed = crossing_committed;
 local_request.body_in_conflict = crossing.body_in_conflict;
 local_request.stop_plane_active = mapped_crossing_relevant &&
 local_stop_plane_active && governor_out.hold_active && governor_out.stop_valid && !crossing_committed;
 local_request.stop_point = governor_out.stop_point;
 local_request.stop_direction = governor_out.stop_direction;
 local_request.stop_tolerance_m = kStopControllerToleranceM;
 local_request.committed_progress_floor_mps = committed_floor;
 // The stopping tail is based on the delivered controller bandwidth and the
 // tire/contact deceleration contract. It applies to actuator-level wheeled
 // and tracked plants; the trajectory validator then checks the resulting
 // swept path in the chassis' own command space.
 if (chassis_->accepts_actuator_duty()) {
 const auto* local_motor = resolved_build_.motor();
 const double local_controller_bandwidth = local_motor
 ? local_motor->controller_bandwidth_radps
 : robot_definition_.motor.controller_bandwidth_radps;
 local_request.actuator_response_delay_s =
 1.0 / std::max(0.1, local_controller_bandwidth);
 local_request.emergency_stop_delay_s = 1.0 / decision_hz() +
 local_request.actuator_response_delay_s;
 local_request.emergency_decel_mps2 = safe_stop_decel_mps2;
 }
 // Path/progress/oscillation critics are shared guidance for twist-space
 // planners, including tracked and holonomic chassis. A tracked plant still
 // includes scrub, while a holonomic plant may make progress laterally;
 // both need the global path to distinguish useful motion from a rollout
 // that merely points toward the goal through an obstacle.
 if (local_request.command_space != sim::LocalCommandSpace::SteeringAngle) {
 local_request.reference.previous_angular_command_radps = held_command_.target_w;
 local_request.reference.path.push_back({nav_x, nav_y});
 if (local_request.stop_plane_active) {
 // A behavior hold inserts a temporary terminal pose into the local plan.
 // Referencing route vertices beyond the closed virtual wall asks the path
 // critic to continue through the crossing while the validator forbids it.
 // Truncate the local reference at the latched stop pose so all critics and
 // the behavior constraint describe the same maneuver.
 if (distance({nav_x, nav_y}, local_request.stop_point) > 1e-6)
 local_request.reference.path.push_back(local_request.stop_point);
 } else if (!planned_route_.empty()) {
 const std::size_t guide_index = std::min(
 route_tracking_system_->path_index(), planned_route_.size() - 1);
 const std::size_t search_begin = guide_index > 8 ? guide_index - 8 : 0;
 const std::size_t search_end = std::min(planned_route_.size(), guide_index + 32);
 std::size_t closest_index = search_begin;
 double closest_distance = std::numeric_limits<double>::infinity();
 for (std::size_t index = search_begin; index < search_end; ++index) {
 const double candidate_distance = distance(
 {nav_x, nav_y}, planned_route_[index].position);
 if (candidate_distance < closest_distance) {
 closest_distance = candidate_distance;
 closest_index = index;
 }
 }
 const double reference_length_m =
 speed_cap * prediction_horizon + property(behavior_, "lookahead_m");
 double accumulated_m = 0.0;
 sim::Vec2 previous{nav_x, nav_y};
 for (std::size_t index = closest_index;
 index < planned_route_.size() && accumulated_m <= reference_length_m;
 ++index) {
 const auto point = planned_route_[index].position;
 const double leg_m = distance(previous, point);
 if (leg_m > 1e-6) {
 local_request.reference.path.push_back(point);
 accumulated_m += leg_m;
 previous = point;
 }
 }
 }
 }
 const auto local_plan = autonomy_pipeline_->plan_local(
 local_request, local_planning_map, local_predicted_frames,
 [this](sim::Vec2 from, sim::Vec2 to) {
 return std::isfinite(navigation_traversal_cost(from, to)) &&
 collision_monitor_->static_motion_free(
 from, to, level_.planning_localization_margin_m);
 }, &local_physical_collision_map,
 [this](sim::Vec2 from, sim::Vec2 to) {
 return std::isfinite(navigation_traversal_cost(from, to)) &&
 collision_monitor_->static_motion_free(
 from, to, level_.planning_localization_margin_m);
 });
 const auto& dwa = local_plan.proposals;
 const auto& validated_selection = local_plan.selection;
 snapshot_.control.dwa_active =
 local_request.command_space == sim::LocalCommandSpace::DifferentialTwist;
 snapshot_.control.dwa_proposed_v_mps = dwa.best.command.v;
 snapshot_.control.dwa_proposed_w_radps = dwa.best.command.w;
 snapshot_.control.dwa_accepted_candidates = validated_selection.accepted_candidates;
 const auto& selected_trajectory = validated_selection.trajectory;
 for (const auto& sample : selected_trajectory.samples)
  snapshot_.control.dwa_selected.push_back(sample.position);
 const auto& trajectory_validation = validated_selection.validation;
 snapshot_.control.trajectory_validated = trajectory_validation.accepted();
 snapshot_.control.trajectory_verdict = static_cast<int>(trajectory_validation.verdict);
 snapshot_.control.trajectory_diagnostic = trajectory_validation.diagnostic;
 snapshot_.control.trajectory_min_dynamic_clearance_m = trajectory_validation.min_dynamic_clearance_m;
 double predicted_separation_m = std::isfinite(trajectory_validation.min_dynamic_clearance_m)
                                   ? trajectory_validation.min_dynamic_clearance_m
                                   : std::numeric_limits<double>::quiet_NaN();
 constexpr std::size_t kVisibleDwaCandidates = 24;
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
  });
 }
 const bool trajectory_accepted = trajectory_validation.accepted();
 if (!trajectory_accepted) {
 // A rejected rollout requests a stop, but zero is not an instantaneous
 // actuator state. Stage the emergency command through the assembled support
 // polygon and the same wheel-space rate contract used by DWA. This prevents
 // an otherwise harmless soft-margin failure from pitching a high-COM load
 // over its short front support while leaving MuJoCo free to realize the
 // commanded torque, slip, and stopping distance.
 const auto stop_terrain = terrain_.sample({nav_x, nav_y});
 const double stop_grade = sim::longitudinal_grade(stop_terrain, nav_heading);
 const bool steering_command =
 local_request.command_space == sim::LocalCommandSpace::SteeringAngle;
 const sim::SafeStopMotion stop_motion{
 local_request.forward_mps,
 local_request.lateral_mps,
 steering_command ? local_request.steering_angle_rad : local_request.yaw_rate_radps,
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
 const auto stop_command = sim::stage_safe_stop(
 chassis_caps_, stop_motion, stop_limits);
 held_command_.target_v = stop_command.forward_mps;
 held_command_.target_lateral_v = stop_command.lateral_mps;
 held_command_.target_w = stop_command.turn;
 held_command_.controlled_stop = true;
  // A local-planner failure enters the zero-speed safety state. A recovery
  // planner needs its own dynamically validated trajectory and termination
  // contract; an unverified steering heuristic must never bypass DWA.
  if (!snapshot_.waiting_for_clearance) {
   std::ostringstream waiting;
   waiting << std::fixed << std::setprecision(1) << nearest_actor;
   record_event("waiting", catalog_.trf("event.waiting", {waiting.str()}));
  }
  snapshot_.waiting_for_clearance = true;
 shadow_open_speed_ = known_tracks.empty() ? -1.0 : 0.0;
 for (const auto& track : known_tracks) {
 shadow_open_speed_ = std::max(shadow_open_speed_, std::hypot(track.state.v.x, track.state.v.y));
 }
 shadow_open_nearest_ = std::isfinite(nearest_actor) ? nearest_actor : -1.0;
  update_shadow_episode();
 } else {
  const auto& command = selected_trajectory.command;
  // This is the only command publication point for a validated local
  // trajectory. Every speed/turn cap was applied before rollout generation.
  constexpr double kCreepSpeedMps = 0.15;
  held_command_ = {command.v, command.w};
  held_command_.target_lateral_v = command.lateral_v;
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
 autonomy_pipeline_->accept_trajectory(snapshot_.time_s, selected_trajectory, tracking_limits);
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
     ? "no-safe-local-trajectory"
 : (committed_transit
 ? "crossing-committed"
     : (governor_out.hold_active
          ? "entry-window-occupied"
 : (snapshot_.waiting_for_clearance ? "predicted-path-conflict" : "safe-local-trajectory")));
 publish(reason_code == "safe-local-trajectory" ? "none" : "dynamic-clearance", reason_code, action_code,
         observation_time_s, std::move(observation_refs), prediction_horizon,
         std::isfinite(nearest_actor) ? nearest_actor : unavailable, predicted_separation_m);
}

} // namespace mission
