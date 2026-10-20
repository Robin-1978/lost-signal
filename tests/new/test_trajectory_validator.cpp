#include "robot/autonomy/planning/validation/trajectory_validator.hpp"
#include "robot/autonomy/planning/local/local_planning_system.hpp"

#include <cmath>
#include <iostream>
#include <limits>

#define CHECK(condition)                                                                                     \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n";                           \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

namespace {

sim::ExecutableTrajectory straight_trajectory() {
 sim::ExecutableTrajectory trajectory;
 trajectory.command_space = sim::LocalCommandSpace::DifferentialTwist;
 trajectory.command = {1.0, 0.0, true, 3.0, 0.0};
 trajectory.samples = {
   {0.0, {1.0, 1.0}, 0.0, 1.0, 0.0, 0.0, 0.0},
   {0.5, {1.5, 1.0}, 0.0, 1.0, 0.0, 0.0, 0.0},
   {1.0, {2.0, 1.0}, 0.0, 1.0, 0.0, 0.0, 0.0},
 };
 return trajectory;
}

} // namespace

int main() {
 sim::GridMap map(100, 100, 0.1);
 const auto trajectory = straight_trajectory();

 const auto accepted = sim::validate_executable_trajectory(trajectory, map, {}, 0.1, 0.2, 0.1);
 CHECK(accepted.accepted());
 CHECK(std::isinf(accepted.min_dynamic_clearance_m));

 auto unordered = trajectory;
 unordered.samples[2].t_s = 0.5;
 CHECK(sim::validate_executable_trajectory(unordered, map, {}, 0.1, 0.2, 0.1).verdict ==
       sim::TrajectoryVerdict::InvalidStructure);

 auto occupied_map = map;
 occupied_map.addBox(1.4, 0.9, 1.6, 1.1);
 CHECK(sim::validate_executable_trajectory(trajectory, occupied_map, {}, 0.1, 0.2, 0.1).verdict ==
       sim::TrajectoryVerdict::StaticCollision);

 auto swept_static = trajectory;
 swept_static.samples = {
   {0.0, {3.0, 1.0}, 0.0, 1.0, 0.0, 0.0, 0.0},
   {1.0, {5.0, 1.0}, 0.0, 1.0, 0.0, 0.0, 0.0},
 };
 auto midpoint_map = map;
 midpoint_map.addBox(3.9, 0.9, 4.1, 1.1);
 CHECK(midpoint_map.isFree(3.0, 1.0));
 CHECK(midpoint_map.isFree(5.0, 1.0));
 CHECK(sim::validate_executable_trajectory(swept_static, midpoint_map, {}, 0.1, 0.2, 0.1).verdict ==
       sim::TrajectoryVerdict::StaticCollision);

 CHECK(
   sim::validate_executable_trajectory(trajectory, map, {}, 0.1, 0.2, 0.1, [](sim::Vec2 from, sim::Vec2 to) {
    return !(from.x < 1.5 && to.x >= 1.5);
   }).verdict == sim::TrajectoryVerdict::StaticCollision);

 // Both discrete endpoints are clear, but ego and actor cross at the same
 // time between them. The swept same-time oracle must catch that collision.
 auto crossing = trajectory;
 crossing.samples = {
   {0.0, {0.5, 1.0}, 0.0, 1.0, 0.0, 0.0, 0.0},
   {1.0, {1.5, 1.0}, 0.0, 1.0, 0.0, 0.0, 0.0},
 };
 const std::vector<std::vector<sim::DynamicObjectState>> crossing_frames{
   {{{1.0, 0.0}, {0.0, 2.0}, 0.1}},
   {{{1.0, 2.0}, {0.0, 2.0}, 0.1}},
 };
 const auto swept = sim::validate_executable_trajectory(crossing, map, crossing_frames, 1.0, 0.1, 0.0);
 CHECK(swept.verdict == sim::TrajectoryVerdict::DynamicCollision);
 CHECK(swept.min_dynamic_clearance_m < 0.0);

 // A short local rollout at a stop-turn-go corner ends in a stationary
 // hold, but the committed behavior contract continues on the next leg. A
 // moving actor may invade that artificial hold while the exact remaining
 // timed execution stays clear. Validate the candidate joined to the
 // current reference suffix rather than replaying elapsed motion or assuming
 // the corner is occupied forever.
 sim::ExecutableTrajectory corner_action;
 corner_action.command_space = sim::LocalCommandSpace::DifferentialTwist;
 corner_action.command = {1.0, 0.0, true, 1.0, 0.0};
 corner_action.samples = {
 {0.0, {1.0, 1.0}, 0.0, 1.0, 0.0, 0.0, 0.0},
 {0.1, {1.1, 1.0}, 0.0, 1.0, 0.0, 0.0, 0.0},
 {1.0, {1.1, 1.0}, 0.0, 0.0, 0.0, 0.0, 0.0},
 };
 std::vector<std::vector<sim::DynamicObjectState>> corner_frames;
 for (int frame = 0; frame <= 10; ++frame) {
 const double time_s = 0.1 * frame;
 corner_frames.push_back({{{1.1, 2.0 - time_s}, {0.0, -1.0}, 0.1}});
 }
 sim::TrajectoryConstraints timed_commitment;
 timed_commitment.committed_reference_path = {
 {1.0, 1.0}, {1.1, 1.0}, {2.0, 1.0},
 };
 timed_commitment.committed_tracking_corridor_m = 0.15;
 timed_commitment.committed_reference_trajectory = {
 {0.0, {1.0, 1.0}},
 {0.1, {1.1, 1.0}},
 {1.0, {2.0, 1.0}},
 };
 const auto timed_corner = sim::validate_executable_trajectory(
 corner_action, map, corner_frames, 0.1, 0.1, 0.1, {},
 timed_commitment);
 CHECK(timed_corner.accepted());
 CHECK(timed_corner.committed_transit());
 CHECK(timed_corner.preserves_committed_schedule);
 CHECK(timed_corner.diagnostic.find("timed crossing") != std::string::npos);
 const auto timed_corner_without_actor = sim::validate_executable_trajectory(
 corner_action, map, {}, 0.1, 0.1, 0.1, {}, timed_commitment);
 CHECK(timed_corner_without_actor.committed_transit());
 CHECK(timed_corner_without_actor.preserves_committed_schedule);
 auto off_schedule = timed_commitment;
 off_schedule.committed_reference_trajectory[1].p = {1.5, 1.0};
 CHECK(!sim::validate_executable_trajectory(
 corner_action, map, corner_frames, 0.1, 0.1, 0.1, {},
 off_schedule).accepted());

 // A locally safe, higher-score detour must not outrank a lower-score
 // candidate that preserves the complete crossing schedule. The detour is
 // within the spatial tube for this command interval, but joining it back to
 // the timed reference intersects the actor later. Treating both verdicts
 // as equivalent spends an admitted traffic window while apparently safe.
 std::vector<std::vector<sim::DynamicObjectState>> schedule_frames(
 11, {{{1.5, 1.31}, {}, 0.1}});
 auto scheduled_action = corner_action;
 scheduled_action.command.score = 1.0;
 auto local_detour = corner_action;
 local_detour.command.score = 10.0;
 local_detour.samples = {
 {0.0, {1.0, 1.0}, 0.0, 0.0, 0.0, 0.0, 0.0},
 {0.1, {1.1, 1.14}, 0.0, 0.0, 0.0, 0.0, 0.0},
 {1.0, {1.1, 1.14}, 0.0, 0.0, 0.0, 0.0, 0.0},
 };
 const auto detour_validation = sim::validate_executable_trajectory(
 local_detour, map, schedule_frames, 0.1, 0.1, 0.1, {},
 timed_commitment);
 CHECK(detour_validation.accepted());
 CHECK(!detour_validation.preserves_committed_schedule);
 sim::DwaResult scheduled_proposals;
 scheduled_proposals.best = local_detour;
 scheduled_proposals.rollouts = {
 {local_detour, false}, {scheduled_action, false},
 };
 const auto scheduled_selection = sim::select_executable_trajectory(
 scheduled_proposals, map, schedule_frames, 0.1, 0.1, 0.1, {},
 timed_commitment);
 CHECK(scheduled_selection.validation.preserves_committed_schedule);
 CHECK(scheduled_selection.trajectory.command.score == 1.0);

 // Among two schedule-preserving candidates, the one that establishes the
 // upcoming curve heading wins before ordinary DWA score. Waiting until
 // position leaves the tube is too late for a rate-limited yaw actuator.
 auto curve_commitment = timed_commitment;
 curve_commitment.committed_reference_path = {
 {1.0, 1.0}, {1.1, 1.0}, {1.2, 1.1},
 };
 curve_commitment.committed_tracking_corridor_m = 0.20;
 curve_commitment.committed_reference_trajectory = {
 {0.0, {1.0, 1.0}},
 {0.1, {1.1, 1.0}},
 {0.2, {1.2, 1.1}},
 {1.0, {2.0, 1.9}},
 };
 auto curve_aligned = corner_action;
 curve_aligned.command.score = 1.0;
 curve_aligned.samples[1].heading_rad = sim::kPi * 0.25;
curve_aligned.samples[2].heading_rad = sim::kPi * 0.25;
 auto curve_late = corner_action;
 curve_late.command.score = 10.0;
 curve_late.samples[1].heading_rad = 0.0;
 sim::DwaResult curve_proposals;
 curve_proposals.best = curve_late;
 curve_proposals.rollouts = {
 {curve_late, false}, {curve_aligned, false},
 };
 const auto curve_selection = sim::select_executable_trajectory(
 curve_proposals, map, {}, 0.1, 0.1, 0.1, {},
 curve_commitment);
 CHECK(curve_selection.validation.preserves_committed_schedule);
 CHECK(curve_selection.trajectory.command.score == 1.0);
const auto late_curve_validation = sim::validate_executable_trajectory(
 curve_late, map, {}, 0.1, 0.1, 0.1, {}, curve_commitment);
CHECK(curve_selection.validation.committed_tracking_cost <
 late_curve_validation.committed_tracking_cost);

 // A stop-turn-go schedule contains a velocity-state contract as well as a
 // pose contract. Two candidates can land at the same next pose while only
 // one has shed enough speed for the upcoming dwell. Schedule arbitration
 // must choose that candidate before an ordinary DWA score rewards the
 // faster overshoot.
 auto stop_commitment = timed_commitment;
 stop_commitment.committed_reference_trajectory = {
 {0.0, {1.0, 1.0}},
 {0.1, {1.1, 1.0}},
 {0.2, {1.1, 1.0}},
 {1.0, {2.0, 1.0}},
 };
 auto settled_for_stop = corner_action;
 settled_for_stop.command.score = 1.0;
 settled_for_stop.samples[1].forward_mps = 0.0;
 settled_for_stop.samples[2].forward_mps = 0.0;
 auto fast_at_stop = settled_for_stop;
 fast_at_stop.command.score = 10.0;
 fast_at_stop.samples[1].forward_mps = 1.0;
 fast_at_stop.samples[2].forward_mps = 1.0;
 sim::DwaResult stop_proposals;
 stop_proposals.best = fast_at_stop;
 stop_proposals.rollouts = {
 {fast_at_stop, false}, {settled_for_stop, false},
 };
 const auto stop_selection = sim::select_executable_trajectory(
 stop_proposals, map, {}, 0.1, 0.1, 0.1, {}, stop_commitment);
 CHECK(stop_selection.validation.preserves_committed_schedule);
 CHECK(stop_selection.trajectory.command.score == 1.0);
 const auto fast_stop_validation = sim::validate_executable_trajectory(
 fast_at_stop, map, {}, 0.1, 0.1, 0.1, {}, stop_commitment);
 CHECK(stop_selection.validation.committed_speed_error_mps <
 fast_stop_validation.committed_speed_error_mps);

 // A merely local-safe action has no authority to spend an active behavior
 // schedule. With no schedule-preserving candidate, selection must request a
 // controlled stop/reassessment rather than publish the detour.
 sim::DwaResult detour_only;
 detour_only.best = local_detour;
 detour_only.rollouts = {{local_detour, false}};
 const auto rejected_detour = sim::select_executable_trajectory(
 detour_only, map, schedule_frames, 0.1, 0.1, 0.1, {},
 timed_commitment);
 CHECK(!rejected_detour.validation.accepted());
 CHECK(rejected_detour.validation.verdict ==
 sim::TrajectoryVerdict::BehaviorConstraint);
 CHECK(rejected_detour.validation.diagnostic.find(
 "committed schedule has no executable candidate") != std::string::npos);

 // On the following control cycle both the admitted trajectory and the
 // forecast clock advance once. The remaining suffix starts exactly at the
 // measured reference pose and remains a valid execution; elapsed motion is
 // neither replayed nor charged twice.
 auto next_cycle_constraints = timed_commitment;
 next_cycle_constraints.committed_reference_trajectory =
 sim::remaining_timed_trajectory(
 timed_commitment.committed_reference_trajectory, 0.1);
 next_cycle_constraints.committed_reference_path = {
 {1.1, 1.0}, {2.0, 1.0},
 };
 sim::ExecutableTrajectory next_cycle_action = scheduled_action;
 next_cycle_action.samples = {
 {0.0, {1.1, 1.0}, 0.0, 0.0, 0.0, 0.0, 0.0},
 {0.1, {1.2, 1.0}, 0.0, 1.0, 0.0, 0.0, 0.0},
 {1.0, {1.2, 1.0}, 0.0, 0.0, 0.0, 0.0, 0.0},
 };
 const auto next_cycle = sim::validate_executable_trajectory(
 next_cycle_action, map, schedule_frames, 0.1, 0.1, 0.1, {},
 next_cycle_constraints);
 CHECK(next_cycle.accepted());
 CHECK(next_cycle.preserves_committed_schedule);

 // An existing soft-margin incursion has a separate contract: physical
 // physical overlap may never occur, motion must be real, and the endpoint
 // must increase separation. A stationary hold cannot claim this verdict.
 auto escape = trajectory;
 escape.samples = {
   {0.0, {1.0, 3.0}, 0.0, 1.0, 0.0, 0.0, 0.0},
   {1.0, {2.0, 3.0}, 0.0, 1.0, 0.0, 0.0, 0.0},
 };
 const std::vector<std::vector<sim::DynamicObjectState>> fixed_threat{
   {{{0.5, 3.0}, {}, 0.2}},
   {{{0.5, 3.0}, {}, 0.2}},
 };
 const auto escape_result = sim::validate_executable_trajectory(escape, map, fixed_threat, 1.0, 0.2, 0.5);
 CHECK(escape_result.accepted());
 CHECK(escape_result.minimum_risk_escape());
 auto trapped_hold = escape;
 trapped_hold.command.v = 0.0;
 trapped_hold.samples.back().position = trapped_hold.samples.front().position;
 trapped_hold.samples.back().forward_mps = 0.0;
 const auto hold_result = sim::validate_executable_trajectory(trapped_hold, map, fixed_threat, 1.0, 0.2, 0.5);
 CHECK(!hold_result.accepted());
 CHECK(hold_result.verdict == sim::TrajectoryVerdict::DynamicCollision);

 sim::DwaResult proposals;
 auto unsafe_high_score = crossing;
 unsafe_high_score.command.score = 10.0;
 auto safe_lower_score = escape;
 safe_lower_score.command.score = 5.0;
 safe_lower_score.samples = {
 {0.0, {4.0, 4.0}, 0.0, 1.0, 0.0, 0.0, 0.0},
 {1.0, {5.0, 4.0}, 0.0, 1.0, 0.0, 0.0, 0.0},
 };
 proposals.best = unsafe_high_score;
 proposals.rollouts = {
 {unsafe_high_score, false},
 {safe_lower_score, false},
 };
 const auto selection = sim::select_executable_trajectory(
 proposals, map, crossing_frames, 1.0, 0.1, 0.0);
 CHECK(selection.validation.accepted());
 CHECK(selection.accepted_candidates == 1);
 CHECK(selection.trajectory.command.score == 5.0);
 CHECK(selection.trajectory.samples.front().position.x == 4.0);

 const std::vector<std::vector<sim::DynamicObjectState>> empty_prediction(2);
 CHECK(sim::validate_executable_trajectory(
 trajectory, map, empty_prediction, 0.25, 0.2, 0.1).accepted());
 const std::vector<std::vector<sim::DynamicObjectState>> short_prediction{
 {{{50.0, 50.0}, {}, 0.1}},
 {{{50.0, 50.0}, {}, 0.1}},
 };
 CHECK(sim::validate_executable_trajectory(trajectory, map, short_prediction, 0.25, 0.2, 0.1).verdict ==
       sim::TrajectoryVerdict::PredictionHorizonShort);

 auto steering = trajectory;
 steering.command_space = sim::LocalCommandSpace::SteeringAngle;
 steering.command.w = 0.42;
 for (auto& sample : steering.samples)
  sample.steering_angle_rad = 0.42;
 const auto steering_result = sim::validate_executable_trajectory(steering, map, {}, 0.1, 0.2, 0.1);
 CHECK(steering_result.accepted());
 CHECK(steering.command_space == sim::LocalCommandSpace::SteeringAngle);
 CHECK(steering.command.w == 0.42);
 CHECK(steering.samples.back().steering_angle_rad == 0.42);

 // Planner-score equivalence is limited to floating-point noise and has an
 // explicit safety-oriented secondary order. Reversing enumeration cannot
 // turn an ulp-level score change into a different command.
 const sim::DwaCmd near_tie_turn{
 0.8, 0.6, true, std::nextafter(5.0, 6.0), 0.0};
 const sim::DwaCmd near_tie_straight{0.8, 0.0, true, 5.0, 0.0};
 CHECK(sim::dwa_command_preferred(near_tie_straight, near_tie_turn));
 CHECK(!sim::dwa_command_preferred(near_tie_turn, near_tie_straight));
 auto meaningful_score = near_tie_turn;
 meaningful_score.score += 1e-8;
 CHECK(sim::dwa_command_preferred(meaningful_score, near_tie_straight));

 // Candidate stability uses an independent analytic envelope: the terrain
 // gravity component and v*w are compared at the same sample time against
 // the support-width/COM wheel-lift angle. Low speed and a counter-turn stay
 // below the boundary; speed, same-direction turn, or a higher COM do not.
 const auto side_slope = [](sim::Vec2) {
 return sim::TerrainSample{0.0, 0.0, 0.30, 0.0};
 };
 sim::TrajectoryConstraints stability;
 stability.terrain_sample = side_slope;
 stability.support_width_m = 0.60;
 stability.support_polygon = {
 {-0.30, -0.30}, {0.30, -0.30}, {0.30, 0.30}, {-0.30, 0.30},
 };
 stability.center_of_mass_height_m = 0.40;
 stability.max_dynamic_rollover_risk = 0.50;
 auto low_speed_turn = trajectory;
 for (auto& sample : low_speed_turn.samples) {
 sample.forward_mps = 0.20;
 sample.yaw_rate_radps = 0.80;
 }
 const auto low_speed_stability = sim::validate_executable_trajectory(
 low_speed_turn, map, {}, 0.1, 0.2, 0.1, {}, stability);
 CHECK(low_speed_stability.accepted());
 CHECK(low_speed_stability.max_dynamic_rollover_risk < 0.50);
 auto fast_turn = low_speed_turn;
 for (auto& sample : fast_turn.samples)
 sample.forward_mps = 1.0;
 const auto fast_stability = sim::validate_executable_trajectory(
 fast_turn, map, {}, 0.1, 0.2, 0.1, {}, stability);
 CHECK(fast_stability.verdict == sim::TrajectoryVerdict::DynamicStabilityViolation);
 CHECK(fast_stability.max_dynamic_rollover_risk > 0.50);
 auto counter_turn = fast_turn;
 for (auto& sample : counter_turn.samples)
 sample.yaw_rate_radps = -0.80;
 CHECK(sim::validate_executable_trajectory(
 counter_turn, map, {}, 0.1, 0.2, 0.1, {}, stability).accepted());
 auto high_com_stability = stability;
 high_com_stability.center_of_mass_height_m = 0.60;
 CHECK(sim::validate_executable_trajectory(
 low_speed_turn, map, {}, 0.1, 0.2, 0.1, {}, high_com_stability).verdict ==
 sim::TrajectoryVerdict::DynamicStabilityViolation);

 // Passive safety includes the braking tail. A constant-speed nominal
 // rollout can remain inside the support polygon while the required stop
 // transfers the load line beyond the short front edge of a three-point
 // chassis; score arbitration must never publish that command.
 auto stop_stability = stability;
 stop_stability.terrain_sample = [](sim::Vec2) { return sim::TerrainSample{}; };
 stop_stability.support_polygon = {
 {-0.28, 0.0}, {0.12, -0.29}, {0.12, 0.29},
 };
 stop_stability.center_of_mass_height_m = 0.45;
 stop_stability.max_dynamic_rollover_risk = 0.70;
 stop_stability.emergency_decel_mps2 = 2.0;
 const auto unstable_stop = sim::validate_executable_trajectory(
 trajectory, map, {}, 0.1, 0.2, 0.1, {}, stop_stability);
 CHECK(unstable_stop.verdict == sim::TrajectoryVerdict::UnsafeEmergencyStop);
 CHECK(unstable_stop.diagnostic.find("support-polygon") != std::string::npos);

 // A behavior stop plane is independent of occupancy. The same statically
 // and dynamically clear rollout is refused when it crosses the uncommitted
 // virtual wall, while an approach ending on the allowed side is accepted.
 sim::TrajectoryConstraints stop_plane;
 stop_plane.stop_plane_active = true;
 stop_plane.stop_point = {1.6, 1.0};
 stop_plane.stop_direction = {1.0, 0.0};
 stop_plane.stop_tolerance_m = 0.02;
 CHECK(sim::validate_executable_trajectory(
 trajectory, map, {}, 0.1, 0.2, 0.1, {}, stop_plane).verdict ==
 sim::TrajectoryVerdict::BehaviorConstraint);
 auto stop_approach = trajectory;
 stop_approach.samples.back().position.x = 1.61;
 CHECK(sim::validate_executable_trajectory(
 stop_approach, map, {}, 0.1, 0.2, 0.1, {}, stop_plane).accepted());

 // A receding-horizon stop approach validates the next command plus its full
 // braking tail. Holding a positive candidate for the complete route-to-line
 // duration before adding that tail would reject every useful approach and
 // leave only a permanent zero command upstream of the stop pose.
 sim::LocalPlanningRequest stop_request;
 stop_request.position = {1.0, 3.0};
 stop_request.heading_rad = 0.0;
 stop_request.target = {1.6, 3.0};
 stop_request.max_speed_mps = 0.5;
 stop_request.max_yaw_rate_radps = 1.0;
 stop_request.max_accel_mps2 = 1.0;
 stop_request.max_decel_mps2 = 1.0;
 stop_request.max_yaw_accel_radps2 = 2.0;
 stop_request.prediction_horizon_s = 3.0;
 stop_request.step_s = 0.1;
 stop_request.robot_radius_m = 0.2;
 stop_request.differential_track_m = 0.5;
 stop_request.max_wheel_speed_mps = 1.0;
 stop_request.stop_plane_active = true;
 stop_request.stop_point = stop_request.target;
 stop_request.stop_direction = {1.0, 0.0};
 stop_request.emergency_stop_required = true;
 stop_request.emergency_stop_delay_s = 0.1;
 stop_request.emergency_decel_mps2 = 1.0;
 const auto stop_progress = sim::LocalPlanningSystem{}.plan(
 stop_request, map, {});
 CHECK(stop_progress.selection.validation.accepted());
 CHECK(stop_progress.selection.trajectory.command.v > 0.0);
 for (const auto& sample : stop_progress.selection.trajectory.samples)
 CHECK(sample.position.x <= stop_request.stop_point.x + 1e-9);

 // A short nominal rollout may be collision-free while the physically
 // required braking tail is not. The hard validator must reject that command
 // before score arbitration.
 auto cannot_stop = trajectory;
 cannot_stop.samples = {
 {0.0, {6.0, 2.0}, 0.0, 1.0, 0.0, 0.0, 0.0},
 {0.2, {6.2, 2.0}, 0.0, 1.0, 0.0, 0.0, 0.0},
 };
 auto braking_map = map;
 braking_map.addBox(6.45, 1.9, 6.65, 2.1);
 CHECK(sim::validate_executable_trajectory(
 cannot_stop, braking_map, {}, 0.1, 0.2, 0.1).accepted());
 sim::TrajectoryConstraints braking;
 braking.emergency_decel_mps2 = 1.0;
 CHECK(sim::validate_executable_trajectory(
 cannot_stop, braking_map, {}, 0.1, 0.2, 0.1, {}, braking).verdict ==
 sim::TrajectoryVerdict::UnsafeEmergencyStop);
 braking.committed_transit = true;
 braking.committed_progress_floor_mps = 0.5;
 const auto committed_stop = sim::validate_executable_trajectory(
 cannot_stop, braking_map, {}, 0.1, 0.2, 0.1, {}, braking);
 CHECK(committed_stop.accepted());
 CHECK(committed_stop.committed_transit());

 // A configured brake contract with zero safe authority is a hard physical
 // result (for example an overloaded downhill support envelope), not the
 // sentinel for "braking model disabled". Moving commands must be rejected.
 auto no_safe_brake = braking;
 no_safe_brake.committed_transit = false;
 no_safe_brake.emergency_stop_required = true;
 no_safe_brake.emergency_decel_mps2 = 0.0;
 const auto no_brake_result = sim::validate_executable_trajectory(
 cannot_stop, map, {}, 0.1, 0.2, 0.1, {}, no_safe_brake);
 CHECK(no_brake_result.verdict == sim::TrajectoryVerdict::UnsafeEmergencyStop);
 CHECK(no_brake_result.diagnostic.find("no safe braking authority") != std::string::npos);

 // Braking authority is evaluated along the tail, not borrowed from a remote
 // lookahead point. Flat ground can stop this trajectory inside the forecast
 // even when the scalar fallback represents a later low-authority slope. If
 // the predicted stop actually crosses onto that slope, the same forecast is
 // correctly too short.
 std::vector<std::vector<sim::DynamicObjectState>> three_second_forecast(
 31, {{{50.0, 50.0}, {}, 0.1}});
 sim::TrajectoryConstraints spatial_braking;
 spatial_braking.emergency_stop_required = true;
 spatial_braking.emergency_stop_delay_s = 0.1;
 spatial_braking.emergency_decel_mps2 = 0.1;
 spatial_braking.safe_stop_decel = [](sim::Vec2, double) { return 2.0; };
 CHECK(sim::validate_executable_trajectory(
 trajectory, map, three_second_forecast, 0.1, 0.2, 0.1, {},
 spatial_braking).accepted());
 spatial_braking.safe_stop_decel = [](sim::Vec2 position, double) {
 return position.x < 1.15 ? 2.0 : 0.1;
 };
 CHECK(sim::validate_executable_trajectory(
 trajectory, map, three_second_forecast, 0.1, 0.2, 0.1, {},
 spatial_braking).verdict == sim::TrajectoryVerdict::PredictionHorizonShort);

 // Differential braking acts on each wheel. With asymmetric reverse wheel
 // speeds the slow wheel stops first and the other wheel sweeps a longer arc;
 // scaling body v and yaw together would incorrectly accept this obstacle.
 auto asymmetric_reverse = trajectory;
 asymmetric_reverse.samples = {
 {0.0, {6.0, 6.0}, 0.0, -0.294, 0.0, 0.72, 0.0},
 {0.2, {6.0, 6.0}, 0.0, -0.294, 0.0, 0.72, 0.0},
 };
 sim::GridMap asymmetric_map(1000, 1000, 0.01);
 asymmetric_map.addBox(5.935, 5.985, 5.945, 5.995);
 sim::TrajectoryConstraints differential_braking;
 differential_braking.emergency_decel_mps2 = 1.0;
 differential_braking.differential_track_m = 0.5;
 CHECK(sim::validate_executable_trajectory(
 asymmetric_reverse, asymmetric_map, {}, 0.01, 0.2, 0.1, {},
 differential_braking).verdict == sim::TrajectoryVerdict::UnsafeEmergencyStop);

 // Braking includes the stationary hold after velocity reaches zero. If a
 // moving actor will invade that hold pose while the nominal trajectory stays
 // fully clear and increases separation, the latter is a named minimum-risk
 // escape rather than an unclassified safety waiver.
 auto leave_invasion = trajectory;
 leave_invasion.samples = {
 {0.0, {1.0, 3.0}, 0.0, 1.0, 0.0, 0.0, 0.0},
 {1.0, {2.0, 3.0}, 0.0, 1.0, 0.0, 0.0, 0.0},
 {2.0, {3.0, 3.0}, 0.0, 1.0, 0.0, 0.0, 0.0},
 };
 const std::vector<std::vector<sim::DynamicObjectState>> invading_hold{
 {{{0.0, 3.0}, {0.75, 0.0}, 0.2}},
 {{{0.75, 3.0}, {0.75, 0.0}, 0.2}},
 {{{1.5, 3.0}, {0.75, 0.0}, 0.2}},
 };
 sim::TrajectoryConstraints escape_braking;
 escape_braking.emergency_decel_mps2 = 1.0;
 const auto leave_result = sim::validate_executable_trajectory(
 leave_invasion, map, invading_hold, 1.0, 0.2, 0.1, {}, escape_braking);
 CHECK(leave_result.accepted());
 CHECK(leave_result.minimum_risk_escape());

 // The maintained-intent forecast can be longer than one local rollout. A
 // stationary candidate that is clear over its one-second action horizon is
 // still unsafe when a predicted actor reaches its emergency hold pose at
 // two seconds. This is an independent passive-safety contract, rather than
 // a mission score or a simulator collision assertion.
 auto late_invasion = trajectory;
 late_invasion.samples = {
{0.0, {4.0, 3.0}, 0.0, 1.0, 0.0, 0.0, 0.0},
{1.0, {5.0, 3.0}, 0.0, 1.0, 0.0, 0.0, 0.0},
 };
 const std::vector<std::vector<sim::DynamicObjectState>> late_frames{
{{{4.5, 5.0}, {0.0, -1.0}, 0.2}},
{{{4.5, 4.0}, {0.0, -1.0}, 0.2}},
{{{4.5, 3.0}, {0.0, -1.0}, 0.2}},
 };
 sim::TrajectoryConstraints late_stop;
 late_stop.emergency_decel_mps2 = 1.0;
 const auto late_result = sim::validate_executable_trajectory(
late_invasion, map, late_frames, 1.0, 0.2, 0.05, {}, late_stop);
 CHECK(late_result.verdict == sim::TrajectoryVerdict::UnsafeEmergencyStop);

 // A zero progress floor stages a physically checked in-place alignment; it
 // is not forward contingency progress and cannot waive a dynamic overlap.
 auto stationary_threat = late_invasion;
 stationary_threat.command = {0.0, 0.0, true, 0.0};
 stationary_threat.samples = {
 {0.0, {4.0, 3.0}, 0.0, 0.0, 0.0, 0.0, 0.0},
 {1.0, {4.0, 3.0}, 0.0, 0.0, 0.0, 0.0, 0.0},
 };
 const std::vector<std::vector<sim::DynamicObjectState>> overlapping_actor{
 {{{4.0, 3.0}, {}, 0.2}},
 {{{4.0, 3.0}, {}, 0.2}},
 };
 sim::TrajectoryConstraints zero_floor_commitment;
 zero_floor_commitment.committed_transit = true;
 zero_floor_commitment.committed_progress_floor_mps = 0.0;
 CHECK(sim::validate_executable_trajectory(
 stationary_threat, map, overlapping_actor, 1.0, 0.2, 0.1, {},
 zero_floor_commitment).verdict == sim::TrajectoryVerdict::DynamicCollision);

 // When every candidate is already inside a soft dynamic margin, select the
 // physically safe escape that gains the most separation before considering
 // the ordinary route score. A goal-biased score must not steer a contingency
 // along the hazard when a stronger escape exists.
 auto weak_escape = escape;
 weak_escape.command.score = 100.0;
 weak_escape.samples.back().position = {1.1, 3.0};
 auto strong_escape = escape;
 strong_escape.command.score = 1.0;
 strong_escape.samples.back().position = {2.0, 3.0};
 sim::DwaResult escape_proposals;
 escape_proposals.best = weak_escape;
 escape_proposals.rollouts = {{weak_escape, false}, {strong_escape, false}};
 const auto safest_escape = sim::select_executable_trajectory(
 escape_proposals, map, fixed_threat, 1.0, 0.2, 0.5);
 CHECK(safest_escape.validation.minimum_risk_escape());
 CHECK(safest_escape.trajectory.samples.back().position.x == 2.0);

 // A long soft-inflation incursion can require two receding-horizon stages:
 // first rotate in physically free space, then translate out. Refusing every
 // rollout whose first endpoint is still in the soft margin deadlocks a real
 // differential robot even though the physical map is clear.
 sim::GridMap preferred(100, 100, 0.1);
 preferred.addBox(3.5, 4.5, 6.5, 5.5);
 sim::GridMap physical(100, 100, 0.1);
 sim::LocalPlanningRequest staged_request;
 staged_request.command_space = sim::LocalCommandSpace::DifferentialTwist;
 staged_request.position = {5.0, 5.0};
 staged_request.heading_rad = 0.0;
 staged_request.target = {5.0, 7.0};
 staged_request.max_speed_mps = 0.5;
 staged_request.max_yaw_rate_radps = 1.6;
 staged_request.max_accel_mps2 = 1.0;
 staged_request.max_yaw_accel_radps2 = 4.0;
 staged_request.prediction_horizon_s = 1.0;
 staged_request.step_s = 0.1;
 staged_request.robot_radius_m = 0.2;
 staged_request.differential_track_m = 0.5;
 staged_request.max_wheel_speed_mps = 2.0;
 const auto staged = sim::LocalPlanningSystem{}.plan(
 staged_request, preferred, {}, {}, &physical);
 CHECK(staged.selection.validation.accepted());
 CHECK(staged.selection.validation.minimum_risk_escape());
 CHECK(staged.selection.validation.diagnostic.find("in-place alignment") != std::string::npos);
 CHECK(std::abs(staged.selection.trajectory.command.v) < 1e-12);
CHECK(std::abs(staged.selection.trajectory.command.w) > 0.0);

 // A conservative raster inflation may mark both the current pose and a
 // physically valid recovery target as preferred-cost occupancy near a box
 // corner. The first in-place turn is judged at the current physical pose;
 // requiring the eventual target to already occupy a preferred free cell
 // prevents the staged recovery from ever starting.
 sim::GridMap target_in_margin(100, 100, 0.1);
 target_in_margin.addBox(3.5, 4.5, 6.5, 7.5);
 const auto staged_toward_margin = sim::LocalPlanningSystem{}.plan(
 staged_request, target_in_margin, {}, {}, &physical);
 CHECK(staged_toward_margin.selection.validation.accepted());
 CHECK(staged_toward_margin.selection.validation.minimum_risk_escape());
 CHECK(staged_toward_margin.selection.validation.diagnostic.find(
 "in-place alignment") != std::string::npos);

 // RO-1 turns around a driven axle that is ahead of base_link. Near a
 // rasterized margin boundary, turning the wrong way can move base_link into
 // a free cell even while the body heading moves away from the route. That
 // cell transition is rigid-transform motion, not a translational escape;
 // recovery must choose the turn that improves the axle-referenced exit
 // heading and let the physical swept validator guard the body arc.
 sim::GridMap axle_margin(160, 120, 0.1);
 axle_margin.addBox(7.0, 0.0, 9.0, 11.0);
 sim::GridMap axle_physical(160, 120, 0.1);
 axle_physical.addBox(8.0, 0.0, 9.0, 11.0);
 auto axle_request = staged_request;
 axle_request.position = {7.0002, 10.4229};
 axle_request.heading_rad = 1.24303;
 axle_request.target = {6.875, 11.125};
 axle_request.prediction_horizon_s = 1.2;
 axle_request.max_yaw_rate_radps = 0.8;
 axle_request.drive_axle_x_m = 0.12;
 axle_request.robot_radius_m = 0.38;
 axle_request.committed_no_return = true;
 const auto axle_alignment = sim::LocalPlanningSystem{}.plan(
 axle_request, axle_margin, {}, {}, &axle_physical);
 CHECK(axle_alignment.selection.validation.accepted());
 CHECK(axle_alignment.selection.validation.minimum_risk_escape());
 CHECK(axle_alignment.selection.validation.diagnostic.find(
 "in-place alignment") != std::string::npos);
 CHECK(std::abs(axle_alignment.selection.trajectory.command.v) < 1e-12);
 CHECK(axle_alignment.selection.trajectory.command.w > 0.0);

 // A zero-v command cannot be called an in-place recovery while measured
 // translation still needs more than the alignment displacement budget to
 // stop. The recovery must brake and replan from the stopped measurement.
 auto moving_staged_request = staged_request;
 moving_staged_request.forward_mps = -0.20;
 moving_staged_request.emergency_stop_delay_s = 0.20;
 moving_staged_request.emergency_decel_mps2 = 2.0;
 const auto moving_staged = sim::LocalPlanningSystem{}.plan(
 moving_staged_request, preferred, {}, {}, &physical);
 CHECK(moving_staged.selection.validation.diagnostic.find("in-place alignment") ==
 std::string::npos);

 // Once the measured body is already inside soft inflation, waiting cannot
 // restore clearance. If the ordinary envelope cannot move, recovery may
 // immediately use the physical map, but only for a trajectory that reduces
 // the incursion and passes the same swept validators. The selected gear and
 // exit direction remain active on the next cycle.
 sim::GridMap deep_preferred(100, 100, 0.1);
 deep_preferred.addBox(3.5, 4.5, 6.5, 7.5);
 auto deep_request = staged_request;
 deep_request.position = {5.05, 5.05};
 deep_request.heading_rad = sim::kPi * 0.5;
 deep_request.target = {5.05, 9.0};
 sim::LocalPlanningSystem delayed_recovery;
 const auto strict_static = [](sim::Vec2 from, sim::Vec2 to) {
 return std::hypot(to.x - from.x, to.y - from.y) <= 1e-9;
 };
 const auto physical_only = [](sim::Vec2, sim::Vec2) { return true; };
 sim::LocalPlanningOutput delayed;
 delayed = delayed_recovery.plan(
 deep_request, deep_preferred, {}, strict_static, &physical, physical_only);
 CHECK(delayed.selection.validation.minimum_risk_escape());
CHECK(delayed.selection.validation.diagnostic.find("in-place alignment") ==
 std::string::npos);
CHECK(std::abs(delayed.selection.trajectory.command.v) > 0.0);
 const auto persistent_recovery = delayed_recovery.plan(
 deep_request, deep_preferred, {}, strict_static, &physical, physical_only);
 CHECK(persistent_recovery.selection.validation.minimum_risk_escape());
CHECK(std::abs(persistent_recovery.selection.trajectory.command.v) > 0.0);

 // An upcoming conflict zone must not disable a physically validated escape
 // from unrelated static inflation several metres before the stop line. Only
 // commitment or the body actually entering the conflict forbids reversal.
 sim::GridMap reverse_preferred(100, 100, 0.1);
 reverse_preferred.addBox(5.0, 4.5, 6.5, 5.5);
 auto upcoming_crossing = staged_request;
 upcoming_crossing.position = {5.0, 5.0};
 upcoming_crossing.target = {7.0, 5.0};
 upcoming_crossing.max_speed_mps = 0.15;
 upcoming_crossing.crossing_relevant = true;
 const auto reverse_escape = sim::LocalPlanningSystem{}.plan(
 upcoming_crossing, reverse_preferred, {}, {}, &physical);
 CHECK(reverse_escape.selection.validation.accepted());
 CHECK(reverse_escape.selection.validation.minimum_risk_escape());
 CHECK(reverse_escape.selection.trajectory.command.v < 0.0);
 CHECK(std::abs(reverse_escape.selection.trajectory.command.v) <=
 upcoming_crossing.max_speed_mps + 1e-9);

 upcoming_crossing.committed_no_return = true;
 const auto no_reverse_inside = sim::LocalPlanningSystem{}.plan(
 upcoming_crossing, reverse_preferred, {}, {}, &physical);
CHECK(no_reverse_inside.selection.validation.accepted());
CHECK(no_reverse_inside.selection.validation.minimum_risk_escape());
CHECK(no_reverse_inside.selection.trajectory.command.v >= -1e-12);

 // Entering a preferred static inflation band must not invalidate a
 // previously accepted crossing commitment. The recovery cap is lower than
 // the committed floor here; planning must preserve a non-empty forward
 // window and validate the resulting physical-map transit instead of
 // returning an invalid empty proposal set.
 auto committed_inside = staged_request;
 committed_inside.position = {5.0, 5.0};
 committed_inside.heading_rad = sim::kPi * 0.5;
 committed_inside.target = {5.0, 7.0};
 committed_inside.max_speed_mps = 0.8;
 committed_inside.recovery_max_speed_mps = 0.2;
 committed_inside.forward_mps = 0.75;
 committed_inside.crossing_committed = true;
 committed_inside.committed_no_return = true;
 committed_inside.committed_progress_floor_mps = 0.7;
 const auto committed_escape = sim::LocalPlanningSystem{}.plan(
 committed_inside, deep_preferred, {}, {}, &physical, physical_only);
 CHECK(committed_escape.selection.validation.accepted());
 CHECK(committed_escape.selection.trajectory.command.valid);
 CHECK(committed_escape.selection.trajectory.command.v >= 0.7 - 1e-9);
CHECK(committed_escape.selection.validation.committed_transit());
CHECK(committed_escape.selection.validation.diagnostic.find("continues") !=
 std::string::npos);

 // A safety stop inside a no-return crossing restarts through the actuator's
 // next reachable command.  The nominal behavior floor is 0.15 m/s, but a
 // 0.4 m/s^2 drive can reach only 0.04 m/s in this 100 ms update.  Requiring
 // the nominal floor immediately would reject the complete candidate set and
 // deadlock the robot in the conflict zone.
 auto stopped_committed = committed_inside;
 stopped_committed.forward_mps = 0.0;
 stopped_committed.max_accel_mps2 = 0.4;
 stopped_committed.max_decel_mps2 = 0.8;
 stopped_committed.step_s = 0.1;
 stopped_committed.committed_progress_floor_mps = 0.15;
 const auto reachable_restart = sim::LocalPlanningSystem{}.plan(
 stopped_committed, deep_preferred, {}, {}, &physical, physical_only);
 CHECK(reachable_restart.selection.validation.accepted());
 CHECK(reachable_restart.selection.validation.committed_transit());
 CHECK(reachable_restart.selection.trajectory.command.v > 0.0);
 CHECK(reachable_restart.selection.trajectory.command.v <= 0.04 + 1e-9);

 // The current pose may still be outside soft inflation while every
 // reachable committed rollout enters it. The same physical-map contract
 // must keep the conflict-zone transit moving in this boundary case.
 sim::GridMap ahead_preferred(100, 100, 0.1);
 ahead_preferred.addBox(3.5, 5.2, 6.5, 7.5);
 CHECK(ahead_preferred.isFree(
 committed_inside.position.x, committed_inside.position.y));
 const auto committed_ahead = sim::LocalPlanningSystem{}.plan(
 committed_inside, ahead_preferred, {}, {}, &physical, physical_only);
 CHECK(committed_ahead.selection.validation.accepted());
 CHECK(committed_ahead.selection.validation.committed_transit());
 CHECK(committed_ahead.selection.trajectory.command.v >= 0.7 - 1e-9);
 CHECK(committed_ahead.selection.validation.diagnostic.find(
 "existing preferred static-margin incursion") == std::string::npos);

 // A physical-map fallback that begins outside preferred inflation cannot
 // claim that it exits an existing static-margin incursion merely because
 // its terminal sample is also free. The only authority available here is
 // the already-declared committed transit; without it the same fallback is
 // not a named static recovery.
 auto free_start_without_commitment = committed_inside;
 free_start_without_commitment.crossing_committed = false;
 free_start_without_commitment.committed_no_return = false;
 free_start_without_commitment.committed_progress_floor_mps = 0.0;
 const auto ordinary_free_start = sim::LocalPlanningSystem{}.plan(
 free_start_without_commitment, ahead_preferred, {}, {}, &physical,
 physical_only);
 CHECK(!ordinary_free_start.selection.validation.minimum_risk_escape());
 CHECK(ordinary_free_start.selection.validation.diagnostic.find(
 "existing preferred static-margin incursion") == std::string::npos);

 // A behavior-layer hold is not a safety proof. When a tracked object will
 // enter the stopped footprint, the local safety planner may temporarily use
 // its bounded collision-escape budget even though the ordinary speed cap is
 // zero. The selected reverse command still passes the physical map,
 // actuator window, complete forecast and braking-tail validator.
 sim::GridMap interaction_map(120, 100, 0.1);
 sim::LocalPlanningRequest held_request;
 held_request.position = {5.0, 5.0};
 held_request.heading_rad = 0.0;
 held_request.target = {8.0, 5.0};
 held_request.max_speed_mps = 0.0;
 held_request.collision_escape_max_speed_mps = 0.25;
 held_request.max_yaw_rate_radps = 1.0;
 held_request.max_accel_mps2 = 1.0;
 held_request.max_decel_mps2 = 1.0;
 held_request.max_yaw_accel_radps2 = 2.0;
 held_request.prediction_horizon_s = 3.0;
 held_request.step_s = 0.1;
 held_request.required_clearance_m = 0.10;
 held_request.robot_radius_m = 0.20;
 held_request.differential_track_m = 0.50;
 held_request.max_wheel_speed_mps = 1.0;
 held_request.emergency_stop_required = true;
 held_request.emergency_stop_delay_s = 0.1;
 held_request.emergency_decel_mps2 = 1.0;
 held_request.stop_plane_active = true;
 held_request.stop_point = {5.2, 5.0};
 held_request.stop_direction = {1.0, 0.0};
 std::vector<std::vector<sim::DynamicObjectState>> approaching_frames;
 for (int frame = 0; frame <= 30; ++frame) {
 const double time_s = held_request.step_s * frame;
 approaching_frames.push_back({
 {{6.5 - 0.35 * time_s, 5.0}, {-0.35, 0.0}, 0.20},
 });
 }
 const auto held_escape = sim::LocalPlanningSystem{}.plan(
 held_request, interaction_map, approaching_frames, {}, &interaction_map);
 if (!held_escape.selection.validation.accepted()) {
 std::cerr << "held escape rejected: verdict="
 << static_cast<int>(held_escape.selection.validation.verdict)
 << " diagnostic=" << held_escape.selection.validation.diagnostic
 << " rollouts=" << held_escape.proposals.rollouts.size() << '\n';
 }
 CHECK(held_escape.selection.validation.accepted());
 CHECK(held_escape.selection.validation.minimum_risk_escape());
 CHECK(held_escape.selection.trajectory.command.v < 0.0);
 CHECK(held_escape.selection.validation.escape_clearance_gain_m > 0.02);

 // Minimum-risk is comparative, not permission to collide. A second object
 // occupying the only reverse refuge makes both forward and reverse motion
 // unsafe; the planner must report no accepted escape instead of choosing
 // the least-overlapping candidate.
 auto blocked_frames = approaching_frames;
 for (auto& frame : blocked_frames)
 frame.push_back({{4.45, 5.0}, {}, 0.20});
 const auto blocked_escape = sim::LocalPlanningSystem{}.plan(
 held_request, interaction_map, blocked_frames, {}, &interaction_map);
 CHECK(!blocked_escape.selection.validation.accepted());
 CHECK(!blocked_escape.selection.validation.minimum_risk_escape());

 return 0;
}
