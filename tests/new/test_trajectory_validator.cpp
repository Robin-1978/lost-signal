#include "robot/autonomy/planning/validation/trajectory_validator.hpp"
#include "robot/autonomy/planning/local/local_planning_system.hpp"

#include <cmath>
#include <iostream>

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

 const std::vector<std::vector<sim::DynamicObjectState>> short_prediction(2);
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
 CHECK(staged.selection.trajectory.command.w > 0.0);

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

 // Soft-map recovery escalates only after a persistent standstill. Ten full
 // measured seconds preserve the normal clearance contract; the following
 // decision may use a physically validated incremental escape, and the
 // recovery mode remains active on the next cycle.
 sim::GridMap deep_preferred(100, 100, 0.1);
 deep_preferred.addBox(3.5, 4.5, 6.5, 7.5);
 auto deep_request = staged_request;
 deep_request.heading_rad = sim::kPi * 0.5;
 deep_request.target = {5.0, 9.0};
 sim::LocalPlanningSystem delayed_recovery;
 const auto strict_static = [](sim::Vec2 from, sim::Vec2 to) {
 return std::hypot(to.x - from.x, to.y - from.y) <= 1e-9;
 };
 const auto physical_only = [](sim::Vec2, sim::Vec2) { return true; };
 sim::LocalPlanningOutput delayed;
 for (int cycle = 0; cycle < 100; ++cycle) {
 delayed = delayed_recovery.plan(
 deep_request, deep_preferred, {}, strict_static, &physical, physical_only);
 CHECK(!delayed.selection.validation.accepted());
 }
 delayed = delayed_recovery.plan(
 deep_request, deep_preferred, {}, strict_static, &physical, physical_only);
 CHECK(delayed.selection.validation.minimum_risk_escape());
 CHECK(delayed.selection.validation.diagnostic.find("reduces") != std::string::npos);
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

 upcoming_crossing.body_in_conflict = true;
 const auto no_reverse_inside = sim::LocalPlanningSystem{}.plan(
 upcoming_crossing, reverse_preferred, {}, {}, &physical);
 CHECK(!no_reverse_inside.selection.validation.accepted());

 return 0;
}
