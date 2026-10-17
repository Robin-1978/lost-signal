#include "autonomy/planning/validation/trajectory_validator.hpp"

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

 return 0;
}
