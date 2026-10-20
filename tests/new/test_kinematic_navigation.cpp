#include "robot/autonomy/planning/local/ackermann_local.hpp"
#include "robot/autonomy/planning/local/local_planning_system.hpp"
#include "robot/autonomy/planning/global/global_planner.hpp"
#include "robot/autonomy/planning/global/hybrid_astar.hpp"
#include "robot/autonomy/control/follow.hpp"
#include "robot/autonomy/control/route_tracking_system.hpp"
#include "sim/hardware/vehicle/ackermann.hpp"
#include "sim/hardware/vehicle/holonomic.hpp"

#include <algorithm>
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

int main() {
 // Occupancy is discrete, but local-planner clearance is continuous inside
 // each free cell and measures distance to complete occupied cell squares.
 // The map boundary is implicit occupied space under the same contract.
 {
 sim::GridMap clearance_map(10, 10, 1.0);
 clearance_map.addBox(5.0, 3.0, 6.0, 4.0);
 CHECK(std::abs(clearance_map.clearance(4.25, 3.5) - 0.75) < 1e-12);
 CHECK(std::abs(clearance_map.clearance(4.75, 3.5) - 0.25) < 1e-12);
 CHECK(clearance_map.clearance(5.25, 3.5) == 0.0);
 CHECK(std::abs(clearance_map.clearance(0.2, 8.0) - 0.2) < 1e-12);
 CHECK(clearance_map.distanceToFreeSpace(5.25, 3.5) == 0.25);
 CHECK(clearance_map.distanceToFreeSpace(4.75, 3.5) == 0.0);
 }
 {
 sim::GridMap segment_map(10, 10, 1.0);
 segment_map.setOccupied(4.5, 4.5);
 CHECK(segment_map.segmentIsFree({1.5, 1.5}, {3.5, 3.5}));
 CHECK(!segment_map.segmentIsFree({1.5, 1.5}, {7.5, 7.5}));
 // Free endpoints do not prove a line free when it crosses an occupied
 // interior cell; this is the shared behavior/executor map contract.
 CHECK(segment_map.isFree(1.5, 4.5));
 CHECK(segment_map.isFree(7.5, 4.5));
 CHECK(!segment_map.segmentIsFree({1.5, 4.5}, {7.5, 4.5}));
 }

 sim::GridMap map(40, 40, 0.25);
 // A wall with a broad opening forces a real turn rather than a straight
 // interpolation, while leaving enough room for the configured car radius.
 map.addBox(4.0, 0.0, 4.5, 5.0);
 sim::HybridAStarConfig hybrid;
 hybrid.wheel_base_m = 0.62;
 hybrid.max_steer_rad = 0.55;
 hybrid.primitive_length_m = 0.375;
 const auto plan = sim::hybrid_astar_plan(map, {1.0, 1.0}, 0.0, {7.5, 7.5}, sim::kPi * 0.5, hybrid);
 CHECK(plan.ok);
 CHECK(plan.path.size() > 8);
 CHECK(plan.path.front().arrival_direction == sim::TravelDirection::Unspecified);
 CHECK(std::hypot(plan.path.back().position.x - 7.5, plan.path.back().position.y - 7.5) <=
 hybrid.goal_position_tolerance_m + 1e-9);
 for (const auto& point : plan.path)
  CHECK(map.isFree(point.position.x, point.position.y));
 for (std::size_t point = 1; point < plan.path.size(); ++point) {
  CHECK(std::hypot(plan.path[point].position.x - plan.path[point - 1].position.x,
                   plan.path[point].position.y - plan.path[point - 1].position.y) > 1e-9);
 }

 // A request already at its goal is a valid one-point route. It must not
 // append a duplicate zero-length terminal segment with an invented gear.
 const auto stationary_plan = sim::hybrid_astar_plan(map, {1.0, 1.0}, 0.0, {1.0, 1.0}, 0.0, hybrid);
 CHECK(stationary_plan.ok);
 CHECK(stationary_plan.path.size() == 1);
 CHECK(stationary_plan.path.front().arrival_direction == sim::TravelDirection::Unspecified);

 // A distinct goal inside the tolerance still has to search at least one
 // motion primitive, otherwise its terminal anchor would have no real gear.
 const auto nearby_plan = sim::hybrid_astar_plan(map, {1.0, 1.0}, 0.0, {1.1, 1.0}, 0.0, hybrid);
 CHECK(nearby_plan.ok);
 CHECK(nearby_plan.path.size() >= 2);
 CHECK(nearby_plan.path[1].arrival_direction != sim::TravelDirection::Unspecified);

 // A free authored goal does not justify an unchecked line from the searched
 // terminal state. This obstacle intersects that former synthetic edge while
 // leaving the searched straight primitive and exact goal free.
 sim::GridMap terminal_map(60, 60, 0.05);
 terminal_map.addBox(1.26, 1.06, 1.34, 1.14);
 auto terminal_config = hybrid;
 terminal_config.primitive_length_m = 0.4;
 terminal_config.goal_position_tolerance_m = 0.35;
 terminal_config.goal_heading_tolerance_rad = sim::kPi;
 const sim::Vec2 authored_goal{1.2, 1.2};
 const auto terminal_plan = sim::hybrid_astar_plan(
 terminal_map, {1.0, 1.0}, 0.0, authored_goal, 0.0, terminal_config);
 CHECK(terminal_plan.ok);
 CHECK(terminal_plan.path.size() == 2);
 CHECK(std::hypot(terminal_plan.path.back().position.x - authored_goal.x,
 terminal_plan.path.back().position.y - authored_goal.y) > 1e-3);
 CHECK(std::hypot(terminal_plan.path.back().position.x - authored_goal.x,
 terminal_plan.path.back().position.y - authored_goal.y) <=
 terminal_config.goal_position_tolerance_m + 1e-9);

 sim::AckermannLocalConfig local;
 local.wheel_base_m = 0.62;
 local.max_steer_rad = 0.55;
 local.max_steer_rate_radps = 2.0;
 local.max_speed_mps = 1.5;
 const auto trajectories =
 sim::ackermann_local_compute({1.0, 1.0}, 0.0, 0.0, 0.0, 0.0,
 {3.0, 2.0}, map, {}, {}, local);
 CHECK(trajectories.best.command.valid);
 CHECK(!trajectories.best.samples.empty());
 CHECK(trajectories.best.command_space == sim::LocalCommandSpace::SteeringAngle);
 CHECK(std::abs(trajectories.best.command.w) <= local.max_steer_rad + 1e-9);

 // Planner and hardware consume the same steering response contract. Replay
 // a full-lock rollout through the production servo and compare every sample,
 // including the hidden rate state exposed as actuator feedback.
 {
 sim::AckermannLocalConfig response_config = local;
 response_config.max_speed_mps = 0.0;
 response_config.predict_time_s = 1.0;
 response_config.dt_s = 0.1;
 response_config.steering_response_s = 0.35;
 const auto response_rollouts = sim::ackermann_local_compute(
 {1.0, 1.0}, 0.0, 0.0, 0.0, 0.0,
 {1.0, 2.0}, map, {}, {}, response_config);
 const auto chosen = std::find_if(
 response_rollouts.rollouts.begin(), response_rollouts.rollouts.end(),
 [&](const auto& rollout) {
 return std::abs(rollout.trajectory.command.w - response_config.max_steer_rad) < 1e-12;
 });
 CHECK(chosen != response_rollouts.rollouts.end());
 sim::AckermannParams actuator;
 actuator.wheel_base_m = response_config.wheel_base_m;
 actuator.max_steer_rad = response_config.max_steer_rad;
 actuator.max_steer_rate_radps = response_config.max_steer_rate_radps;
 actuator.servo_response_s = response_config.steering_response_s;
 const double servo_kp = 1.0 / std::max(
 0.05, actuator.servo_response_s * actuator.max_steer_rate_radps);
 sim::AckermannDrive hardware(actuator, {}, servo_kp);
 sim::DiffState hardware_state;
 for (std::size_t sample = 1; sample < chosen->trajectory.samples.size(); ++sample) {
 hardware_state = hardware.step(
 hardware_state, {0.0, chosen->trajectory.command.w},
 response_config.dt_s, {});
 CHECK(std::abs(hardware.telemetry().steer.angle_rad -
 chosen->trajectory.samples[sample].steering_angle_rad) < 1e-12);
 }
 CHECK(std::abs(hardware.telemetry().steer.rate_radps) <=
 response_config.max_steer_rate_radps + 1e-12);
 }

 // A narrow backing task requires reverse primitives. Hybrid A* must retain
 // their gear in its typed path, and the local sampler must honor that gear.
 sim::GridMap backing_map(40, 12, 0.25);
 const auto backing_plan = sim::hybrid_astar_plan(backing_map, {7.5, 1.5}, 0.0, {2.0, 1.5}, 0.0, hybrid);
 CHECK(backing_plan.ok);
 CHECK(std::any_of(backing_plan.path.begin(), backing_plan.path.end(), [](const auto& point) {
  return point.arrival_direction == sim::TravelDirection::Reverse;
 }));
 sim::AckermannLocalConfig reverse_local = local;
 reverse_local.preferred_direction = sim::TravelDirection::Reverse;
 const auto reverse_trajectories =
 sim::ackermann_local_compute({7.5, 1.5}, 0.0, 0.0, 0.0, 0.0,
 {5.5, 1.8}, backing_map, {}, {}, reverse_local);
 CHECK(reverse_trajectories.best.command.valid);
 CHECK(reverse_trajectories.best.command.v <= 0.0);

 // A short reverse primitive ending at a gear cusp must not be swallowed by
 // the ordinary 0.55 m waypoint tolerance. The controller reaches the cusp
 // in reverse, then and only then exposes the following forward section.
 {
 const std::vector<sim::RoutePoint> cusp_route{
 {{1.0, 1.0}, 0.0, sim::TravelDirection::Unspecified},
 {{0.625, 1.0}, 0.0, sim::TravelDirection::Reverse},
 {{0.95, 0.8}, -0.4, sim::TravelDirection::Forward},
 };
 sim::GuideState guide;
 sim::GridMap cusp_map(40, 40, 0.1);
 const auto before_cusp = sim::ackermann_guide_step(
 cusp_route, guide, 1.0, 1.0, 0.0, 0.62, 0.55, 1.5,
 0.75, 0.1, cusp_map);
 CHECK(before_cusp.tv < 0.0);
 CHECK(guide.idx == 1);
 const auto after_cusp = sim::ackermann_guide_step(
 cusp_route, guide, 0.63, 1.0, 0.0, 0.62, 0.55, 1.5,
 0.75, 0.1, cusp_map);
 CHECK(after_cusp.tv >= 0.0);
 CHECK(guide.idx == 2);
 }

 // A controller can enter a preferred inflation band through finite tracking
 // error while remaining outside physical collision occupancy. The local
 // planner must produce a bounded escape back to preferred free space rather
 // than reject every rollout at t=0. Physical occupancy remains inviolable.
 sim::GridMap margin_source(100, 40, 0.1);
 margin_source.addBox(5.0, 0.0, 6.0, 4.0);
 const auto preferred_margin = margin_source.inflated(0.75);
 const auto physical_collision = margin_source;
 sim::LocalPlanningRequest escape_request;
 escape_request.position = {4.45, 2.0};
 escape_request.heading_rad = sim::kPi;
 escape_request.target = {3.0, 2.0};
 escape_request.max_speed_mps = 1.0;
 escape_request.max_yaw_rate_radps = 1.5;
 escape_request.max_accel_mps2 = 2.0;
 escape_request.max_yaw_accel_radps2 = 4.0;
 escape_request.prediction_horizon_s = 2.0;
 escape_request.step_s = 0.1;
 escape_request.robot_radius_m = 0.25;
 CHECK(!preferred_margin.isFree(escape_request.position.x, escape_request.position.y));
 CHECK(physical_collision.isFree(escape_request.position.x, escape_request.position.y));
 sim::LocalPlanningSystem local_system;
 // A stop pose keeps its route-tangent heading even when position error is
 // zero. Point-only scoring is singular here and used to align with atan2(0,0).
 sim::GridMap pose_map(40, 40, 0.1);
 sim::LocalPlanningRequest pose_request;
 pose_request.position = {2.0, 2.0};
 pose_request.heading_rad = 0.0;
 pose_request.target = pose_request.position;
 pose_request.target_heading_rad = sim::kPi * 0.5;
 pose_request.max_speed_mps = 0.0;
 pose_request.max_yaw_rate_radps = 1.5;
 pose_request.max_accel_mps2 = 2.0;
 pose_request.max_yaw_accel_radps2 = 4.0;
 pose_request.prediction_horizon_s = 2.0;
 pose_request.step_s = 0.1;
 const auto pose_plan = local_system.plan(pose_request, pose_map, {});
 CHECK(pose_plan.selection.validation.accepted());
 CHECK(pose_plan.selection.trajectory.command.v == 0.0);
 CHECK(pose_plan.selection.trajectory.command.w > 0.0);

 const auto escape = local_system.plan(
 escape_request, preferred_margin, {}, [](sim::Vec2, sim::Vec2 to) {
 return to.x <= 4.75;
 }, &physical_collision);
 CHECK(escape.selection.validation.minimum_risk_escape());
 CHECK(escape.selection.trajectory.command.valid);
 CHECK(preferred_margin.isFree(
 escape.selection.trajectory.samples.back().position.x,
 escape.selection.trajectory.samples.back().position.y));
 CHECK(std::all_of(escape.selection.trajectory.samples.begin(),
 escape.selection.trajectory.samples.end(), [&](const auto& sample) {
 return physical_collision.isFree(sample.position.x, sample.position.y);
 }));

 // A rollout endpoint is only a prediction. If the measured pose remains in
 // the same soft margin, the planner must eventually enter incremental
 // physical-map recovery even though each nominal rollout claims that it can
 // leave the margin. This regression keeps the measured pose deliberately
 // fixed to represent contact or tracking error.
 sim::LocalPlanningSystem stalled_recovery;
 int physical_feasibility_calls = 0;
 sim::LocalPlanningRequest stalled_request = escape_request;
 for (int cycle = 0; cycle < 105; ++cycle) {
 const auto stalled = stalled_recovery.plan(
 stalled_request, preferred_margin, {}, {}, &physical_collision,
 [&](sim::Vec2, sim::Vec2) {
 ++physical_feasibility_calls;
 return true;
 });
 CHECK(stalled.selection.validation.accepted());
 }
 CHECK(physical_feasibility_calls > 0);

 // Global replanning is triggered by persistent static evidence on the
 // remaining route, while a live tracked actor at the same blockage keeps
 // ownership in prediction/behavior. This prevents both permanent local
 // recovery loops and global detours around a passing vehicle.
 const std::vector<sim::RoutePoint> monitored_route{
 {{1.0, 1.0}, 0.0, sim::TravelDirection::Unspecified},
 {{2.0, 1.0}, 0.0, sim::TravelDirection::Forward},
 {{4.0, 1.0}, 0.0, sim::TravelDirection::Forward},
 };
 sim::EstimatedState monitored_estimate;
 monitored_estimate.position = {1.0, 1.0};
 sim::GridMap blocked_route_map(40, 40, 0.1);
 blocked_route_map.addBox(1.8, 0.8, 2.2, 1.2);
 sim::RouteTrackingSystem route_monitor(1.0, 0.0, 0.0, 1.0, 1.0);
 for (int cycle = 0; cycle < 9; ++cycle) {
 CHECK(!route_monitor.should_replan(
 monitored_route, monitored_estimate, blocked_route_map, {}, 0.1));
 }
 CHECK(route_monitor.should_replan(
 monitored_route, monitored_estimate, blocked_route_map, {}, 0.1));
 route_monitor.reset();
 sim::DynamicTrack passing_actor;
 passing_actor.state.p = {2.0, 1.0};
 passing_actor.state.radius = 0.3;
 for (int cycle = 0; cycle < 20; ++cycle) {
 CHECK(!route_monitor.should_replan(
 monitored_route, monitored_estimate, blocked_route_map,
 {passing_actor}, 0.1));
 }

 const auto mecanum = sim::mecanum_inverse(1.2, -0.4, 0.7, 0.6, 0.5);
 double recovered_forward = 0.0, recovered_left = 0.0, recovered_yaw = 0.0;
 sim::mecanum_forward(mecanum, 0.6, 0.5, recovered_forward, recovered_left, recovered_yaw);
 CHECK(std::abs(recovered_forward - 1.2) < 1e-9);
 CHECK(std::abs(recovered_left + 0.4) < 1e-9);
 CHECK(std::abs(recovered_yaw - 0.7) < 1e-9);

 const auto swerve = sim::swerve_inverse(0.0, 1.0, 0.0, 0.6, 0.5);
 for (const auto& module : swerve) {
  CHECK(std::abs(std::abs(module.speed_mps) - 1.0) < 1e-9);
  CHECK(std::abs(std::abs(module.steer_rad) - sim::kPi * 0.5) < 1e-9);
 }
 sim::HolonomicDrive drive({sim::HolonomicKind::Mecanum, 0.6, 0.5, 2.4, 4.0, 6.0, 26.0, 2.8});
 sim::DiffState state;
 sim::DiffCmd strafe;
 strafe.target_lateral_v = 1.0;
 for (int step = 0; step < 100; ++step)
  state = drive.step(state, strafe, 0.02, {});
 CHECK(state.y > 1.0);
 CHECK(std::abs(state.x) < 1e-9);
 // Lethal inflation and graded clearance have different jobs. The former
 // rejects overlap; the latter keeps a nominal global route away from the
 // edge of an occupied cell so ordinary tracking error does not immediately
 // force local recovery. This is the default ROBOT planner contract and must
 // not depend on a level-specific game setting.
 {
 sim::GridMap route_map(14, 12, 1.0);
 // The central block gives equal-length upper and lower routes. The lower
 // route remains physically free but runs beside a second obstacle for
 // longer. Plain A* resolves that exact tie by grid index; the default
 // graded critic should resolve it toward the route with more usable room.
 route_map.addBox(5.0, 4.0, 9.0, 7.0);
 route_map.addBox(4.0, 2.0, 10.0, 3.0);
 const std::vector<autonomy::PlanningPose> request{
 {{1.5, 5.5}, 0.0}, {{12.5, 5.5}, 0.0},
 };
 autonomy::GlobalPlanningSystem shortest(
 autonomy::default_global_planner(), 0.0);
 autonomy::GlobalPlanningSystem clearance_aware(
 autonomy::default_global_planner());
 const auto shortest_plan = shortest.plan(route_map, request, {});
 const auto robust_plan = clearance_aware.plan(route_map, request, {});
 CHECK(shortest_plan.ok);
 CHECK(robust_plan.ok);
 const auto y_extent = [](const auto& path) {
 std::pair<double, double> value{
 std::numeric_limits<double>::infinity(),
 -std::numeric_limits<double>::infinity()};
 for (const auto& point : path)
 value = {std::min(value.first, point.position.y),
 std::max(value.second, point.position.y)};
 return value;
 };
 CHECK(y_extent(shortest_plan.path).first <= 3.5 + 1e-9);
 CHECK(y_extent(robust_plan.path).second >= 7.5 - 1e-9);

 // Clearance is a real soft cost, not only a deterministic tie breaker. The
 // direct path below this wall is shorter and physically free, but it stays
 // one half-cell from inflation. The default planner accepts a modest detour
 // through the wider corridor so finite-bandwidth tracking does not start in
 // local recovery.
 sim::GridMap shortcut_map(12, 9, 1.0);
 shortcut_map.addBox(4.0, 4.0, 8.0, 5.0);
 const std::vector<autonomy::PlanningPose> shortcut_request{
 {{1.5, 3.5}, 0.0}, {{10.5, 3.5}, 0.0},
 };
 const auto tight_shortcut = shortest.plan(shortcut_map, shortcut_request, {});
 const auto margin_route = clearance_aware.plan(shortcut_map, shortcut_request, {});
 CHECK(tight_shortcut.ok && margin_route.ok);
 const auto path_length = [](const auto& path) {
 double length = 0.0;
 for (std::size_t index = 1; index < path.size(); ++index) {
 length += std::hypot(
 path[index].position.x - path[index - 1].position.x,
 path[index].position.y - path[index - 1].position.y);
 }
 return length;
 };
 const auto minimum_clearance = [&](const auto& path) {
 double clearance = std::numeric_limits<double>::infinity();
 for (const auto& point : path)
 clearance = std::min(clearance,
 shortcut_map.clearance(point.position.x, point.position.y));
 return clearance;
 };
 CHECK(path_length(margin_route.path) > path_length(tight_shortcut.path) + 0.1);
 CHECK(minimum_clearance(margin_route.path) >
 minimum_clearance(tight_shortcut.path) + 0.5);
 }

 return 0;
}
