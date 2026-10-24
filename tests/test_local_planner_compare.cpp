#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

#include "robot/autonomy/planning/dwa_planner.hpp"
#include "robot/autonomy/planning/teb_planner.hpp"

namespace {

lost_signal::robot::autonomy::LocalMap open_map() {
 using namespace lost_signal::robot::autonomy;
 LocalMap map;
 map.valid = true;
 map.provenance = LocalMapProvenance::kSensorRays;
 map.resolution_m = 0.1;
 map.width = 120;
 map.height = 120;
 map.origin_x_m = -6.0;
 map.origin_y_m = -6.0;
 map.occupancy.assign(map.width * map.height, 0);
 return map;
}

lost_signal::robot::autonomy::Route straight_route() {
 using namespace lost_signal::robot::autonomy;
 Route route;
 route.valid = true;
 for (int i = 0; i <= 8; ++i) {
 route.points.push_back({static_cast<double>(i), 0.0, 0.0, 1});
 }
 return route;
}

} // namespace

int main() {
 using namespace lost_signal::robot;
 using namespace lost_signal::robot::autonomy;
 platform::ChassisLimits limits{.max_forward_mps = 2.0,
 .max_yaw_rate_radps = 1.5,
 .footprint_radius_m = 0.4};
 EstimatedState est;
 est.valid = true;
 est.stamp_ns = 1'000'000'000;
 PerceptionResult perception;
 perception.scan_ok = true;
 perception.stamp_ns = est.stamp_ns;
 const auto map = open_map();
 const auto route = straight_route();
    const auto empty_prediction = predict_perception(perception, 1.0);

    // Name-based construction lives in planning and rejects unknown policy.
    assert(dynamic_cast<DwaPlanner*>(make_local_planner("dwa").get()) !=
        nullptr);
    assert(dynamic_cast<TebPlanner*>(make_local_planner("teb").get()) !=
        nullptr);
    assert(make_local_planner("magic") == nullptr);

 // Same open route and limits: both planners must return a reachable forward
 // command, and neither may jump by more than a single 0.1 s acceleration
 // interval from standstill.
 DwaPlanner dwa;
 TebPlanner teb;
 const LocalPlan dwa_open = dwa.plan(
 est, perception, empty_prediction, map, route, limits);
 const LocalPlan teb_open = teb.plan(
 est, perception, empty_prediction, map, route, limits);
 assert(dwa_open.has_plan && teb_open.has_plan);
 assert(dwa_open.best.forward_mps > 0.0);
 assert(teb_open.best.forward_mps > 0.0);
 assert(teb_open.best.forward_mps <= 0.2 + 1e-9);
 assert(teb.last_band().size() >= 3);

 // At startup the rolling map may have only a short sensor-cleared corridor.
 // TEB must size its band from the next reachable command rather than demand
 // that the full top-speed horizon is already observed.
 LocalMap observed_prefix = map;
 std::fill(observed_prefix.occupancy.begin(),
 observed_prefix.occupancy.end(), -1);
 for (int gy = 50; gy <= 70; ++gy) {
 for (int gx = 50; gx <= 67; ++gx) {
 observed_prefix.occupancy[
 static_cast<std::size_t>(gy) * observed_prefix.width + gx] = 0;
 }
 }
 TebPlanner teb_startup;
 const LocalPlan startup = teb_startup.plan(
 est, perception, empty_prediction, observed_prefix, route, limits);
 assert(startup.has_plan);
 assert(startup.best.forward_mps > 0.0);
 assert(startup.best.forward_mps <= 0.2 + 1e-9);

 // A fast actor crosses between coarse frame endpoints. Both planners consume
 // the same typed batch; a returned command must not drive rapidly into the
 // crossing, while no-plan is a valid conservative TEB result.
 PerceptionResult crossing = perception;
 DynamicTrack actor;
 actor.id = 7;
 actor.x_m = 0.45;
 actor.y_m = -1.0;
 actor.vy_mps = 5.0;
 actor.radius_m = 0.20;
 actor.prediction_ready = true;
 actor.last_observed_ns = crossing.stamp_ns;
 crossing.tracks.push_back(actor);
 const auto crossing_prediction = predict_perception(
 crossing, 1.0, {.horizon_s = 2.5, .frame_dt_s = 0.2});
 const PredictedClearance swept = minimum_predicted_clearance(
 crossing_prediction, {0.0, 0.0}, {0.4, 0.0}, 0.0, 0.2, 0.4);
 assert(swept.clearance_m < 0.0);
 DwaPlanner dwa_crossing;
 TebPlanner teb_crossing;
 const LocalPlan dwa_dynamic = dwa_crossing.plan(
 est, crossing, crossing_prediction, map, route, limits);
 const LocalPlan teb_dynamic = teb_crossing.plan(
 est, crossing, crossing_prediction, map, route, limits);
 assert(!dwa_dynamic.has_plan || dwa_dynamic.best.forward_mps <= 0.0);
 assert(!teb_dynamic.has_plan || teb_dynamic.best.forward_mps <= 0.2);

 // Ackermann feasibility is identical at the planner boundary: TEB cannot
 // issue in-place yaw and must obey |w| <= |v|*kappa.
 platform::ChassisLimits ackermann = limits;
 ackermann.max_curvature_inv_m = 0.35;
 TebPlanner teb_ackermann;
 const LocalPlan ack = teb_ackermann.plan(
 est, perception, empty_prediction, map, route, ackermann);
 assert(ack.has_plan);
 assert(std::abs(ack.best.yaw_rate_radps) <=
 std::abs(ack.best.forward_mps) * ackermann.max_curvature_inv_m + 1e-9);

 // Position and heading are separate terminal constraints. A differential
 // base already at the final point rotates toward the requested final yaw;
 // an Ackermann base still cannot manufacture in-place steering.
 Route yaw_goal;
 yaw_goal.valid = true;
 yaw_goal.points.push_back({0.0, 0.0, 0.5 * core::kPi, 1});
 TebPlanner teb_terminal;
 const LocalPlan terminal_diff = teb_terminal.plan(
 est, perception, empty_prediction, map, yaw_goal, limits);
 assert(terminal_diff.has_plan);
 assert(std::abs(terminal_diff.best.forward_mps) < 1e-9);
 assert(terminal_diff.best.yaw_rate_radps > 0.0);
 TebPlanner teb_terminal_ackermann;
 const LocalPlan terminal_ack = teb_terminal_ackermann.plan(
 est, perception, empty_prediction, map, yaw_goal, ackermann);
 assert(terminal_ack.has_plan);
 assert(std::abs(terminal_ack.best.forward_mps) < 1e-9);
 assert(std::abs(terminal_ack.best.yaw_rate_radps) < 1e-9);

 // Deterministic micro-comparison for regression visibility. This is reported
 // rather than used as a brittle pass/fail threshold across build machines.
 constexpr int repeats = 100;
 auto run = [&](auto& planner) {
 const auto begin = std::chrono::steady_clock::now();
 for (int i = 0; i < repeats; ++i) {
 const LocalPlan plan = planner.plan(
 est, perception, empty_prediction, map, route, limits);
 assert(plan.has_plan);
 }
 return std::chrono::duration_cast<std::chrono::microseconds>(
 std::chrono::steady_clock::now() - begin).count();
 };
 DwaPlanner timed_dwa;
 TebPlanner timed_teb;
 const auto dwa_us = run(timed_dwa);
 const auto teb_us = run(timed_teb);
 std::printf("local planner compare: dwa=%lldus teb=%lldus runs=%d\n",
 static_cast<long long>(dwa_us), static_cast<long long>(teb_us), repeats);
 return 0;
}
