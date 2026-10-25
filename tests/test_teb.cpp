#include <cassert>
#include <cmath>
#include <cstdio>

#include "robot/autonomy/planning/teb_planner.hpp"

namespace {

using namespace lost_signal::robot::autonomy;

LocalMap open_map() {
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

Route straight_route() {
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
 EstimatedState estimate;
 estimate.valid = true;
 estimate.stamp_ns = 1'000'000'000;
 PerceptionResult perception;
 perception.scan_ok = true;
 perception.stamp_ns = estimate.stamp_ns;
 const LocalMap map = open_map();
 const Route route = straight_route();
 const PredictionBatch empty = predict_perception(perception, 1.0);

 TebPlanner planner;
 const LocalPlan first = planner.plan(
 estimate, perception, empty, map, route, limits);
 assert(first.has_plan);
 assert(first.candidates.size() >= 2);
 assert(first.best.forward_mps > 0.0);
 bool left_seed = false;
 bool right_seed = false;
 for (const auto& candidate : first.candidates) {
 left_seed |= candidate.yaw_rate_radps > 1e-3;
 right_seed |= candidate.yaw_rate_radps < -1e-3;
 }
 assert(left_seed && right_seed);
 const auto first_band = planner.last_band();
 assert(first_band.size() >= 3);
 for (std::size_t i = 1; i < first_band.size(); ++i) {
 assert(first_band[i].time_s > first_band[i - 1].time_s);
 }

 const LocalPlan warm = planner.plan(
 estimate, perception, empty, map, route, limits);
 assert(warm.has_plan);
 assert(planner.last_band().size() == first_band.size());

 DynamicTrack crossing;
 crossing.id = 9;
 crossing.x_m = 0.45;
 crossing.y_m = -1.0;
 crossing.vy_mps = 5.0;
 crossing.radius_m = 0.2;
 crossing.prediction_ready = true;
 crossing.last_observed_ns = estimate.stamp_ns;
 PerceptionResult traffic = perception;
 traffic.tracks.push_back(crossing);
 const PredictionBatch forecast = predict_perception(
 traffic, 1.0, {.horizon_s = 2.5, .frame_dt_s = 0.1});
 TebPlanner dynamic;
 const LocalPlan dynamic_plan = dynamic.plan(
 estimate, traffic, forecast, map, route, limits);
 assert(dynamic_plan.has_plan);
 assert(!dynamic_plan.candidates.empty());

 platform::ChassisLimits ackermann = limits;
 ackermann.max_curvature_inv_m = 0.3;
 TebPlanner car_like;
 const LocalPlan car_plan = car_like.plan(
 estimate, perception, empty, map, route, ackermann);
 assert(car_plan.has_plan);
 for (const auto& candidate : car_plan.candidates) {
 assert(std::abs(candidate.yaw_rate_radps) <=
 std::abs(candidate.forward_mps) * ackermann.max_curvature_inv_m + 1e-9);
 }

 TebPlanner terrain_blocked;
 terrain_blocked.set_terrain_gate(
 [](double, double, double, double, double) { return false; });
 assert(!terrain_blocked.plan(
 estimate, perception, empty, map, route, limits).has_plan);

 std::printf("teb ok\n");
 return 0;
}
