#include <cassert>
#include <cmath>
#include <cstdio>
#include <memory>

#include "robot/autonomy/planning/ackermann_planner.hpp"
#include "robot/autonomy/planning/global_planner.hpp"
#include "robot/autonomy/planning/route_smoothing.hpp"

using lost_signal::robot::autonomy::AckermannPlanner;
using lost_signal::robot::autonomy::EstimatedState;
using lost_signal::robot::autonomy::Route;
using lost_signal::robot::autonomy::RoutePoint;
using lost_signal::robot::iface::MapSnapshot;
using lost_signal::robot::platform::ChassisLimits;

namespace {

MapSnapshot open_map(int width, int height, double resolution,
 double origin_x, double origin_y) {
 MapSnapshot map;
 map.width = static_cast<std::uint32_t>(width);
 map.height = static_cast<std::uint32_t>(height);
 map.resolution_m = resolution;
 map.origin_x_m = origin_x;
 map.origin_y_m = origin_y;
 map.occupancy.assign(static_cast<std::size_t>(width * height), 0);
 return map;
}

EstimatedState pose(double x, double y, double yaw) {
 EstimatedState out;
 out.valid = true;
 out.x_m = x;
 out.y_m = y;
 out.yaw_rad = yaw;
 return out;
}

} // namespace

int main() {
 using namespace lost_signal::robot::autonomy;

 ChassisLimits limits;
 limits.max_curvature_inv_m = 1.1;
 limits.max_steer_rad = 0.55;
 auto selected = make_global_planner(limits, 0.56);
 assert(dynamic_cast<AckermannPlanner*>(selected.get()) != nullptr);

 AckermannPlanner::Config config;
 config.wheel_base_m = 0.56;
 config.max_steer_rad = 0.55;
 config.primitive_length_m = 0.35;
 config.heading_bins = 48;
 config.max_expansions = 60'000;
 AckermannPlanner planner(config);
 MapSnapshot map = open_map(80, 80, 0.25, -10.0, -10.0);

 // A car-like route reaches a displaced goal without an in-place turn and
 // keeps every sampled point in the ROBOT occupancy map.
 Route route = planner.plan(pose(0.0, 0.0, 0.0), {5.0, 2.0, 0.0}, map);
 assert(route.valid && route.points.size() >= 2);
 bool explicit_gear = false;
 for (const auto& point : route.points) {
 assert(!map.blocked(point.x_m, point.y_m));
 assert(std::isfinite(point.x_m) && std::isfinite(point.y_m));
 assert(std::isfinite(point.yaw_rad));
 explicit_gear |= point.gear == 1 || point.gear == -1;
 }
 assert(explicit_gear);
 assert(std::hypot(route.points.back().x_m - 5.0,
 route.points.back().y_m - 2.0) <= config.goal_position_tolerance_m + 1e-9);
 for (std::size_t i = 1; i < route.points.size(); ++i) {
 if (route.points[i].gear != route.points[i - 1].gear) continue;
 const double ds = std::hypot(route.points[i].x_m - route.points[i - 1].x_m,
 route.points[i].y_m - route.points[i - 1].y_m);
 if (ds > 1e-6) {
 const double dyaw = std::abs(lost_signal::robot::core::wrap_angle(
 route.points[i].yaw_rad - route.points[i - 1].yaw_rad));
 assert(dyaw <= 0.55 + 1e-6);
 }
 }
 Route smoothed;
 smoothed.valid = true;
 smoothed.points = {{0.0, 0.0, 0.0, 1}, {1.0, 0.2, 0.1, 1},
 {2.0, 0.5, 0.2, 1}};
 assert(smooth_ackermann_route(smoothed, {0.56, 0.55, 0.55, 2.0}));

 // Invalid kinematic configuration fails closed on a raster instead of
 // returning a generic A* route that the chassis cannot realize.
 AckermannPlanner::Config invalid = config;
 invalid.wheel_base_m = 0.0;
 AckermannPlanner invalid_planner(invalid);
 assert(!invalid_planner.plan(pose(0.0, 0.0, 0.0), {5.0, 0.0, 0.0}, map).valid);

 // A vector-only map has no collision evidence; the planner emits only the
 // straight seed, which remains subject to local-map and validator gates.
 MapSnapshot vector_map;
 Route seed = planner.plan(pose(0.0, 0.0, 0.0), {2.0, 0.0, 0.0}, vector_map);
 assert(seed.valid && seed.points.size() >= 2);
 AckermannPlanner invalid_vector(invalid);
 assert(!invalid_vector.plan(pose(0.0, 0.0, 0.0), {2.0, 0.0, 0.0},
 vector_map).valid);

 std::printf("ackermann planner ok\n");
 return 0;
}
