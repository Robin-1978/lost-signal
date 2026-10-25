#include <algorithm>
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

void fill_box(MapSnapshot& map, double x0, double y0,
 double x1, double y1) {
 for (std::uint32_t y = 0; y < map.height; ++y) {
 for (std::uint32_t x = 0; x < map.width; ++x) {
 const double wx = map.origin_x_m + (x + 0.5) * map.resolution_m;
 const double wy = map.origin_y_m + (y + 0.5) * map.resolution_m;
 if (wx >= x0 && wx < x1 && wy >= y0 && wy < y1) {
 map.occupancy[static_cast<std::size_t>(y) * map.width + x] = 100;
 }
 }
 }
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
 route.points.back().y_m - 2.0) <= 1e-9);
 assert(std::abs(route.points.back().yaw_rad) <= 1e-9);
 int steering_reversals = 0;
 int previous_steering_sign = 0;
 for (std::size_t i = 1; i < route.points.size(); ++i) {
 if (route.points[i].gear != route.points[i - 1].gear) continue;
 const double ds = std::hypot(route.points[i].x_m - route.points[i - 1].x_m,
 route.points[i].y_m - route.points[i - 1].y_m);
 if (ds > 1e-6) {
 const double dyaw = std::abs(lost_signal::robot::core::wrap_angle(
 route.points[i].yaw_rad - route.points[i - 1].yaw_rad));
 assert(dyaw <= 0.55 + 1e-6);
 const double signed_dyaw = lost_signal::robot::core::wrap_angle(
 route.points[i].yaw_rad - route.points[i - 1].yaw_rad);
 const int steering_sign = signed_dyaw > 1e-6
 ? 1 : signed_dyaw < -1e-6 ? -1 : 0;
 if (steering_sign != 0 && previous_steering_sign != 0 &&
 steering_sign != previous_steering_sign) {
 ++steering_reversals;
 }
 if (steering_sign != 0) previous_steering_sign = steering_sign;
 }
 }
 // In an obstacle-free field, a short pose-to-pose route has no reason to
 // alternate steering repeatedly. This catches the lattice zigzag that a
 // distance-only Hybrid-A* cost used to emit.
 assert(steering_reversals <= 4);
 Route smoothed;
 smoothed.valid = true;
 smoothed.points = {{0.0, 0.0, 0.0, 1}, {1.0, 0.2, 0.1, 1},
 {2.0, 0.5, 0.2, 1}};
 assert(smooth_ackermann_route(smoothed, {0.56, 0.55, 0.55, 2.0}));

 // The same request can run cooperatively: map preparation, guide search,
 // and sparse Hybrid-A* refinement remain pending until their bounded work
 // slices complete, then produce a valid car-like route.
 {
 AckermannPlanner incremental(config);
 assert(incremental.supports_incremental_plan());
 incremental.begin_plan(pose(0.0, 0.0, 0.0), {5.0, 2.0, 0.0}, map);
 GlobalPlanProgress progress = incremental.continue_plan(1);
 assert(progress.status == GlobalPlanStatus::kPending);
 for (int cycle = 0;
 cycle < 20'000 && progress.status == GlobalPlanStatus::kPending;
 ++cycle) {
 progress = incremental.continue_plan(32);
 }
 assert(progress.status == GlobalPlanStatus::kSucceeded);
 assert(progress.route.valid && progress.route.points.size() >= 2);
 }

 // A car already at the requested position may still need a bounded
 // multi-point maneuver to attain the requested heading. Grid A* has a
 // zero-cell route here; Hybrid-A* must retain pose planning instead of
 // declaring the route invalid or asking the chassis to rotate in place.
 {
 const Route pose_change = planner.plan(
 pose(0.0, 0.0, 0.0), {0.0, 0.0, 0.6}, map);
 assert(pose_change.valid && pose_change.points.size() >= 3);
 assert(std::hypot(pose_change.points.back().x_m,
 pose_change.points.back().y_m) <= 1e-9);
 assert(std::abs(lost_signal::robot::core::wrap_angle(
 pose_change.points.back().yaw_rad - 0.6)) <= 1e-9);
 }

 // A corridor narrower than the turning diameter requires a real reverse
 // gear; a forward-only planner fails closed instead of inserting a pivot.
 {
 MapSnapshot corridor = open_map(80, 32, 0.25, -10.0, -4.0);
 fill_box(corridor, -10.0, -4.0, 10.0, -0.75);
 fill_box(corridor, -10.0, 0.75, 10.0, 4.0);
 const Route reverse = planner.plan(
 pose(0.0, 0.0, 0.0), {-3.0, 0.0, 0.0}, corridor);
 assert(reverse.valid);
 assert(std::any_of(reverse.points.begin(), reverse.points.end(),
 [](const RoutePoint& point) { return point.gear < 0; }));
 AckermannPlanner::Config forward_config = config;
 forward_config.allow_reverse = false;
 AckermannPlanner forward_only(forward_config);
 assert(!forward_only.plan(
 pose(0.0, 0.0, 0.0), {-3.0, 0.0, 0.0}, corridor).valid);
 }

 // Hybrid A* searches the complete footprint-plus-reserve costmap. The
 // center point fits through this opening, but the configured chassis does
 // not, so no motion primitive or final connector may pass it.
 {
 MapSnapshot narrow = open_map(80, 80, 0.25, -10.0, -10.0);
 fill_box(narrow, 2.0, -10.0, 3.0, -0.25);
 fill_box(narrow, 2.0, 0.25, 3.0, 10.0);
 const Route blocked = planner.plan(
 pose(0.0, 0.0, 0.0), {5.0, 0.0, 0.0}, narrow);
 assert(!blocked.valid);
 }

 // A wider opening remains usable and every returned segment stays outside
 // the same inflated map, including the exact-goal connector after heading
 // smoothing.
 {
 MapSnapshot wide = open_map(80, 80, 0.25, -10.0, -10.0);
 fill_box(wide, 2.0, -10.0, 3.0, -1.0);
 fill_box(wide, 2.0, 1.0, 3.0, 10.0);
 const Route through = planner.plan(
 pose(0.0, 0.0, 0.0), {5.0, 0.0, 0.0}, wide);
 assert(through.valid);
 }

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
