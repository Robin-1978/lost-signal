#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "robot/autonomy/mapping/mapper.hpp"
#include "robot/autonomy/localization/global_search.hpp"
#include "robot/autonomy/localization/scan_matcher.hpp"
#include "robot/autonomy/planning/ackermann_planner.hpp"
#include "robot/autonomy/planning/astar_planner.hpp"
#include "sim/eval/world_io.hpp"
#include "sim/scenario/actor_sidecar.hpp"
#include "sim/scenario/scenario_session.hpp"

namespace {

lost_signal::robot::autonomy::EstimatedState at(double x_m, double y_m) {
 lost_signal::robot::autonomy::EstimatedState state;
 state.valid = true;
 state.x_m = x_m;
 state.y_m = y_m;
 return state;
}

const lost_signal::robot::iface::Place& place(
 const lost_signal::robot::iface::MapSnapshot& map,
 const std::string& place_id) {
 const auto found = std::find_if(
 map.places.begin(), map.places.end(), [&](const auto& candidate) {
 return candidate.place_id == place_id;
 });
 assert(found != map.places.end());
 return *found;
}

void assert_route_clear(
 const lost_signal::robot::autonomy::Route& route,
 const lost_signal::robot::autonomy::LocalMap& lethal) {
 assert(route.valid && route.points.size() >= 2);
 for (std::size_t point = 1; point < route.points.size(); ++point) {
 const auto& from = route.points[point - 1];
 const auto& to = route.points[point];
 const double length = std::hypot(to.x_m - from.x_m, to.y_m - from.y_m);
 const int samples = std::max(
 1, static_cast<int>(std::ceil(length / (0.25 * lethal.resolution_m))));
 for (int sample = 0; sample <= samples; ++sample) {
 const double blend = static_cast<double>(sample) / samples;
 const double x_m = from.x_m + (to.x_m - from.x_m) * blend;
 const double y_m = from.y_m + (to.y_m - from.y_m) * blend;
 assert(!lethal.is_blocked(x_m, y_m));
 }
 }
}

std::size_t soft_band_reentries(
 const lost_signal::robot::autonomy::Route& route,
 const lost_signal::robot::autonomy::LocalCostmap& costmap) {
 std::size_t reentries = 0;
 bool in_soft_band = costmap.cost_at(
 route.points.front().x_m, route.points.front().y_m) > 0.0f;
 for (std::size_t point = 1; point < route.points.size(); ++point) {
 const auto& from = route.points[point - 1];
 const auto& to = route.points[point];
 const double length = std::hypot(to.x_m - from.x_m, to.y_m - from.y_m);
 const int samples = std::max(
 1, static_cast<int>(std::ceil(length / (0.25 * costmap.lethal.resolution_m))));
 for (int sample = 0; sample <= samples; ++sample) {
 const double blend = static_cast<double>(sample) / samples;
 const bool sample_in_soft_band = costmap.cost_at(
 from.x_m + (to.x_m - from.x_m) * blend,
 from.y_m + (to.y_m - from.y_m) * blend) > 0.0f;
 if (sample_in_soft_band && !in_soft_band) ++reentries;
 in_soft_band = sample_in_soft_band;
 }
 }
 return reentries;
}

std::vector<float> scan_from_survey(
 const lost_signal::robot::iface::MapSnapshot& map,
 double x_m, double y_m, double yaw_rad) {
 constexpr std::size_t kRays = 180;
 constexpr double kMaxRangeM = 10.0;
 std::vector<float> ranges(kRays, static_cast<float>(kMaxRangeM));
 for (std::size_t ray = 0; ray < kRays; ++ray) {
 const double angle = yaw_rad - M_PI +
 2.0 * M_PI * static_cast<double>(ray) / kRays;
 for (double distance = 0.05; distance < kMaxRangeM; distance += 0.05) {
 const int gx = static_cast<int>(std::floor(
 (x_m + distance * std::cos(angle) - map.origin_x_m) /
 map.resolution_m));
 const int gy = static_cast<int>(std::floor(
 (y_m + distance * std::sin(angle) - map.origin_y_m) /
 map.resolution_m));
 if (gx < 0 || gy < 0 || gx >= static_cast<int>(map.width) ||
 gy >= static_cast<int>(map.height)) {
 break;
 }
 if (map.occupancy[static_cast<std::size_t>(gy) * map.width + gx] >= 50) {
 ranges[ray] = static_cast<float>(distance);
 break;
 }
 }
 }
 return ranges;
}

} // namespace

int main() {
 using namespace lost_signal;
 const std::string map_path =
 std::string(TEST_CONFIG_DIR) + "/worlds/industrial_200k.json";

 sim::eval::WorldFileContent content;
 std::string error;
 assert(sim::eval::load_world_file(map_path, content, &error));
 assert(content.places.size() == 5);

 double min_x = content.boxes.front().x0;
 double min_y = content.boxes.front().y0;
 double max_x = content.boxes.front().x1;
 double max_y = content.boxes.front().y1;
 for (const auto& box : content.boxes) {
 min_x = std::min(min_x, box.x0);
 min_y = std::min(min_y, box.y0);
 max_x = std::max(max_x, box.x1);
 max_y = std::max(max_y, box.y1);
 }
 assert(max_x - min_x == 500.0);
 assert(max_y - min_y == 400.0);
 assert((max_x - min_x) * (max_y - min_y) == 200000.0);

 sim::scenario::ScenarioSession session;
 sim::eval::apply_world_content(session.setup_world(), content);
 session.fit_map_window_to_world();
 assert(sim::scenario::load_actors_sidecar(
 map_path, session, {}, &error) == 4);
 assert(session.setup_world().actors().size() == 4);
 assert(sim::scenario::validate_actor_routes(
 session.setup_world().actors(), session.setup_world().static_boxes()));

 const robot::iface::MapSnapshot map = session.active_map();
 assert(map.has_raster());
 assert(map.width == 2032);
 assert(map.height == 1632);
 assert(static_cast<std::size_t>(map.width) * map.height > 3200000);
 assert(map.places.size() == 5);

 // The authored startup structure must provide a globally observable scan.
 // The first MuJoCo step builds geometry; the second publishes the first
 // settled lidar frame. Localization consumes only that ROBOT observation
 // and the surveyed raster, never the recorded truth alongside it.
 session.step_once();
 session.step_once();
 assert(!session.episode().steps().empty());
 const robot::iface::ObservationFrame& startup_scan =
 session.episode().steps().back().obs;
 assert(startup_scan.scan_valid);
 robot::autonomy::GlobalSearcher global_search;
 global_search.set_lidar_mount(
 {{0.25, 0.0, 0.35}, {0.0, 0.0, 0.0, 1.0}});
 const auto cold_search_started = std::chrono::steady_clock::now();
 const robot::autonomy::GlobalSearchResult startup = global_search.search(
 startup_scan.ranges_m, startup_scan.scan_angle_min_rad,
 startup_scan.scan_angle_increment_rad, map, startup_scan.stamp_ns);
 const auto cold_search_finished = std::chrono::steady_clock::now();
 assert(startup.accepted);
 assert(std::hypot(startup.x_m, startup.y_m) < 0.35);
 assert(std::abs(startup.yaw_rad) < 0.10);
 assert(startup.map_preparation_cells == 2 * map.occupancy.size());
 const auto warm_search_started = std::chrono::steady_clock::now();
 const robot::autonomy::GlobalSearchResult warm_startup = global_search.search(
 startup_scan.ranges_m, startup_scan.scan_angle_min_rad,
 startup_scan.scan_angle_increment_rad, map, startup_scan.stamp_ns);
 const auto warm_search_finished = std::chrono::steady_clock::now();
 assert(warm_startup.accepted);
 assert(warm_startup.map_preparation_cells == 0);
 const double cold_search_ms = std::chrono::duration<double, std::milli>(
 cold_search_finished - cold_search_started).count();
 const double warm_search_ms = std::chrono::duration<double, std::milli>(
 warm_search_finished - warm_search_started).count();

 // Named work areas represent physical docks and gates, so each has
 // asymmetric surveyed structure within lidar range. A bounded correction
 // from a realistic odometry error must be observable at every endpoint.
 robot::autonomy::ScanMatcher local_matcher;
 for (const auto& goal : map.places) {
 const std::vector<float> ranges = scan_from_survey(
 map, goal.x_m, goal.y_m, goal.yaw_rad);
 const auto match = local_matcher.localize(
 ranges, -M_PI, 2.0 * M_PI / ranges.size(),
 goal.x_m + 0.18, goal.y_m - 0.12, goal.yaw_rad + 0.06,
 map, 1);
 const double match_error_m = std::hypot(
 match.x_m - goal.x_m, match.y_m - goal.y_m);
 if (!match.accepted || match_error_m >= 0.30) {
 std::fprintf(stderr,
 "endpoint localization failed place=%s reason=%s score=%.3f "
 "overlap=%.3f margin=%.3f returns=%zu error=%.3f pose=(%.3f,%.3f)\n",
 goal.place_id.c_str(), match.rejection_reason.c_str(), match.score,
 match.overlap, match.score_margin, match.usable_returns, match_error_m,
 match.x_m, match.y_m);
 }
 assert(match.accepted);
 assert(match_error_m < 0.30);
 }

 robot::autonomy::AStarPlanner planner;
 const robot::autonomy::LocalCostmap costmap =
 robot::autonomy::make_global_costmap(
 map, 0.4 + robot::platform::kDefaultNavigationClearanceMarginM,
 robot::platform::kDefaultNavigationSoftInflationRadiusM, true);
 assert(costmap.valid());
 const std::vector<std::string> mission{
 "west_hub", "north_depot", "central_gate",
 "east_terminal", "south_yard"};
 auto current = at(0.0, 0.0);
 std::size_t route_points = 0;
 for (const auto& place_id : mission) {
 const auto& goal = place(map, place_id);
 const auto route = planner.plan(
 current, {goal.x_m, goal.y_m, goal.yaw_rad}, map);
 assert_route_clear(route, costmap.lethal);
 route_points += route.points.size();
 current = at(goal.x_m, goal.y_m);
 current.yaw_rad = goal.yaw_rad;
 }

 // Car-like planning first finds the same map-derived 2D guide and then
 // refines it with sparse, corridor-bounded Hybrid-A* states. The raster is
 // deliberately too large for cells x headings x gears dense allocation.
 robot::autonomy::AckermannPlanner ackermann;
 auto ackermann_current = at(0.0, 0.0);
 std::size_t ackermann_route_points = 0;
 for (const auto& place_id : mission) {
 const auto& goal = place(map, place_id);
 const auto route = ackermann.plan(
 ackermann_current, {goal.x_m, goal.y_m, goal.yaw_rad}, map);
 assert_route_clear(route, costmap.lethal);
 ackermann_route_points += route.points.size();
 ackermann_current = at(goal.x_m, goal.y_m);
 ackermann_current.yaw_rad = goal.yaw_rad;
 }

 // Replanning must work from actual mid-mission poses, not only authored
 // endpoints. These two poses previously exhausted a whole-route Hybrid-A*
 // budget and a local refinement deadline respectively.
 const auto& north_depot = place(map, "north_depot");
 robot::autonomy::AckermannPlanner::Config recovery_config;
 recovery_config.wheel_base_m = 0.62;
 for (const auto& start_pose : std::vector<robot::autonomy::EstimatedState>{
 [] { auto p = at(46.59, -118.57); p.yaw_rad = 3.13; return p; }(),
 [] { auto p = at(42.045, -119.736); p.yaw_rad = -1.762; return p; }(),
 [] { auto p = at(27.35, -70.05); p.yaw_rad = -1.02; return p; }()}) {
 robot::autonomy::AckermannPlanner recovery(recovery_config);
 const auto route = recovery.plan(
 start_pose, start_pose.x_m < 30.0
 ? robot::autonomy::RoutePoint{42.0, -120.0, -M_PI_2}
 : robot::autonomy::RoutePoint{
 north_depot.x_m, north_depot.y_m, north_depot.yaw_rad}, map);
 assert_route_clear(route, costmap.lethal);
 if (start_pose.x_m < 30.0) {
 // A car may enter the soft band once while backing out and changing
 // curvature. It must then leave instead of repeatedly weaving through the
 // same clearance cost beside a long wall.
 assert(soft_band_reentries(route, costmap) <= 1);
 }
 }

 // A long mission can replan just after leaving the cluttered startup bay.
 // The local Hybrid-A* refinement must reconnect this measured pose to the
 // map-derived long-distance guide instead of exhausting its first segment.
 {
  auto replanning_pose = at(3.948585579, -1.266295216);
  replanning_pose.yaw_rad = -0.559507758;
  const auto& central_gate = place(map, "central_gate");
  robot::autonomy::AckermannPlanner replanner(recovery_config);
 const auto route = replanner.plan(
   replanning_pose,
   {central_gate.x_m, central_gate.y_m, central_gate.yaw_rad}, map);
  assert_route_clear(route, costmap.lethal);
 }

 // Replanning must also remain finite after the local controller stops at an
 // intermediate Ackermann pose. This measured-only pose is far from the map
 // boundary and has a valid 2D guide; refining it must not spend an unbounded
 // number of states trying to reconnect to the guide's first lattice sample.
 {
 auto replanning_pose = at(197.410, -18.428);
 replanning_pose.yaw_rad = -0.262;
 const auto& central_gate = place(map, "central_gate");
 robot::autonomy::AckermannPlanner replanner(recovery_config);
 replanner.begin_plan(
 replanning_pose,
 {central_gate.x_m, central_gate.y_m, central_gate.yaw_rad}, map);
 robot::autonomy::GlobalPlanProgress progress;
 std::size_t slices = 0;
 do {
 progress = replanner.continue_plan(replanner.incremental_plan_budget());
 ++slices;
 } while (progress.status == robot::autonomy::GlobalPlanStatus::kPending);
 // At the production 20 ms decision cadence, 64 deterministic slices are a
 // 1.28 s upper bound from a stopped pose to a usable route. This is a work
 // budget contract, so it is independent of host CPU speed and wall clocks.
 constexpr std::size_t kMaximumReplanDecisionCycles = 64;
 assert(slices <= kMaximumReplanDecisionCycles);
 assert(progress.status == robot::autonomy::GlobalPlanStatus::kSucceeded);
 const auto& route = progress.route;
 assert_route_clear(route, costmap.lethal);
 }

 // Terminal replanning must also stay bounded when translation and heading
 // are both outstanding. This is the measured ROBOT pose reached by the
 // direct central-gate mission before its old route was exhausted; a global
 // planner that remains pending here leaves an otherwise localized robot
 // stopped indefinitely.
 {
  auto replanning_pose = at(249.200, -3.136);
  replanning_pose.yaw_rad = 1.181;
  const auto& central_gate = place(map, "central_gate");
  robot::autonomy::AckermannPlanner replanner(recovery_config);
  replanner.begin_plan(
   replanning_pose,
   {central_gate.x_m, central_gate.y_m, central_gate.yaw_rad}, map);
  robot::autonomy::GlobalPlanProgress progress;
  std::size_t slices = 0;
  constexpr std::size_t kMaximumTerminalReplanDecisionCycles = 64;
  do {
   progress = replanner.continue_plan(replanner.incremental_plan_budget());
   ++slices;
  } while (progress.status == robot::autonomy::GlobalPlanStatus::kPending &&
           slices <= kMaximumTerminalReplanDecisionCycles);
  assert(slices <= kMaximumTerminalReplanDecisionCycles);
  assert(progress.status == robot::autonomy::GlobalPlanStatus::kSucceeded);
  assert_route_clear(progress.route, costmap.lethal);
 }

 // Dynamic avoidance can leave the vehicle farther behind the terminal
 // while retaining a large heading error.  The incremental planner must
 // still return a trackable pose-to-pose manoeuvre instead of repeatedly
 // consuming the local planner's replan requests.
 {
 auto replanning_pose = at(249.160, -4.970);
 replanning_pose.yaw_rad = 1.220;
 const auto& central_gate = place(map, "central_gate");
 robot::autonomy::AckermannPlanner replanner(recovery_config);
 replanner.begin_plan(
 replanning_pose,
 {central_gate.x_m, central_gate.y_m, central_gate.yaw_rad}, map);
 robot::autonomy::GlobalPlanProgress progress;
 std::size_t slices = 0;
 constexpr std::size_t kMaximumAvoidanceReplanDecisionCycles = 64;
 do {
 progress = replanner.continue_plan(replanner.incremental_plan_budget());
 ++slices;
 } while (progress.status == robot::autonomy::GlobalPlanStatus::kPending &&
 slices <= kMaximumAvoidanceReplanDecisionCycles);
 assert(slices <= kMaximumAvoidanceReplanDecisionCycles);
 assert(progress.status == robot::autonomy::GlobalPlanStatus::kSucceeded);
 assert_route_clear(progress.route, costmap.lethal);
 }

 std::printf(
 "industrial_200k ok: extent=%.0fx%.0f cells=%ux%u actors=%zu "
 "route_points=%zu ackermann_route_points=%zu "
 "startup=(%.3f,%.3f,%.3f) global_cold_ms=%.1f global_warm_ms=%.1f\n",
 max_x - min_x, max_y - min_y, map.width, map.height,
 session.setup_world().actors().size(), route_points,
 ackermann_route_points,
 startup.x_m, startup.y_m, startup.yaw_rad,
 cold_search_ms, warm_search_ms);
 return 0;
}
