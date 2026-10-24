#include "robot/autonomy/planning/ackermann_planner.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

#include "robot/core/geometry.hpp"
#include "robot/autonomy/planning/route_smoothing.hpp"

namespace lost_signal::robot::autonomy {
namespace {

struct Node {
 double x = 0.0;
 double y = 0.0;
 double yaw = 0.0;
 double g = 0.0;
 int parent = -1;
 int gear = 1;
};

int heading_bin(double yaw, int bins) {
 const double wrapped = core::wrap_angle(yaw) + core::kPi;
 return std::clamp(static_cast<int>(wrapped * bins / (2.0 * core::kPi)),
 0, bins - 1);
}

} // namespace

AckermannPlanner::AckermannPlanner() : config_() {}

AckermannPlanner::AckermannPlanner(Config config) : config_(config) {}

Route AckermannPlanner::plan(const EstimatedState& start,
 const RoutePoint& goal, const iface::MapSnapshot& map) {
 // A car-like planner must never silently fall back to a holonomic/grid
 // route: that would produce a path the chassis cannot realize. A vector
 // map is still allowed to use the straight-line seed because it contains
 // no collision evidence; the local ROBOT map and validator remain the
 // admission gates for motion.
 if (!start.valid || !std::isfinite(start.x_m) ||
 !std::isfinite(start.y_m) || !std::isfinite(start.yaw_rad) ||
 !std::isfinite(goal.x_m) || !std::isfinite(goal.y_m) ||
 !std::isfinite(goal.yaw_rad)) {
 return Route{};
 }
 if (!(config_.wheel_base_m > 0.0) ||
 !(config_.max_steer_rad > 0.0) ||
 !(config_.primitive_length_m > 0.0) ||
 !(config_.reverse_cost > 0.0) || config_.gear_change_cost < 0.0 ||
 !(config_.goal_position_tolerance_m >= 0.0) ||
 !(config_.goal_heading_tolerance_rad >= 0.0) ||
 config_.heading_bins < 8 || config_.max_expansions <= 0 ||
 config_.max_states < 8 ||
 !std::isfinite(config_.wheel_base_m) ||
 !std::isfinite(config_.max_steer_rad) ||
 !std::isfinite(config_.primitive_length_m) ||
 !std::isfinite(config_.reverse_cost) ||
 !std::isfinite(config_.gear_change_cost) ||
 !std::isfinite(config_.goal_position_tolerance_m) ||
 !std::isfinite(config_.goal_heading_tolerance_rad)) {
 return Route{};
 }
 if (!map.has_raster()) {
 StraightLinePlanner fallback;
 return fallback.plan(start, goal, map);
 }
 if (map.width > static_cast<std::uint32_t>(std::numeric_limits<int>::max()) ||
 map.height > static_cast<std::uint32_t>(std::numeric_limits<int>::max())) {
 return Route{};
 }
 const int w = static_cast<int>(map.width);
 const int h = static_cast<int>(map.height);
 const int dirs = config_.allow_reverse ? 2 : 1;
 const double res = map.resolution_m;
 if (w <= 0 || h <= 0 || !std::isfinite(res) || !(res > 0.0) ||
 !std::isfinite(map.origin_x_m) || !std::isfinite(map.origin_y_m)) {
 return Route{};
 }
 const std::size_t cells = static_cast<std::size_t>(w) *
 static_cast<std::size_t>(h);
 if (cells == 0 || cells > config_.max_states /
 static_cast<std::size_t>(dirs)) {
 return Route{};
 }
 const std::size_t max_bins = config_.max_states /
 (cells * static_cast<std::size_t>(dirs));
 const int bins = static_cast<int>(std::min<std::size_t>(
 static_cast<std::size_t>(config_.heading_bins), max_bins));
 if (bins < 8) return Route{};
 auto grid = [&](double x, double y, int& gx, int& gy) {
 gx = static_cast<int>(std::floor((x - map.origin_x_m) / res));
 gy = static_cast<int>(std::floor((y - map.origin_y_m) / res));
 };
 auto inside = [&](int gx, int gy) {
 return gx >= 0 && gy >= 0 && gx < w && gy < h;
 };
 auto key = [&](double x, double y, double yaw, int gear) {
 int gx = 0, gy = 0;
 grid(x, y, gx, gy);
 if (!inside(gx, gy)) return -1;
 const int gb = (dirs == 2 && gear < 0) ? 1 : 0;
 return (((gy * w + gx) * bins + heading_bin(yaw, bins)) * dirs + gb);
 };
 int sx = 0, sy = 0, gx = 0, gy = 0;
 grid(start.x_m, start.y_m, sx, sy);
 grid(goal.x_m, goal.y_m, gx, gy);
 if (!inside(sx, sy) || !inside(gx, gy) || map.blocked(start.x_m, start.y_m) ||
 map.blocked(goal.x_m, goal.y_m)) {
 return Route{};
 }
 const double initial_distance =
 std::hypot(goal.x_m - start.x_m, goal.y_m - start.y_m);
 if (initial_distance <= config_.goal_position_tolerance_m &&
 std::abs(core::wrap_angle(goal.yaw_rad - start.yaw_rad)) <=
 config_.goal_heading_tolerance_rad) {
 Route out;
 out.points = {{start.x_m, start.y_m, core::wrap_angle(start.yaw_rad), 1},
 {goal.x_m, goal.y_m, core::wrap_angle(goal.yaw_rad), 1}};
 out.valid = true;
 return out;
 }
 const std::size_t state_count = cells * static_cast<std::size_t>(bins) *
 static_cast<std::size_t>(dirs);
 std::vector<double> best(static_cast<std::size_t>(state_count),
 std::numeric_limits<double>::infinity());
 std::vector<Node> nodes;
 nodes.reserve(std::min<std::size_t>(
 static_cast<std::size_t>(config_.max_expansions), state_count));
 using Entry = std::pair<double, int>;
 std::priority_queue<Entry, std::vector<Entry>, std::greater<Entry>> open;
 nodes.push_back({start.x_m, start.y_m, core::wrap_angle(start.yaw_rad), 0.0,
 -1, 1});
 const int start_key = key(start.x_m, start.y_m, start.yaw_rad, 1);
 if (start_key < 0) return Route{};
 best[static_cast<std::size_t>(start_key)] = 0.0;
 open.emplace(std::hypot(goal.x_m - start.x_m, goal.y_m - start.y_m), 0);

 const std::array<double, 5> steer_factors{-1.0, -0.5, 0.0, 0.5, 1.0};
 const std::array<int, 2> gears{1, -1};
 int goal_node = -1;
 int expanded = 0;
 while (!open.empty() && expanded < config_.max_expansions) {
 const int index = open.top().second;
 open.pop();
 const Node current = nodes[static_cast<std::size_t>(index)];
 const int current_key = key(current.x, current.y, current.yaw,
 current.gear);
 if (current_key < 0 || current.g > best[static_cast<std::size_t>(current_key)] + 1e-9)
 continue;
 ++expanded;
 const double distance = std::hypot(goal.x_m - current.x,
 goal.y_m - current.y);
 if (index != 0 && distance <= config_.goal_position_tolerance_m &&
 std::abs(core::wrap_angle(goal.yaw_rad - current.yaw)) <=
 config_.goal_heading_tolerance_rad) {
 goal_node = index;
 break;
 }
 for (int gear_slot = 0; gear_slot < dirs; ++gear_slot) {
 const int gear = gears[static_cast<std::size_t>(gear_slot)];
 for (double factor : steer_factors) {
 const double steer = factor * config_.max_steer_rad;
 const double curvature = std::tan(steer) / config_.wheel_base_m;
 const int samples = std::max(3, static_cast<int>(std::ceil(
 config_.primitive_length_m / std::max(0.05, 0.45 * res))));
 double x = current.x, y = current.y, yaw = current.yaw;
 bool feasible = true;
 double previous_x = x, previous_y = y;
 for (int sample = 0; sample < samples; ++sample) {
 const double ds = gear * config_.primitive_length_m / samples;
 const double mid = yaw + 0.5 * ds * curvature;
 x += ds * std::cos(mid);
 y += ds * std::sin(mid);
 yaw = core::wrap_angle(yaw + ds * curvature);
 if (map.blocked(x, y)) {
 feasible = false;
 break;
 }
 // Keep the swept primitive conservative even when a cell corner is
 // crossed between integration samples.
 int ax = 0, ay = 0, bx = 0, by = 0;
 grid(previous_x, previous_y, ax, ay);
 grid(x, y, bx, by);
 if (std::abs(bx - ax) > 1 || std::abs(by - ay) > 1) {
 feasible = false;
 break;
 }
 previous_x = x;
 previous_y = y;
 }
 if (!feasible) continue;
 const int next_key = key(x, y, yaw, gear);
 if (next_key < 0) continue;
 double edge = config_.primitive_length_m;
 if (gear < 0) edge *= config_.reverse_cost;
 if (gear != current.gear) edge += config_.gear_change_cost;
 // Traversal is a ROBOT-side belief. It can bias the route but never
 // substitutes for map collision checks.
 if (traversal_ != nullptr) {
 double confidence = 0.0;
 const double slip = traversal_->slip_at(x, y, &confidence);
 edge *= 1.0 + std::clamp(slip, 0.0, 1.0) * std::clamp(confidence, 0.0, 1.0);
 }
 const double next_g = current.g + edge + 0.01 * std::abs(factor);
 if (next_g + 1e-9 >= best[static_cast<std::size_t>(next_key)]) continue;
 best[static_cast<std::size_t>(next_key)] = next_g;
 nodes.push_back({x, y, yaw, next_g, index, gear});
 const int next_index = static_cast<int>(nodes.size() - 1);
 const double heuristic = std::hypot(goal.x_m - x, goal.y_m - y) +
 0.08 * config_.wheel_base_m *
 std::abs(core::wrap_angle(goal.yaw_rad - yaw));
 open.emplace(next_g + heuristic, next_index);
 }
 }
 }
 if (goal_node < 0) return Route{};
 std::vector<Node> path;
 for (int index = goal_node; index >= 0;
 index = nodes[static_cast<std::size_t>(index)].parent) {
 path.push_back(nodes[static_cast<std::size_t>(index)]);
 }
 std::reverse(path.begin(), path.end());
 Route out;
 out.valid = path.size() >= 2;
 out.points.reserve(path.size());
 for (const auto& node : path)
 out.points.push_back({node.x, node.y, node.yaw, node.gear});
 if (!smooth_ackermann_route(out, {config_.wheel_base_m,
 config_.max_steer_rad, 0.55, 2.0})) {
 return Route{};
 }
 return out;
}

} // namespace lost_signal::robot::autonomy
