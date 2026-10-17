#include "robot/autonomy/planning/global/hybrid_astar.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

namespace sim {
namespace {

struct Node {
 double x{0.0};
 double y{0.0};
 double heading{0.0};
 double g{0.0};
 int parent{-1};
 int direction{1};
 int steer_index{0};
};

int heading_bin(double heading, int bins) {
 const double normalized = wrap_angle(heading) + kPi;
 return std::clamp(static_cast<int>(normalized * bins / (2.0 * kPi)), 0, bins - 1);
}

} // namespace

AStarResult hybrid_astar_plan(const GridMap& map, Vec2 start, double start_heading_rad, Vec2 goal,
 double goal_heading_rad, const HybridAStarConfig& config,
 const TraversalCost& traversal_cost) {
 AStarResult result;
 if (config.wheel_base_m <= 0.0 || config.max_steer_rad <= 0.0 ||
 config.primitive_length_m <= 0.0 || config.heading_bins < 8) {
 return result;
 }
 const auto [sx, sy] = map.worldToGrid(start.x, start.y);
 const auto [gx, gy] = map.worldToGrid(goal.x, goal.y);
 if (!map.inside(sx, sy) || !map.inside(gx, gy) || !map.isFree(start.x, start.y)) {
 return result;
 }
 const int directions = config.allow_reverse ? 2 : 1;
 const int states_per_cell = config.heading_bins * directions;
 const auto key = [&](double x, double y, double heading, int direction) {
 const auto [cx, cy] = map.worldToGrid(x, y);
 if (!map.inside(cx, cy))
 return -1;
 const int direction_index = config.allow_reverse && direction < 0 ? 1 : 0;
 return ((cy * map.w() + cx) * config.heading_bins + heading_bin(heading, config.heading_bins)) *
 directions + direction_index;
 };
 const int state_count = map.w() * map.h() * states_per_cell;
 std::vector<double> best(static_cast<std::size_t>(state_count),
 std::numeric_limits<double>::infinity());
 std::vector<Node> nodes;
 nodes.reserve(16384);
 using Open = std::pair<double, int>;
 std::priority_queue<Open, std::vector<Open>, std::greater<Open>> open;
 nodes.push_back({start.x, start.y, wrap_angle(start_heading_rad), 0.0, -1, 1, 0});
 best[static_cast<std::size_t>(key(start.x, start.y, start_heading_rad, 1))] = 0.0;
 open.emplace(std::hypot(goal.x - start.x, goal.y - start.y), 0);

 const std::array<double, 3> steer_factors{-1.0, 0.0, 1.0};
 const int direction_values[2] = {1, -1};
 int goal_node = -1;
 while (!open.empty() && result.expanded < config.max_expansions) {
 const int node_index = open.top().second;
 open.pop();
 const Node current = nodes[static_cast<std::size_t>(node_index)];
 const int current_key = key(current.x, current.y, current.heading, current.direction);
 if (current_key < 0 || current.g > best[static_cast<std::size_t>(current_key)] + 1e-9) {
 continue;
 }
 ++result.expanded;
 const double goal_distance = std::hypot(goal.x - current.x, goal.y - current.y);
 const bool exact_stationary_goal = node_index == 0 && goal_distance <= 1e-9;
 if ((node_index != 0 || exact_stationary_goal) &&
 goal_distance <= config.goal_position_tolerance_m &&
 std::abs(wrap_angle(goal_heading_rad - current.heading)) <= config.goal_heading_tolerance_rad) {
 goal_node = node_index;
 break;
 }

 for (int direction_slot = 0; direction_slot < directions; ++direction_slot) {
 const int direction = direction_values[direction_slot];
 for (int steer_index = -1; steer_index <= 1; ++steer_index) {
 const double steer = steer_factors[static_cast<std::size_t>(steer_index + 1)] *
 config.max_steer_rad;
 const double signed_length = direction * config.primitive_length_m;
 const double curvature = std::tan(steer) / config.wheel_base_m;
 const int samples = std::max(2, static_cast<int>(
 std::ceil(config.primitive_length_m / std::max(0.05, map.res() * 0.45))));
 double x = current.x;
 double y = current.y;
 double heading = current.heading;
 Vec2 previous{x, y};
 double terrain_multiplier_sum = 0.0;
 bool feasible = true;
 for (int sample = 1; sample <= samples; ++sample) {
 const double ds = signed_length / samples;
 const double heading_mid = heading + 0.5 * ds * curvature;
 x += ds * std::cos(heading_mid);
 y += ds * std::sin(heading_mid);
 heading = wrap_angle(heading + ds * curvature);
 const Vec2 next{x, y};
 const double multiplier = traversal_cost ? traversal_cost(previous, next) : 1.0;
 if (!map.isFree(x, y) || !(multiplier > 0.0) || !std::isfinite(multiplier)) {
 feasible = false;
 break;
 }
 terrain_multiplier_sum += multiplier;
 previous = next;
 }
 if (!feasible)
 continue;
 const int next_key = key(x, y, heading, direction);
 if (next_key < 0)
 continue;
 double edge_cost = config.primitive_length_m * terrain_multiplier_sum / samples;
 if (direction < 0)
 edge_cost *= config.reverse_cost;
 if (direction != current.direction)
 edge_cost += config.direction_change_cost_m;
 edge_cost += config.steer_cost * std::abs(steer_index);
 edge_cost += 0.02 * std::abs(steer_index - current.steer_index);
 const double next_g = current.g + edge_cost;
 if (next_g + 1e-9 >= best[static_cast<std::size_t>(next_key)])
 continue;
 best[static_cast<std::size_t>(next_key)] = next_g;
 nodes.push_back({x, y, heading, next_g, node_index, direction, steer_index});
 const int next_index = static_cast<int>(nodes.size() - 1);
 const double heuristic = std::hypot(goal.x - x, goal.y - y) +
 0.12 * config.wheel_base_m * std::abs(wrap_angle(goal_heading_rad - heading));
 open.emplace(next_g + heuristic, next_index);
 }
 }
 }
 if (goal_node < 0)
 return result;
 result.ok = true;
 std::vector<Node> route_nodes;
 for (int index = goal_node; index >= 0; index = nodes[static_cast<std::size_t>(index)].parent) {
 route_nodes.push_back(nodes[static_cast<std::size_t>(index)]);
 }
 std::reverse(route_nodes.begin(), route_nodes.end());
 result.path.reserve(route_nodes.size() + 1);
 for (std::size_t index = 0; index < route_nodes.size(); ++index) {
 const auto& node = route_nodes[index];
 result.path.push_back({
 {node.x, node.y},
 node.heading,
 index == 0 ? TravelDirection::Unspecified
 : (node.direction < 0 ? TravelDirection::Reverse : TravelDirection::Forward),
 });
 }
 // Keep an exact endpoint for waypoint composition. The terminal anchor
 // inherits the final searched state's body heading and gear; it must not
 // invent the requested yaw as if the search had reached it exactly.
 const TravelDirection terminal_direction = route_nodes.size() > 1
 ? (route_nodes.back().direction < 0 ? TravelDirection::Reverse : TravelDirection::Forward)
 : TravelDirection::Unspecified;
 const double terminal_distance =
 std::hypot(goal.x - route_nodes.back().x, goal.y - route_nodes.back().y);
 // An authored goal may sit inside the optional clearance band while its
 // arrival region still contains safe poses. Do not append an unsafe exact
 // anchor in that case; the searched terminal pose already satisfies the
 // requested goal-region contract.
 if (terminal_distance > 1e-9 && map.isFree(goal.x, goal.y)) {
 result.path.push_back({goal, route_nodes.back().heading, terminal_direction});
 } else if (terminal_distance <= 1e-9) {
 result.path.back().position = goal;
 }
 return result;
}

} // namespace sim
