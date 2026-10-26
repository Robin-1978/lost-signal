#include "robot/autonomy/planning/ackermann_planner.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

#include "robot/core/geometry.hpp"

namespace lost_signal::robot::autonomy {
namespace {

int heading_bin(double yaw, int bins) {
 const double wrapped = core::wrap_angle(yaw) + core::kPi;
 return std::clamp(static_cast<int>(wrapped * bins / (2.0 * core::kPi)),
 0, bins - 1);
}

bool valid_config(const AckermannPlanner::Config& config) {
 return config.wheel_base_m > 0.0 && config.max_steer_rad > 0.0 &&
 config.primitive_length_m > 0.0 && config.heading_bins >= 8 &&
 config.max_curvature_delta_per_m > 0.0 &&
 config.curvature_cost_per_m >= 0.0 &&
 config.curvature_change_cost_m >= 0.0 &&
 config.guide_heuristic_weight >= 1.0 &&
 config.reverse_cost > 0.0 && config.gear_change_cost >= 0.0 &&
 config.goal_position_tolerance_m >= 0.0 &&
 config.goal_heading_tolerance_rad >= 0.0 &&
 config.connector_heading_tolerance_rad >= 0.0 &&
 (config.max_expansions == -1 || config.max_expansions > 0) &&
 (config.max_segment_compute_time_s == -1.0 ||
 config.max_segment_compute_time_s > 0.0) &&
 config.expansions_per_cycle > 0 &&
 config.corridor_radius_m > 0.0 && config.unknown_cost_per_m >= 0.0 &&
 config.clearance_cost_per_m >= 0.0 && config.footprint_radius_m > 0.0 &&
 config.hard_clearance_margin_m >= 0.0 &&
 config.soft_inflation_radius_m >= 0.0 && config.max_states >= 8 &&
 std::isfinite(config.wheel_base_m) &&
 std::isfinite(config.max_steer_rad) &&
 std::isfinite(config.primitive_length_m) &&
 std::isfinite(config.max_curvature_delta_per_m) &&
 std::isfinite(config.curvature_cost_per_m) &&
 std::isfinite(config.curvature_change_cost_m) &&
 std::isfinite(config.guide_heuristic_weight) &&
 std::isfinite(config.reverse_cost) &&
 std::isfinite(config.gear_change_cost) &&
 std::isfinite(config.goal_position_tolerance_m) &&
 std::isfinite(config.goal_heading_tolerance_rad) &&
 std::isfinite(config.connector_heading_tolerance_rad) &&
 std::isfinite(config.max_segment_compute_time_s) &&
 std::isfinite(config.corridor_radius_m) &&
 std::isfinite(config.unknown_cost_per_m) &&
 std::isfinite(config.clearance_cost_per_m) &&
 std::isfinite(config.footprint_radius_m) &&
 std::isfinite(config.hard_clearance_margin_m) &&
 std::isfinite(config.soft_inflation_radius_m);
}

AStarPlanner::Config guide_config(const AckermannPlanner::Config& config) {
 AStarPlanner::Config guide;
 guide.expansions_per_cycle = config.expansions_per_cycle;
 guide.unknown_cost_per_m = config.unknown_cost_per_m;
 guide.clearance_cost_per_m = config.clearance_cost_per_m;
 guide.footprint_radius_m = config.footprint_radius_m;
 guide.hard_clearance_margin_m = config.hard_clearance_margin_m;
 guide.soft_inflation_radius_m = config.soft_inflation_radius_m;
 return guide;
}

bool feasible_terminal_connector(double x, double y, double yaw,
 double curvature, int gear, const RoutePoint& goal,
 const AckermannPlanner::Config& config) {
 const double dx = goal.x_m - x;
 const double dy = goal.y_m - y;
 const double chord_m = std::hypot(dx, dy);
 if (!(chord_m > 1e-6)) {
 return std::abs(core::wrap_angle(goal.yaw_rad - yaw)) <= 1e-6;
 }
 const double heading_delta = core::wrap_angle(goal.yaw_rad - yaw);
 const double signed_chord_m = gear < 0 ? -chord_m : chord_m;
 const double connector_curvature =
 2.0 * std::sin(0.5 * heading_delta) / signed_chord_m;
 const double maximum_curvature =
 std::tan(config.max_steer_rad) / config.wheel_base_m;
 if (std::abs(connector_curvature) > maximum_curvature + 1e-6) {
 return false;
 }
 const double body_mid_yaw = core::wrap_angle(yaw + 0.5 * heading_delta);
 const double chord_yaw = std::atan2(dy, dx);
 const double travel_yaw = core::wrap_angle(
 body_mid_yaw + (gear < 0 ? core::kPi : 0.0));
 if (std::abs(core::wrap_angle(chord_yaw - travel_yaw)) >
 config.connector_heading_tolerance_rad) {
 return false;
 }
 return std::abs(connector_curvature - curvature) <=
 config.max_curvature_delta_per_m * chord_m + 1e-9;
}

} // namespace

struct AckermannPlanner::SearchState {
 enum class Phase { kGuide, kHybrid };

 struct Node {
 double x = 0.0;
 double y = 0.0;
 double yaw = 0.0;
 double g = 0.0;
 int parent = -1;
 int gear = 1;
 double curvature = 0.0;
 std::size_t corridor_index = 0;
 };

 struct Entry {
 double f = 0.0;
 double g = 0.0;
 int node = -1;
 };

 struct EntryGreater {
 bool operator()(const Entry& a, const Entry& b) const {
 if (a.f != b.f) return a.f > b.f;
 if (a.g != b.g) return a.g < b.g;
 return a.node > b.node;
 }
 };

 const iface::MapSnapshot* map = nullptr;
 const LocalCostmap* planning_map = nullptr;
 EstimatedState start;
 RoutePoint goal;
 Route guide;
 std::vector<double> guide_remaining_m;
 RoutePoint segment_goal;
 std::size_t segment_start_index = 0;
 std::size_t segment_goal_index = 0;
 int segment_start_gear = 1;
 double segment_start_curvature = 0.0;
 Route assembled;
 Phase phase = Phase::kGuide;
 int width = 0;
 int height = 0;
 int bins = 0;
 int dirs = 0;
 double resolution_m = 0.0;
 int expanded = 0;
 int segment_expanded = 0;
 double segment_compute_time_s = 0.0;
 std::unordered_map<std::uint64_t, double> best;
 std::vector<Node> nodes;
 std::priority_queue<Entry, std::vector<Entry>, EntryGreater> open;
};

AckermannPlanner::AckermannPlanner() : AckermannPlanner(Config{}) {}

AckermannPlanner::AckermannPlanner(Config config)
 : config_(config), guide_planner_(guide_config(config)) {}

AckermannPlanner::~AckermannPlanner() = default;
AckermannPlanner::AckermannPlanner(AckermannPlanner&&) noexcept = default;
AckermannPlanner& AckermannPlanner::operator=(AckermannPlanner&&) noexcept =
 default;

void AckermannPlanner::cancel_plan() {
 guide_planner_.cancel_plan();
 search_.reset();
 completed_ = GlobalPlanProgress{};
 completed_ready_ = false;
}

void AckermannPlanner::begin_plan(const EstimatedState& start,
 const RoutePoint& goal, const iface::MapSnapshot& map) {
 cancel_plan();
 auto complete = [&](GlobalPlanStatus status, Route route = {}) {
 completed_.status = status;
 completed_.route = std::move(route);
 completed_ready_ = true;
 };

 if (!start.valid || !std::isfinite(start.x_m) ||
 !std::isfinite(start.y_m) || !std::isfinite(start.yaw_rad) ||
 !std::isfinite(goal.x_m) || !std::isfinite(goal.y_m) ||
 !std::isfinite(goal.yaw_rad) || !valid_config(config_)) {
 complete(GlobalPlanStatus::kFailed);
 return;
 }
 if (!map.has_raster()) {
 StraightLinePlanner fallback;
 Route route = fallback.plan(start, goal, map);
 complete(route.valid ? GlobalPlanStatus::kSucceeded
 : GlobalPlanStatus::kFailed,
 std::move(route));
 return;
 }
 if (map.width > static_cast<std::uint32_t>(
 std::numeric_limits<int>::max()) ||
 map.height > static_cast<std::uint32_t>(
 std::numeric_limits<int>::max()) ||
 !std::isfinite(map.resolution_m) || !(map.resolution_m > 0.0) ||
 !std::isfinite(map.origin_x_m) || !std::isfinite(map.origin_y_m)) {
 complete(GlobalPlanStatus::kFailed);
 return;
 }

 auto state = std::make_unique<SearchState>();
 state->map = &map;
 state->start = start;
 state->goal = goal;
 state->width = static_cast<int>(map.width);
 state->height = static_cast<int>(map.height);
 state->bins = config_.heading_bins;
 state->dirs = config_.allow_reverse ? 2 : 1;
 state->resolution_m = map.resolution_m;
 search_ = std::move(state);
 guide_planner_.begin_plan(start, goal, map);
}

GlobalPlanProgress AckermannPlanner::continue_plan(
 std::size_t expansion_budget) {
 if (completed_ready_) {
 completed_ready_ = false;
 return std::move(completed_);
 }
 if (search_ == nullptr) return {GlobalPlanStatus::kFailed, {}};
 if (expansion_budget == 0) return {GlobalPlanStatus::kPending, {}};

 SearchState& state = *search_;
 const auto fail = [&](std::string diagnostic) {
 search_.reset();
 return GlobalPlanProgress{
 GlobalPlanStatus::kFailed, {}, std::move(diagnostic)};
 };
 const iface::MapSnapshot& map = *state.map;
 const auto grid = [&](double x, double y, int& gx, int& gy) {
 gx = static_cast<int>(std::floor((x - map.origin_x_m) /
 state.resolution_m));
 gy = static_cast<int>(std::floor((y - map.origin_y_m) /
 state.resolution_m));
 };
 const auto inside = [&](int gx, int gy) {
 return gx >= 0 && gy >= 0 && gx < state.width && gy < state.height;
 };
 const auto segment_is_free = [&](double x0, double y0,
 double x1, double y1) {
 if (state.planning_map == nullptr) return false;
 const double length = std::hypot(x1 - x0, y1 - y0);
 const int samples = std::max(1, static_cast<int>(std::ceil(
 length / std::max(0.05, 0.45 * state.resolution_m))));
 for (int sample = 0; sample <= samples; ++sample) {
 const double u = static_cast<double>(sample) / samples;
 if (state.planning_map->lethal.is_blocked(
 x0 + u * (x1 - x0), y0 + u * (y1 - y0))) {
 return false;
 }
 }
 return true;
 };

 const double maximum_curvature =
 std::tan(config_.max_steer_rad) / config_.wheel_base_m;
 const auto key = [&](double x, double y, double yaw, int gear,
 double curvature) {
 int cell_x = 0, cell_y = 0;
 grid(x, y, cell_x, cell_y);
 if (!inside(cell_x, cell_y)) {
 return std::numeric_limits<std::uint64_t>::max();
 }
 const std::uint64_t cell = static_cast<std::uint64_t>(cell_y) *
 static_cast<std::uint64_t>(state.width) +
 static_cast<std::uint64_t>(cell_x);
 const std::uint64_t gear_bin =
 state.dirs == 2 && gear < 0 ? 1ULL : 0ULL;
 const int steer_bin = std::clamp(static_cast<int>(std::llround(
 2.0 * curvature / maximum_curvature)) + 2, 0, 4);
 return (((cell * static_cast<std::uint64_t>(state.bins) +
 static_cast<std::uint64_t>(heading_bin(yaw, state.bins))) *
 static_cast<std::uint64_t>(state.dirs) + gear_bin) *
 static_cast<std::uint64_t>(5)) +
 static_cast<std::uint64_t>(steer_bin);
 };
 const auto begin_hybrid_segment = [&]() {
 // Keep each kinematic search local while preserving one continuous route.
 // The horizon is derived from both the map corridor and the chassis minimum
 // turning radius, so work scales with route length without a level-specific
 // distance or an unbounded whole-route lattice.
 const double minimum_turning_radius_m = 1.0 / maximum_curvature;
 // Keep refinement local enough that branching cannot grow with mission
 // length, but long enough to contain a complete minimum-radius turnaround.
 // Both terms come from configured geometry rather than map or mission size.
 const double refinement_horizon_m = std::max(
 config_.corridor_radius_m, 2.0 * minimum_turning_radius_m);
 double guide_distance_m = 0.0;
 std::size_t endpoint = state.segment_start_index;
 while (endpoint + 1 < state.guide.points.size() &&
 guide_distance_m < refinement_horizon_m) {
 const RoutePoint& from = state.guide.points[endpoint];
 const RoutePoint& to = state.guide.points[endpoint + 1];
 guide_distance_m += std::hypot(to.x_m - from.x_m, to.y_m - from.y_m);
 ++endpoint;
 }
 state.segment_goal_index = endpoint;
 if (endpoint + 1 == state.guide.points.size()) {
 state.segment_goal = state.goal;
 } else {
 const std::size_t before = endpoint > 0 ? endpoint - 1 : endpoint;
 const std::size_t after = std::min(
 endpoint + 1, state.guide.points.size() - 1);
 const RoutePoint& target = state.guide.points[endpoint];
 const RoutePoint& tangent_from = state.guide.points[before];
 const RoutePoint& tangent_to = state.guide.points[after];
 state.segment_goal = target;
 state.segment_goal.yaw_rad = std::atan2(
 tangent_to.y_m - tangent_from.y_m,
 tangent_to.x_m - tangent_from.x_m);
 state.segment_goal.gear = state.segment_start_gear;
 }

 state.best.clear();
 state.nodes.clear();
 state.open = {};
 state.segment_expanded = 0;
 state.segment_compute_time_s = 0.0;
 const RoutePoint& start_pose = state.assembled.points.back();
 const std::uint64_t start_key = key(
 start_pose.x_m, start_pose.y_m, start_pose.yaw_rad,
 state.segment_start_gear, state.segment_start_curvature);
 if (start_key == std::numeric_limits<std::uint64_t>::max()) return false;
 const std::size_t initial_sparse_capacity = std::min<std::size_t>(
 config_.max_states,
 std::max<std::size_t>(16'384, config_.expansions_per_cycle * 4));
 state.best.reserve(initial_sparse_capacity);
 state.nodes.reserve(initial_sparse_capacity);
 state.nodes.push_back({start_pose.x_m, start_pose.y_m,
 core::wrap_angle(start_pose.yaw_rad), 0.0, -1,
 state.segment_start_gear, state.segment_start_curvature,
 state.segment_start_index});
 state.best.emplace(start_key, 0.0);
 const double remaining_to_segment_m = std::max(
 0.0, state.guide_remaining_m[state.segment_start_index] -
 state.guide_remaining_m[state.segment_goal_index]);
 state.open.push({config_.guide_heuristic_weight * remaining_to_segment_m,
 0.0, 0});
 return true;
 };

 if (state.phase == SearchState::Phase::kGuide) {
 GlobalPlanProgress guide = guide_planner_.continue_plan(expansion_budget);
 if (guide.status == GlobalPlanStatus::kPending) return guide;
 const bool pose_only_maneuver = std::hypot(
 state.goal.x_m - state.start.x_m,
 state.goal.y_m - state.start.y_m) <=
 config_.goal_position_tolerance_m;
 if ((guide.status != GlobalPlanStatus::kSucceeded || !guide.route.valid ||
 guide.route.points.size() < 2) && pose_only_maneuver) {
 // Grid A* has no route when start and goal occupy the same cell, while a
 // car may still need a multi-point maneuver to change heading. Its failed
 // search has already prepared the same inflated costmap; seed Hybrid-A*
 // with a zero-length two-pose guide and retain every kinematic/collision
 // check in the refinement stage.
 guide.route.valid = true;
 guide.route.points = {
 {state.start.x_m, state.start.y_m, state.start.yaw_rad, 1},
 state.goal};
 } else if (guide.status != GlobalPlanStatus::kSucceeded ||
 !guide.route.valid || guide.route.points.size() < 2) {
 return fail(guide.diagnostic.empty()
 ? "2D guide search failed"
 : "2D guide search failed: " + guide.diagnostic);
 }
 state.planning_map = guide_planner_.prepared_costmap(map);
 if (state.planning_map == nullptr) {
 return fail("2D guide has no prepared costmap");
 }
 state.guide = std::move(guide.route);
 state.guide_remaining_m.assign(state.guide.points.size(), 0.0);
 for (std::size_t index = state.guide.points.size() - 1; index > 0;
 --index) {
 const RoutePoint& from = state.guide.points[index - 1];
 const RoutePoint& to = state.guide.points[index];
 state.guide_remaining_m[index - 1] = state.guide_remaining_m[index] +
 std::hypot(to.x_m - from.x_m, to.y_m - from.y_m);
 }
 state.phase = SearchState::Phase::kHybrid;

 int sx = 0, sy = 0, gx = 0, gy = 0;
 grid(state.start.x_m, state.start.y_m, sx, sy);
 grid(state.goal.x_m, state.goal.y_m, gx, gy);
 if (!inside(sx, sy) || !inside(gx, gy) ||
 state.planning_map->lethal.is_blocked(state.start.x_m, state.start.y_m) ||
 state.planning_map->lethal.is_blocked(state.goal.x_m, state.goal.y_m)) {
 return fail("start or goal is inside the inflated lethal map");
 }

 // Hybrid A* uses analytic expansions for simple pose connections before
 // opening the lattice. Without this fast path, cell/heading quantization
 // can discover a long looping path before the exact straight primitive,
 // even in open space. The same curvature, steering-rate and swept lethal
 // checks used by a terminal connector make the shortcut fully kinematic.
 const std::array<int, 2> direct_gears{1, -1};
 for (std::size_t gear_index = 0;
 gear_index < (config_.allow_reverse ? direct_gears.size() : 1U);
 ++gear_index) {
 const int gear = direct_gears[gear_index];
 if (!feasible_terminal_connector(
 state.start.x_m, state.start.y_m, state.start.yaw_rad, 0.0,
 gear, state.goal, config_) ||
 !segment_is_free(state.start.x_m, state.start.y_m,
 state.goal.x_m, state.goal.y_m)) {
 continue;
 }
 Route direct;
 direct.valid = true;
 direct.points = {
 {state.start.x_m, state.start.y_m,
 core::wrap_angle(state.start.yaw_rad), gear},
 {state.goal.x_m, state.goal.y_m,
 core::wrap_angle(state.goal.yaw_rad), gear}};
 search_.reset();
 return {GlobalPlanStatus::kSucceeded, std::move(direct)};
 }

 state.assembled.valid = true;
 state.assembled.points = {{state.start.x_m, state.start.y_m,
 core::wrap_angle(state.start.yaw_rad), 1}};
 if (!begin_hybrid_segment()) {
 return fail("cannot initialize first Hybrid-A* segment");
 }
 // Guide preparation and Hybrid-A* expansion have separate deterministic
 // budgets. Start fine refinement on the next decision cycle.
 return {GlobalPlanStatus::kPending, {}};
 }

 const LocalCostmap& planning_map = *state.planning_map;
 const auto blocked = [&](double x, double y) {
 return planning_map.lethal.is_blocked(x, y);
 };
 const auto original_unknown = [&](double x, double y) {
 int cell_x = 0, cell_y = 0;
 grid(x, y, cell_x, cell_y);
 return inside(cell_x, cell_y) &&
 map.occupancy[static_cast<std::size_t>(cell_y) * map.width +
 static_cast<std::size_t>(cell_x)] < 0;
 };
 const auto corridor_match = [&](double x, double y,
 std::size_t hint, std::size_t& match, double& distance_to_guide) {
 const std::size_t window = static_cast<std::size_t>(std::ceil(
 2.0 * config_.primitive_length_m /
 std::max(1e-6, state.resolution_m))) + 8;
 const std::size_t first = hint > window ? hint - window : 0;
 const std::size_t last = std::min(
 state.guide.points.size() - 1, hint + window);
 double best_distance = std::numeric_limits<double>::infinity();
 std::size_t best_index = hint;
 for (std::size_t index = first; index <= last; ++index) {
 const RoutePoint& point = state.guide.points[index];
 const double distance = std::hypot(x - point.x_m, y - point.y_m);
 if (distance < best_distance) {
 best_distance = distance;
 best_index = index;
 }
 }
 match = best_index;
 distance_to_guide = best_distance;
 return best_distance <= config_.corridor_radius_m + 1e-9;
 };
 const std::array<double, 5> steer_factors{-1.0, -0.5, 0.0, 0.5, 1.0};
 const std::array<int, 2> gears{1, -1};
 int goal_node = -1;
 std::size_t work = 0;
 const auto slice_started = std::chrono::steady_clock::now();
 const auto segment_compute_time_s = [&]() {
 return state.segment_compute_time_s +
 std::chrono::duration<double>(
 std::chrono::steady_clock::now() - slice_started).count();
 };
 while (!state.open.empty() && work < expansion_budget &&
 (config_.max_expansions < 0 ||
 state.segment_expanded < config_.max_expansions) &&
 (config_.max_segment_compute_time_s < 0.0 ||
 segment_compute_time_s() < config_.max_segment_compute_time_s)) {
 const SearchState::Entry entry = state.open.top();
 state.open.pop();
 ++work;
 const SearchState::Node current =
 state.nodes[static_cast<std::size_t>(entry.node)];
 const std::uint64_t current_key = key(
 current.x, current.y, current.yaw, current.gear, current.curvature);
 const auto current_best = state.best.find(current_key);
 if (current_best == state.best.end() ||
 current.g > current_best->second + 1e-9) {
 continue;
 }
 ++state.expanded;
 ++state.segment_expanded;
 const double distance = std::hypot(
 state.segment_goal.x_m - current.x,
 state.segment_goal.y_m - current.y);
 const double connector_heading_limit = std::min(
 config_.goal_heading_tolerance_rad,
 maximum_curvature * std::max(distance, 1e-6));
 if (entry.node != 0 && distance <= config_.goal_position_tolerance_m &&
 std::abs(core::wrap_angle(state.segment_goal.yaw_rad - current.yaw)) <=
 connector_heading_limit + 1e-9) {
 // The exact-goal connector is part of the route and must respect the same
 // bicycle curvature and steering-rate envelope as every search primitive.
 // A near-goal lattice state is not sufficient by itself: an arbitrary
 // straight snap can introduce an impossible final steering reversal.
 if (segment_is_free(current.x, current.y,
 state.segment_goal.x_m, state.segment_goal.y_m) &&
 feasible_terminal_connector(
 current.x, current.y, current.yaw, current.curvature,
 current.gear, state.segment_goal, config_)) {
 goal_node = entry.node;
 break;
 }
 }

 for (int gear_slot = 0; gear_slot < state.dirs; ++gear_slot) {
 const int gear = gears[static_cast<std::size_t>(gear_slot)];
 for (double factor : steer_factors) {
 const double steer = factor * config_.max_steer_rad;
 const double curvature = std::tan(steer) / config_.wheel_base_m;
 if (std::abs(curvature - current.curvature) >
 config_.max_curvature_delta_per_m * config_.primitive_length_m + 1e-9) {
 continue;
 }
 const int samples = std::max(3, static_cast<int>(std::ceil(
 config_.primitive_length_m /
 std::max(0.05, 0.45 * state.resolution_m))));
 double x = current.x, y = current.y, yaw = current.yaw;
 double previous_x = x, previous_y = y;
 double accumulated_clearance_cost = 0.0;
 double accumulated_unknown = 0.0;
 std::size_t corridor_index = current.corridor_index;
 double distance_to_guide = 0.0;
 bool feasible = true;
 for (int sample = 0; sample < samples; ++sample) {
 const double ds = gear * config_.primitive_length_m / samples;
 const double mid = yaw + 0.5 * ds * curvature;
 x += ds * std::cos(mid);
 y += ds * std::sin(mid);
 yaw = core::wrap_angle(yaw + ds * curvature);
 if (blocked(x, y)) {
 feasible = false;
 break;
 }
 accumulated_clearance_cost += planning_map.cost_at(x, y);
 if (original_unknown(x, y)) accumulated_unknown += 1.0;
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
 // The published route is a polyline between primitive endpoints. A
 // collision-free bicycle arc is insufficient when its chord clips an
 // inflated corner, because the local planner will consume that chord.
 // Reject the state here so search can choose a route that is safe under
 // the same representation delivered downstream.
 if (feasible && !segment_is_free(current.x, current.y, x, y)) {
 feasible = false;
 }
 if (feasible && !corridor_match(
 x, y, corridor_index, corridor_index, distance_to_guide)) {
 feasible = false;
 }
 if (!feasible) continue;

 const std::uint64_t next_key = key(x, y, yaw, gear, curvature);
 if (next_key == std::numeric_limits<std::uint64_t>::max()) continue;
 double edge = config_.primitive_length_m;
 if (gear < 0) edge *= config_.reverse_cost;
 if (gear != current.gear) edge += config_.gear_change_cost;
 edge += config_.primitive_length_m *
 (config_.clearance_cost_per_m * accumulated_clearance_cost / samples +
 config_.unknown_cost_per_m * accumulated_unknown / samples);
 // The 2-D guide already prices ROBOT-observed traction and roughness over
 // the complete map. Reapplying a point-sampled slip multiplier inside the
 // fine lattice changes expansion order without changing the corridor; on a
 // long route it can exhaust the bounded Hybrid-A* search even though the
 // guide remains feasible. Fine refinement owns kinematics, swept collision,
 // steering effort and gear changes. Terrain preference stays in the guide,
 // once, where its spatial integral is meaningful.
 const double normalized_curvature =
 std::abs(curvature) / maximum_curvature;
 const double normalized_curvature_change =
 std::abs(curvature - current.curvature) / maximum_curvature;
 edge += config_.curvature_cost_per_m * config_.primitive_length_m *
 normalized_curvature;
 edge += config_.curvature_change_cost_m * normalized_curvature_change;
 const double next_g = current.g + edge;
 const auto known = state.best.find(next_key);
 if (known != state.best.end() && next_g + 1e-9 >= known->second) {
 continue;
 }
 if (known == state.best.end() && state.best.size() >= config_.max_states) {
 continue;
 }
 state.best[next_key] = next_g;
 state.nodes.push_back({x, y, yaw, next_g, entry.node, gear,
 curvature, corridor_index});
 const int next_node = static_cast<int>(state.nodes.size() - 1);
 const double guide_to_segment_m = corridor_index <=
 state.segment_goal_index
 ? state.guide_remaining_m[corridor_index] -
 state.guide_remaining_m[state.segment_goal_index]
 : std::hypot(state.segment_goal.x_m - x,
 state.segment_goal.y_m - y);
 const double heuristic = config_.guide_heuristic_weight *
 (std::max(0.0, guide_to_segment_m) + distance_to_guide) +
 0.08 * config_.wheel_base_m *
 std::abs(core::wrap_angle(state.segment_goal.yaw_rad - yaw));
 state.open.push({next_g + heuristic, next_g, next_node});
 }
 }
 }

 if (goal_node < 0 && !state.open.empty() &&
 (config_.max_expansions < 0 ||
 state.segment_expanded < config_.max_expansions) &&
 (config_.max_segment_compute_time_s < 0.0 ||
 segment_compute_time_s() < config_.max_segment_compute_time_s)) {
 state.segment_compute_time_s = segment_compute_time_s();
 return {GlobalPlanStatus::kPending, {}};
 }
 if (goal_node < 0) {
 if (state.open.empty()) {
 return fail("Hybrid-A* segment exhausted its reachable sparse states");
 }
 if (config_.max_expansions >= 0 &&
 state.segment_expanded >= config_.max_expansions) {
 return fail("Hybrid-A* segment reached its diagnostic expansion limit");
 }
 return fail("Hybrid-A* segment reached its diagnostic compute limit");
 }

 std::vector<SearchState::Node> path;
 for (int index = goal_node; index >= 0;
 index = state.nodes[static_cast<std::size_t>(index)].parent) {
 path.push_back(state.nodes[static_cast<std::size_t>(index)]);
 }
 std::reverse(path.begin(), path.end());
 if (path.size() < 2) {
 return fail("Hybrid-A* segment produced no motion primitive");
 }
 for (std::size_t index = 1; index < path.size(); ++index) {
 const auto& node = path[index];
 state.assembled.points.push_back(
 {node.x, node.y, node.yaw, node.gear});
 }
 const RoutePoint last = state.assembled.points.back();
 const double connector_distance = std::hypot(
 state.segment_goal.x_m - last.x_m,
 state.segment_goal.y_m - last.y_m);
 double terminal_curvature = path.back().curvature;
 if (connector_distance > 1e-6) {
 if (!segment_is_free(last.x_m, last.y_m,
 state.segment_goal.x_m, state.segment_goal.y_m)) {
 return fail("Hybrid-A* terminal connector intersects inflated geometry");
 }
 const double heading_delta = core::wrap_angle(
 state.segment_goal.yaw_rad - last.yaw_rad);
 const double signed_distance = last.gear < 0
 ? -connector_distance : connector_distance;
 terminal_curvature =
 2.0 * std::sin(0.5 * heading_delta) / signed_distance;
 state.assembled.points.push_back({
 state.segment_goal.x_m, state.segment_goal.y_m,
 core::wrap_angle(state.segment_goal.yaw_rad), last.gear});
 } else {
 state.assembled.points.back().x_m = state.segment_goal.x_m;
 state.assembled.points.back().y_m = state.segment_goal.y_m;
 state.assembled.points.back().yaw_rad =
 core::wrap_angle(state.segment_goal.yaw_rad);
 }
 // Search nodes already carry the physical bicycle yaw and a steering
 // curvature state. Replacing those headings with chord averages would
 // discard the joint state that Hybrid-A* actually validated.
 for (std::size_t index = 1; index < state.assembled.points.size(); ++index) {
 const RoutePoint& from = state.assembled.points[index - 1];
 const RoutePoint& to = state.assembled.points[index];
 if (!segment_is_free(from.x_m, from.y_m, to.x_m, to.y_m)) {
 return fail("assembled Hybrid-A* chord intersects inflated geometry");
 }
 }

 if (state.segment_goal_index + 1 < state.guide.points.size()) {
 state.segment_start_index = state.segment_goal_index;
 state.segment_start_gear = last.gear;
 state.segment_start_curvature = terminal_curvature;
 if (!begin_hybrid_segment()) {
 return fail("cannot initialize next Hybrid-A* segment");
 }
 return {GlobalPlanStatus::kPending, {}};
 }
 Route out = std::move(state.assembled);
 out.valid = out.points.size() >= 2;
 search_.reset();
 return {GlobalPlanStatus::kSucceeded, std::move(out)};
}

Route AckermannPlanner::plan(const EstimatedState& start,
 const RoutePoint& goal, const iface::MapSnapshot& map) {
 begin_plan(start, goal, map);
 for (;;) {
 GlobalPlanProgress progress = continue_plan(
 std::numeric_limits<std::size_t>::max());
 if (progress.status == GlobalPlanStatus::kPending) continue;
 return progress.status == GlobalPlanStatus::kSucceeded
 ? std::move(progress.route)
 : Route{};
 }
}

} // namespace lost_signal::robot::autonomy
