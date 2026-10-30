#include "robot/autonomy/planning/astar_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>
#include <vector>

#include "robot/autonomy/mapping/mapper.hpp"

namespace lost_signal::robot::autonomy {
namespace {

constexpr double kDiagonalCost = 1.4142135623730951;

std::uint64_t anonymous_map_fingerprint(const iface::MapSnapshot& map) {
 // Anonymous maps are common in unit tests and local mapping seams. Pointer
 // identity is insufficient because successive snapshots may reuse the same
 // stack/vector storage. Named deployment maps use the O(1) id+revision key.
 std::uint64_t hash = 1469598103934665603ULL;
 const auto add = [&](std::uint64_t value) {
 hash ^= value;
 hash *= 1099511628211ULL;
 };
 add(map.width);
 add(map.height);
 add(static_cast<std::uint64_t>(std::llround(map.resolution_m * 1e9)));
 add(static_cast<std::uint64_t>(std::llround(map.origin_x_m * 1e6)));
 add(static_cast<std::uint64_t>(std::llround(map.origin_y_m * 1e6)));
 for (const int8_t cell : map.occupancy) {
 add(static_cast<std::uint8_t>(cell));
 }
 return hash;
}

bool valid_config(const AStarPlanner::Config& config) {
 return config.expansions_per_cycle > 0 &&
 config.preparation_cells_per_cycle > 0 &&
 config.unknown_cost_per_m >= 0.0 && config.grade_cost_per_m >= 0.0 &&
 config.max_grade >= 0.0 && config.clearance_cost_per_m >= 0.0 &&
 config.footprint_radius_m > 0.0 &&
 config.hard_clearance_margin_m >= 0.0 &&
 config.soft_inflation_radius_m >= 0.0 &&
 config.roughness_cost_per_m >= 0.0 && config.slip_cost_per_m >= 0.0 &&
 std::isfinite(config.unknown_cost_per_m) &&
 std::isfinite(config.grade_cost_per_m) &&
 std::isfinite(config.max_grade) &&
 std::isfinite(config.clearance_cost_per_m) &&
 std::isfinite(config.footprint_radius_m) &&
 std::isfinite(config.hard_clearance_margin_m) &&
 std::isfinite(config.soft_inflation_radius_m) &&
 std::isfinite(config.roughness_cost_per_m) &&
 std::isfinite(config.slip_cost_per_m);
}

} // namespace

struct AStarPlanner::SearchState {
 enum class Phase {
 kAllocateLethal,
 kCopyObstacles,
 kInflateHard,
 kAllocateLinear,
 kMarkLethalCost,
 kInflateSoft,
 kApplyBoundary,
 kValidateEndpoints,
 kAllocateScores,
 kAllocateParents,
 kAllocateClosed,
 kStartSearch,
 kSearch,
 };
 struct Entry {
 float f = 0.0f;
 float g = 0.0f;
 int index = -1;
 };
 struct EntryGreater {
 bool operator()(const Entry& a, const Entry& b) const {
 if (a.f != b.f) return a.f > b.f;
 // Equal lower bounds remain optimal; prefer greater progress so an
 // open grid does not expand a broad equal-cost wavefront.
 return a.g < b.g;
 }
 };

 const iface::MapSnapshot* map = nullptr;
 RoutePoint goal;
 int width = 0;
 int height = 0;
 int start_x = 0;
 int start_y = 0;
 double start_world_x_m = 0.0;
 double start_world_y_m = 0.0;
 double start_world_yaw_rad = 0.0;
 int goal_x = 0;
 int goal_y = 0;
 double resolution_m = 0.0;
 double minimum_cost_per_m = 1.0;
 bool uniform_traversal_prior = false;
 bool has_known_free = false;
 Phase phase = Phase::kValidateEndpoints;
 std::size_t cursor = 0;
 bool expanding_seed = false;
 std::uint64_t offset_cursor = 0;
 int hard_radius_cells = 0;
 int soft_radius_cells = 0;
 std::vector<float> g_score;
 std::vector<int> parent;
 std::vector<std::uint8_t> closed;
 std::priority_queue<Entry, std::vector<Entry>, EntryGreater> open;
};

AStarPlanner::AStarPlanner(Config config) : config_(config) {}

AStarPlanner::AStarPlanner() : config_() {}

AStarPlanner::~AStarPlanner() = default;

AStarPlanner::AStarPlanner(AStarPlanner&& other) noexcept
 : config_(std::move(other.config_)),
 fallback_(std::move(other.fallback_)),
 traversal_(other.traversal_),
 cached_costmap_(std::move(other.cached_costmap_)),
 cached_map_id_(std::move(other.cached_map_id_)),
 cached_map_revision_(other.cached_map_revision_),
 cached_anonymous_fingerprint_(other.cached_anonymous_fingerprint_),
 costmap_cached_(other.costmap_cached_),
 cached_has_known_free_(other.cached_has_known_free_) {
 other.cancel_plan();
}

AStarPlanner& AStarPlanner::operator=(AStarPlanner&& other) noexcept {
 if (this == &other) return *this;
 cancel_plan();
 config_ = std::move(other.config_);
 fallback_ = std::move(other.fallback_);
 traversal_ = other.traversal_;
 cached_costmap_ = std::move(other.cached_costmap_);
 cached_map_id_ = std::move(other.cached_map_id_);
 cached_map_revision_ = other.cached_map_revision_;
 cached_anonymous_fingerprint_ = other.cached_anonymous_fingerprint_;
 costmap_cached_ = other.costmap_cached_;
 cached_has_known_free_ = other.cached_has_known_free_;
 other.cancel_plan();
 return *this;
}

AStarPlanner::AStarPlanner(double unknown_cost_per_m) : config_() {
 config_.unknown_cost_per_m = unknown_cost_per_m;
}

const LocalCostmap* AStarPlanner::prepared_costmap(
 const iface::MapSnapshot& map) const {
 const bool same_identity = !map.map_id.empty()
 ? cached_map_id_ == map.map_id && cached_map_revision_ == map.revision
 : cached_anonymous_fingerprint_ == anonymous_map_fingerprint(map);
 if (!costmap_cached_ || !same_identity || !cached_costmap_.valid()) {
 return nullptr;
 }
 return &cached_costmap_;
}

void AStarPlanner::cancel_plan() {
 search_.reset();
 completed_ = GlobalPlanProgress{};
 completed_ready_ = false;
}

void AStarPlanner::begin_plan(const EstimatedState& start,
 const RoutePoint& goal, const iface::MapSnapshot& map) {
 cancel_plan();
 auto complete = [&](GlobalPlanStatus status, Route route = {},
 std::string diagnostic = {}) {
 completed_.status = status;
 completed_.route = std::move(route);
 completed_.diagnostic = std::move(diagnostic);
 completed_ready_ = true;
 };

 if (!start.valid || !std::isfinite(start.x_m) ||
 !std::isfinite(start.y_m) || !std::isfinite(goal.x_m) ||
 !std::isfinite(goal.y_m) || !std::isfinite(goal.yaw_rad) ||
 !valid_config(config_)) {
 complete(GlobalPlanStatus::kFailed, {}, "invalid A* request or config");
 return;
 }
 if (!map.has_raster()) {
 Route route = fallback_.plan(start, goal, map);
 complete(route.valid ? GlobalPlanStatus::kSucceeded
 : GlobalPlanStatus::kFailed,
 std::move(route));
 return;
 }
 if (!std::isfinite(map.origin_x_m) ||
 !std::isfinite(map.origin_y_m)) {
 complete(GlobalPlanStatus::kFailed, {}, "map origin is invalid");
 return;
 }
 if (map.width > static_cast<std::uint32_t>(
 std::numeric_limits<int>::max()) ||
 map.height > static_cast<std::uint32_t>(
 std::numeric_limits<int>::max())) {
 complete(GlobalPlanStatus::kFailed, {}, "map dimensions exceed index range");
 return;
 }
 const std::size_t cell_count =
 static_cast<std::size_t>(map.width) * map.height;
 if (cell_count > static_cast<std::size_t>(
 std::numeric_limits<int>::max())) {
 complete(GlobalPlanStatus::kFailed, {}, "map cell count exceeds index range");
 return;
 }

 auto state = std::make_unique<SearchState>();
 state->map = &map;
 state->goal = goal;
 state->width = static_cast<int>(map.width);
 state->height = static_cast<int>(map.height);
 state->resolution_m = map.resolution_m;
 state->start_world_x_m = start.x_m;
 state->start_world_y_m = start.y_m;
 state->start_world_yaw_rad = start.yaw_rad;
 const auto to_grid = [&](double wx, double wy, int& x, int& y) {
 x = static_cast<int>(std::floor(
 (wx - map.origin_x_m) / map.resolution_m));
 y = static_cast<int>(std::floor(
 (wy - map.origin_y_m) / map.resolution_m));
 };
 to_grid(start.x_m, start.y_m, state->start_x, state->start_y);
 to_grid(goal.x_m, goal.y_m, state->goal_x, state->goal_y);
 const auto inside = [&](int x, int y) {
 return x >= 0 && y >= 0 && x < state->width && y < state->height;
 };
 if (!inside(state->start_x, state->start_y) ||
 !inside(state->goal_x, state->goal_y)) {
 complete(GlobalPlanStatus::kFailed, {}, "start or goal is outside the map");
 return;
 }
 const bool same_identity = !map.map_id.empty()
 ? cached_map_id_ == map.map_id && cached_map_revision_ == map.revision
 : cached_anonymous_fingerprint_ == anonymous_map_fingerprint(map);
 const bool reuse_costmap = costmap_cached_ && same_identity;
 if (reuse_costmap) {
 state->has_known_free = cached_has_known_free_;
 state->phase = SearchState::Phase::kValidateEndpoints;
 } else {
 cached_costmap_ = LocalCostmap{};
 costmap_cached_ = false;
 state->phase = SearchState::Phase::kAllocateLethal;
 const int max_radius_cells = std::max(state->width, state->height);
 const auto radius_cells = [&](double radius_m) {
 const double requested =
 std::ceil(radius_m / state->resolution_m + 0.5);
 return requested >= max_radius_cells
 ? max_radius_cells
 : static_cast<int>(requested);
 };
 state->hard_radius_cells = radius_cells(
 config_.footprint_radius_m + config_.hard_clearance_margin_m);
 state->soft_radius_cells =
 radius_cells(config_.soft_inflation_radius_m);
 }
 search_ = std::move(state);
}

GlobalPlanProgress AStarPlanner::continue_plan(
 std::size_t expansion_budget) {
 if (completed_ready_) {
 completed_ready_ = false;
 return std::move(completed_);
 }
 if (search_ == nullptr) {
 return {GlobalPlanStatus::kFailed, {}, "no active A* request"};
 }
 if (expansion_budget == 0) return {GlobalPlanStatus::kPending, {}};

 SearchState& state = *search_;
 const iface::MapSnapshot& map = *state.map;
 const LocalCostmap& planning_map = cached_costmap_;
 const auto inside = [&](int x, int y) {
 return x >= 0 && y >= 0 && x < state.width && y < state.height;
 };
 const auto index = [&](int x, int y) { return y * state.width + x; };
 const auto cell_value = [&](int x, int y) -> int {
 if (!inside(x, y)) return 100;
 return planning_map.lethal.occupancy[index(x, y)];
 };
 const auto cell_elevation = [&](int x, int y) -> float {
 if (!map.has_elevation()) return std::numeric_limits<float>::quiet_NaN();
 return map.elevation_m[index(x, y)];
 };
 const auto heuristic = [&](int x, int y) {
 const int dx = std::abs(x - state.goal_x);
 const int dy = std::abs(y - state.goal_y);
 const int diagonal = std::min(dx, dy);
 const int straight = std::max(dx, dy) - diagonal;
 return state.minimum_cost_per_m * state.resolution_m *
 (straight + diagonal * kDiagonalCost);
 };

 const std::size_t cell_count =
 static_cast<std::size_t>(state.width) * state.height;
 const std::size_t preparation_budget =
 config_.preparation_cells_per_cycle;
 // Eight full-raster passes cover base occupancy, hard/soft costs,
 // boundary handling, and the three search arrays. Small revisions can
 // complete those passes within the same declared preparation budget.
 constexpr std::size_t kFullRasterPasses = 8;
 const bool immediate_preparation =
 cell_count <= preparation_budget / kFullRasterPasses;
 const auto occupied_boundary = [&](int x, int y,
 const std::vector<int8_t>& occupancy) {
 for (int adjacent_y = -1; adjacent_y <= 1; ++adjacent_y) {
 for (int adjacent_x = -1; adjacent_x <= 1; ++adjacent_x) {
 if (adjacent_x == 0 && adjacent_y == 0) continue;
 const int neighbor_x = x + adjacent_x;
 const int neighbor_y = y + adjacent_y;
 if (inside(neighbor_x, neighbor_y) &&
 occupancy[index(neighbor_x, neighbor_y)] < 50) {
 return true;
 }
 }
 }
 return false;
 };
 const auto apply_inflation = [&](int seed_x, int seed_y,
 int radius_cells, double radius_m, std::uint64_t& offset_cursor,
 std::size_t& work, auto&& apply) {
 const std::uint64_t span =
 2ULL * static_cast<std::uint64_t>(radius_cells) + 1ULL;
 const std::uint64_t offset_count = span * span;
 while (offset_cursor < offset_count && work < preparation_budget) {
 const std::int64_t offset_x =
 static_cast<std::int64_t>(offset_cursor % span) - radius_cells;
 const std::int64_t offset_y =
 static_cast<std::int64_t>(offset_cursor / span) - radius_cells;
 ++offset_cursor;
 ++work;
 const std::int64_t target_x = seed_x + offset_x;
 const std::int64_t target_y = seed_y + offset_y;
 if (target_x < 0 || target_y < 0 || target_x >= state.width ||
 target_y >= state.height) {
 continue;
 }
 const double nearest_x = std::max(
 0.0, std::abs(static_cast<double>(offset_x)) *
 state.resolution_m - 0.5 * state.resolution_m);
 const double nearest_y = std::max(
 0.0, std::abs(static_cast<double>(offset_y)) *
 state.resolution_m - 0.5 * state.resolution_m);
 const double distance = std::hypot(nearest_x, nearest_y);
 if (distance <= radius_m + 1e-12) {
 apply(static_cast<int>(target_x), static_cast<int>(target_y), distance);
 }
 }
 return offset_cursor == offset_count;
 };

 if (state.phase == SearchState::Phase::kAllocateLethal) {
 cached_costmap_.lethal.valid = true;
 cached_costmap_.lethal.resolution_m = state.resolution_m;
 cached_costmap_.lethal.width = map.width;
 cached_costmap_.lethal.height = map.height;
 cached_costmap_.lethal.origin_x_m = map.origin_x_m;
 cached_costmap_.lethal.origin_y_m = map.origin_y_m;
 cached_costmap_.lethal.occupancy.assign(cell_count, 0);
 state.phase = SearchState::Phase::kCopyObstacles;
 if (!immediate_preparation) {
 return {GlobalPlanStatus::kPending, {}};
 }
 }
 if (state.phase == SearchState::Phase::kCopyObstacles) {
 const std::size_t end = state.cursor +
 std::min(preparation_budget, cell_count - state.cursor);
 for (; state.cursor < end; ++state.cursor) {
 const int8_t value = map.occupancy[state.cursor];
 cached_costmap_.lethal.occupancy[state.cursor] =
 value >= 50 ? 100 : 0;
 if (value >= 0 && value < 50) state.has_known_free = true;
 }
 if (state.cursor == cell_count) {
 state.cursor = 0;
 state.phase = SearchState::Phase::kInflateHard;
 }
 if (state.phase == SearchState::Phase::kCopyObstacles ||
 !immediate_preparation) {
 return {GlobalPlanStatus::kPending, {}};
 }
 }
 if (state.phase == SearchState::Phase::kInflateHard) {
 std::size_t work = 0;
 const double radius_m =
 config_.footprint_radius_m + config_.hard_clearance_margin_m;
 while (state.cursor < cell_count && work < preparation_budget) {
 const int seed_x = static_cast<int>(state.cursor % state.width);
 const int seed_y = static_cast<int>(state.cursor / state.width);
 if (!state.expanding_seed) {
 ++work;
 if (map.occupancy[state.cursor] < 50 ||
 !occupied_boundary(seed_x, seed_y, map.occupancy)) {
 ++state.cursor;
 continue;
 }
 state.expanding_seed = true;
 state.offset_cursor = 0;
 }
 const bool complete = apply_inflation(
 seed_x, seed_y, state.hard_radius_cells, radius_m,
 state.offset_cursor, work,
 [&](int x, int y, double) {
 cached_costmap_.lethal.occupancy[index(x, y)] = 100;
 });
 if (complete) {
 state.expanding_seed = false;
 ++state.cursor;
 }
 }
 if (state.cursor == cell_count) {
 state.cursor = 0;
 state.phase = SearchState::Phase::kAllocateLinear;
 }
 if (state.phase == SearchState::Phase::kInflateHard ||
 !immediate_preparation) {
 return {GlobalPlanStatus::kPending, {}};
 }
 }
 if (state.phase == SearchState::Phase::kAllocateLinear) {
 cached_costmap_.soft_radius_m =
 std::max(0.0, config_.soft_inflation_radius_m);
 cached_costmap_.linear_cost.assign(cell_count, 0.0f);
 state.phase = SearchState::Phase::kMarkLethalCost;
 if (!immediate_preparation) {
 return {GlobalPlanStatus::kPending, {}};
 }
 }
 if (state.phase == SearchState::Phase::kMarkLethalCost) {
 const std::size_t end = state.cursor +
 std::min(preparation_budget, cell_count - state.cursor);
 for (; state.cursor < end; ++state.cursor) {
 if (cached_costmap_.lethal.occupancy[state.cursor] >= 50) {
 cached_costmap_.linear_cost[state.cursor] = 1.0f;
 }
 }
 if (state.cursor == cell_count) {
 state.cursor = 0;
 state.phase = SearchState::Phase::kInflateSoft;
 }
 if (state.phase == SearchState::Phase::kMarkLethalCost ||
 !immediate_preparation) {
 return {GlobalPlanStatus::kPending, {}};
 }
 }
 if (state.phase == SearchState::Phase::kInflateSoft) {
 const double radius_m = cached_costmap_.soft_radius_m;
 if (!(radius_m > 0.0)) {
 state.phase = SearchState::Phase::kApplyBoundary;
 } else {
 std::size_t work = 0;
 while (state.cursor < cell_count && work < preparation_budget) {
 const int seed_x = static_cast<int>(state.cursor % state.width);
 const int seed_y = static_cast<int>(state.cursor / state.width);
 if (!state.expanding_seed) {
 ++work;
 if (cached_costmap_.lethal.occupancy[state.cursor] < 50 ||
 !occupied_boundary(
 seed_x, seed_y, cached_costmap_.lethal.occupancy)) {
 ++state.cursor;
 continue;
 }
 state.expanding_seed = true;
 state.offset_cursor = 0;
 }
 const bool complete = apply_inflation(
 seed_x, seed_y, state.soft_radius_cells, radius_m,
 state.offset_cursor, work,
 [&](int x, int y, double distance) {
 const std::size_t target = index(x, y);
 if (cached_costmap_.lethal.occupancy[target] >= 50) {
 cached_costmap_.linear_cost[target] = 1.0f;
 return;
 }
 const float cost = static_cast<float>(1.0 - distance / radius_m);
 cached_costmap_.linear_cost[target] =
 std::max(cached_costmap_.linear_cost[target], cost);
 });
 if (complete) {
 state.expanding_seed = false;
 ++state.cursor;
 }
 }
 if (state.cursor == cell_count) {
 state.cursor = 0;
 state.phase = SearchState::Phase::kApplyBoundary;
 }
 if (state.phase == SearchState::Phase::kInflateSoft ||
 !immediate_preparation) {
 return {GlobalPlanStatus::kPending, {}};
 }
 }
 }
 if (state.phase == SearchState::Phase::kApplyBoundary) {
 const double hard_radius = config_.footprint_radius_m +
 config_.hard_clearance_margin_m;
 const double soft_radius = cached_costmap_.soft_radius_m;
 const double width_m = state.width * state.resolution_m;
 const double height_m = state.height * state.resolution_m;
 const std::size_t end = state.cursor +
 std::min(preparation_budget, cell_count - state.cursor);
 for (; state.cursor < end; ++state.cursor) {
 const std::size_t x = state.cursor % state.width;
 const std::size_t y = state.cursor / state.width;
 const double center_x = (x + 0.5) * state.resolution_m;
 const double center_y = (y + 0.5) * state.resolution_m;
 const double boundary = std::min({
 center_x, center_y, width_m - center_x, height_m - center_y});
 if (boundary <= hard_radius + 1e-12) {
 cached_costmap_.lethal.occupancy[state.cursor] = 100;
 cached_costmap_.linear_cost[state.cursor] = 1.0f;
 } else if (soft_radius > 0.0 &&
 boundary < hard_radius + soft_radius) {
 const float cost = static_cast<float>(
 1.0 - (boundary - hard_radius) / soft_radius);
 cached_costmap_.linear_cost[state.cursor] =
 std::max(cached_costmap_.linear_cost[state.cursor], cost);
 }
 }
 if (state.cursor == cell_count) {
 cached_map_id_ = map.map_id;
 cached_map_revision_ = map.revision;
 cached_anonymous_fingerprint_ = map.map_id.empty()
 ? anonymous_map_fingerprint(map)
 : 0;
 cached_has_known_free_ = state.has_known_free;
 costmap_cached_ = true;
 state.phase = SearchState::Phase::kValidateEndpoints;
 }
 if (state.phase == SearchState::Phase::kApplyBoundary ||
 !immediate_preparation) {
 return {GlobalPlanStatus::kPending, {}};
 }
 }
 if (state.phase == SearchState::Phase::kValidateEndpoints) {
 if (!cached_costmap_.valid()) {
 search_.reset();
 return {GlobalPlanStatus::kFailed, {},
 "inflated global costmap is unavailable"};
 }
 if (cached_costmap_.lethal.occupancy[
 index(state.start_x, state.start_y)] >= 50) {
 search_.reset();
 return {GlobalPlanStatus::kFailed, {},
 "start is inside the inflated lethal map"};
 }
 if (cached_costmap_.lethal.occupancy[
 index(state.goal_x, state.goal_y)] >= 50) {
 search_.reset();
 return {GlobalPlanStatus::kFailed, {},
 "goal is inside the inflated lethal map"};
 }
 state.phase = SearchState::Phase::kAllocateScores;
 }
 if (state.phase == SearchState::Phase::kAllocateScores) {
 state.g_score.assign(
 cell_count, std::numeric_limits<float>::infinity());
 state.phase = SearchState::Phase::kAllocateParents;
 if (!immediate_preparation) {
 return {GlobalPlanStatus::kPending, {}};
 }
 }
 if (state.phase == SearchState::Phase::kAllocateParents) {
 state.parent.assign(cell_count, -1);
 state.phase = SearchState::Phase::kAllocateClosed;
 if (!immediate_preparation) {
 return {GlobalPlanStatus::kPending, {}};
 }
 }
 if (state.phase == SearchState::Phase::kAllocateClosed) {
 state.closed.assign(cell_count, 0);
 state.phase = SearchState::Phase::kStartSearch;
 if (!immediate_preparation) {
 return {GlobalPlanStatus::kPending, {}};
 }
 }
 if (state.phase == SearchState::Phase::kStartSearch) {
 const double minimum_roughness_m = traversal_ != nullptr
 ? traversal_->minimum_roughness_m()
 : 0.0;
 state.minimum_cost_per_m =
 1.0 + (state.has_known_free ? 0.0 : config_.unknown_cost_per_m) +
 config_.roughness_cost_per_m * minimum_roughness_m;
 state.uniform_traversal_prior =
 traversal_ != nullptr && !traversal_->has_spatial_beliefs();
 const int start_index = index(state.start_x, state.start_y);
 state.g_score[start_index] = 0.0f;
 state.open.push({
 static_cast<float>(heuristic(state.start_x, state.start_y)),
 0.0f, start_index});
 state.phase = SearchState::Phase::kSearch;
 }

 constexpr int kDx[8] = {1, 1, 0, -1, -1, -1, 0, 1};
 constexpr int kDy[8] = {0, 1, 1, 1, 0, -1, -1, -1};
 int found = -1;
 std::size_t work = 0;
 while (!state.open.empty() && work < expansion_budget) {
 const SearchState::Entry entry = state.open.top();
 state.open.pop();
 ++work;
 const int current = entry.index;
 if (entry.g > state.g_score[current] || state.closed[current]) continue;
 state.closed[current] = 1;
 const int current_x = current % state.width;
 const int current_y = current / state.width;
 if (current_x == state.goal_x && current_y == state.goal_y) {
 found = current;
 break;
 }

 for (int direction = 0; direction < 8; ++direction) {
 const int next_x = current_x + kDx[direction];
 const int next_y = current_y + kDy[direction];
 if (!inside(next_x, next_y) || cell_value(next_x, next_y) >= 50) {
 continue;
 }
 if (direction % 2 == 1 &&
 (cell_value(current_x + kDx[direction], current_y) >= 50 ||
 cell_value(current_x, current_y + kDy[direction]) >= 50)) {
 continue;
 }

 const double step =
 (direction % 2 == 0 ? 1.0 : kDiagonalCost) * state.resolution_m;
 double cost = step;
 if (map.occupancy[index(next_x, next_y)] < 0) {
 cost += config_.unknown_cost_per_m * step;
 }
 const double world_x =
 map.origin_x_m + (next_x + 0.5) * state.resolution_m;
 const double world_y =
 map.origin_y_m + (next_y + 0.5) * state.resolution_m;
 cost += config_.clearance_cost_per_m * step *
 planning_map.cost_at(world_x, world_y);

 const float height0 = cell_elevation(current_x, current_y);
 const float height1 = cell_elevation(next_x, next_y);
 if (!std::isnan(height0) && !std::isnan(height1)) {
 const double grade = std::abs(height1 - height0) / step;
 if (grade > config_.max_grade) continue;
 cost += config_.grade_cost_per_m * std::abs(height1 - height0);
 }
 if (traversal_ != nullptr) {
 if (state.uniform_traversal_prior) {
 cost += config_.roughness_cost_per_m *
 traversal_->default_roughness_m() * step;
 } else {
 cost += config_.roughness_cost_per_m *
 traversal_->roughness_at(world_x, world_y) * step;
 cost += config_.slip_cost_per_m *
 traversal_->slip_at(world_x, world_y) * step;
 }
 }

 const double next_g = static_cast<double>(state.g_score[current]) + cost;
 const int next = index(next_x, next_y);
 if (next_g < state.g_score[next]) {
 state.g_score[next] = static_cast<float>(next_g);
 state.parent[next] = current;
 state.open.push({
 static_cast<float>(next_g + heuristic(next_x, next_y)),
 static_cast<float>(next_g), next});
 }
 }
 }

 if (found < 0 && !state.open.empty()) {
 return {GlobalPlanStatus::kPending, {}};
 }
 if (found < 0) {
 search_.reset();
 return {GlobalPlanStatus::kFailed, {},
 "A* exhausted all reachable free cells"};
 }

 Route route;
 const int start_index = index(state.start_x, state.start_y);
 for (int current = found; current >= 0;) {
 const int x = current % state.width;
 const int y = current / state.width;
 route.points.push_back({
 map.origin_x_m + (x + 0.5) * state.resolution_m,
 map.origin_y_m + (y + 0.5) * state.resolution_m,
 state.goal.yaw_rad});
 if (current == start_index) break;
 current = state.parent[current];
 }
 std::reverse(route.points.begin(), route.points.end());
 // A raster can place the estimated start and semantic goal in one cell.
 // Preserve a two-pose route in that case so the local planner can still
 // settle the requested terminal heading. Returning an invalid one-point
 // route would make the route pipeline latch `global-plan-failed` even though
 // the position is reachable and only orientation remains.
 if (route.points.size() == 1) {
   route.points.insert(route.points.begin(),
                       {state.start_world_x_m, state.start_world_y_m,
                        state.start_world_yaw_rad});
 }
 const RoutePoint cell_goal = route.points.back();
 if (std::hypot(cell_goal.x_m - state.goal.x_m,
 cell_goal.y_m - state.goal.y_m) > 1e-9) {
 route.points.push_back(state.goal);
 } else {
 route.points.back().yaw_rad = state.goal.yaw_rad;
 }
 for (std::size_t i = 0; i + 1 < route.points.size(); ++i) {
 route.points[i].yaw_rad = std::atan2(
 route.points[i + 1].y_m - route.points[i].y_m,
 route.points[i + 1].x_m - route.points[i].x_m);
 }
 route.valid = route.points.size() >= 2;
 search_.reset();
 return {route.valid ? GlobalPlanStatus::kSucceeded
 : GlobalPlanStatus::kFailed,
 std::move(route)};
}

Route AStarPlanner::plan(const EstimatedState& start, const RoutePoint& goal,
 const iface::MapSnapshot& map) {
 begin_plan(start, goal, map);
 for (;;) {
 GlobalPlanProgress progress =
 continue_plan(std::numeric_limits<std::size_t>::max());
 if (progress.status == GlobalPlanStatus::kPending) continue;
 return progress.status == GlobalPlanStatus::kSucceeded
 ? std::move(progress.route)
 : Route{};
 }
}

} // namespace lost_signal::robot::autonomy
