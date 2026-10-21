#include "field_dispatch_system.hpp"

#include <algorithm>
#include <cmath>

namespace mission {
namespace {

// Mission markers use the authored acceptance radius; localization uncertainty
// is handled by the ROBOT planner and estimator, never by silently widening a
// game-side success test.
constexpr double kArrivalRadiusM = 0.85;
constexpr double kStoppedSpeedMps = 0.18;
constexpr double kConfirmDwellS = 0.7;

double distance(sim::Vec2 a, sim::Vec2 b) {
 return std::hypot(a.x - b.x, a.y - b.y);
}

std::size_t portable_index(std::mt19937& rng, std::size_t size) {
 // Lemire's multiply-high rejection sampler fixes the mapping from mt19937
 // outputs to an index. std::uniform_int_distribution is allowed to consume
 // the engine differently in libc++, libstdc++, and MSVC's library.
 const auto bound = static_cast<std::uint32_t>(size);
 std::uint64_t product = static_cast<std::uint64_t>(rng()) * bound;
 std::uint32_t low = static_cast<std::uint32_t>(product);
 if (low < bound) {
 const std::uint32_t threshold = static_cast<std::uint32_t>(-bound) % bound;
 while (low < threshold) {
 product = static_cast<std::uint64_t>(rng()) * bound;
 low = static_cast<std::uint32_t>(product);
 }
 }
 return static_cast<std::size_t>(product >> 32U);
}

} // namespace

void FieldDispatchSystem::reset(std::uint64_t seed) {
 rng_.seed(static_cast<std::uint32_t>(seed ^ (seed >> 32U)));
 state_ = {};
 next_open_s_ = 2.0;
 issued_count_ = 0;
}

std::vector<FieldDispatchTransition> FieldDispatchSystem::update(
 double time_s, double dt_s, const MapDefinition& map, sim::Vec2 robot_position,
 double robot_speed_mps) {
 std::vector<FieldDispatchTransition> transitions;
 if (!state_.active) {
 if (time_s + 1e-9 >= next_open_s_) {
 open_next(time_s, map, robot_position, transitions);
 }
 return transitions;
 }

 state_.distance_m = distance(robot_position, state_.target);
 if (state_.distance_m <= kArrivalRadiusM && std::abs(robot_speed_mps) <= kStoppedSpeedMps) {
 state_.dwell_progress += dt_s / kConfirmDwellS;
 } else {
 state_.dwell_progress = 0.0;
 }
 if (state_.dwell_progress + 1e-9 < 1.0) {
 return transitions;
 }

 const std::string reached_marker = state_.marker_id;
 if (state_.kind == "delivery" && state_.stage == 0) {
 state_.stage = 1;
 state_.dwell_progress = 0.0;
 const sim::Vec2 previous_target = state_.target;
 if (choose_target(map, previous_target, reached_marker)) {
 state_.distance_m = distance(robot_position, state_.target);
 transitions.push_back({FieldDispatchTransitionKind::PickedUp, state_.kind, reached_marker});
 return transitions;
 }
 }

 transitions.push_back({FieldDispatchTransitionKind::Completed, state_.kind, reached_marker});
 ++state_.completed_count;
 state_.active = false;
 state_.dwell_progress = 0.0;
 state_.distance_m = 0.0;
 next_open_s_ = time_s + 6.0;
 return transitions;
}

bool FieldDispatchSystem::open_next(double time_s, const MapDefinition& map,
 sim::Vec2 robot_position, std::vector<FieldDispatchTransition>& transitions) {
 static constexpr const char* kKinds[] = {"survey", "delivery", "assist"};
 state_.kind = kKinds[issued_count_ % 3];
 state_.stage = 0;
 state_.stage_count = state_.kind == "delivery" ? 2 : 1;
 state_.opened_at_s = time_s;
 state_.dwell_progress = 0.0;
 if (!choose_target(map, robot_position, {})) {
 next_open_s_ = time_s + 5.0;
 return false;
 }
 state_.active = true;
 state_.distance_m = distance(robot_position, state_.target);
 ++issued_count_;
 transitions.push_back({FieldDispatchTransitionKind::Opened, state_.kind, state_.marker_id});
 return true;
}

bool FieldDispatchSystem::choose_target(const MapDefinition& map, sim::Vec2 robot_position,
 const std::string& excluded_marker) {
 std::vector<std::pair<std::string, Pose>> candidates;
 for (const auto& [id, pose] : map.markers) {
 if (id == excluded_marker)
 continue;
 if (distance(robot_position, {pose.x, pose.y}) < 2.5)
 continue;
 candidates.emplace_back(id, pose);
 }
 if (candidates.empty()) {
 for (const auto& [id, pose] : map.markers) {
 if (id != excluded_marker)
 candidates.emplace_back(id, pose);
 }
 }
 if (candidates.empty())
 return false;
 const auto& selected = candidates[portable_index(rng_, candidates.size())];
 state_.marker_id = selected.first;
 state_.target = {selected.second.x, selected.second.y};
 return true;
}

} // namespace mission
