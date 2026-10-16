#pragma once

#include "game/content.hpp"

#include <cstdint>
#include <random>
#include <string>
#include <vector>

namespace game {

enum class FieldDispatchTransitionKind { Opened, PickedUp, Completed };

struct FieldDispatchTransition {
 FieldDispatchTransitionKind kind{FieldDispatchTransitionKind::Opened};
 std::string dispatch_kind;
 std::string marker_id;
};

struct FieldDispatchState {
 bool active{false};
 std::string kind;
 std::string marker_id;
 sim::Vec2 target;
 int stage{0};
 int stage_count{1};
 int completed_count{0};
 double opened_at_s{0.0};
 double dwell_progress{0.0};
 double distance_m{0.0};
};

// Deterministic, endless-world work generator. It creates small field jobs
// from authored map markers without mutating mission objectives or owning any
// rendering. A frontend can present the state; a future contract generator
// can reuse the same transitions for rewards and persistence.
class FieldDispatchSystem {
 public:
 void reset(std::uint64_t seed);
 std::vector<FieldDispatchTransition> update(double time_s, double dt_s,
 const MapDefinition& map, sim::Vec2 robot_position, double robot_speed_mps);
 const FieldDispatchState& state() const { return state_; }

 private:
 bool open_next(double time_s, const MapDefinition& map, sim::Vec2 robot_position,
 std::vector<FieldDispatchTransition>& transitions);
 bool choose_target(const MapDefinition& map, sim::Vec2 robot_position,
 const std::string& excluded_marker);

 std::mt19937 rng_{0};
 FieldDispatchState state_;
 double next_open_s_{2.0};
 int issued_count_{0};
};

} // namespace game
