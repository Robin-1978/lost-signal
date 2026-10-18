#pragma once

#include "sim/game/mission/content.hpp"

#include <cstdint>
#include <optional>
#include <string>

namespace mission {

enum class FieldIntentKind { Follow, GoTo, Search, Wait, Return };
enum class FieldIntentAcknowledgement { Idle, Pending, Accepted, Completed, Rejected };

const char* field_intent_kind_id(FieldIntentKind kind);
const char* field_intent_acknowledgement_id(FieldIntentAcknowledgement acknowledgement);

struct FieldOperatorState {
 sim::Vec2 position;
 sim::Vec2 velocity;
 sim::Vec2 home_position;
 double heading_rad{0.0};
 bool moving{false};
};

struct FieldIntentState {
 bool active{false};
 FieldIntentKind kind{FieldIntentKind::Wait};
 sim::Vec2 target;
 std::uint64_t revision{0};
 std::uint64_t acknowledged_revision{0};
 FieldIntentAcknowledgement acknowledgement{FieldIntentAcknowledgement::Idle};
};

struct FieldExpeditionCheckpoint {
 std::string level_id;
 FieldOperatorState field_operator;
 FieldIntentState intent;
};

// Runtime state for the human field engineer and the high-level instructions
// they give Luo Yi. This system owns no robot motion: ScenarioSession turns
// accepted intentions into normal planner goals and chassis commands.
class FieldOperatorSystem {
 public:
 static constexpr double walk_speed_mps = 2.2;
 static constexpr double collision_radius_m = 0.28;

 void reset(const MapDefinition& map, sim::Vec2 home_position, const std::string& level_id);
 void set_move_command(sim::Vec2 direction);
 void update(double dt_s, const MapDefinition& map, const std::string& level_id);
 bool issue_intent(FieldIntentKind kind, std::optional<sim::Vec2> target = std::nullopt);
 void acknowledge(std::uint64_t revision, bool accepted);
 void complete(std::uint64_t revision);
 FieldExpeditionCheckpoint checkpoint() const { return {{}, state_, intent_}; }
 void restore(const FieldExpeditionCheckpoint& checkpoint, const MapDefinition& map,
 const std::string& level_id);

 const FieldOperatorState& state() const { return state_; }
 const FieldIntentState& intent() const { return intent_; }

 private:
 static bool walkable(const MapDefinition& map, sim::Vec2 position,
 const std::string& level_id);

 FieldOperatorState state_;
 FieldIntentState intent_;
 sim::Vec2 move_command_;
};

} // namespace mission
