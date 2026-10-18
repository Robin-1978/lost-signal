#include "field_operator_system.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace mission {
namespace {

double length(sim::Vec2 value) {
 return std::hypot(value.x, value.y);
}

bool finite(sim::Vec2 value) {
 return std::isfinite(value.x) && std::isfinite(value.y);
}

} // namespace

const char* field_intent_kind_id(FieldIntentKind kind) {
 switch (kind) {
 case FieldIntentKind::Follow:
 return "follow";
 case FieldIntentKind::GoTo:
 return "go_to";
 case FieldIntentKind::Search:
 return "search";
 case FieldIntentKind::Wait:
 return "wait";
 case FieldIntentKind::Return:
 return "return";
 }
 return "wait";
}

const char* field_intent_acknowledgement_id(
 FieldIntentAcknowledgement acknowledgement) {
 switch (acknowledgement) {
 case FieldIntentAcknowledgement::Idle:
 return "idle";
 case FieldIntentAcknowledgement::Pending:
 return "pending";
 case FieldIntentAcknowledgement::Accepted:
 return "accepted";
 case FieldIntentAcknowledgement::Completed:
 return "completed";
 case FieldIntentAcknowledgement::Rejected:
 return "rejected";
 }
 return "idle";
}

void FieldOperatorSystem::reset(const MapDefinition& map, sim::Vec2 home_position,
 const std::string& level_id) {
 state_ = {};
 state_.home_position = home_position;
 state_.position = home_position;
 move_command_ = {};
 intent_ = {};

 // Spawn beside the robot when space permits. Candidate order is fixed, so
 // the same content always produces the same expedition starting tableau.
 static constexpr sim::Vec2 kOffsets[] = {
 {0.0, 0.9}, {0.0, -0.9}, {0.9, 0.0}, {-0.9, 0.0},
 };
 for (const auto offset : kOffsets) {
 const sim::Vec2 candidate{home_position.x + offset.x, home_position.y + offset.y};
 if (walkable(map, candidate, level_id)) {
 state_.position = candidate;
 break;
 }
 }
}

void FieldOperatorSystem::set_move_command(sim::Vec2 direction) {
 if (!finite(direction)) {
 move_command_ = {};
 return;
 }
 const double magnitude = length(direction);
 if (magnitude > 1.0) {
 direction.x /= magnitude;
 direction.y /= magnitude;
 }
 move_command_ = direction;
}

void FieldOperatorSystem::update(double dt_s, const MapDefinition& map,
 const std::string& level_id) {
 const sim::Vec2 before = state_.position;
 const sim::Vec2 requested_velocity{
 move_command_.x * walk_speed_mps, move_command_.y * walk_speed_mps};
 if (length(requested_velocity) > 1e-6)
 state_.heading_rad = std::atan2(requested_velocity.y, requested_velocity.x);

 const sim::Vec2 requested{
 state_.position.x + requested_velocity.x * std::max(0.0, dt_s),
 state_.position.y + requested_velocity.y * std::max(0.0, dt_s),
 };
 // Resolve axes independently so walking along a wall remains responsive.
 const sim::Vec2 move_x{requested.x, state_.position.y};
 if (walkable(map, move_x, level_id))
 state_.position.x = move_x.x;
 const sim::Vec2 move_y{state_.position.x, requested.y};
 if (walkable(map, move_y, level_id))
 state_.position.y = move_y.y;

 if (dt_s > 1e-9) {
 state_.velocity = {
 (state_.position.x - before.x) / dt_s,
 (state_.position.y - before.y) / dt_s,
 };
 } else {
 state_.velocity = {};
 }
 state_.moving = length(state_.velocity) > 1e-6;

 if (intent_.active && intent_.kind == FieldIntentKind::Follow)
 intent_.target = state_.position;
}

void FieldOperatorSystem::restore(const FieldExpeditionCheckpoint& checkpoint,
 const MapDefinition& map, const std::string& level_id) {
 if (!walkable(map, checkpoint.field_operator.position, level_id) ||
 !finite(checkpoint.field_operator.home_position) ||
 !std::isfinite(checkpoint.field_operator.heading_rad)) {
 throw std::invalid_argument("invalid field expedition operator checkpoint");
 }
 if (checkpoint.intent.active &&
 (checkpoint.intent.revision == 0 || !finite(checkpoint.intent.target))) {
 throw std::invalid_argument("invalid field expedition intent checkpoint");
 }
 state_ = checkpoint.field_operator;
 state_.velocity = {};
 state_.moving = false;
 move_command_ = {};
 intent_ = checkpoint.intent;
 if (intent_.active)
 intent_.acknowledgement = FieldIntentAcknowledgement::Pending;
}

bool FieldOperatorSystem::issue_intent(FieldIntentKind kind, std::optional<sim::Vec2> target) {
 if ((kind == FieldIntentKind::GoTo || kind == FieldIntentKind::Search) &&
 (!target || !finite(*target))) {
 return false;
 }
 intent_.active = true;
 intent_.kind = kind;
 ++intent_.revision;
 intent_.acknowledgement = FieldIntentAcknowledgement::Pending;
 switch (kind) {
 case FieldIntentKind::Follow:
 case FieldIntentKind::Wait:
 intent_.target = state_.position;
 break;
 case FieldIntentKind::Return:
 intent_.target = state_.home_position;
 break;
 case FieldIntentKind::GoTo:
 case FieldIntentKind::Search:
 intent_.target = *target;
 break;
 }
 return true;
}

void FieldOperatorSystem::acknowledge(std::uint64_t revision, bool accepted) {
 if (!intent_.active || revision != intent_.revision)
 return;
 intent_.acknowledged_revision = revision;
 intent_.acknowledgement = accepted ? FieldIntentAcknowledgement::Accepted
 : FieldIntentAcknowledgement::Rejected;
}

void FieldOperatorSystem::complete(std::uint64_t revision) {
 if (!intent_.active || revision != intent_.revision)
 return;
 intent_.active = false;
 intent_.acknowledged_revision = revision;
 intent_.acknowledgement = FieldIntentAcknowledgement::Completed;
}

bool FieldOperatorSystem::walkable(const MapDefinition& map, sim::Vec2 position,
 const std::string& level_id) {
 if (!finite(position) || position.x < collision_radius_m || position.y < collision_radius_m ||
 position.x > map.size_m.x - collision_radius_m ||
 position.y > map.size_m.y - collision_radius_m) {
 return false;
 }
 for (const auto& obstacle : map.obstacles) {
 if (!active_in_level(obstacle.active_in, level_id))
 continue;
 const double nearest_x = std::clamp(position.x, obstacle.min.x, obstacle.max.x);
 const double nearest_y = std::clamp(position.y, obstacle.min.y, obstacle.max.y);
 if (std::hypot(position.x - nearest_x, position.y - nearest_y) < collision_radius_m)
 return false;
 }
 return true;
}

} // namespace mission
