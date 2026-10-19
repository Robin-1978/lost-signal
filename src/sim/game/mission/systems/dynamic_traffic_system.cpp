#include "sim/game/mission/systems/dynamic_traffic_system.hpp"

namespace mission {

std::vector<sim::CrossingZoneSpec> DynamicTrafficSystem::active_zones(
 const std::vector<ConflictZoneDefinition>& definitions,
 const std::string& level_id) const {
 std::vector<sim::CrossingZoneSpec> active;
 active.reserve(definitions.size());
 for (const auto& definition : definitions) {
 if (!active_in_level(definition.active_in, level_id))
 continue;
 std::vector<sim::CrossingObservationApproach> observation_approaches;
 observation_approaches.reserve(definition.observation_approaches.size());
 for (const auto& approach : definition.observation_approaches) {
 observation_approaches.push_back({
 approach.centerline,
 approach.half_width_m,
 approach.maximum_speed_mps,
 });
 }
 active.push_back({
 {definition.id, definition.min, definition.max,
 definition.participant_radius_m, std::move(observation_approaches)},
 definition.authority == "control"
 ? sim::CrossingAuthority::Control
 : sim::CrossingAuthority::Observe,
 });
 }
 return active;
}

} // namespace mission
