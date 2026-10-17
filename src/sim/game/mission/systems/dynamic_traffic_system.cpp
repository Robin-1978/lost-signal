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
 active.push_back({
 {definition.id, definition.min, definition.max},
 definition.authority == "control"
 ? sim::CrossingAuthority::Control
 : sim::CrossingAuthority::Observe,
 });
 }
 return active;
}

} // namespace mission
