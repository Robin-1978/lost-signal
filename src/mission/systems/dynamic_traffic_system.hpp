#pragma once

#include "autonomy/behavior/crossing_behavior.hpp"
#include "mission/content.hpp"

#include <string>
#include <vector>

namespace mission {

// Stateless world-content adapter. It only filters authored regions for the
// current level and translates their authority enum. Stop poses, prediction,
// state transitions, and motion decisions all belong to autonomy.
class DynamicTrafficSystem {
 public:
 std::vector<sim::CrossingZoneSpec> active_zones(
 const std::vector<ConflictZoneDefinition>& definitions,
 const std::string& level_id) const;
};

} // namespace mission
