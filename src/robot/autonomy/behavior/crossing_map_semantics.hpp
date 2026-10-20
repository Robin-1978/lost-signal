#pragma once

#include "robot/autonomy/behavior/crossing_behavior.hpp"
#include "robot/runtime/map_provider.hpp"

#include <string>
#include <vector>

namespace sim {

// Converts the deployable robot-map semantic layer into the typed contract
// consumed by crossing behavior. This is a ROBOT-side decoder: simulation,
// SLAM, survey and HD-map providers all publish the same MapSnapshot and may
// not call behavior with private content types.
struct CrossingMapSemantics {
 bool valid{true};
 std::string diagnostic;
 std::vector<CrossingZoneSpec> zones;
};

CrossingMapSemantics crossing_semantics_from_map(
 const runtime::MapSnapshot& map);

} // namespace sim
