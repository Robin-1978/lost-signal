#pragma once

#include "sim/game/mission/content.hpp"
#include "robot/runtime/map_provider.hpp"

#include <cstdint>
#include <string_view>

namespace mission {

// Scenario/content adapter. Only robot-relevant geometry and semantics cross
// this boundary; actors, visuals, scripts, scoring, and physics truth do not.
sim::runtime::MapSnapshot make_deployable_map_snapshot(
 const MapDefinition& map, std::string_view level_id,
 sim::runtime::TimePoint sample_time = {}, std::uint64_t revision = 1);

} // namespace mission
