#pragma once

#include "robot/runtime/map_provider.hpp"
#include "robot/runtime/observation.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace sim::autonomy {

enum class MappingState {
 Uninitialized,
 Mapping,
 Degraded,
 Completed,
};

// Provider-neutral output for online SLAM backends. The pose is an internal
// local-frame estimate even when no external localization exists; published
// maps are immutable revisions and never mutate a planner's view.
struct MapBuilderUpdate {
 MappingState state{MappingState::Uninitialized};
 PoseObservation pose;
 std::shared_ptr<const runtime::MapSnapshot> published_map;
 std::uint64_t map_revision{0};
 std::string diagnostic;
};

class IMapBuilder {
 public:
 virtual ~IMapBuilder() = default;

 virtual void reset() = 0;
 virtual MapBuilderUpdate update(
 const ObservationFrame& observations, const EstimatedState& prior,
 runtime::TimePoint sample_time) = 0;
};

} // namespace sim::autonomy
