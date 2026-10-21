#pragma once

#include "robot/autonomy/mapping/keyframe_map_rebuilder.hpp"
#include "robot/runtime/map_provider.hpp"

#include <cstdint>
#include <string>

namespace sim::autonomy {

struct SlamMapPublicationResult {
 bool success{false};
 bool graph_optimized{false};
 runtime::MapUpdateResult map_result{runtime::MapUpdateResult::Invalid};
 std::uint64_t revision{0};
 std::string diagnostic;
};

struct SlamMapPublicationConfig {
 PoseGraphOptimizationConfig optimization;
 // A single accepted association must not teleport the map. These bounds are
 // deliberately generous relative to a survey loop, so they reject a wrong
 // association or a diverged solve instead of tuning normal corrections.
 double maximum_translation_correction_m{25.0};
 double maximum_heading_correction_rad{1.5};
};

// Coordinates the backend-to-map transaction. A pose graph is optimized and
// the complete candidate map is rebuilt before MapManager::replace is called.
// Therefore readers keep their old immutable revision if optimization,
// reconstruction, validation, or publication fails. The class owns no
// planner state and exposes no solver types outside the mapping package.
class SlamMapPublication final {
 public:
 explicit SlamMapPublication(SlamMapPublicationConfig config = {});

 SlamMapPublicationResult optimize_rebuild_publish(
 MappingSession& session, const KeyframeMapRebuilder& rebuilder,
 runtime::MapManager& map_manager, double publication_time_s) const;

 private:
 SlamMapPublicationConfig config_;
};

} // namespace sim::autonomy
