#pragma once

#include "robot/autonomy/mapping/mapping_session.hpp"
#include "robot/autonomy/mapping/elevation_fusion.hpp"
#include "robot/autonomy/mapping/occupancy_log_odds.hpp"
#include "robot/runtime/map_provider.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace sim::autonomy {

struct KeyframeMapRebuildConfig {
 std::uint32_t width{240};
 std::uint32_t height{240};
 double resolution_m{0.05};
 double maximum_range_m{12.0};
 double sensor_x_m{0.0};
 double sensor_y_m{0.0};
 double sensor_yaw_rad{0.0};
 double sensor_height_m{0.30};
 double ground_rejection_band_m{0.06};
 bool publish_elevation_layer{true};
 // Multi-view height fusion policy. A cell is published only while its fused
 // uncertainty stays inside these limits.
 ElevationFusionConfig elevation{};
 // Occupancy evidence policy. Repeated free observations can clear a
 // transient return instead of leaving a permanent ghost obstacle.
 OccupancyLogOddsConfig occupancy{};
 std::string map_id{"online-slam"};
};

struct KeyframeMapRebuildResult {
 bool success{false};
 std::string diagnostic;
 std::shared_ptr<const runtime::MapSnapshot> snapshot;
};

// Reconstructs a complete immutable map from optimized keyframes. This is a
// separate stage from the pose-graph solver: the solver changes poses, while
// this builder owns raster semantics. It never edits the previously published
// snapshot in place.
class KeyframeMapRebuilder final {
 public:
 explicit KeyframeMapRebuilder(KeyframeMapRebuildConfig config = {});

 KeyframeMapRebuildResult rebuild(
 const std::vector<MappingSessionKeyframe>& keyframes,
 std::uint64_t revision, double publication_time_s) const;

 private:
 bool valid_config() const;
 KeyframeMapRebuildConfig config_;
};

} // namespace sim::autonomy
