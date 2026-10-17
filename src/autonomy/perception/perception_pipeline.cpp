#include "autonomy/perception/perception_pipeline.hpp"

#include <stdexcept>

namespace sim {

PerceptionPipeline::PerceptionPipeline(const GridMap& static_map, int width,
 int height, double resolution_m, std::uint32_t seed,
 ObservedObstacleLayerConfig obstacle_config)
 : lidar_(static_map, seed),
 obstacles_(width, height, resolution_m, obstacle_config) {}

void PerceptionPipeline::reset(std::uint32_t seed) {
 lidar_.reset(seed);
 obstacles_.reset();
}

LidarPerceptionFrame PerceptionPipeline::process(const Scan& scan,
 const Scan& static_reference, const LidarProjectionState& state,
 const LidarProjectionConfig& config, double track_max_age_s) {
 auto frame = lidar_.process(scan, static_reference, state, config,
 track_max_age_s);
 if (!obstacles_.observe(scan.header.stamp_s, frame.obstacle_rays)) {
 throw std::invalid_argument(
 "perception pipeline rejected an out-of-order LiDAR observation");
 }
 return frame;
}

GridMap PerceptionPipeline::compose_local_map(const GridMap& static_costmap,
 double now_s, double inflation_radius_m) const {
 return obstacles_.compose(static_costmap, now_s, inflation_radius_m);
}

} // namespace sim
