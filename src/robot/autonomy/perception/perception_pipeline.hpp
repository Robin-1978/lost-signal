#pragma once

#include "robot/autonomy/mapping/observed_obstacle_layer.hpp"
#include "robot/autonomy/perception/lidar_perception_system.hpp"

#include <cstdint>
#include <limits>
#include <vector>

namespace sim {

// Cohesive autonomy front end for one planar LiDAR stream. Sensor simulation
// and transport end at Scan; this pipeline owns projection, foreground
// extraction, transient occupancy, tracking and prediction state. It can be
// fed identically by simulation, replay, or a future hardware provider.
class PerceptionPipeline {
 public:
 PerceptionPipeline(const GridMap& static_map, int width, int height,
 double resolution_m, std::uint32_t seed,
 ObservedObstacleLayerConfig obstacle_config = {});

 void reset(std::uint32_t seed);
 LidarPerceptionFrame process(const Scan& scan, const Scan& static_reference,
 const LidarProjectionState& state, const LidarProjectionConfig& config,
 double track_max_age_s);

 GridMap compose_local_map(const GridMap& static_costmap, double now_s,
 double inflation_radius_m) const;

 const DynamicObstacleMap& dynamic_obstacles() const {
 return lidar_.dynamic_obstacles();
 }
 DynamicObstacleMap& dynamic_obstacles() { return lidar_.dynamic_obstacles(); }
 const ObservedObstacleLayer& obstacle_layer() const { return obstacles_; }
 const std::vector<ObservedRay2D>& latest_visibility_rays() const {
 return latest_visibility_rays_;
 }
 double latest_visibility_stamp_s() const { return latest_visibility_stamp_s_; }

 private:
 LidarPerceptionSystem lidar_;
 ObservedObstacleLayer obstacles_;
 ObservedObstacleLayerConfig obstacle_config_;
 std::vector<ObservedRay2D> latest_visibility_rays_;
 double latest_visibility_stamp_s_{-
 std::numeric_limits<double>::infinity()};
};

} // namespace sim
