#pragma once

#include "robot/autonomy/mapping/grid_map.hpp"
#include "robot/core/dynamic_object_state.hpp"
#include "sim/hardware/sensors/lidar_truth.hpp"

#include <vector>

namespace sim {

Scan planar_lidar_raycast(const GridMap& map, Vec2 position, double heading,
 int ray_count, double max_range_m,
 const std::vector<DynamicObjectState>& dynamic_objects = {});

LidarRaycastTruth planar_lidar_truth(const GridMap& map, Vec2 position,
 double heading, int ray_count, double max_range_m,
 const std::vector<DynamicObjectState>& dynamic_objects = {});

} // namespace sim
