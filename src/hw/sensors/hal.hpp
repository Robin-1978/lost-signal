#pragma once

#include "algo/dynobs.hpp"
#include "algo/grid_map.hpp"
#include "sensor.hpp"

#include <vector>

namespace sim {

// Simulation geometry backend. It returns an ideal scan; SensorSystem applies
// the configured observation model afterward. A real device adapter belongs
// at the timestamped Scan message boundary because it must not accept world
// truth or receive synthetic noise twice.
class LidarBackend {
 public:
 virtual ~LidarBackend() = default;
 virtual Scan scan(Vec2 position, double heading, int rays, double range,
                   const std::vector<DynState>& dynamic_objects) = 0;
};

class SimLidar : public LidarBackend {
 public:
 explicit SimLidar(const GridMap& map) : map_(map) {}
 Scan scan(Vec2 position, double heading, int rays, double range,
           const std::vector<DynState>& dynamic_objects) override {
  return lidar_raycast(map_, position, heading, rays, range, dynamic_objects);
 }

 private:
 const GridMap& map_;
};

} // namespace sim
