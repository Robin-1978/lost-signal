#pragma once

#include "algo/dynobs.hpp"
#include "algo/grid_map.hpp"
#include "sensor.hpp"

#include <vector>

namespace sim {

// Hardware abstraction layer, phase A1 (lidar only; encoders/IMU follow).
// The decision stack consumes sensor data through these interfaces; simulation
// and real hardware provide different backends. SimLidar must reproduce
// lidar_scan bit-for-bit (covered by test); a future ROS/hardware backend
// implements the same interface against device drivers.
class LidarBackend {
 public:
 virtual ~LidarBackend() = default;
 virtual Scan scan(Vec2 position, double heading, int rays, double range, double noise_sigma, Rng& rng,
                   const std::vector<DynState>& dynamic_objects) = 0;
};

class SimLidar : public LidarBackend {
 public:
 explicit SimLidar(const GridMap& map) : map_(map) {}
 Scan scan(Vec2 position, double heading, int rays, double range, double noise_sigma, Rng& rng,
           const std::vector<DynState>& dynamic_objects) override {
  return lidar_scan(map_, position, heading, rays, range, noise_sigma, rng, dynamic_objects);
 }

 private:
 const GridMap& map_;
};

} // namespace sim
