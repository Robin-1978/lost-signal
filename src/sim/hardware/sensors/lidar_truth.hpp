#pragma once

#include "robot/runtime/sensor_messages.hpp"

namespace sim {

// Ideal physical returns at the simulated sensor aperture. Simulation
// backends produce this value before a virtual device adds sampling effects,
// latency, noise, clipping, and dropouts.
struct LidarRaycastTruth {
 Scan combined;
 Scan static_environment;
};

struct LidarPointCloudRaycastTruth {
 LidarPointCloud combined;
 LidarPointCloud static_environment;
};

} // namespace sim
