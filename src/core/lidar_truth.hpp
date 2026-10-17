#pragma once

#include "runtime/sensor_messages.hpp"

namespace sim {

// Ideal physical returns at the sensor aperture. Simulation backends produce
// this value; the selected LiDAR hardware model converts it into measurements.
struct LidarRaycastTruth {
 Scan combined;
 Scan static_environment;
};

} // namespace sim
