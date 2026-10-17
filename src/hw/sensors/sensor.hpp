#pragma once

#include "core/random.hpp"
#include "runtime/sensor_messages.hpp"

#include <vector>

namespace sim {

// A LiDAR-specific observation model. These parameters deliberately live at
// the sensor boundary: geometry backends return ideal ranges, then simulation
// applies bias, white noise, range quantization and missed returns. A hardware
// backend publishes its measured Scan directly and never receives simulation
// noise a second time.
struct LidarNoiseModel {
 double range_stddev_m{0.0};
 double range_bias_m{0.0};
 double range_resolution_m{0.0};
 double dropout_probability{0.0};
};

// Normalized first-return model used after ideal 3D geometry selects the
// nearest surface. Values describe a named sensor/material profile; they are
// observation parameters and never alter collision or tire friction.
struct LidarReturnModel {
 double terrain_reflectivity{0.30};
 double obstacle_reflectivity{0.68};
 double actor_reflectivity{0.52};
 double default_reflectivity{0.50};
 double incidence_floor{0.15};
 double distance_attenuation_per_m2{0.018};
};

Scan apply_lidar_noise(Scan ideal_scan, const LidarNoiseModel& model, Rng& rng);
Scan apply_lidar_noise(Scan ideal_scan, double noise_std, Rng& rng);

} // namespace sim
