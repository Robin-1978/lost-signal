#include "sensor.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace sim {

Scan apply_lidar_noise(Scan ideal_scan, const LidarNoiseModel& model, Rng& rng) {
 if (model.range_stddev_m < 0.0 || model.range_resolution_m < 0.0 ||
  model.dropout_probability < 0.0 || model.dropout_probability > 1.0 ||
 !std::isfinite(model.range_bias_m) ||
  ideal_scan.ranges.size() != ideal_scan.hit.size() ||
 (!ideal_scan.intensities.empty() && ideal_scan.intensities.size() != ideal_scan.ranges.size()) ||
 ideal_scan.ranges.size() != static_cast<std::size_t>(ideal_scan.n_rays) ||
 ideal_scan.n_rays < 0 || ideal_scan.max_range <= 0.0) {
 throw std::invalid_argument("invalid ideal lidar scan");
 }
 for (std::size_t index = 0; index < ideal_scan.ranges.size(); ++index) {
 double& range = ideal_scan.ranges[index];
 if (!ideal_scan.hit[index]) {
 range = ideal_scan.max_range;
 continue;
 }
  if (model.dropout_probability > 0.0 && rng.uniform01() < model.dropout_probability) {
   ideal_scan.hit[index] = 0;
   range = ideal_scan.max_range;
 if (!ideal_scan.intensities.empty()) {
 ideal_scan.intensities[index] = 0.0;
 }
   continue;
  }
 range += model.range_bias_m + rng.gauss(model.range_stddev_m);
  if (model.range_resolution_m > 0.0) {
   range = std::round(range / model.range_resolution_m) * model.range_resolution_m;
  }
 // A measurement driven outside the device interval is a no-return, not a
 // valid obstacle invented exactly at the range boundary. The minimum-range
 // blind zone is applied by SensorSystem because it belongs to the selected
 // device profile; this layer owns the universal maximum-range behavior.
 if (range < 0.0 || range >= ideal_scan.max_range) {
 ideal_scan.hit[index] = 0;
 range = ideal_scan.max_range;
 if (!ideal_scan.intensities.empty())
 ideal_scan.intensities[index] = 0.0;
 }
 }
 return ideal_scan;
}

Scan apply_lidar_noise(Scan ideal_scan, double noise_std, Rng& rng) {
 return apply_lidar_noise(std::move(ideal_scan), {noise_std, 0.0, 0.0, 0.0}, rng);
}

} // namespace sim
