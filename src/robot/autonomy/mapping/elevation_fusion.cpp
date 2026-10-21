#include "robot/autonomy/mapping/elevation_fusion.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace sim::autonomy {
namespace {

bool valid_config(const ElevationFusionConfig& config) {
 return std::isfinite(config.return_sigma_m) && config.return_sigma_m > 0.0 &&
 std::isfinite(config.pose_sigma_m) && config.pose_sigma_m >= 0.0 &&
 std::isfinite(config.maximum_sigma_m) && config.maximum_sigma_m > 0.0 &&
 config.maximum_observations >= 2;
}

} // namespace

ElevationFusion::ElevationFusion(ElevationFusionConfig config)
 : config_(std::move(config)) {}

void ElevationFusion::reset(std::size_t cells) { cells_.assign(cells, Cell{}); }

ElevationEstimate ElevationFusion::estimate(std::size_t cell) const {
 ElevationEstimate output;
 if (cell >= cells_.size())
 return output;
 const auto& stored = cells_[cell];
 if (stored.observations == 0)
 return output;
 output.valid = true;
 output.height_m = stored.mean_m;
 output.observations = stored.observations;
 const double precision = stored.precision;
 if (!(precision > 0.0) || !std::isfinite(precision)) {
 output.valid = false;
 return output;
 }
 // Variance of the fused mean plus the observed disagreement between views.
 const double disagreement = stored.observations > 1
 ? std::max(0.0, stored.weighted_square) /
 (precision * static_cast<double>(stored.observations - 1))
 : 0.0;
 output.sigma_m = std::sqrt(1.0 / precision + disagreement);
 if (!std::isfinite(output.sigma_m) || output.sigma_m > config_.maximum_sigma_m) {
 output.valid = false;
 }
 return output;
}

ElevationEstimate ElevationFusion::fuse(std::size_t cell, double height_m,
 double sigma_m) {
 if (!valid_config(config_) || cell >= cells_.size() || !std::isfinite(height_m) ||
 !std::isfinite(sigma_m) || sigma_m < 0.0) {
 return {};
 }
 const double observation_variance = config_.return_sigma_m * config_.return_sigma_m +
 config_.pose_sigma_m * config_.pose_sigma_m + sigma_m * sigma_m;
 if (!(observation_variance > 0.0) || !std::isfinite(observation_variance))
 return {};
 const double weight = 1.0 / observation_variance;
 auto& stored = cells_[cell];
 const double decay = static_cast<double>(config_.maximum_observations - 1) /
 static_cast<double>(config_.maximum_observations);
 if (stored.observations == 0) {
 stored.mean_m = height_m;
 stored.precision = weight;
 stored.weighted_square = 0.0;
 } else {
 stored.precision *= decay;
 stored.weighted_square *= decay;
 stored.precision += weight;
 const double previous_mean = stored.mean_m;
 stored.mean_m = (previous_mean * (stored.precision - weight) +
 height_m * weight) / stored.precision;
 const double residual = height_m - stored.mean_m;
 stored.weighted_square += weight * residual * residual;
 }
 if (stored.observations < std::numeric_limits<std::uint16_t>::max())
 ++stored.observations;
 // The observation is fused even when the resulting cell is too uncertain to
 // publish; the caller sees valid == false rather than losing the evidence.
 return estimate(cell);
}

} // namespace sim::autonomy
