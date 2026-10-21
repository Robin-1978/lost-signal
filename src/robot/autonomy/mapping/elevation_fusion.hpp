#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace sim::autonomy {

struct ElevationFusionConfig {
 // Ranging noise of one return, before any multi-return averaging.
 double return_sigma_m{0.03};
 // Localization/pose uncertainty is correlated across one scan, so it adds
 // to the observation uncertainty instead of averaging away.
 double pose_sigma_m{0.04};
 // Sustained disagreement beyond this level means the cell is unreliable
 // (occlusion edge, moving obstacle, bad return). It is reported, not hidden.
 double maximum_sigma_m{0.50};
 // Bound on the fusion weight so old evidence is discounted rather than
 // owning the estimate forever. This is a fixed-memory forgetting factor.
 std::uint16_t maximum_observations{64};
};

struct ElevationEstimate {
 bool valid{false};
 double height_m{0.0};
 double sigma_m{0.0};
 std::uint16_t observations{0};
};

// Weighted multi-view height fusion for one raster cell. Each observation is
// combined by inverse variance, the variance is propagated explicitly, and
// the weight is capped so a cell can still follow real terrain changes. This
// follows the robot-centric elevation-map practice of publishing height
// together with its uncertainty instead of a bare last-sample value.
class ElevationFusion final {
 public:
 explicit ElevationFusion(ElevationFusionConfig config = {});

 void reset(std::size_t cells);
 std::size_t cell_count() const { return cells_.size(); }

 // Fuses one return. Returns the fused state after the update. A non-finite
 // height, a non-positive sigma, or an out-of-range cell is rejected and the
 // stored estimate is left untouched (the return value is then invalid).
 ElevationEstimate fuse(std::size_t cell, double height_m,
 double sigma_m = 0.0);
 // Restores a fused estimate when a raster is rigidly resampled.
 void assign(std::size_t cell, double height_m, double sigma_m);
 ElevationEstimate estimate(std::size_t cell) const;

 const ElevationFusionConfig& config() const { return config_; }

 private:
 struct Cell {
 double mean_m{0.0};
 // Sum of inverse observation variances, with a bounded forgetting factor.
 double precision{0.0};
 // Precision-weighted residual scatter, used to expose disagreement between
 // views instead of silently averaging an occlusion edge into one height.
 double weighted_square{0.0};
 std::uint16_t observations{0};
 };

 ElevationFusionConfig config_;
 std::vector<Cell> cells_;
};

} // namespace sim::autonomy
