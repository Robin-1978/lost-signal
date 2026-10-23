#pragma once

// ROBOT-owned live elevation fusion (pure belief update over measurement
// evidence, never SIM truth). Surveyed ground returns carry a height
// observation (the reconstructed 3D endpoint); each cell fuses multi-view
// heights by inverse variance with a bounded forgetting factor, and
// reports disagreement instead of silently averaging an occlusion edge
// into one height. A cell whose sigma exceeds the cap publishes invalid
// (unknown): downstream falls back to the survey prior, never to a guess.
//
// The fusion math mirrors the device-side practice of publishing height
// together with its uncertainty instead of a bare last-sample value.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace lost_signal::robot::autonomy {

struct ElevationFusionConfig {
  // Ranging noise of one return (ROBOT-owned device assumption, documented
  // in the mapper; pose uncertainty adds per observation from the estimate).
  double return_sigma_m = 0.03;
  // Pose uncertainty is correlated across one scan: added to the
  // observation variance instead of averaging away.
  double pose_sigma_m = 0.0;
  // Sustained disagreement beyond this reports unreliable (stale survey,
  // occlusion edge, moving obstacle) instead of publishing a fiction.
  double maximum_sigma_m = 0.50;
  // Forgetting factor bound: old evidence discounts so a cell still follows
  // real terrain changes instead of being owned by history forever.
  std::uint16_t maximum_observations = 64;
};

struct ElevationEstimate {
  bool valid = false;
  double height_m = 0.0;
  double sigma_m = 0.0;
  std::uint16_t observations = 0;
};

class ElevationFusion {
 public:
  explicit ElevationFusion(ElevationFusionConfig config = {})
      : config_(config) {}

  void reset(std::size_t cells) { cells_.assign(cells, Cell{}); }
  std::size_t cell_count() const { return cells_.size(); }

  // One height return into a cell. Non-finite height, negative extra sigma,
  // or out-of-range cell is rejected with stored evidence untouched.
  // The return evidence fuses even when the cell stays unpublished; the
  // caller sees valid == false rather than losing the observation.
  ElevationEstimate fuse(std::size_t cell, double height_m,
                         double extra_sigma_m = 0.0) {
    if (!valid_config() || cell >= cells_.size() ||
        !std::isfinite(height_m) || !std::isfinite(extra_sigma_m) ||
        extra_sigma_m < 0.0) {
      return {};
    }
    const double observation_variance =
        config_.return_sigma_m * config_.return_sigma_m +
        config_.pose_sigma_m * config_.pose_sigma_m +
        extra_sigma_m * extra_sigma_m;
    if (!(observation_variance > 0.0) || !std::isfinite(observation_variance)) {
      return {};
    }
    const double weight = 1.0 / observation_variance;
    Cell& c = cells_[cell];
    const double decay =
        static_cast<double>(config_.maximum_observations - 1) /
        static_cast<double>(config_.maximum_observations);
    if (c.observations == 0) {
      c.mean_m = height_m;
      c.precision = weight;
      c.weighted_square = 0.0;
    } else {
      c.precision *= decay;
      c.weighted_square *= decay;
      c.precision += weight;
      const double previous_mean = c.mean_m;
      c.mean_m = (previous_mean * (c.precision - weight) + height_m * weight) /
                 c.precision;
      const double residual = height_m - c.mean_m;
      c.weighted_square += weight * residual * residual;
    }
    if (c.observations < std::numeric_limits<std::uint16_t>::max()) {
      ++c.observations;
    }
    return estimate(cell);
  }

  ElevationEstimate estimate(std::size_t cell) const {    ElevationEstimate out;
    if (cell >= cells_.size()) return out;
    const Cell& c = cells_[cell];
    if (c.observations == 0) return out;
    if (!(c.precision > 0.0) || !std::isfinite(c.precision)) return out;
    out.valid = true;
    out.height_m = c.mean_m;
    out.observations = c.observations;
    // Fused-mean variance plus observed between-view disagreement.
    const double disagreement =
        c.observations > 1
            ? std::max(0.0, c.weighted_square) /
                  (c.precision * (c.observations - 1))
            : 0.0;
    out.sigma_m = std::sqrt(1.0 / c.precision + disagreement);
    if (!std::isfinite(out.sigma_m) ||
        out.sigma_m > config_.maximum_sigma_m) {
      out.valid = false;
    }
    return out;
  }

  const ElevationFusionConfig& config() const { return config_; }

  // Full belief state per cell (grid owner remaps it on window recenter;
  // out-of-range reads default). Round-trips exactly by construction.
  struct CellState {
    double mean_m = 0.0;
    double precision = 0.0;
    double weighted_square = 0.0;
    std::uint16_t observations = 0;
  };
  CellState cell_state(std::size_t cell) const {
    if (cell >= cells_.size()) return CellState{};
    const Cell& c = cells_[cell];
    return CellState{c.mean_m, c.precision, c.weighted_square,
                     c.observations};
  }
  void set_cell_state(std::size_t cell, CellState state) {
    if (cell >= cells_.size()) return;
    Cell& c = cells_[cell];
    c.mean_m = std::isfinite(state.mean_m) ? state.mean_m : 0.0;
    c.precision =
        (std::isfinite(state.precision) && state.precision >= 0.0)
            ? state.precision
            : 0.0;
    c.weighted_square =
        (std::isfinite(state.weighted_square) && state.weighted_square >= 0.0)
            ? state.weighted_square
            : 0.0;
    c.observations = state.observations;
  }

 private:
  struct Cell {
    double mean_m = 0.0;
    double precision = 0.0;  // inverse-variance sum (forgetting-bounded)
    double weighted_square = 0.0;
    std::uint16_t observations = 0;
  };
  bool valid_config() const {
    return std::isfinite(config_.return_sigma_m) &&
           config_.return_sigma_m > 0.0 &&
           std::isfinite(config_.pose_sigma_m) &&
           config_.pose_sigma_m >= 0.0 &&
           std::isfinite(config_.maximum_sigma_m) &&
           config_.maximum_sigma_m > 0.0 &&
           config_.maximum_observations >= 2;
  }
  ElevationFusionConfig config_;
  std::vector<Cell> cells_;
};

// Non-owning view of a published live elevation raster (mapper output) for
// the validator: measurement heights where confident, NaN elsewhere. The
// validator prefers live where present and falls back to the survey prior
// — same ground doctrine, two sources. Empty (null data) disables live.
struct LiveElevationView {
  const float* height_m = nullptr;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  double resolution_m = 0.1;
  double origin_x_m = 0.0;
  double origin_y_m = 0.0;

  bool has(std::uint32_t w, std::uint32_t h) const {
    return attached() && width == w && height == h;
  }
  bool attached() const {
    return height_m != nullptr && width > 0 && height > 0;
  }
  // Live surface height at (wx, wy); NaN when absent or unknown.
  float at(double wx, double wy) const {
    if (height_m == nullptr || width == 0 || height == 0) {
      return std::numeric_limits<float>::quiet_NaN();
    }
    const int gx = static_cast<int>(
        std::floor((wx - origin_x_m) / resolution_m));
    const int gy = static_cast<int>(
        std::floor((wy - origin_y_m) / resolution_m));
    if (gx < 0 || gy < 0 || gx >= static_cast<int>(width) ||
        gy >= static_cast<int>(height)) {
      return std::numeric_limits<float>::quiet_NaN();
    }
    return height_m[static_cast<std::size_t>(gy) * width + gx];
  }
};

// Live-first ground test shared by the validator call sites: a return is
// ground when the LIVE surface at the endpoint meets the sensor plane
// (measurement wins where confident); otherwise the survey prior decides.
template <typename SurveyGroundFn>
inline bool endpoint_is_live_ground(const LiveElevationView& live,
                                    double end_x_m, double end_y_m,
                                    double sensor_z_m, double tol_m,
                                    SurveyGroundFn survey_ground) {
  if (tol_m > 0.0 && std::isfinite(sensor_z_m)) {
    const float h = live.at(end_x_m, end_y_m);
    if (std::isfinite(h) &&
        std::abs(static_cast<double>(h) - sensor_z_m) <= tol_m) {
      return true;
    }
  }
  return survey_ground();
}

}  // namespace lost_signal::robot::autonomy
