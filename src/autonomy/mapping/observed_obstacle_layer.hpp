#pragma once

#include "autonomy/mapping/grid_map.hpp"

#include <cstddef>
#include <limits>
#include <vector>

namespace sim {

// One projected planar range observation in the map frame. Projection and
// scan deskew happen before mapping so the layer has no sensor- or TF-specific
// dependency and can consume simulated, replayed, or hardware measurements.
struct ObservedRay2D {
 Vec2 origin;
 Vec2 endpoint;
 bool hit{false};
};

struct ObservedObstacleLayerConfig {
 double obstacle_persistence_s{0.6};
 double expected_update_period_s{0.1};
};

// Transient occupancy evidence derived only from observations. A complete
// scan is applied atomically: rays clear free space, hit endpoints mark
// obstacles, and marking wins where neighboring rays touch the same cell.
// Static-map ownership and inflation remain separate composition steps.
class ObservedObstacleLayer {
 public:
 ObservedObstacleLayer(int width, int height, double resolution,
 ObservedObstacleLayerConfig config = {});

 void reset();
 bool observe(double stamp_s, const std::vector<ObservedRay2D>& rays);

 bool fresh(double now_s) const;
 std::size_t active_obstacle_cells(double now_s) const;
 GridMap compose(const GridMap& static_costmap, double now_s, double inflation_radius_m) const;

 double last_observation_stamp_s() const { return last_observation_stamp_s_; }

 private:
 struct CellEvidence {
 double marked_s{-std::numeric_limits<double>::infinity()};
 double cleared_s{-std::numeric_limits<double>::infinity()};
 };

 std::vector<std::size_t> ray_cells(Vec2 origin, Vec2 endpoint, bool include_endpoint) const;
 bool active(const CellEvidence& cell, double now_s) const;

 int width_{0};
 int height_{0};
 double resolution_{0.0};
 ObservedObstacleLayerConfig config_;
 std::vector<CellEvidence> cells_;
 double last_observation_stamp_s_{-std::numeric_limits<double>::infinity()};
};

} // namespace sim
