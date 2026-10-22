#pragma once

#include "robot/autonomy/mapping/grid_map.hpp"
#include "robot/autonomy/mapping/observed_obstacle_layer.hpp"
#include "robot/autonomy/perception/dynamic_obstacle_map.hpp"
#include "robot/core/random.hpp"
#include "robot/runtime/sensor_messages.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace sim {

struct LidarProjectionState {
 Vec2 base_position;
 double base_heading_rad{0.0};
 double linear_velocity_mps{0.0};
 double lateral_velocity_mps{0.0};
 double angular_velocity_radps{0.0};
 double base_height_m{0.0};
 double base_roll_rad{0.0};
 double base_pitch_rad{0.0};
 bool attitude_valid{false};
 // Dynamic classification is map-relative. When the pose is degraded, a
 // static tree or wall can appear as a moving foreground cluster. Keep raw
 // returns in the local obstacle layer, but do not publish map-frame tracks
 // until the localization estimate is trustworthy again.
 double localization_confidence{1.0};
 bool localization_valid{true};
};

struct LidarProjectionConfig {
 Vec2 position_in_base;
 double yaw_in_base_rad{0.0};
 double range_noise_sigma_m{0.0};
 double map_alignment_tolerance_m{0.45};
 double height_in_base_m{0.0};
 double roll_in_base_rad{0.0};
 double pitch_in_base_rad{0.0};
 double ground_alignment_tolerance_m{0.30};
 // Dynamic classification compares foreground geometry with the loaded map.
 // Below this confidence the raw sensor frame remains usable for local
 // safety, but map-frame tracks must not be published.
 double min_dynamic_classification_confidence{0.35};
};

struct LidarPerceptionFrame {
 std::vector<Vec2> hits;
 std::vector<ObservedRay2D> obstacle_rays;
 std::vector<ObservedClearingRegion> dynamic_clearing_regions;
 bool static_obstacle_near{false};
 bool dynamic_object_detected{false};
};

// Autonomy front end for a planar scan. It projects and deskews raw ranges,
// produces occupancy rays, separates map-consistent geometry from foreground,
// and owns anonymous dynamic-object tracking. It consumes no simulator truth.
class LidarPerceptionSystem {
 public:
 LidarPerceptionSystem(const GridMap& static_map, std::uint32_t seed);

 void reset(std::uint32_t seed);
 LidarPerceptionFrame process(const Scan& scan, const Scan& static_reference,
 const LidarProjectionState& state, const LidarProjectionConfig& config,
 double track_max_age_s, double dynamic_occupancy_history_s = 0.0);

 const DynamicObstacleMap& dynamic_obstacles() const { return dynamic_obstacles_; }
 DynamicObstacleMap& dynamic_obstacles() { return dynamic_obstacles_; }

 private:
 struct PendingDetection {
 std::size_t id{0};
 Vec2 position;
 double radius_m{0.45};
 double last_seen_s{0.0};
 int confirmations{0};
 };

 const GridMap& static_map_;
 Rng perception_rng_;
 DynamicObstacleMap dynamic_obstacles_;
 std::vector<PendingDetection> pending_;
 std::size_t next_track_id_{0};
};

} // namespace sim
