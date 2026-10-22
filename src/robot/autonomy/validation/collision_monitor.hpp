#pragma once

#include "robot/autonomy/mapping/grid_map.hpp"
#include "robot/core/dynamic_object_state.hpp"
#include "robot/runtime/sensor_messages.hpp"

#include <vector>

namespace sim::autonomy {

struct CollisionBox {
 Vec2 min;
 Vec2 max;
 double yaw_rad{0.0};
};

struct CollisionActorObservation {
 DynamicObjectState state;
 bool social{false};
};

struct CollisionObservation {
 Vec2 position;
 double time_s{0.0};
 double dt_s{0.0};
 const GridMap* occupancy{nullptr};
 std::vector<CollisionActorObservation> actors;
};

struct CollisionAssessment {
 bool static_collision{false};
 bool dynamic_collision{false};
 int social_contacts{0};
 double static_clearance_m{0.0};
 double dynamic_clearance_m{999.0};
 Vec2 nearest_actor_position;
};

// Configuration for the short-range safety field measured directly in the
// LiDAR frame. This field is independent of global localization: a drifting
// map pose must not be able to turn a measured object into free space.
struct LidarSafetyConfig {
 double half_angle_rad{0.5};
 double reaction_time_s{0.30};
 double braking_decel_mps2{0.80};
 double sensor_standoff_m{0.05};
 double range_growth_m_per_mps{0.10};
 double zero_range_m{0.01};
};

struct LidarSafetyObservation {
 const Scan* scan{nullptr};
 Vec2 position_in_base;
 double yaw_in_base_rad{0.0};
 double min_range_m{0.0};
 double forward_speed_mps{0.0};
 bool measurement_valid{true};
 bool measurement_fresh{true};
};

struct LidarSafetyAssessment {
 bool stop_required{true};
 // Absence of evidence is not valid evidence. Consumers may construct this
 // struct before a scan is available and must fail closed without reporting
 // that a healthy sensor observed free space.
 bool measurement_valid{false};
 bool target_in_path{false};
 double forward_clearance_m{-1.0};
 double required_forward_clearance_m{0.0};
 double braking_distance_m{0.0};
 Vec2 nearest_target_in_base;
};

// ROBOT safety monitor. Inputs are deployment-map geometry and observed or
// predicted tracks only. A simulator's rigid contacts are deliberately not a
// valid input here; SIM owns the separate physical-world judge.
class CollisionMonitor {
 public:
 CollisionMonitor(Vec2 world_size, double robot_radius_m,
 std::vector<CollisionBox> static_obstacles);

 void reset();
 CollisionAssessment assess(const CollisionObservation& observation);
 // Evaluate the raw sensor-frame field. The result is deliberately usable
 // without a map pose or localization confidence: missing, stale or invalid
 // measurements fail closed, and a target inside the conservative stopping
 // distance forbids further motion in that direction.
 LidarSafetyAssessment assess_lidar(const LidarSafetyObservation& observation,
 const LidarSafetyConfig& config) const;
 // Continuous circular-footprint check used by local trajectory validation.
 // This is analytic world geometry, independent of the raster costmap used
 // for route preference and observed-obstacle layering.
 bool static_motion_free(Vec2 from, Vec2 to,
 double execution_margin_m = 0.0) const;
 double static_clearance(Vec2 position) const;

 private:
 Vec2 world_size_;
 double robot_radius_m_{0.0};
 std::vector<CollisionBox> static_obstacles_;
};

} // namespace sim::autonomy
