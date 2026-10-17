#pragma once

#include "robot/autonomy/control/trajectory_tracking_system.hpp"
#include "robot/autonomy/perception/perception_pipeline.hpp"
#include "robot/autonomy/planning/local/local_planning_system.hpp"

#include <cstdint>

namespace sim {

// Owns the stateful autonomy stages that must survive between scheduler
// ticks. Mission/runtime code supplies immutable observations, intent and
// constraints; this object owns perception state, local motion generation,
// validation and tracking evidence. It has no access to actor truth, goals,
// scoring, rendering or physics bodies, so the same pipeline can be driven
// by simulation, replay or a hardware provider.
class AutonomyPipeline {
 public:
 AutonomyPipeline(const GridMap& static_map, int width, int height,
 double resolution_m, std::uint32_t seed,
 ObservedObstacleLayerConfig obstacle_config = {});

 void reset(std::uint32_t seed);

 LidarPerceptionFrame process_lidar(
 const Scan& scan, const Scan& static_reference,
 const LidarProjectionState& state, const LidarProjectionConfig& config,
 double track_max_age_s);

 GridMap compose_local_map(const GridMap& static_costmap, double now_s,
 double inflation_radius_m) const;

 const DynamicObstacleMap& dynamic_obstacles() const;
 const ObservedObstacleLayer& observed_obstacles() const;

 LocalPlanningOutput plan_local(
 const LocalPlanningRequest& request, const GridMap& planning_map,
 const std::vector<std::vector<DynamicObjectState>>& predicted_frames,
 StaticFeasibility terrain_feasible = {},
 const GridMap* physical_collision_map = nullptr) const;

 TrajectoryTrackingReport observe_trajectory(
 double time_s, Vec2 position, double heading_rad,
 double forward_mps) const;
 std::uint64_t accept_trajectory(
 double generated_at_s, const ExecutableTrajectory& trajectory,
 const TrajectoryTrackingLimits& limits);

 private:
 PerceptionPipeline perception_;
 LocalPlanningSystem local_planner_;
 TrajectoryTrackingSystem trajectory_tracker_;
};

} // namespace sim
