#include "robot/autonomy/pipeline/autonomy_pipeline.hpp"

namespace sim {

AutonomyPipeline::AutonomyPipeline(const GridMap& static_map, int width,
 int height, double resolution_m, std::uint32_t seed,
 ObservedObstacleLayerConfig obstacle_config)
 : perception_(static_map, width, height, resolution_m, seed,
 std::move(obstacle_config)) {}

void AutonomyPipeline::reset(std::uint32_t seed) {
 perception_.reset(seed);
 local_planner_.reset();
 trajectory_tracker_.reset();
}

LidarPerceptionFrame AutonomyPipeline::process_lidar(
 const Scan& scan, const Scan& static_reference,
 const LidarProjectionState& state, const LidarProjectionConfig& config,
 double track_max_age_s) {
 return perception_.process(
 scan, static_reference, state, config, track_max_age_s);
}

GridMap AutonomyPipeline::compose_local_map(
 const GridMap& static_costmap, double now_s,
 double inflation_radius_m) const {
 return perception_.compose_local_map(
 static_costmap, now_s, inflation_radius_m);
}

const DynamicObstacleMap& AutonomyPipeline::dynamic_obstacles() const {
 return perception_.dynamic_obstacles();
}

const ObservedObstacleLayer& AutonomyPipeline::observed_obstacles() const {
 return perception_.obstacle_layer();
}

LocalPlanningOutput AutonomyPipeline::plan_local(
 const LocalPlanningRequest& request, const GridMap& planning_map,
 const std::vector<std::vector<DynamicObjectState>>& predicted_frames,
 StaticFeasibility terrain_feasible,
 const GridMap* physical_collision_map,
 StaticFeasibility physical_terrain_feasible) {
 return local_planner_.plan(request, planning_map, predicted_frames,
 std::move(terrain_feasible), physical_collision_map,
 std::move(physical_terrain_feasible));
}

TrajectoryTrackingReport AutonomyPipeline::observe_trajectory(
 double time_s, Vec2 position, double heading_rad,
 double forward_mps) const {
 return trajectory_tracker_.observe(
 time_s, position, heading_rad, forward_mps);
}

std::uint64_t AutonomyPipeline::accept_trajectory(
 double generated_at_s, const ExecutableTrajectory& trajectory,
 const TrajectoryTrackingLimits& limits) {
 return trajectory_tracker_.accept(generated_at_s, trajectory, limits);
}

} // namespace sim
