#include "robot/autonomy/pipeline/autonomy_pipeline.hpp"
#include "robot/autonomy/behavior/crossing_map_semantics.hpp"

#include <stdexcept>

namespace sim {

AutonomyPipeline::AutonomyPipeline(const GridMap& static_map, int width,
 int height, double resolution_m, std::uint32_t seed,
 ObservedObstacleLayerConfig obstacle_config,
 const runtime::IMapProvider* map_provider)
 : perception_(static_map, width, height, resolution_m, seed,
 std::move(obstacle_config)),
 map_provider_(map_provider) {
 refresh_crossing_semantics();
}

void AutonomyPipeline::reset(std::uint32_t seed) {
 perception_.reset(seed);
 known_map_localizer_.reset();
 local_planner_.reset();
 trajectory_tracker_.reset();
 crossing_behavior_.reset();
 entry_governor_.reset();
 crossing_execution_feedback_ = {};
 refresh_crossing_semantics();
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

PredictionBatch AutonomyPipeline::predict_dynamic_objects(
 double time_s, double horizon_s, double frame_dt_s,
 double max_age_s) const {
 return perception_.dynamic_obstacles().predict_typed(
 time_s, horizon_s, frame_dt_s, max_age_s);
}

const ObservedObstacleLayer& AutonomyPipeline::observed_obstacles() const {
 return perception_.obstacle_layer();
}

const std::vector<ObservedRay2D>& AutonomyPipeline::latest_visibility_rays() const {
 return perception_.latest_visibility_rays();
}

double AutonomyPipeline::latest_visibility_stamp_s() const {
 return perception_.latest_visibility_stamp_s();
}

KnownMapLocalizationResult AutonomyPipeline::localize_known_map(
 const Scan& scan, Vec2 odom_position, double odom_heading_rad,
 const runtime::MapSnapshot& map, double timestamp_s) const {
 return known_map_localizer_.localize(
 scan, odom_position, odom_heading_rad, map, timestamp_s);
}

void AutonomyPipeline::refresh_crossing_semantics() {
 if (!map_provider_) {
 crossing_zones_.clear();
 crossing_map_id_.clear();
 crossing_map_revision_ = 0;
 return;
 }
 const auto map = map_provider_->current_snapshot();
 if (!map) {
 crossing_zones_.clear();
 crossing_map_id_.clear();
 crossing_map_revision_ = 0;
 return;
 }
 if (map->map_id == crossing_map_id_ &&
 map->revision == crossing_map_revision_)
 return;
 const auto semantics = crossing_semantics_from_map(*map);
 if (!semantics.valid) {
 throw std::invalid_argument(
 "invalid crossing semantics in robot map '" + map->map_id +
 "': " + semantics.diagnostic);
 }
 crossing_behavior_.reset();
 entry_governor_.reset();
 crossing_execution_feedback_ = {};
 crossing_zones_ = semantics.zones;
 crossing_map_id_ = map->map_id;
 crossing_map_revision_ = map->revision;
}

CrossingBehaviorDecision AutonomyPipeline::assess_crossing(
 const std::vector<RoutePoint>& route, std::size_t path_index,
 Vec2 ego_position, double ego_speed_mps, double ego_heading_rad,
 const std::vector<std::vector<DynamicObjectState>>& predicted_frames,
 double frame_dt_s, const CrossingSafetyEnvelope& safety,
 const CrossingExecutionCaps& execution, double decision_dt_s,
 double evaluation_stamp_s,
 CrossingPathFeasibility path_feasible) {
 refresh_crossing_semantics();
 return crossing_behavior_.assess(
 crossing_zones_, route, path_index, ego_position, ego_speed_mps,
 ego_heading_rad, predicted_frames, frame_dt_s, safety, execution,
 decision_dt_s, perception_.latest_visibility_rays(),
 perception_.latest_visibility_stamp_s(),
 evaluation_stamp_s,
 crossing_execution_feedback_.committed &&
 crossing_execution_feedback_.blocked,
 crossing_execution_feedback_.committed &&
 crossing_execution_feedback_.contingency_escape,
 std::move(path_feasible));
}

GovernorOutput AutonomyPipeline::update_entry_governor(
 const GovernorInput& input) {
 return entry_governor_.update(input);
}

EntryWindowDecision AutonomyPipeline::assess_entry_window(
 const EntryWindowRequest& request) {
 return entry_governor_.assess(request);
}

bool AutonomyPipeline::entry_governor_holding() const {
 return entry_governor_.holding();
}

void AutonomyPipeline::report_crossing_execution(
 bool committed, const TrajectoryValidation& validation,
 std::uint64_t trajectory_sequence, double stamp_s) {
 crossing_execution_feedback_.trajectory_sequence = trajectory_sequence;
 crossing_execution_feedback_.stamp_s = stamp_s;
 crossing_execution_feedback_.committed = committed;
 crossing_execution_feedback_.blocked = committed &&
 !validation.preserves_behavior_authority();
 crossing_execution_feedback_.contingency_escape = committed &&
 validation.requires_committed_contingency();
}

LocalPlanningOutput AutonomyPipeline::plan_local(
 const LocalPlanningRequest& request, const GridMap& planning_map,
 const std::vector<std::vector<DynamicObjectState>>& predicted_frames,
 StaticFeasibility terrain_feasible,
 const GridMap* physical_collision_map,
 StaticFeasibility physical_terrain_feasible,
 TerrainSampleQuery terrain_sample) {
 return local_planner_.plan(request, planning_map, predicted_frames,
 std::move(terrain_feasible), physical_collision_map,
 std::move(physical_terrain_feasible), std::move(terrain_sample));
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
