#pragma once

#include "robot/autonomy/behavior/crossing_behavior.hpp"
#include "robot/autonomy/behavior/entry_governor.hpp"
#include "robot/autonomy/control/trajectory_tracking_system.hpp"
#include "robot/autonomy/perception/perception_pipeline.hpp"
#include "robot/autonomy/planning/local/local_planning_system.hpp"
#include "robot/runtime/map_provider.hpp"

#include <cstdint>
#include <limits>
#include <string>

namespace sim {

struct CrossingExecutionFeedback {
 std::uint64_t trajectory_sequence{0};
 double stamp_s{-std::numeric_limits<double>::infinity()};
 bool committed{false};
 bool blocked{false};
 bool contingency_escape{false};
};

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
 ObservedObstacleLayerConfig obstacle_config = {},
 const runtime::IMapProvider* map_provider = nullptr);

 void reset(std::uint32_t seed);

 LidarPerceptionFrame process_lidar(
 const Scan& scan, const Scan& static_reference,
 const LidarProjectionState& state, const LidarProjectionConfig& config,
 double track_max_age_s);

 GridMap compose_local_map(const GridMap& static_costmap, double now_s,
 double inflation_radius_m) const;

 const DynamicObstacleMap& dynamic_obstacles() const;
 PredictionBatch predict_dynamic_objects(
 double time_s, double horizon_s, double frame_dt_s,
 double max_age_s) const;
 const ObservedObstacleLayer& observed_obstacles() const;
 const std::vector<ObservedRay2D>& latest_visibility_rays() const;
 double latest_visibility_stamp_s() const;

 // Behavior obtains deployable semantics through IMapProvider. Simulation
 // callers provide route, state and sensor evidence; they never select or
 // translate map features, and cannot own the behavior state machine.
 CrossingBehaviorDecision assess_crossing(
 const std::vector<RoutePoint>& route, std::size_t path_index,
 Vec2 ego_position, double ego_speed_mps, double ego_heading_rad,
 const std::vector<std::vector<DynamicObjectState>>& predicted_frames,
 double frame_dt_s, const CrossingSafetyEnvelope& safety,
 const CrossingExecutionCaps& execution, double decision_dt_s,
 double evaluation_stamp_s,
 CrossingPathFeasibility path_feasible = {});

 GovernorOutput update_entry_governor(const GovernorInput& input);
 EntryWindowDecision assess_entry_window(const EntryWindowRequest& request);
 bool entry_governor_holding() const;

 void report_crossing_execution(
 bool committed, const TrajectoryValidation& validation,
 std::uint64_t trajectory_sequence, double stamp_s);
 const CrossingExecutionFeedback& crossing_execution_feedback() const {
 return crossing_execution_feedback_;
 }

 LocalPlanningOutput plan_local(
 const LocalPlanningRequest& request, const GridMap& planning_map,
 const std::vector<std::vector<DynamicObjectState>>& predicted_frames,
 StaticFeasibility terrain_feasible = {},
 const GridMap* physical_collision_map = nullptr,
 StaticFeasibility physical_terrain_feasible = {},
 TerrainSampleQuery terrain_sample = {});

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
 const runtime::IMapProvider* map_provider_{nullptr};
 std::string crossing_map_id_;
 std::uint64_t crossing_map_revision_{0};
 std::vector<CrossingZoneSpec> crossing_zones_;
 CrossingBehaviorSystem crossing_behavior_;
 EntryGovernorSystem entry_governor_;
 CrossingExecutionFeedback crossing_execution_feedback_;

 void refresh_crossing_semantics();
};

} // namespace sim
