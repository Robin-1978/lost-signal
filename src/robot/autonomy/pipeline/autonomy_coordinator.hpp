#pragma once

#include "robot/autonomy/behavior/high_level_policy.hpp"
#include "robot/autonomy/control/route_tracking_system.hpp"
#include "robot/autonomy/control/controlled_stop_latch.hpp"
#include "robot/autonomy/control/safe_stop_limiter.hpp"
#include "robot/autonomy/diagnostics/chassis_diagnostics.hpp"
#include "robot/autonomy/localization/estimation_system.hpp"
#include "robot/autonomy/localization/localization_supervisor.hpp"
#include "robot/autonomy/planning/global/global_planner.hpp"
#include "robot/autonomy/pipeline/autonomy_pipeline.hpp"
#include "robot/autonomy/pipeline/robot_motion_runtime.hpp"
#include "robot/autonomy/validation/collision_monitor.hpp"
#include "robot/control/ackermann_actuator_controller.hpp"
#include "robot/control/differential_wheel_controller.hpp"
#include "robot/control/holonomic_actuator_controller.hpp"
#include "robot/runtime/runtime_channel.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace sim::autonomy {

// The coordinator is the deployable owner of the upper autonomy policy
// boundary.  A composition root (simulation, replay, or a native robot
// process) supplies only estimated state and provider-neutral safety facts.
// It never receives a level, actor, physics body, or simulator truth object.
struct AutonomyCoordinatorInput {
 StrategyObservation observation;
 StrategySafetyContext safety;
};

struct AutonomyCoordinatorOutput {
 StrategyDecision decision;
 std::string policy_name;
};

// The ROBOT-owned actuator transaction.  A composition adapter supplies only
// timestamped LSRI feedback and a body-motion intent; this method performs the
// chassis-specific control law and returns a typed, expiring LSRI command.
// Provider and simulator types must not appear in this result.
struct ChassisControlResult {
 runtime::ActuatorCommand command;
};

// Provider-neutral local-planning transaction. Composition roots assemble
// timestamped observations and map views, while the ROBOT coordinator owns
// the planning call and its state. Pointers are non-owning and valid only for
// the duration of plan_local_motion(); they avoid copying large maps per tick.
struct RobotLocalPlanningInput {
 const LocalPlanningRequest* request{nullptr};
 const GridMap* planning_map{nullptr};
 const std::vector<std::vector<DynamicObjectState>>* predicted_frames{nullptr};
 StaticFeasibility terrain_feasible;
 const GridMap* physical_collision_map{nullptr};
 StaticFeasibility physical_terrain_feasible;
 TerrainSampleQuery terrain_sample;
};

// One provider-neutral decision cycle. The ROBOT composition root may run
// policy and local planning in the same transaction; SIM and native providers
// only assemble these typed observations and consume the typed result.
struct RobotAutonomyCycleInput {
 AutonomyCoordinatorInput autonomy;
 const RobotLocalPlanningInput* local_planning{nullptr};
};

struct RobotAutonomyCycleOutput {
 AutonomyCoordinatorOutput autonomy;
 std::optional<LocalPlanningOutput> local_planning;
};

class AutonomyCoordinator {
 public:
 explicit AutonomyCoordinator(std::shared_ptr<IHighLevelPolicy> policy = {});

 void reset(std::uint64_t seed);
 AutonomyCoordinatorOutput update(const AutonomyCoordinatorInput& input);
 RobotAutonomyCycleOutput run_cycle(const RobotAutonomyCycleInput& input);

 // Route tracking is stateful actuator-facing autonomy. It is configured by
 // the composition root from the selected chassis profile, but its state is
 // owned by ROBOT so SIM cannot become the long-term owner of PID progress.
 void configure_route_tracking(double kp, double ki, double kd,
 double output_limit, double integral_limit);
 bool has_route_tracking() const { return route_tracking_ != nullptr; }
 RouteTrackingSystem& route_tracking();
 const RouteTrackingSystem& route_tracking() const;
 std::size_t route_path_index() const;
 void set_route_path_index(std::size_t index);
 bool route_reversing() const;
 void set_route_reversing(bool reversing);
 void reset_route_tracking();
 sim::GuideOut follow_route(const std::vector<sim::RoutePoint>& path,
 const sim::EstimatedState& estimate, double lookahead_m, double cruise_mps,
 double dt_s, const GridMap& planning_map, const ChassisCaps& chassis_caps);
 TravelDirection route_travel_direction(const std::vector<sim::RoutePoint>& path) const;
 bool route_should_replan(const std::vector<sim::RoutePoint>& path,
 const sim::EstimatedState& estimate, const GridMap& local_map,
 const std::vector<DynamicTrack>& dynamic_tracks, double dt_s);
 double route_heading_command(double error_rad, double dt_s);
 double route_pid_p() const;
 double route_pid_i() const;
 double route_pid_d() const;

 bool update_controlled_stop(bool carrying_stop, double peak_wheel_speed_mps,
 const ControlledStopLatch::Config& config);
 void reset_controlled_stop();

 void configure_collision_monitor(Vec2 world_size, double robot_radius_m,
 std::vector<CollisionBox> static_obstacles);
 CollisionMonitor& collision_monitor();
 const CollisionMonitor& collision_monitor() const;

 void reset_chassis_controllers();
 RobotMotionRuntime& motion_runtime() { return motion_runtime_; }
 const RobotMotionRuntime& motion_runtime() const { return motion_runtime_; }
 void configure_differential_controller(
 DifferentialWheelControllerConfig config, WheelPidGains gains);
 void configure_ackermann_controller(
 AckermannActuatorControllerConfig config, WheelPidGains gains);
 void configure_holonomic_controller(
 HolonomicActuatorControllerConfig config, WheelPidGains gains);
 bool has_differential_controller() const { return differential_controller_ != nullptr; }
 bool has_ackermann_controller() const { return ackermann_controller_ != nullptr; }
 bool has_holonomic_controller() const { return holonomic_controller_ != nullptr; }
 DifferentialWheelController& differential_controller();
 AckermannActuatorController& ackermann_controller();
 HolonomicActuatorController& holonomic_controller();

 ChassisControlResult update_chassis_actuators(
 const DiffCmd& command, const ObservationFrame& feedback,
 double dt_s, runtime::TimePoint now, runtime::TimePoint valid_until,
 std::uint64_t sequence);

 // Safety limiting is a ROBOT-owned computation. A composition root may
 // supply the current chassis profile and measured terrain/COM facts, but it
 // must not call the limiter implementation directly from a simulator.
 double safe_stop_deceleration_limit(
 const ChassisCaps& chassis, const SafeStopLimits& limits) const;

 LocalPlanningOutput plan_local_motion(const RobotLocalPlanningInput& input);
 void set_motion_policy(const MotionPolicy& policy);

 // Provider-neutral autonomy surface. Simulation and hardware composition
 // roots use these typed views instead of reaching through the concrete
 // AutonomyPipeline. This keeps the pipeline replaceable without letting a
 // provider become the owner of perception, prediction, localization or
 // crossing state.
 GridMap compose_local_map(const GridMap& static_costmap, double now_s,
 double inflation_radius_m) const;
 std::vector<Vec2> active_observed_obstacles(double now_s) const;
 std::vector<DynamicTrack> active_dynamic_tracks(double time_s,
 double max_age_s) const;
 PredictionBatch predict_dynamic_objects(double time_s, double horizon_s,
 double frame_dt_s, double max_age_s) const;
 LidarPerceptionFrame process_lidar(
 const Scan& scan, const Scan& static_reference,
 const LidarProjectionState& state, const LidarProjectionConfig& config,
 double track_max_age_s);
 KnownMapLocalizationResult localize_known_map(
 const Scan& scan, Vec2 odom_position, double odom_heading_rad,
 const runtime::MapSnapshot& map, double timestamp_s) const;
 autonomy::LocalizationResult localize_known_map_global(
 const Scan& scan, const runtime::MapSnapshot& map, double timestamp_s) const;
 CrossingBehaviorDecision assess_crossing(
 const std::vector<RoutePoint>& route, std::size_t path_index,
 Vec2 ego_position, double ego_speed_mps, double ego_heading_rad,
 const std::vector<std::vector<DynamicObjectState>>& predicted_frames,
 double frame_dt_s, const CrossingSafetyEnvelope& safety,
 const CrossingExecutionCaps& execution, double decision_dt_s,
 double evaluation_stamp_s, CrossingPathFeasibility path_feasible = {});
 GovernorOutput update_entry_governor(const GovernorInput& input);
 EntryWindowDecision assess_entry_window(const EntryWindowRequest& request);
 bool entry_governor_holding() const;
 void report_crossing_execution(bool committed,
 const TrajectoryValidation& validation, std::uint64_t trajectory_sequence,
 double stamp_s);
 const CrossingExecutionFeedback& crossing_execution_feedback() const;
 TrajectoryTrackingReport observe_trajectory(double time_s, Vec2 position,
 double heading_rad, double forward_mps) const;
 std::uint64_t accept_trajectory(double generated_at_s,
 const ExecutableTrajectory& trajectory, const TrajectoryTrackingLimits& limits);

 ChassisDiagnosticsSystem& diagnostics() { return diagnostics_; }
 const ChassisDiagnosticsSystem& diagnostics() const { return diagnostics_; }

 void set_estimator_motion_policy(const MotionPolicy& policy) {
 estimator_.set_motion_policy(policy);
 }
 EstimationSystem& estimator() { return estimator_; }
 const EstimationSystem& estimator() const { return estimator_; }

 void set_localization_config(const autonomy::LocalizationSupervisorConfig& config) {
 localization_supervisor_.set_config(config);
 }
 autonomy::LocalizationSupervisor& localization_supervisor() { return localization_supervisor_; }
 const autonomy::LocalizationSupervisor& localization_supervisor() const {
 return localization_supervisor_;
 }

 void configure_global_planner(std::shared_ptr<const ::autonomy::IGlobalPlanner> planner);
 ::autonomy::GlobalPlanningSystem& global_planning();
 const ::autonomy::GlobalPlanningSystem& global_planning() const;

 void configure_pipeline(const GridMap& static_map, int width, int height,
 double resolution_m, std::uint32_t seed,
 ObservedObstacleLayerConfig obstacle_config = {},
 const runtime::IMapProvider* map_provider = nullptr);
 bool has_pipeline() const { return pipeline_ != nullptr; }
 AutonomyPipeline& pipeline();
 const AutonomyPipeline& pipeline() const;
 void reset_pipeline(std::uint32_t seed);

 const std::string& policy_name() const { return policy_name_; }

 private:
 std::shared_ptr<IHighLevelPolicy> policy_;
 std::string policy_name_;
 std::unique_ptr<RouteTrackingSystem> route_tracking_;
 ControlledStopLatch controlled_stop_latch_;
 ChassisDiagnosticsSystem diagnostics_;
 std::unique_ptr<CollisionMonitor> collision_monitor_;
 std::unique_ptr<DifferentialWheelController> differential_controller_;
 std::unique_ptr<AckermannActuatorController> ackermann_controller_;
 std::unique_ptr<HolonomicActuatorController> holonomic_controller_;
 RobotMotionRuntime motion_runtime_;
 EstimationSystem estimator_;
 autonomy::LocalizationSupervisor localization_supervisor_;
 std::unique_ptr<::autonomy::GlobalPlanningSystem> global_planning_;
 std::unique_ptr<AutonomyPipeline> pipeline_;
};

} // namespace sim::autonomy
