#pragma once

#include "robot/autonomy/pipeline/autonomy_coordinator.hpp"
#include "robot/executive/robot_executive.hpp"
#include "robot/executive/task_executive.hpp"
#include "robot/runtime/runtime_channel.hpp"
#include "robot/runtime/map_provider.hpp"
#include "robot/runtime/endpoint_registry.hpp"
#include "robot/runtime/provider_conformance.hpp"
#include "robot/autonomy/planning/local/predictor.hpp"
#include "robot/autonomy/control/safe_stop_limiter.hpp"
#include "robot/autonomy/control/follow.hpp"
#include "robot/autonomy/mapping/terrain.hpp"
#include "robot/autonomy/mapping/raster_traversal.hpp"
#include "robot/autonomy/behavior/crossing_behavior.hpp"
#include "robot/autonomy/behavior/entry_governor.hpp"
#include "robot/autonomy/planning/local/dynamic_clearance.hpp"

#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <cstdint>
#include <utility>
#include <vector>

namespace sim::robot {

// Native-process composition root. It owns only ROBOT objects and a selected
// LSRI channel. SIM uses the same stack in-process, but supplies a simulated
// provider from its own composition root. No game, renderer or physics type
// can enter this class.
struct RobotApplicationConfig {
 runtime::RuntimeRegistry runtime_registry{runtime::RuntimeRegistry::with_builtins()};
 std::string runtime_provider_id{"channel.in-process"};
 std::shared_ptr<::sim::IHighLevelPolicy> high_level_policy;
 runtime::RuntimeChannelConfig channel;
 // Endpoint ownership is deployment data. The same descriptors are used by
 // simulated, replay and native providers; only backend/provider_id changes.
 runtime::EndpointRegistry endpoints;
 runtime::ProviderConformanceConfig conformance;
 // Map ownership stays with the deployment's map service. The application
 // keeps a non-owning provider view so task/UI adapters can use the same
 // immutable map revisions without exposing coordinator internals.
 runtime::IMapProvider* map_provider{nullptr};
};

// One provider-neutral native/SIM cycle. Task and perception composition may
// supply a body-motion intent after local planning; the application owns the
// final ROBOT transaction and never accepts provider-native objects.
struct RobotCycleInput {
 autonomy::AutonomyCoordinatorInput autonomy;
 // Optional local-planning transaction. The caller provides only ROBOT
 // contracts; the application runs it before publishing the actuator/body
 // command. Native and simulated composition roots therefore share one
 // autonomy cycle instead of duplicating planner orchestration.
 const autonomy::RobotLocalPlanningInput* local_planning{nullptr};
 // A composition root may already have run the behavior transaction while
 // assembling this cycle's local-planning facts. In that case ROBOT still
 // owns local planning and validation, but must not update the policy twice
 // in one decision tick.
 bool run_autonomy{true};
 // Optional task observation for the executive.  When present, the same
 // application tick updates task lifecycle before autonomy runs.  Keeping
 // this here makes the ordering explicit without making the executive a
 // second scheduler or asking SIM to call it separately.
 std::optional<executive::TaskObservation> task_observation;
 std::optional<DiffCmd> motion_intent;
 double dt_s{0.0};
 runtime::TimePoint now;
 runtime::TimePoint valid_until;
};

struct RobotCycleOutput {
 autonomy::AutonomyCoordinatorOutput autonomy;
 std::optional<LocalPlanningOutput> local_planning;
 std::optional<runtime::MotionCommand> motion_command;
 std::optional<runtime::ActuatorCommand> actuator_command;
};

class RobotApplication {
 public:
 explicit RobotApplication(RobotApplicationConfig config);
 // In-process composition constructor. The SIM adapter owns the provider
 // channel and supplies it here without transferring ownership. ROBOT still
 // owns the cycle and controller transaction; this is the same boundary used
 // by the native composition root, without requiring IPC.
 RobotApplication(runtime::IRuntimeChannel& channel,
  autonomy::AutonomyCoordinator& coordinator,
  runtime::ProviderConformanceConfig conformance = {});
 // Owning composition constructor used by SIM and native deployment roots.
 // The coordinator lifetime is then unambiguously inside ROBOT.
 RobotApplication(runtime::IRuntimeChannel& channel,
  std::unique_ptr<autonomy::AutonomyCoordinator> coordinator,
  runtime::ProviderConformanceConfig conformance = {});
 // Composition constructor for a deployment that supplies only the policy.
 // RobotApplication constructs and owns the coordinator; a SIM adapter never
 // owns or schedules an autonomy object itself.
 RobotApplication(runtime::IRuntimeChannel& channel,
 std::shared_ptr<::sim::IHighLevelPolicy> policy,
 runtime::ProviderConformanceConfig conformance = {});
 ~RobotApplication();

 RobotApplication(const RobotApplication&) = delete;
 RobotApplication& operator=(const RobotApplication&) = delete;

 void configure();
 void activate();
 void deactivate();

 runtime::LifecycleState state() const;
 runtime::RuntimeHealth health() const;
 const runtime::IRuntimeChannel& channel() const;
 runtime::IAutonomyPort& autonomy_port();
 runtime::IPlatformPort& platform_port();
 void bind_map_provider(runtime::IMapProvider& provider) { map_provider_ = &provider; }
 void bind_map_catalog(runtime::MapCatalog& catalog, runtime::MapManager& active_map);
 bool map_switch_pending() const { return executive_.pending_map_request().has_value(); }
 runtime::IMapProvider& map_provider();
 const runtime::IMapProvider& map_provider() const;

 // The terrain consumed by planning is reconstructed from the active Robot
 // Map revision plus calibrated tire priors. Weather, authored contact
 // coefficients and physics-engine state are deliberately absent.
 void set_traversal_config(RasterTraversalConfig config) {
 traversal_.set_config(std::move(config));
 }
 TerrainSample estimated_terrain(Vec2 position);
 TraversalContactEstimate estimated_contact(Vec2 position);
 double estimated_traversal_cost(Vec2 from, Vec2 to,
 const TerrainCapability& capability);
 // Feed only LSRI motion observations into the ROBOT terrain belief. SIM
 // contact coefficients and authored relief never enter this method.
 void observe_terrain_motion(const EstimatedState& estimate,
 const ImuObservation& imu, double dt_s) {
 traversal_.observe_motion(estimate, imu, dt_s);
 }

 // Provider adapters should use these helpers instead of bypassing the
 // conformance monitor when publishing frames or checking liveness.
 bool publish_observation(ObservationFrame observation, runtime::TimePoint now);
 void check_provider_liveness(runtime::TimePoint now);
 // The single ROBOT cycle entry point for native and in-process composition.
 // Executive and autonomy are ordered here; callers do not schedule either
 // object independently.
 RobotCycleOutput tick(const RobotCycleInput& input);
 // Execute an already selected body-motion intent through the ROBOT
 // controller and LSRI provider. Planning must have produced the intent;
 // this entry point only owns feedback ingestion, actuator conversion,
 // expiry and command publication.
 RobotCycleOutput execute_motion(const DiffCmd& motion_intent, double dt_s,
 runtime::TimePoint now, runtime::TimePoint valid_until);
 const runtime::ProviderConformanceMonitor& conformance() const { return conformance_; }
 runtime::ProviderConformanceMonitor& conformance() { return conformance_; }

 // ROBOT autonomy surface. These methods are the application boundary for
 // SIM, replay and future UI/API adapters; the concrete Coordinator remains
 // private to this composition root.
 void reset(std::uint64_t seed) { active_coordinator().reset(seed); }
 void reset_autonomy(std::uint64_t seed) { active_coordinator().reset(seed); }
 const std::string& policy_name() const { return active_coordinator().policy_name(); }
 EstimationSystem& estimator() { return active_coordinator().estimator(); }
 const EstimationSystem& estimator() const { return active_coordinator().estimator(); }
 autonomy::LocalizationSupervisor& localization_supervisor() { return active_coordinator().localization_supervisor(); }
 const autonomy::LocalizationSupervisor& localization_supervisor() const { return active_coordinator().localization_supervisor(); }
 const autonomy::BrakingMeasurement& update_braking_diagnostics(double time_s, Vec2 position,
 double speed_mps, double target_speed_mps) {
 return active_coordinator().diagnostics().update_braking(time_s, position, speed_mps, target_speed_mps);
 }
 bool update_slip_diagnostics(bool tread_observable, double tread_speed_mps,
 double body_speed_mps, double dt_s) {
 return active_coordinator().diagnostics().update_slip(tread_observable, tread_speed_mps, body_speed_mps, dt_s);
 }
 bool update_slip_from_sensors(const EncoderObservation& encoder,
 const ImuObservation& imu, double dt_s) {
 return active_coordinator().diagnostics().update_slip_from_sensors(encoder, imu, dt_s);
 }
 bool slip_detected() const { return active_coordinator().diagnostics().slip_detected(); }
 int slip_episodes() const { return active_coordinator().diagnostics().slip_episodes(); }
 autonomy::RobotMotionRuntime& motion_runtime() { return active_coordinator().motion_runtime(); }
 const autonomy::RobotMotionRuntime& motion_runtime() const { return active_coordinator().motion_runtime(); }

 void set_estimator_motion_policy(const MotionPolicy& policy) { active_coordinator().set_estimator_motion_policy(policy); }
 void set_localization_config(const autonomy::LocalizationSupervisorConfig& config) {
 active_coordinator().set_localization_config(config);
 }
 void configure_global_planner(std::shared_ptr<const ::autonomy::IGlobalPlanner> planner) {
 active_coordinator().configure_global_planner(std::move(planner));
 }
 ::autonomy::GlobalPlanningSystem& global_planning() { return active_coordinator().global_planning(); }
 const ::autonomy::GlobalPlanningSystem& global_planning() const { return active_coordinator().global_planning(); }
 ::autonomy::GlobalPlan plan_global(const GridMap& map,
 const std::vector<::autonomy::PlanningPose>& waypoints,
 const sim::TraversalCost& traversal_cost) const {
 return active_coordinator().global_planning().plan(map, waypoints, traversal_cost);
 }
 std::string_view global_planner_id() const {
 return active_coordinator().global_planning().planner().id();
 }
 void configure_pipeline(const GridMap& static_map, int width, int height,
 double resolution_m, std::uint32_t seed,
 ObservedObstacleLayerConfig obstacle_config = {},
 const runtime::IMapProvider* map_provider = nullptr) {
 active_coordinator().configure_pipeline(static_map, width, height, resolution_m, seed,
 std::move(obstacle_config), map_provider);
 }
 bool has_pipeline() const { return active_coordinator().has_pipeline(); }
 void reset_pipeline(std::uint32_t seed) { active_coordinator().reset_pipeline(seed); }
 void set_motion_policy(const MotionPolicy& policy) { active_coordinator().set_motion_policy(policy); }

 void configure_route_tracking(double kp, double ki, double kd,
 double output_limit, double integral_limit) {
 active_coordinator().configure_route_tracking(kp, ki, kd, output_limit, integral_limit);
 }
 bool has_route_tracking() const { return active_coordinator().has_route_tracking(); }
 void reset_route_tracking() { active_coordinator().reset_route_tracking(); }
 std::size_t route_path_index() const { return active_coordinator().route_path_index(); }
 void set_route_path_index(std::size_t index) { active_coordinator().set_route_path_index(index); }
 bool route_reversing() const { return active_coordinator().route_reversing(); }
 void set_route_reversing(bool value) { active_coordinator().set_route_reversing(value); }
 sim::GuideOut follow_route(const std::vector<sim::RoutePoint>& path,
 const sim::EstimatedState& estimate, double lookahead_m, double cruise_mps,
 double dt_s, const GridMap& planning_map, const ChassisCaps& chassis_caps) {
 return active_coordinator().follow_route(path, estimate, lookahead_m, cruise_mps,
 dt_s, planning_map, chassis_caps);
 }
 TravelDirection route_travel_direction(const std::vector<sim::RoutePoint>& path) const {
 return active_coordinator().route_travel_direction(path);
 }
 bool route_should_replan(const std::vector<sim::RoutePoint>& path,
 const sim::EstimatedState& estimate, const GridMap& local_map,
 const std::vector<DynamicTrack>& dynamic_tracks, double dt_s) {
 return active_coordinator().route_should_replan(path, estimate, local_map, dynamic_tracks, dt_s);
 }
 double route_heading_command(double error_rad, double dt_s) {
 return active_coordinator().route_heading_command(error_rad, dt_s);
 }
 double route_pid_p() const { return active_coordinator().route_pid_p(); }
 double route_pid_i() const { return active_coordinator().route_pid_i(); }
 double route_pid_d() const { return active_coordinator().route_pid_d(); }

 GridMap compose_local_map(const GridMap& static_costmap, double now_s,
 double inflation_radius_m) const {
 return active_coordinator().compose_local_map(static_costmap, now_s, inflation_radius_m);
 }
 std::vector<Vec2> active_observed_obstacles(double now_s) const {
 return active_coordinator().active_observed_obstacles(now_s);
 }
 std::vector<DynamicTrack> active_dynamic_tracks(double time_s, double max_age_s) const {
 return active_coordinator().active_dynamic_tracks(time_s, max_age_s);
 }
 PredictionBatch predict_dynamic_objects(double time_s, double horizon_s,
 double frame_dt_s, double max_age_s) const {
 return active_coordinator().predict_dynamic_objects(time_s, horizon_s, frame_dt_s, max_age_s);
 }
 // Provider-neutral geometry and scheduling helpers. These keep the SIM
 // adapter from including or invoking ROBOT algorithm implementations while
 // retaining plain value contracts at the application boundary.
 double heading_error(Vec2 target, Vec2 current, double heading_rad) const {
 return sim::heading_err(target, current.x, current.y, heading_rad);
 }
 double scheduled_speed(double heading_error_rad, double cruise_mps) const {
 return sim::sched_v(heading_error_rad, cruise_mps);
 }
 std::vector<Vec2> forward_route_reference(const std::vector<RoutePoint>& path,
 std::size_t begin, Vec2 current, double maximum_length_m) const {
 return sim::forward_route_reference(path, begin, current, maximum_length_m);
 }
 double longitudinal_grade(const TerrainSample& terrain, double heading_rad) const {
 return sim::longitudinal_grade(terrain, heading_rad);
 }
 double lateral_grade(const TerrainSample& terrain, double heading_rad) const {
 return sim::lateral_grade(terrain, heading_rad);
 }
 double static_rollover_angle(double support_width_m,
 double center_of_mass_height_m) const {
 return sim::static_rollover_angle(support_width_m, center_of_mass_height_m);
 }
 double ride_speed_limit(double roughness_m, double support_base_m,
 double gravity_fraction) const {
 return sim::roughness_ride_speed_limit(roughness_m, support_base_m, gravity_fraction);
 }
 double corner_speed_cap(const std::vector<RoutePoint>& path, std::size_t index,
 double cruise_mps, double lateral_accel_mps2, double preview_m,
 double yaw_rate_max_radps) const {
 return sim::corner_speed_cap(path, index, cruise_mps, lateral_accel_mps2,
 preview_m, yaw_rate_max_radps);
 }
 DwaWeights local_planner_weights(double preferred_clearance_m) const {
 return {1.0, 1.3 + preferred_clearance_m * 1.8, 0.25};
 }
 std::vector<std::vector<DynamicObjectState>> flatten_prediction(
 const PredictionBatch& prediction) const {
 return sim::flatten_prediction_frames(prediction);
 }
 DynamicClearance minimum_stationary_clearance(Vec2 ego_position,
 const std::vector<std::vector<DynamicObjectState>>& frames,
 double frame_dt_s, double ego_radius_m) const {
 return sim::minimum_stationary_clearance(ego_position, frames, frame_dt_s,
 ego_radius_m);
 }
 GovernorOutput merge_entry_constraints(const GovernorOutput& first,
 const GovernorOutput& second, const std::vector<Vec2>& route) const {
 return sim::merge_entry_constraints(first, second, route);
 }
 double match_timed_trajectory_phase(const std::vector<TimedPose>& trajectory,
 Vec2 measured_position, double nominal_elapsed_s,
 double admitted_time_error_s) const {
 return sim::match_timed_trajectory_phase(trajectory, measured_position,
 nominal_elapsed_s, admitted_time_error_s);
 }
 std::vector<TimedPose> remaining_timed_trajectory(
 const std::vector<TimedPose>& trajectory, double elapsed_s) const {
 return sim::remaining_timed_trajectory(trajectory, elapsed_s);
 }
 CrossingPoseApproach crossing_pose_approach(Vec2 ego_position,
 double ego_heading_rad, Vec2 stop_point, Vec2 stop_direction,
 bool force_reverse, double drive_axle_x_m, double acquisition_radius_m) const {
 return sim::crossing_pose_approach(ego_position, ego_heading_rad, stop_point,
 stop_direction, force_reverse, drive_axle_x_m, acquisition_radius_m);
 }
 LidarPerceptionFrame process_lidar(const Scan& scan, const Scan& static_reference,
 const LidarProjectionState& state, const LidarProjectionConfig& config,
 double track_max_age_s) {
 return active_coordinator().process_lidar(scan, static_reference, state, config, track_max_age_s);
 }
 KnownMapLocalizationResult localize_known_map(const Scan& scan, Vec2 odom_position,
 double odom_heading_rad, const runtime::MapSnapshot& map, double timestamp_s) const {
 return active_coordinator().localize_known_map(scan, odom_position, odom_heading_rad, map, timestamp_s);
 }
 autonomy::LocalizationResult localize_known_map_global(const Scan& scan,
 const runtime::MapSnapshot& map, double timestamp_s) const {
 return active_coordinator().localize_known_map_global(scan, map, timestamp_s);
 }
 CrossingBehaviorDecision assess_crossing(
 const std::vector<RoutePoint>& route, std::size_t path_index, Vec2 ego_position,
 double ego_speed_mps, double ego_heading_rad,
 const std::vector<std::vector<DynamicObjectState>>& predicted_frames,
 double frame_dt_s, const CrossingSafetyEnvelope& safety,
 const CrossingExecutionCaps& execution, double decision_dt_s,
 double evaluation_stamp_s, CrossingPathFeasibility path_feasible = {}) {
 return active_coordinator().assess_crossing(route, path_index, ego_position,
 ego_speed_mps, ego_heading_rad, predicted_frames, frame_dt_s, safety,
 execution, decision_dt_s, evaluation_stamp_s, std::move(path_feasible));
 }
 EntryWindowDecision assess_entry_window(const EntryWindowRequest& request) {
 return active_coordinator().assess_entry_window(request);
 }
 bool entry_governor_holding() const { return active_coordinator().entry_governor_holding(); }
 const CrossingExecutionFeedback& crossing_execution_feedback() const {
 return active_coordinator().crossing_execution_feedback();
 }
 void report_crossing_execution(bool committed, const TrajectoryValidation& validation,
 std::uint64_t trajectory_sequence, double stamp_s) {
 active_coordinator().report_crossing_execution(committed, validation, trajectory_sequence, stamp_s);
 }
 TrajectoryTrackingReport observe_trajectory(double time_s, Vec2 position,
 double heading_rad, double forward_mps) const {
 return active_coordinator().observe_trajectory(time_s, position, heading_rad, forward_mps);
 }
 std::uint64_t accept_trajectory(double generated_at_s, const ExecutableTrajectory& trajectory,
 const TrajectoryTrackingLimits& limits) {
 return active_coordinator().accept_trajectory(generated_at_s, trajectory, limits);
 }
 bool update_controlled_stop(bool carrying_stop, double peak_wheel_speed_mps,
 const ControlledStopLatch::Config& config) {
 return active_coordinator().update_controlled_stop(carrying_stop, peak_wheel_speed_mps, config);
 }
 void reset_controlled_stop() { active_coordinator().reset_controlled_stop(); }
 bool static_motion_free(Vec2 from, Vec2 to, double execution_margin_m = 0.0) const {
 return active_coordinator().collision_monitor().static_motion_free(from, to, execution_margin_m);
 }
 double static_clearance(Vec2 position) const {
 return active_coordinator().collision_monitor().static_clearance(position);
 }
 autonomy::CollisionAssessment assess_collision(const autonomy::CollisionObservation& observation) {
 return active_coordinator().collision_monitor().assess(observation);
 }
 void configure_collision_monitor(Vec2 world_size, double robot_radius_m,
 std::vector<autonomy::CollisionBox> static_obstacles) {
 active_coordinator().configure_collision_monitor(world_size, robot_radius_m,
 std::move(static_obstacles));
 }
 double safe_stop_deceleration_limit(const ChassisCaps& chassis, const SafeStopLimits& limits) const {
 return active_coordinator().safe_stop_deceleration_limit(chassis, limits);
 }
 DepartureAssessment assess_departure(const std::vector<Vec2>& path, std::size_t start_idx,
 double cruise_mps, const std::vector<std::vector<DynamicObjectState>>& frames,
 double frame_dt_s, double ego_radius_m, double margin_m) const {
 return sim::assess_departure(path, start_idx, cruise_mps, frames, frame_dt_s,
 ego_radius_m, margin_m);
 }
 PocketSearch sample_pockets(const GridMap& inflated_map, const std::vector<Vec2>& route,
 Vec2 start, double start_theta, double cruise_mps, const ChassisExecCaps& exec,
 double hold_s, const PocketSampleParams& params,
 const std::vector<std::vector<DynamicObjectState>>& frames, double frame_dt_s,
 double ego_radius_m, double margin_m) const {
 return sim::sample_pockets(inflated_map, route, start, start_theta, cruise_mps,
 exec, hold_s, params, frames, frame_dt_s, ego_radius_m, margin_m);
 }
 SafeStopCommand stage_safe_stop(const ChassisCaps& chassis, const SafeStopMotion& motion,
 const SafeStopLimits& limits) const {
 return sim::stage_safe_stop(chassis, motion, limits);
 }
 void reset_chassis_controllers() { active_coordinator().reset_chassis_controllers(); }
 void configure_differential_controller(DifferentialWheelControllerConfig config, WheelPidGains gains) {
 active_coordinator().configure_differential_controller(std::move(config), gains);
 }
 void configure_ackermann_controller(AckermannActuatorControllerConfig config, WheelPidGains gains) {
 active_coordinator().configure_ackermann_controller(std::move(config), gains);
 }
 void configure_holonomic_controller(HolonomicActuatorControllerConfig config, WheelPidGains gains) {
 active_coordinator().configure_holonomic_controller(std::move(config), gains);
 }
 bool has_differential_controller() const { return active_coordinator().has_differential_controller(); }
 bool has_ackermann_controller() const { return active_coordinator().has_ackermann_controller(); }
 bool has_holonomic_controller() const { return active_coordinator().has_holonomic_controller(); }

 RobotExecutive& executive() { return executive_; }
 const RobotExecutive& executive() const { return executive_; }

 private:
 std::unique_ptr<runtime::IRuntimeChannel> channel_;
 runtime::IRuntimeChannel* borrowed_channel_{nullptr};
 runtime::ProviderConformanceMonitor conformance_;
 autonomy::AutonomyCoordinator coordinator_;
 std::unique_ptr<autonomy::AutonomyCoordinator> owned_coordinator_;
 autonomy::AutonomyCoordinator* borrowed_coordinator_{nullptr};
 RobotExecutive executive_;
 runtime::IMapProvider* map_provider_{nullptr};
 runtime::MapCatalog* map_catalog_{nullptr};
 runtime::MapManager* active_map_manager_{nullptr};
 RasterTraversalModel traversal_;
 std::shared_ptr<const runtime::MapSnapshot> traversal_map_;

 runtime::IRuntimeChannel& active_channel();
 const runtime::IRuntimeChannel& active_channel() const;
 autonomy::AutonomyCoordinator& active_coordinator();
 const autonomy::AutonomyCoordinator& active_coordinator() const;
 void refresh_traversal_map();
};

} // namespace sim::robot
