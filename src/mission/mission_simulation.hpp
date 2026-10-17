#pragma once

#include "content_catalog.hpp"
#include "component_registry.hpp"
#include "autonomy/control/route_tracking_system.hpp"
#include "mission/systems/collision_monitor.hpp"
#include "mission/systems/dynamic_traffic_system.hpp"
#include "mission/systems/deterministic_scheduler.hpp"
#include "mission/systems/decision_evidence_system.hpp"
#include "autonomy/pipeline/autonomy_pipeline.hpp"
#include "mission/systems/field_dispatch_system.hpp"
#include "mission/systems/field_operator_system.hpp"
#include "mission/systems/goal_system.hpp"
#include "autonomy/planning/global/global_planner.hpp"
#include "mission/systems/world_activity_system.hpp"
#include "sim/simulated_platform.hpp"
#include "mission/world/actor_behavior_registry.hpp"
#include "autonomy/behavior/entry_governor.hpp"
#include "autonomy/behavior/crossing_behavior.hpp"
#include "autonomy/planning/validation/entry_shadow.hpp"
#include "hw/chassis/diff_drive.hpp"
#include "autonomy/planning/local/dwa.hpp"
#include "autonomy/planning/local/ackermann_local.hpp"
#include "autonomy/planning/local/holonomic_local.hpp"
#include "autonomy/behavior/high_level_policy.hpp"
#include "core/event_log.hpp"
#include "autonomy/control/follow.hpp"
#include "autonomy/localization/estimation_system.hpp"
#include "autonomy/mapping/terrain.hpp"
#include "hw/sensors/sensor_system.hpp"
#include "physics/frame_transform.hpp"
#include "autonomy/mapping/grid_map.hpp"
#include "autonomy/localization/slip_detect.hpp"
#include "hw/chassis/motor_drive.hpp"
#include "autonomy/control/pid.hpp"
#include "hw/sensors/sensor.hpp"
#include "runtime/runtime_channel.hpp"

#include <map>
#include <cstdint>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace mission {

enum class MissionStatus { Ready, Running, Succeeded, Failed };

// Bullet is the production pose integrator and the default. Domain keeps the
// deterministic 2.5D reference model for explicit comparison, while Shadow
// runs the same wheel forces through Bullet without letting it drive gameplay.
// Auto resolves to Bullet because Bullet is a required build dependency.
enum class PhysicsMode { Domain, BulletShadow, Bullet, Auto };

PhysicsMode parse_physics_mode(const std::string& value);
const char* physics_mode_name(PhysicsMode mode);

struct MissionEvent {
 double time_s{0.0};
 std::string code;
 std::string message;
};

// Gate-3 shadow wait episode (structured): one waiting interval with the
// first/last departure assessments and the actual waited time. reasons code
// sim::DepartReason (0=GoNow..5=Invalid). Stored alongside the event log so
// integrity (open/close pairing, field ranges) and distribution (counts and
// durations by reason) are machine-checkable, not parsed from prose.
struct ShadowEpisode {
 double start_s{0.0};
 double first_pred_s{-1.0};
 int first_reason{-1};
 double end_s{0.0};
 double last_pred_s{-1.0};
 int last_reason{-1};
 double waited_s{0.0};
 // Gate-B stratification: fastest known threat and nearest clearance at wait
 // start (max |v| over active tracks, meters and m/s, -1 = uncaptured).
 double entry_threat_speed_mps{-1.0};
 double entry_nearest_m{-1.0};
};

struct ActorSnapshot {
 std::string id;
 sim::Vec2 position;
 sim::Vec2 velocity;
 double radius_m{0.0};
 bool detected{false};
 std::string visual_kind{"vehicle"};
 // Stable presentation heading. It follows authored motion and is held at
 // zero-speed dwells, so renderers never derive yaw from noisy scan-to-scan
 // velocity estimates.
 double heading_rad{0.0};
 // Visual self-description copied from the actor registry (content): the
 // renderer poses this model, never branches on agent kinds.
 std::string visual_model;
 double visual_ref_radius_m{0.45};
};

struct DynamicObstacleSnapshot {
 std::string id;
 sim::Vec2 estimated_position;
 sim::Vec2 estimated_velocity;
 double radius_m{0.0};
 double last_observed_s{0.0};
 double learned_endpoint_dwell_s{0.0};
 std::string visual_kind{"vehicle"};
 double heading_rad{0.0};
};

struct LocalTrajectorySnapshot {
 std::vector<sim::Vec2> points;
 bool collision{false};
 double score{0.0};
};

struct MountActuationSnapshot {
 std::string mount_id;
 double yaw_rad{0.0};
 double roll_rad{0.0};
 // Articulated physics supplies the complete base_link_T_child transform.
 // This includes caster-trail translation and supersedes yaw/roll synthesis.
 std::optional<sim::physics::RigidTransform> base_from_link;
};

struct ControlSnapshot {
 bool dwa_active{false};
 sim::Vec2 local_target;
 std::vector<LocalTrajectorySnapshot> dwa_candidates;
 std::vector<sim::Vec2> dwa_selected;
 double dwa_proposed_v_mps{0.0};
 double dwa_proposed_w_radps{0.0};
 std::size_t dwa_accepted_candidates{0};
 // Exact local trajectory contract. The validator checks the same timed
 // rollout whose command is handed to control; tracking compares the next
 // estimate against that accepted rollout.
 bool trajectory_validated{false};
 int trajectory_verdict{-1};
 double trajectory_min_dynamic_clearance_m{std::numeric_limits<double>::infinity()};
 std::uint64_t trajectory_sequence{0};
 bool trajectory_tracking_active{false};
 bool trajectory_within_envelope{false};
 double trajectory_position_error_m{0.0};
 double trajectory_heading_error_rad{0.0};
 double trajectory_speed_error_mps{0.0};
 // Shadow departure assessment (gate 3): observed only, never acted on.
 // shadow_reason codes sim::DepartReason for the departure verdict
 // (assess_departure never emits kUnassessed/kClear), -1 = unassessed.
 bool shadow_assessed{false};
 double shadow_depart_s{-1.0};
 int shadow_reason{-1};
 // Authored conflict-zone hypothesis. Shadow authority publishes evidence;
 // control authority additionally permits the crossing state machine to
 // constrain motion after its executable-trajectory gate is satisfied.
 bool crossing_assessed{false};
 bool crossing_control_authority{false};
 std::string crossing_zone_id;
 sim::Vec2 crossing_stop{0, 0};
 double crossing_wait_s{-1.0};
 int crossing_reason{-1};
 int crossing_phase{0};
 bool crossing_committed{false};
 bool crossing_ready_to_commit{false};
 bool crossing_safe_to_commit{false};
 bool crossing_position_ready{false};
 bool crossing_speed_ready{false};
 bool crossing_heading_ready{false};
 double crossing_stop_longitudinal_m{0.0};
 double crossing_stop_lateral_m{0.0};
 double crossing_stop_distance_m{0.0};
 double crossing_stop_heading_error_rad{0.0};
 bool crossing_prediction_evidence_available{false};
 bool crossing_threats_moving_away{false};
 sim::Vec2 crossing_exit{0, 0};
 double crossing_transit_speed_mps{0.0};
 double crossing_execution_duration_s{0.0};
 double crossing_execution_min_gap_m{std::numeric_limits<double>::infinity()};
 double crossing_execution_required_gap_m{0.0};
 // Spatial pocket shadow (Gate B): last sampling while waiting. verdict codes
 // sim::SearchVerdict (0=FoundFeasible/1=AllRefused/2=Incomplete), -1 = unsampled.
 int shadow_pocket_verdict{-1};
 int shadow_pocket_feasible{0};
 int shadow_pocket_total{0};
 // Entry governor (Phase-2, Behavior one-sided brake): hold flag + stop
 // point issued this step. Absent hold means DWA decides, never "go".
 bool governor_hold{false};
 sim::Vec2 governor_stop{0, 0};
 // Gate-B reason distribution: counts of the three leg reasons per candidate
 // (transit+hold+rejoin), bins code sim::DepartReason 0..8. Sum == 3*total.
 int shadow_pocket_reasons[10]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
 double heading_error_rad{0.0};
 double heading_pid_p{0.0};
 double heading_pid_i{0.0};
 double heading_pid_d{0.0};
 double target_v_mps{0.0};
 double target_w_radps{0.0};
 double left_target_mps{0.0};
 double right_target_mps{0.0};
 double left_measured_mps{0.0};
 double right_measured_mps{0.0};
 double left_target_rpm{0.0};
 double right_target_rpm{0.0};
 double left_measured_rpm{0.0};
 double right_measured_rpm{0.0};
 double left_pwm{0.0};
 double right_pwm{0.0};
 double left_speed_pid_p{0.0};
 double left_speed_pid_i{0.0};
 double left_speed_pid_d{0.0};
 double right_speed_pid_p{0.0};
 double right_speed_pid_i{0.0};
 double right_speed_pid_d{0.0};
 // Ackermann only: drive-unit values mirror into the left_* fields above and
 // the servo state lands here. Differential leaves these zero (never fiction).
 double steer_target_rad{0.0};
 double steer_angle_rad{0.0};
 double steer_pwm{0.0};
 bool steering_servo_present{false};
 // Generic actuator transforms already bound to robot mount ids. Renderers
 // never branch on chassis type or infer steering from body motion.
 std::vector<MountActuationSnapshot> mount_actuations;
};

struct PayloadPickup {
 std::string goal_id;
 double time_s{0.0};
 int item_count{0};
 double mass_kg{0.0};
 double cog_height_m{0.0};
 double value{0.0};
 double fragility{0.0};
};

// Authoritative runtime load state. Content describes the starting load and
// collectible trays; simulation owns their accumulated mass and first moment.
// Replay snapshots copy this value so a seek cannot show a different plant
// from the one that produced the recorded motion.
struct PayloadState {
 double mass_kg{0.0};
 double first_moment_kgm{0.0};
 double cog_height_m{0.0};
 int item_count{0};
 double value{0.0};
 double collected_mass_kg{0.0};
 double fragility_mass_sum{0.0};
 double integrity{1.0};
 double max_shock_mps2{0.0};
 double max_jerk_mps3{0.0};
 std::vector<PayloadPickup> history;
};

struct EnvironmentState {
 double wetness{0.0};
 double visibility{1.0};
 double daylight{1.0};
 std::size_t keyframe_index{0};
 std::size_t next_keyframe_index{0};
 double blend{0.0};
};

struct MissionSnapshot {
 double time_s{0.0};
 sim::DiffState robot;
 sim::Vec2 estimated_position;
 double estimated_heading{0.0};
 std::vector<ActorSnapshot> actors;
 std::vector<DynamicObstacleSnapshot> dynamic_obstacles;
 std::vector<sim::Vec2> lidar_hits;
 ControlSnapshot control;
 bool static_obstacle_detected{false};
 bool dynamic_object_detected{false};
 bool waiting_for_clearance{false};
 bool via_reached{false};
 // Number completed. Autonomous task order is represented separately by the
 // stable active/completed/abandoned ids and must never use this as an index.
 std::size_t goals_reached{0};
 std::size_t active_goal_index{std::numeric_limits<std::size_t>::max()};
 std::string active_goal_id;
 std::vector<std::string> completed_goal_ids;
 std::vector<std::string> abandoned_goal_ids;
 PayloadState payload;
 EnvironmentState environment;
 MissionStatus status{MissionStatus::Ready};
 double robot_height_m{0.0};
 double robot_roll_rad{0.0};
 double robot_pitch_rad{0.0};
 double terrain_grade{0.0};
 double cross_slope{0.0};
 double rollover_risk{0.0};
 bool traction_saturated{false};
 // Onboard-observable estimate from encoder/IMU disagreement. Autonomy may
 // consume this; traction_saturated above remains simulator plant truth.
 bool traction_slip_observed{false};
 // Bullet-shadow comparison. Zero in Domain mode; in Bullet authority these
 // describe the live rigid body and shadow_error_* stay zero.
 double shadow_position_error_m{0.0};
 double shadow_heading_error_rad{0.0};
 double shadow_height_error_m{0.0};
 int rigid_contact_count{0};
 std::vector<std::string> ground_contact_links;
 double max_drive_slip_ratio{0.0};
 // Regional conditions are copied into every replay frame. Physics, sensing,
 // rendering and UI therefore describe the same instant when replay seeks.
 WorldActivityState world_activity;
 sim::StrategyDecision strategy;
 std::string strategy_policy{"route-baseline"};
 DecisionEvidence decision;
};

struct MissionResult {
 MissionStatus status{MissionStatus::Ready};
 std::string failure_reason;
 double time_s{0.0};
 double path_length_m{0.0};
 double energy_wh{0.0};
 double min_clearance_m{999.0};
 double scored_clearance_m{999.0};
 double min_static_clearance_m{999.0};
 double min_static_clearance_time_s{0.0};
 sim::Vec2 min_static_robot_position;
 double min_dynamic_clearance_m{999.0};
 double min_dynamic_clearance_time_s{0.0};
 sim::Vec2 min_dynamic_robot_position;
 sim::Vec2 min_dynamic_actor_position;
 double odometry_error_m{0.0};
 double max_odometry_error_m{0.0};
 double odometry_heading_error_rad{0.0};
 double max_odometry_heading_error_rad{0.0};
 // Navigation estimate in map. In odometry-only mode these equal the raw
 // odometry errors above; fused localization corrects map->odom while raw
 // odom->base_footprint remains continuous and may continue to drift.
 double fused_pose_error_m{0.0};
 double max_fused_pose_error_m{0.0};
 double fused_heading_error_rad{0.0};
 double max_fused_heading_error_rad{0.0};
 int collisions{0};
 int pedestrian_contacts{0};
 // Perception diagnostics compare anonymous tracks with truth after planning;
 // they never provide identity to the tracker or controller.
 long assoc_agree{0};
 long assoc_total{0};
 int slip_events{0};
 double max_drive_slip_ratio{0.0};
 int astar_expanded{0};
 int local_decisions{0};
 bool via_reached{false};
 bool safe_score{false};
 bool fast_score{false};
 bool efficient_score{false};
 double max_grade{0.0};
 double max_abs_roll_rad{0.0};
 double max_rollover_risk{0.0};
 double max_shadow_position_error_m{0.0};
 double max_shadow_heading_error_rad{0.0};
 double max_shadow_height_error_m{0.0};
 int rigid_contacts{0};
 double cargo_integrity{1.0};
 int collected_items{0};
 double collected_value{0.0};
 double final_payload_mass_kg{0.0};
 double final_payload_cog_height_m{0.0};
 double peak_wetness{0.0};
 double min_visibility{1.0};
};

// Named composition options keep extension points discoverable and avoid a
// positional constructor call when an application replaces one subsystem.
struct MissionSimulationOptions {
 std::optional<std::uint64_t> seed_override;
 std::vector<std::string> owned_abilities;
 ComponentRegistry component_registry{ComponentRegistry::with_builtins()};
 std::shared_ptr<const autonomy::IGlobalPlanner> global_planner;
 PhysicsMode physics_mode{PhysicsMode::Bullet};
 std::shared_ptr<sim::IHighLevelPolicy> high_level_policy;
 std::string contract_id;
 std::string runtime_provider_id{"channel.in-process"};
 sim::runtime::RuntimeRegistry runtime_registry{sim::runtime::RuntimeRegistry::with_builtins()};
};

class MissionSimulation {
 public:
 static constexpr double fixed_step_s = 0.02;

 MissionSimulation(const ContentCatalog& catalog, std::string level_id, BuildSelection build,
                   const MissionSimulationOptions& options);

 MissionSimulation(
   const ContentCatalog& catalog, std::string level_id, BuildSelection build,
   std::optional<uint64_t> seed_override = std::nullopt, const std::vector<std::string>& owned_abilities = {},
   const ComponentRegistry& component_registry = ComponentRegistry::builtins(),
   std::shared_ptr<const autonomy::IGlobalPlanner> global_planner = {},
   PhysicsMode physics_mode = PhysicsMode::Bullet,
   std::shared_ptr<sim::IHighLevelPolicy> high_level_policy = {}, std::string contract_id = {},
   std::string runtime_provider_id = "channel.in-process",
   sim::runtime::RuntimeRegistry runtime_registry = sim::runtime::RuntimeRegistry::with_builtins());

 void reset();
 void start();
 void step();
 void run_to_completion();
 void set_manual_command(double forward, double turn);
 void set_endless_mode(bool enabled);
 bool endless_mode() const { return endless_mode_; }
 void restore_endless_world(const WorldActivityCheckpoint& checkpoint);
 WorldActivityCheckpoint endless_world_checkpoint() const;
 void restore_field_expedition(const FieldExpeditionCheckpoint& checkpoint);
 FieldExpeditionCheckpoint field_expedition_checkpoint() const;
 const FieldDispatchState& field_dispatch() const { return field_dispatch_system_.state(); }
 const FieldOperatorState& field_operator() const { return field_operator_system_.state(); }
 const FieldIntentState& field_intent() const { return field_operator_system_.intent(); }
 bool set_field_operator_command(double world_x, double world_y);
 bool issue_field_intent(FieldIntentKind kind, std::optional<sim::Vec2> target = std::nullopt);
 const WorldActivityState& world_activity() const { return snapshot_.world_activity; }
 const GoalStep* active_goal() const {
  const auto index = goal_system_.active_goal_index();
  return index && *index < level_.objective.goals.size() ? &level_.objective.goals[*index] : nullptr;
 }

 const LevelDefinition& level() const { return level_; }
 const MapDefinition& map_definition() const { return map_definition_; }
 const BuildSelection& build() const { return build_; }
 const RobotDefinition& robot() const { return robot_definition_; }
 static const std::string& level_robot_id(const ContentCatalog& catalog, const std::string& level_id,
                                          const BuildSelection& build);
 uint64_t effective_seed() const { return seed_override_.value_or(level_.seed); }
 const sim::GridMap& occupancy_map() const { return *truth_map_; }
 const sim::GridMap& navigation_map() const { return *planning_map_; }
 const sim::TerrainModel& terrain_model() const { return terrain_; }
 const sim::TerrainCapability& terrain_capability() const { return terrain_capability_; }
 const sim::ChassisCaps& chassis_caps() const { return chassis_caps_; }
 std::optional<sim::physics::RigidTransform> lookup_transform(const std::string& target_frame,
                                                              const std::string& source_frame) const {
  return frame_tree_.lookup(target_frame, source_frame);
 }
 std::vector<std::string> frame_ids() const { return frame_tree_.frame_ids(); }
 double frame_timestamp_s() const { return estimation_system_.state().timestamp_s; }
 const std::vector<sim::RoutePoint>& planned_route() const { return planned_route_; }
 const MissionSnapshot& snapshot() const { return snapshot_; }
 const PayloadState& payload_state() const { return payload_; }
 const MissionResult& result() const { return result_; }
 const std::vector<MissionEvent>& events() const { return events_; }
 const std::vector<DecisionEvidence>& decisions() const {
 return decision_evidence_system_.records();
 }
 const std::vector<MissionSnapshot>& replay() const { return replay_; }
 void seek_replay(double time_s);
 const std::vector<ShadowEpisode>& shadow_episodes() const { return shadow_episodes_; }
 bool shadow_episode_open() const { return shadow_episode_open_; }
 int slip_episodes() const { return slip_detector_.episodes(); }
 const std::vector<double>& slip_episode_times() const { return slip_episode_times_; }
 const char* physics_backend_name() const;
 const std::string& runtime_provider_id() const { return runtime_provider_id_; }
 std::uint32_t runtime_api_version() const { return sim::runtime::kApiVersion; }
 sim::runtime::LifecycleState runtime_state() const {
  return runtime_channel_ ? runtime_channel_->state() : sim::runtime::LifecycleState::Unconfigured;
 }
 sim::runtime::RuntimeHealth runtime_health() const {
  return runtime_channel_ ? runtime_channel_->health() : sim::runtime::RuntimeHealth{};
 }
 const std::string& strategy_policy_name() const { return snapshot_.strategy_policy; }
 std::string_view global_planner_id() const { return planning_system_.planner().id(); }
 std::string_view local_planner_id() const {
  switch (chassis_caps_.command_space) {
  case sim::ChassisCommandSpace::SteeringAngle:
   return "ackermann-rollout";
  case sim::ChassisCommandSpace::HolonomicTwist:
   return "holonomic-rollout";
  case sim::ChassisCommandSpace::YawRate:
   return "dwa-yaw-rate";
  }
  return "unknown";
 }
 const ContractJob* active_contract() const { return active_contract_; }
 PhysicsMode physics_mode() const { return physics_mode_; }
 double navigation_traversal_cost(sim::Vec2 from, sim::Vec2 to) const;

 private:
 void build_world();
 void reset_payload();
 void collect_payload(const GoalStep& goal);
 bool update_payload_integrity(const sim::DiffState& before, const sim::DiffState& after,
                               double lateral_accel_mps2, double contact_impulse_Ns);
 void synchronize_payload_dynamics();
 void update_environment();
 double environment_grip_multiplier(const std::string& material) const;
 double total_mass_kg() const;
 double combined_center_height_m() const;
 double combined_yaw_inertia_kgm2() const;
 void select_next_goal(const Pose& start);
 void plan_route();
 void finish_route_failure();
 void update_actors();
 void scan_environment();
 void refresh_dynamic_obstacles();
 void decide_control();
 void apply_motion();
 void finish_success();
 void finish_failure(std::string reason, std::string message);
 void record_event(std::string code, std::string message, double throttle_s = 0.0);
 void publish_decision(DecisionEvidence decision);
 void close_decision(std::string outcome_code);
 void update_shadow_episode();
 void record_replay();
 double surface_traction(sim::Vec2 position) const;
 sim::Vec2 current_navigation_target();
 double decision_hz() const;
 double corner_speed_cap(double nav_x, double nav_y) const;
 double track_max_age_s() const;
 sim::GroundContact wheel_ground(sim::Vec2 position) const;
 void initialize_rigid_physics();
 void initialize_frame_tree();
 void update_frame_tree();
 void apply_goal_stop_constraint();
 void refresh_endless_route();

 const ContentCatalog& catalog_;
 std::string level_id_;
 BuildSelection build_;
 // Seed override for multi-seed studies. Empty = level JSON seed (the only
 // seed official baselines ever use). Effective seed fans out to all three
 // RNG streams with the same xor constants as before.
 std::optional<uint64_t> seed_override_;
 LevelDefinition level_;
 const MapDefinition& map_definition_;
 const RobotDefinition& robot_definition_;
 PhysicsMode requested_physics_mode_{PhysicsMode::Bullet};
 PhysicsMode physics_mode_{PhysicsMode::Bullet};
 sim::runtime::RuntimeRegistry runtime_registry_;
 std::string runtime_provider_id_;
 // Content is converted once into validated, unit-bearing runtime contracts.
 // BuildSelection remains the stable map used by saves and player packs.
 ResolvedBuild resolved_build_;
 bool was_saturated_{false};
 double rigid_rollover_exposure_s_{0.0};
 double previous_payload_shock_mps2_{0.0};
 double filtered_payload_shock_mps2_{0.0};
 bool payload_shock_initialized_{false};
 // Continuous driving style, piecewise-interpolated across the policy anchors.
 Properties behavior_;

 std::unique_ptr<sim::GridMap> truth_map_;
 std::unique_ptr<sim::SensorSystem> sensor_system_;
 std::unique_ptr<sim::AutonomyPipeline> autonomy_pipeline_;
 std::unique_ptr<sim::GridMap> collision_map_;
 std::unique_ptr<CollisionMonitor> collision_monitor_;
 std::unique_ptr<sim::GridMap> planning_map_;
 double planning_clearance_m_{0.0};
 std::unique_ptr<sim::DiffDrive> drive_;
 std::unique_ptr<sim::ChassisModel> chassis_;
 std::unique_ptr<sim::runtime::IRuntimeChannel> runtime_channel_;
 std::unique_ptr<sim::RouteTrackingSystem> route_tracking_system_;
 double heading_kp_{0.0};
 // Yaw-rate capability used by DWA/manual/clamps. Diff: DiffDrive w_max.
 // Ackermann: v*tan(steer_max)/base at top drive speed. Hardware, not tuned.
 double yaw_cap_rads_{2.0};
 double max_forward_speed_mps_{0.0};
 sim::EstimationSystem estimation_system_;
 sim::physics::FrameTransformTree frame_tree_;
 sim::TerrainModel terrain_;
 sim::TerrainCapability terrain_capability_;
 sim::ChassisCaps chassis_caps_;
 autonomy::GlobalPlanningSystem planning_system_;
 bool custom_global_planner_{false};
 std::unique_ptr<sim::platform::SimulatedPlatform> platform_;
 bool custom_high_level_policy_{false};
 std::shared_ptr<sim::IHighLevelPolicy> high_level_policy_;
 const ContractJob* active_contract_{nullptr};
 GoalSystem goal_system_;
 PayloadState payload_;
 // Wheel phase is simulation state and is copied into replay snapshots via
 // mount_actuations. Renderers never synthesize motion from wall-clock time.
 std::unordered_map<std::string, double> mount_roll_rad_;
 FieldDispatchSystem field_dispatch_system_;
 FieldOperatorSystem field_operator_system_;
 WorldActivitySystem world_activity_system_;
 std::vector<sim::RoutePoint> planned_route_;
 bool route_feasible_{true};
 bool return_reserve_failure_{false};
 // Persistent expedition mode: the same world, sensors and Bullet contacts
 // keep stepping without goal, timeout or collision termination. The player
 // moves the field operator and issues high-level intents; the robot always
 // drives through the autonomous planning and control stack.
 bool endless_mode_{false};
 std::string endless_target_key_{"uninitialized"};
 // One agent list for every behavior (no per-kind parallel arrays): kind
 // dispatches ping-pong vs loop evaluation; collision semantics and visuals
 // ride along from the actor's self-description. Dormant agents wait in
 // dormant_agents_ (invisible, non-collidable) until a summon goal moves one
 // here with t0 = now, so the encounter reproduces from that moment.
 struct SimAgent {
  std::string id;
  std::string kind{"patrol"};
  // Neutral behavior inputs (translated once from the actor registry entry);
  // the registry evaluates them, mission never branches on kinds.
  mission::ActorBehaviorParams behavior;
  bool social{false};
  std::string visual_kind{"vehicle"};
  std::string visual_model;
  double visual_ref_radius_m{0.45};
  double display_heading_rad{0.0};
  double t0{0.0};
 };
 std::vector<SimAgent> agents_;
 std::vector<SimAgent> dormant_agents_;
 std::vector<sim::DynamicObjectState> actual_actors_;

 MissionSnapshot snapshot_;
 MissionResult result_;
 std::vector<MissionEvent> events_;
 DecisionEvidenceSystem decision_evidence_system_;
 std::vector<ShadowEpisode> shadow_episodes_;
 std::vector<MissionSnapshot> replay_;
 // Shadow slip instrument (tracked only, see apply_motion): detector state
 // plus previous-step speeds. Observation only; no consumer yet (consumer
 // design is a behavior gate, not smuggled in here).
 sim::SlipDetector slip_detector_;
 std::vector<double> slip_episode_times_;
 double slip_prev_tread_v_{0.0};
 double slip_prev_body_v_{0.0};
 bool slip_have_prev_{false};
 sim::DiffCmd held_command_;
 // Shadow wait episode bookkeeping (gate 3): first predicted departure when
 // a wait opens, vs the last assessment and the actual waited time when it
 // closes. Observation only; no control path reads these.
 bool shadow_episode_open_{false};
 double shadow_episode_start_s_{0.0};
 double shadow_episode_first_s_{-1.0};
 int shadow_episode_first_reason_{-1};
 double shadow_episode_last_s_{-1.0};
 int shadow_episode_last_reason_{-1};
 double shadow_episode_entry_speed_{-1.0};
 double shadow_episode_entry_nearest_{-1.0};
 // Staging for the next episode open (filled in the DWA branch where tracks
 // are visible, consumed by update_shadow_episode on the rising edge).
 double shadow_open_speed_{-1.0};
 double shadow_open_nearest_{-1.0};
 // Entry governor state (Phase-2 Behavior brake): hysteresis lives here,
 // reset every run like every other per-run state below.
 sim::EntryGovernorSystem entry_governor_system_;
 DynamicTrafficSystem dynamic_traffic_system_;
 sim::CrossingBehaviorSystem crossing_behavior_system_;
 DeterministicScheduler scheduler_;
 double last_event_time_s_{-1000.0};
 std::string last_event_code_;
 std::uint64_t motion_sequence_{0};
 std::uint64_t command_sequence_{0};
 bool manual_control_active_{false};
 sim::DiffCmd manual_command_;
};

const char* mission_status_name(MissionStatus status);

} // namespace mission
