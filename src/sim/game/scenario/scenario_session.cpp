#include "scenario_session.hpp"
#include "sim/game/mission/deployable_map_adapter.hpp"
#include "robot/autonomy/planning/task/task_ordering.hpp"

#include "robot/autonomy/planning/global/astar.hpp"
#include "robot/autonomy/planning/validation/trajectory_validator.hpp"
#include "sim/game/mission/world/actor_behavior_registry.hpp"
#include "sim/hardware/vehicle/chassis_factory.hpp"
#include "robot/autonomy/localization/slip_detect.hpp"
#include "sim/hardware/physics/lidar_raycast.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace mission {
namespace {

constexpr double kBuiltInDecisionHz = 10.0;
constexpr double kDynamicTrackMaxAgeS = 1.25;

sim::Vec2 as_vec(const Pose& pose) {
 return {pose.x, pose.y};
}

double distance(sim::Vec2 a, sim::Vec2 b) {
 return std::hypot(a.x - b.x, a.y - b.y);
}

bool dispatch_has_priority(const FieldDispatchState& dispatch, const WorldIncidentState& incident) {
 if (!dispatch.active)
  return false;
 return !incident.active || dispatch.opened_at_s <= incident.opened_at_s;
}

bool inside(sim::Vec2 point, const SurfaceDefinition& box) {
 return point.x >= box.min.x && point.x < box.max.x && point.y >= box.min.y && point.y < box.max.y;
}

std::string field_kind_label(const ContentCatalog& catalog, const std::string& kind) {
 if (kind == "delivery")
  return catalog.tr("field.kind.delivery");
 if (kind == "assist")
  return catalog.tr("field.kind.assist");
 return catalog.tr("field.kind.survey");
}

std::string weather_label(const ContentCatalog& catalog, WeatherKind weather) {
 switch (weather) {
 case WeatherKind::Clear:
  return catalog.tr("weather.clear");
 case WeatherKind::Overcast:
  return catalog.tr("weather.overcast");
 case WeatherKind::Drizzle:
  return catalog.tr("weather.drizzle");
 case WeatherKind::Fog:
  return catalog.tr("weather.fog");
 }
 return catalog.tr("weather.clear");
}

std::string incident_label(const ContentCatalog& catalog, WorldIncidentKind incident) {
 switch (incident) {
 case WorldIncidentKind::RoadClosure:
  return catalog.tr("incident.road_closure");
 case WorldIncidentKind::FallenCargo:
  return catalog.tr("incident.fallen_cargo");
 case WorldIncidentKind::PowerFault:
  return catalog.tr("incident.power_fault");
 }
 return catalog.tr("incident.road_closure");
}

} // namespace

const char* mission_status_name(MissionStatus status) {
 switch (status) {
 case MissionStatus::Ready:
  return "ready";
 case MissionStatus::Running:
  return "running";
 case MissionStatus::Succeeded:
  return "succeeded";
 case MissionStatus::Failed:
  return "failed";
 }
 return "unknown";
}

PhysicsMode parse_physics_mode(const std::string& value) {
 if (value == "domain")
  return PhysicsMode::Domain;
 if (value == "shadow")
  return PhysicsMode::BulletShadow;
 if (value == "bullet")
  return PhysicsMode::Bullet;
 if (value == "auto")
  return PhysicsMode::Auto;
 throw std::invalid_argument("unknown physics mode '" + value + "' (expected domain|shadow|bullet|auto)");
}

const char* physics_mode_name(PhysicsMode mode) {
 switch (mode) {
 case PhysicsMode::Domain:
  return "domain";
 case PhysicsMode::BulletShadow:
  return "bullet-shadow";
 case PhysicsMode::Bullet:
  return "bullet";
 case PhysicsMode::Auto:
  return "auto";
 }
 return "unknown";
}

const std::string& ScenarioSession::level_robot_id(const ContentCatalog& catalog,
                                                     const std::string& level_id,
                                                     const BuildSelection& build) {
 // Validated player choice wins (validate_build enforces allow-list + unlock);
 // empty request keeps the presentation default / companion path bit-identical.
 if (!build.chassis_robot.empty()) {
  return build.chassis_robot;
 }
 const std::string& presentation_robot = catalog.presentation_for_level(level_id).robot_id;
 return presentation_robot.empty() ? catalog.campaign().companion_robot : presentation_robot;
}

ScenarioSession::ScenarioSession(const ContentCatalog& catalog, std::string level_id,
                                     BuildSelection build, const ScenarioSessionOptions& options)
    : ScenarioSession(catalog, std::move(level_id), std::move(build), options.seed_override,
                        options.owned_abilities, options.component_registry, options.global_planner,
                        options.physics_mode, options.high_level_policy, options.contract_id,
                        options.runtime_provider_id, options.runtime_registry) {}

ScenarioSession::ScenarioSession(
  const ContentCatalog& catalog, std::string level_id, BuildSelection build,
  std::optional<uint64_t> seed_override, const std::vector<std::string>& owned_abilities,
  const ComponentRegistry& component_registry, std::shared_ptr<const autonomy::IGlobalPlanner> global_planner,
  PhysicsMode physics_mode, std::shared_ptr<sim::IHighLevelPolicy> high_level_policy, std::string contract_id,
  std::string runtime_provider_id, sim::runtime::RuntimeRegistry runtime_registry)
    : catalog_(catalog), level_id_(std::move(level_id)), build_(std::move(build)),
      seed_override_(seed_override), level_(catalog_.level(level_id_)),
      map_definition_(catalog_.map(level_.map)),
      robot_definition_(catalog_.robot(level_robot_id(catalog_, level_id_, build_))),
      requested_physics_mode_(physics_mode),
      physics_mode_(physics_mode == PhysicsMode::Auto ? PhysicsMode::Bullet : physics_mode),
      runtime_registry_(std::move(runtime_registry)), runtime_provider_id_(std::move(runtime_provider_id)),
      behavior_(catalog_.style_behavior(build_.style)), estimation_system_(effective_seed() ^ 0x6a09e667U),
      planning_system_(global_planner ? global_planner : autonomy::default_global_planner()),
      custom_global_planner_(static_cast<bool>(global_planner)),
      custom_high_level_policy_(static_cast<bool>(high_level_policy)),
      high_level_policy_(high_level_policy ? std::move(high_level_policy)
                                           : sim::make_route_baseline_policy()) {
 catalog_.validate_build(level_id_, build_, owned_abilities);
 if (!build_.task_policy.empty())
  level_.objective.task_policy = build_.task_policy;
 if (!contract_id.empty()) {
  const auto contract = std::find_if(catalog_.contracts().begin(), catalog_.contracts().end(),
                                     [&](const ContractJob& job) { return job.id == contract_id; });
  if (contract == catalog_.contracts().end() || contract->level != level_id_)
   throw ContentError("contract '" + contract_id + "' does not target level '" + level_id_ + "'");
  active_contract_ = &*contract;
 }
 std::string map_error;
 if (!map_manager_.configure(make_deployable_map_snapshot(map_definition_, level_id_), &map_error) ||
 !map_manager_.activate()) {
 throw ContentError("cannot configure robot map: " + map_error);
 }
 resolved_build_ = component_registry.resolve(catalog_, build_);
 reset();
}

void ScenarioSession::reset() {
 if (runtime_channel_ && runtime_channel_->state() == sim::runtime::LifecycleState::Active) {
  runtime_channel_->deactivate();
 }
 result_ = {};
 snapshot_ = {};
 update_environment();
 mount_roll_rad_.clear();
 reset_payload();
 high_level_policy_->reset(effective_seed() ^ 0x1f83d9abU);
 snapshot_.strategy_policy = high_level_policy_->name();
 lidar_delivery_queue_.clear();
 events_.clear();
 decision_evidence_system_.reset();
 replay_.clear();
 shadow_episodes_.clear();
 shadow_episode_open_ = false;
 shadow_episode_first_s_ = -1.0;
 shadow_episode_first_reason_ = -1;
 shadow_episode_last_s_ = -1.0;
 shadow_episode_last_reason_ = -1;
 held_command_ = {};
 manual_command_ = {};
 manual_control_active_ = false;
 endless_target_key_ = "uninitialized";
 goal_system_.reset(level_.objective, map_manager_);
 snapshot_.goals_reached = goal_system_.goals_reached();
 snapshot_.completed_goal_ids = goal_system_.completed_goal_ids(level_.objective);
 snapshot_.abandoned_goal_ids = goal_system_.abandoned_goal_ids(level_.objective);
 snapshot_.payload = payload_;
 field_dispatch_system_.reset(effective_seed() ^ 0xbb67ae85U);
 world_activity_system_.reset(effective_seed() ^ 0x510e527fU);
 snapshot_.world_activity = world_activity_system_.state();
 const auto* scheduled_lidar = resolved_build_.lidar();
 scheduler_.reset(scheduled_lidar ? 1.0 / scheduled_lidar->scan_hz : 0.0,
 1.0 / decision_hz(), 0.04);
 motion_sequence_ = 0;
 actuator_feedback_sequence_ = 0;
 command_sequence_ = 0;
 last_event_time_s_ = -1000.0;
 last_event_code_.clear();
 entry_governor_system_.reset();
 crossing_behavior_system_.reset();
 was_saturated_ = false;
 rigid_rollover_exposure_s_ = 0.0;
 previous_payload_shock_mps2_ = 0.0;
 filtered_payload_shock_mps2_ = 0.0;
 payload_shock_initialized_ = false;
 build_world();
 initialize_rigid_physics();
 synchronize_payload_dynamics();
 plan_route();

 const auto& spawn = map_definition_.markers.at(level_.objective.spawn);
 snapshot_.robot = {spawn.x, spawn.y, spawn.theta, 0.0, 0.0};
 field_operator_system_.reset(map_definition_, {spawn.x, spawn.y}, level_id_);
 const auto spawn_terrain = terrain_.sample({spawn.x, spawn.y});
 if (platform_) {
  const auto physical = platform_->state();
  snapshot_.robot_height_m = physical.height_m;
  snapshot_.robot_pitch_rad = physical.pitch_rad;
  snapshot_.robot_roll_rad = physical.roll_rad;
 } else {
  snapshot_.robot_height_m = spawn_terrain.height_m + robot_definition_.base_link_height_m;
  snapshot_.robot_pitch_rad = -std::atan(sim::longitudinal_grade(spawn_terrain, spawn.theta));
  snapshot_.robot_roll_rad = std::atan(sim::lateral_grade(spawn_terrain, spawn.theta));
 }
 snapshot_.estimated_position = {spawn.x, spawn.y};
 snapshot_.estimated_heading = spawn.theta;
 snapshot_.status = MissionStatus::Ready;
 estimation_system_.reset(spawn.x, spawn.y, spawn.theta, effective_seed() ^ 0x6a09e667U);
 initialize_frame_tree();
 if (sensor_system_) {
 sensor_system_->reset(effective_seed(), effective_seed() ^ 0x9e3779b9U);
 }
 if (autonomy_pipeline_) {
 autonomy_pipeline_->reset(effective_seed() ^ 0x9e3779b9U);
 }
 update_actors();
 slip_detector_.reset();
 slip_episode_times_.clear();
 slip_prev_tread_v_ = 0.0;
 slip_prev_body_v_ = 0.0;
 slip_have_prev_ = false;
 record_replay();
}

void ScenarioSession::reset_payload() {
 payload_ = {};
 auto add_initial_load = [this](double mass_kg, double cog_height_m) {
  if (mass_kg <= 0.0)
   return;
  payload_.mass_kg += mass_kg;
  payload_.first_moment_kgm += mass_kg * std::max(0.0, cog_height_m);
 };
 add_initial_load(level_.payload_mass_kg, level_.payload_cog_height_m);
 if (active_contract_)
  add_initial_load(active_contract_->mass_kg, active_contract_->cog_height_m);
 payload_.cog_height_m = payload_.mass_kg > 1e-9 ? payload_.first_moment_kgm / payload_.mass_kg : 0.0;
}

void ScenarioSession::update_environment() {
 const auto& keyframes = level_.environment_keyframes;
 if (keyframes.empty()) {
  snapshot_.environment = {};
  return;
 }
 const auto upper = std::upper_bound(
   keyframes.begin(), keyframes.end(), snapshot_.time_s,
   [](double time_s, const EnvironmentKeyframeDefinition& keyframe) { return time_s < keyframe.time_s; });
 const std::size_t next =
   upper == keyframes.end() ? keyframes.size() - 1 : static_cast<std::size_t>(upper - keyframes.begin());
 const std::size_t current = upper == keyframes.end() ? next : (next == 0 ? 0 : next - 1);
 const auto& from = keyframes[current];
 const auto& to = keyframes[next];
 const double duration_s = to.time_s - from.time_s;
 const double blend =
   duration_s > 1e-9 ? std::clamp((snapshot_.time_s - from.time_s) / duration_s, 0.0, 1.0) : 0.0;
 const auto lerp = [blend](double a, double b) { return a + (b - a) * blend; };
 snapshot_.environment = {
   lerp(from.wetness, to.wetness),
   lerp(from.visibility, to.visibility),
   lerp(from.daylight, to.daylight),
   current,
   next,
   blend,
 };
 result_.peak_wetness = std::max(result_.peak_wetness, snapshot_.environment.wetness);
 result_.min_visibility = std::min(result_.min_visibility, snapshot_.environment.visibility);
}

double ScenarioSession::environment_grip_multiplier(const std::string& material) const {
 const auto& keyframes = level_.environment_keyframes;
 if (keyframes.empty())
  return 1.0;
 const auto value = [&](std::size_t index) {
  const auto found = keyframes[index].material_grip.find(material);
  return found == keyframes[index].material_grip.end() ? 1.0 : found->second;
 };
 const double from = value(snapshot_.environment.keyframe_index);
 const double to = value(snapshot_.environment.next_keyframe_index);
 return from + (to - from) * snapshot_.environment.blend;
}

double ScenarioSession::total_mass_kg() const {
 return robot_definition_.mass_kg + payload_.mass_kg;
}

double ScenarioSession::combined_center_height_m() const {
 return std::max(
   0.05, (robot_definition_.mass_kg * robot_definition_.center_of_mass_height_m + payload_.first_moment_kgm) /
           std::max(0.5, total_mass_kg()));
}

double ScenarioSession::combined_yaw_inertia_kgm2() const {
 return robot_definition_.yaw_inertia_kgm2 +
        payload_.mass_kg * robot_definition_.collision_radius_m * robot_definition_.collision_radius_m;
}

void ScenarioSession::synchronize_payload_dynamics() {
 const sim::ChassisBodyDynamics dynamics{total_mass_kg(), combined_yaw_inertia_kgm2()};
 if (chassis_)
  chassis_->set_body_dynamics(dynamics);
 if (platform_)
  platform_->update_load(dynamics.mass_kg, combined_center_height_m(), dynamics.yaw_inertia_kgm2);
 terrain_capability_.mass_kg = dynamics.mass_kg;
 const double support_width = chassis_caps_.geometry.support_width_m;
 terrain_capability_.max_cross_slope_rad = std::atan2(support_width * 0.5, combined_center_height_m()) * 0.98;
 snapshot_.payload = payload_;
 result_.collected_items = payload_.item_count;
 result_.collected_value = payload_.value;
 result_.final_payload_mass_kg = payload_.mass_kg;
 result_.final_payload_cog_height_m = payload_.cog_height_m;
}

void ScenarioSession::collect_payload(const GoalStep& goal) {
 if (goal.action != "collect" || goal.item_count <= 0 || goal.item_mass_kg <= 0.0)
  return;
 const std::string id = goal.id.empty() ? goal.marker : goal.id;
 payload_.mass_kg += goal.item_mass_kg;
 payload_.first_moment_kgm += goal.item_mass_kg * goal.item_cog_height_m;
 payload_.cog_height_m = payload_.first_moment_kgm / payload_.mass_kg;
 payload_.item_count += goal.item_count;
 payload_.value += goal.item_value;
 const double previous_collected_mass = payload_.collected_mass_kg;
 payload_.integrity = (payload_.integrity * previous_collected_mass + goal.item_mass_kg) /
                      (previous_collected_mass + goal.item_mass_kg);
 payload_.collected_mass_kg += goal.item_mass_kg;
 payload_.fragility_mass_sum += goal.item_mass_kg * goal.item_fragility;
 payload_.history.push_back({id, snapshot_.time_s, goal.item_count, goal.item_mass_kg, goal.item_cog_height_m,
                             goal.item_value, goal.item_fragility});
 synchronize_payload_dynamics();
}

bool ScenarioSession::update_payload_integrity(const sim::DiffState& before, const sim::DiffState& after,
                                                 double lateral_accel_mps2, double contact_impulse_Ns) {
 if (payload_.collected_mass_kg <= 1e-9)
  return true;
 const double forward_accel_mps2 = (after.v - before.v) / fixed_step_s;
 const double raw_shock_mps2 = std::hypot(forward_accel_mps2, lateral_accel_mps2);
 // Fruit sits in a tray rather than at an ideal rigid-body point. A 120 ms
 // first-order mount response rejects fixed-step contact chatter while keeping
 // braking, bumps and collisions in the measured cargo acceleration.
 const double filter_alpha = 1.0 - std::exp(-fixed_step_s / 0.12);
 filtered_payload_shock_mps2_ += (raw_shock_mps2 - filtered_payload_shock_mps2_) * filter_alpha;
 const double shock_mps2 = filtered_payload_shock_mps2_;
 const double jerk_mps3 =
   payload_shock_initialized_ ? std::abs(shock_mps2 - previous_payload_shock_mps2_) / fixed_step_s : 0.0;
 previous_payload_shock_mps2_ = shock_mps2;
 payload_shock_initialized_ = true;
 payload_.max_shock_mps2 = std::max(payload_.max_shock_mps2, shock_mps2);
 payload_.max_jerk_mps3 = std::max(payload_.max_jerk_mps3, jerk_mps3);

 const double fragility = std::clamp(payload_.fragility_mass_sum / payload_.collected_mass_kg, 0.0, 1.0);
 const double shock_excess = std::max(0.0, shock_mps2 - 4.0);
 const double jerk_excess = std::max(0.0, jerk_mps3 - 24.0);
 const double roll_excess = std::max(0.0, snapshot_.rollover_risk - 0.55);
 double damage = fragility * fixed_step_s *
                 (0.001 * shock_excess * shock_excess + 0.00002 * jerk_excess + 0.02 * roll_excess);
 damage += fragility * 0.008 * std::max(0.0, contact_impulse_Ns - 4.0);
 payload_.integrity = std::clamp(payload_.integrity - damage, 0.0, 1.0);
 snapshot_.payload = payload_;
 result_.cargo_integrity = payload_.integrity;
 return payload_.integrity >= 0.25;
}

const char* ScenarioSession::physics_backend_name() const {
 return physics_mode_name(physics_mode_);
}

void ScenarioSession::initialize_rigid_physics() {
 platform_.reset();
 if (physics_mode_ == PhysicsMode::Domain) {
  return;
 }
 const auto& spawn = map_definition_.markers.at(level_.objective.spawn);
 platform_ = std::make_unique<sim::platform::SimulatedPlatform>(
   map_definition_, level_, robot_definition_, chassis_caps_, terrain_, sim::Vec2{spawn.x, spawn.y},
   spawn.theta, total_mass_kg(), combined_center_height_m(), combined_yaw_inertia_kgm2());
}

void ScenarioSession::initialize_frame_tree() {
 frame_tree_.clear();
 constexpr double kDegreesToRadians = sim::kPi / 180.0;
 for (const auto& mount : robot_definition_.mounts) {
  frame_tree_.set_transform("base_link", mount.frame_id,
                            {
                              {mount.position_m.x, mount.position_m.y, mount.position_m.z},
                              sim::physics::quaternion_from_euler(mount.rotation_deg.x * kDegreesToRadians,
                                                                  mount.rotation_deg.y * kDegreesToRadians,
                                                                  mount.rotation_deg.z * kDegreesToRadians),
                            });
 }
 if (!frame_tree_.lookup("base_link", "imu_link")) {
  frame_tree_.set_transform("base_link", "imu_link", {});
 }
 update_frame_tree();
}

void ScenarioSession::update_frame_tree() {
 const auto map_from_odom = estimation_system_.map_from_odom_translation();
 frame_tree_.set_transform(
   "map", "odom",
   {
     {map_from_odom.x, map_from_odom.y, 0.0},
     sim::physics::quaternion_from_euler(0.0, 0.0, estimation_system_.map_from_odom_heading_rad()),
   });
 const auto odom_position = estimation_system_.odometry_position();
 const auto estimated_ground = terrain_.sample(estimation_system_.state().position);
 frame_tree_.set_transform(
   "odom", "base_footprint",
   {
     {odom_position.x, odom_position.y, estimated_ground.height_m},
     sim::physics::quaternion_from_euler(0.0, 0.0, estimation_system_.odometry_heading_rad()),
   });
 const double base_height = snapshot_.robot_height_m - estimated_ground.height_m;
 frame_tree_.set_transform(
   "base_footprint", "base_link",
   {
     {0.0, 0.0, base_height},
     sim::physics::quaternion_from_euler(snapshot_.robot_roll_rad, snapshot_.robot_pitch_rad, 0.0),
   });

 constexpr double kDegreesToRadians = sim::kPi / 180.0;
 for (const auto& actuation : snapshot_.control.mount_actuations) {
  const auto mount = std::find_if(robot_definition_.mounts.begin(), robot_definition_.mounts.end(),
                                  [&](const auto& candidate) { return candidate.id == actuation.mount_id; });
  if (mount == robot_definition_.mounts.end())
   continue;
  frame_tree_.set_transform(
    "base_link", mount->frame_id,
    {
      actuation.base_from_link
        ? actuation.base_from_link->translation_m
        : sim::physics::Vec3{mount->position_m.x, mount->position_m.y, mount->position_m.z},
      actuation.base_from_link
        ? actuation.base_from_link->rotation
        : sim::physics::quaternion_multiply(
            sim::physics::quaternion_from_euler(0.0, 0.0, actuation.yaw_rad),
            sim::physics::quaternion_multiply(
              sim::physics::quaternion_from_euler(mount->rotation_deg.x * kDegreesToRadians,
                                                  mount->rotation_deg.y * kDegreesToRadians,
                                                  mount->rotation_deg.z * kDegreesToRadians),
              sim::physics::quaternion_from_euler(0.0, 0.0, actuation.roll_rad))),
    });
 }
}

void ScenarioSession::build_world() {
 const int width = static_cast<int>(std::ceil(map_definition_.size_m.x / map_definition_.grid_resolution_m));
 const int height = static_cast<int>(std::ceil(map_definition_.size_m.y / map_definition_.grid_resolution_m));
 truth_map_ = std::make_unique<sim::GridMap>(width, height, map_definition_.grid_resolution_m);
 std::vector<CollisionBox> collision_obstacles;
 for (const auto& obstacle : map_definition_.obstacles) {
  if (active_in_level(obstacle.active_in, level_id_)) {
   truth_map_->addBox(obstacle.min.x, obstacle.min.y, obstacle.max.x, obstacle.max.y);
 collision_obstacles.push_back({obstacle.min, obstacle.max});
  }
 }
 sensor_system_ = std::make_unique<sim::SensorSystem>(
 effective_seed(), effective_seed() ^ 0x9e3779b9U);
 collision_map_ = std::make_unique<sim::GridMap>(truth_map_->inflated(robot_definition_.collision_radius_m));
 collision_monitor_ = std::make_unique<CollisionMonitor>(
 map_definition_.size_m, robot_definition_.collision_radius_m,
 std::move(collision_obstacles));
 std::vector<sim::HeightPatch> active_terrain;
 for (const auto& patch : map_definition_.terrain) {
  if (!active_in_level(patch.active_in, level_id_)) {
   continue;
  }
  active_terrain.push_back({
    patch.id,
    patch.min,
    patch.max,
    patch.base_height_m,
    patch.grade_x,
    patch.grade_y,
    patch.roughness,
  });
 }
 terrain_ = sim::TerrainModel(std::move(active_terrain));

 sim::PlanarDiffDriveParams params;
 params.max_forward_mps = std::max(1.5, property(behavior_, "cruise_mps") * 1.25);
 params.max_acceleration_mps2 = resolved_build_.tires().brake_decel_mps2;
 params.track_width_m = robot_definition_.wheel_base_m;
 drive_ = std::make_unique<sim::PlanarDiffDrivePlant>(params);
 // Chassis dispatch: drive.type selects the actuator model. Unknown types
 // fail loudly in content, never silently fall back to another chassis.
 // The equipped motor component overrides the robot default; an empty motor
 // slot (early levels) runs the robot stock motor. New motors stay plug-and-play
 // because PID gains are derived from these same numbers.
 const auto* equipped_motor = resolved_build_.motor();
 const double motor_vmax =
   equipped_motor ? equipped_motor->max_wheel_speed_mps : robot_definition_.motor.max_wheel_speed_mps;
 max_forward_speed_mps_ = motor_vmax;
 const double motor_stall =
   equipped_motor ? equipped_motor->stall_force_N : robot_definition_.motor.stall_force_N;
 const double motor_free =
   equipped_motor ? equipped_motor->free_speed_mps : robot_definition_.motor.free_speed_mps;
 const sim::WheelControllerDesign controller_design{
   equipped_motor ? equipped_motor->controller_bandwidth_radps
                  : robot_definition_.motor.controller_bandwidth_radps,
   equipped_motor ? equipped_motor->controller_integral_time_s
                  : robot_definition_.motor.controller_integral_time_s,
   equipped_motor ? equipped_motor->controller_derivative_time_s
                  : robot_definition_.motor.controller_derivative_time_s,
 };
 const double simulation_mass = total_mass_kg();
 double wheel_mass = std::max(0.05, robot_definition_.motor.wheel_equiv_mass_kg);
 if (robot_definition_.drive_type == "differential") {
  // A wheel-speed loop accelerates the wheel/gear train plus the chassis.
  // Common mode reflects M/2 into each wheel; differential mode reflects
  // 2*Iz/track^2. Tuning only against rotor inertia makes yaw response far
  // slower than the stated closed-loop bandwidth.
  const double common_mode_mass = simulation_mass * 0.5;
  const double differential_mode_mass =
    2.0 * combined_yaw_inertia_kgm2() / (robot_definition_.wheel_base_m * robot_definition_.wheel_base_m);
  wheel_mass += std::max(common_mode_mass, differential_mode_mass);
 }
 const sim::WheelPidGains wheel_gains =
   sim::tune_wheel_pid(wheel_mass, motor_stall, motor_free, 0.5 * motor_vmax, controller_design);
 // Chassis construction goes through the factory registry (hw/chassis):
 // new drive.type implementations register themselves; mission only
 // translates robot JSON + equipped specs into neutral build params.
 sim::ChassisBuildParams chassis_params;
 chassis_params.wheel_base_m = robot_definition_.wheel_base_m;
 chassis_params.wheel_radius_m = robot_definition_.wheel_radius_m;
 chassis_params.track_width_m = robot_definition_.track_width_m;
 chassis_params.track_length_m = robot_definition_.track_length_m;
 chassis_params.collision_radius_m = robot_definition_.collision_radius_m;
 chassis_params.max_wheel_speed_mps = motor_vmax;
 chassis_params.max_wheel_accel_mps2 = robot_definition_.motor.max_wheel_accel_mps2;
 chassis_params.stall_force_N = motor_stall;
 chassis_params.free_speed_mps = motor_free;
 chassis_params.pwm_limit = robot_definition_.motor.pwm_limit;
 chassis_params.integral_limit = robot_definition_.motor.integral_limit;
 chassis_params.wheel_equiv_mass_kg = robot_definition_.motor.wheel_equiv_mass_kg;
 chassis_params.coulomb_smoothing_speed_mps = equipped_motor
                                                ? equipped_motor->coulomb_smoothing_speed_mps
                                                : robot_definition_.motor.coulomb_smoothing_speed_mps;
 chassis_params.viscous_drag_N_per_mps =
   equipped_motor ? equipped_motor->viscous_drag_N_per_mps : robot_definition_.motor.viscous_drag_N_per_mps;
 chassis_params.encoder_counts_per_revolution = robot_definition_.encoder.counts_per_revolution;
 chassis_params.mass_kg = simulation_mass;
 chassis_params.yaw_inertia_kgm2 = combined_yaw_inertia_kgm2();
 chassis_params.max_steer_rad = robot_definition_.steering.max_steer_rad;
 chassis_params.max_steer_rate_radps = robot_definition_.steering.max_steer_rate_radps;
 chassis_params.servo_response_s = robot_definition_.steering.servo_response_s;
 chassis_params.steering_integral_limit = robot_definition_.steering.integral_limit;
 chassis_params.tracked_max_yaw_rate_rads = robot_definition_.tracked_max_yaw_rate_rads;
 chassis_params.behavior_w_max = params.max_yaw_rate_radps;
 chassis_params.behavior_accel_max = params.max_acceleration_mps2;
 chassis_params.max_yaw_accel_radps2 = robot_definition_.max_yaw_accel_radps2;
 // Resolve the support footprint from the assembled rolling contacts. Robot
 // data is the source of truth for where wheels touch; collision radius is a
 // body envelope and must not masquerade as a stability footprint.
 std::vector<const MountDefinition*> rolling_contacts;
 const auto tire_slot_definition =
   std::find_if(robot_definition_.slots.begin(), robot_definition_.slots.end(),
                [](const auto& slot) { return slot.slot == "tires"; });
 if (tire_slot_definition != robot_definition_.slots.end()) {
  double drive_axle_x_sum = 0.0;
  std::size_t drive_wheel_count = 0;
  for (const auto& mount_id : tire_slot_definition->mounts) {
   const auto mount = std::find_if(robot_definition_.mounts.begin(), robot_definition_.mounts.end(),
                                   [&](const auto& candidate) { return candidate.id == mount_id; });
   if (mount == robot_definition_.mounts.end())
    continue;
   rolling_contacts.push_back(&*mount);
   if (robot_definition_.drive_type == "differential" && mount->motion == "wheel") {
    drive_axle_x_sum += mount->position_m.x;
    ++drive_wheel_count;
   }
  }
  if (drive_wheel_count > 0)
   chassis_params.drive_axle_x_m = drive_axle_x_sum / static_cast<double>(drive_wheel_count);
 }
 for (const auto& support : robot_definition_.passive_supports) {
  const auto mount = std::find_if(robot_definition_.mounts.begin(), robot_definition_.mounts.end(),
                                  [&](const auto& candidate) { return candidate.id == support.steer_mount; });
  if (mount != robot_definition_.mounts.end()) {
   rolling_contacts.push_back(&*mount);
  }
 }
 if (rolling_contacts.size() >= 2) {
  double min_x = rolling_contacts.front()->position_m.x;
  double max_x = min_x;
  double min_y = rolling_contacts.front()->position_m.y;
  double max_y = min_y;
  for (const auto* mount : rolling_contacts) {
   min_x = std::min(min_x, mount->position_m.x);
   max_x = std::max(max_x, mount->position_m.x);
   min_y = std::min(min_y, mount->position_m.y);
   max_y = std::max(max_y, mount->position_m.y);
  }
  if (max_x - min_x > 0.05)
   chassis_params.support_length_m = max_x - min_x;
  if (max_y - min_y > 0.05)
   chassis_params.support_width_m = max_y - min_y;
 }
 // Builtin registration is explicit at the composition root (idempotent):
 // archive linkers drop unreferenced self-registering objects, so static
 // registrars are platform luck. Custom types: register_creator + JSON type.
 sim::ChassisFactory::register_builtin_chassis();
 mission::ActorBehaviorRegistry::register_builtin_behaviors();
 sim::BuiltChassis built =
   sim::ChassisFactory::create(robot_definition_.drive_type, chassis_params, wheel_gains);
 chassis_ = std::move(built.model);
 chassis_caps_ = chassis_->caps();
 yaw_cap_rads_ = chassis_caps_.max_yaw_rate_radps;

 differential_wheel_controller_.reset();
 const auto has_actuator_channel = [&](const std::string& channel) {
 return std::any_of(robot_definition_.articulation.actuators.begin(),
 robot_definition_.articulation.actuators.end(),
 [&](const auto& actuator) { return actuator.channel == channel; });
 };
 const bool deployable_actuator_control =
 physics_mode_ != PhysicsMode::Domain &&
 robot_definition_.drive_type == "differential" &&
 chassis_->accepts_normalized_duty() &&
 has_actuator_channel("left") && has_actuator_channel("right");
 if (deployable_actuator_control) {
 differential_wheel_controller_ =
 std::make_unique<sim::DifferentialWheelController>(
 sim::DifferentialWheelControllerConfig{
 chassis_params.wheel_base_m,
 chassis_params.wheel_radius_m,
 chassis_params.max_wheel_speed_mps,
 chassis_params.encoder_counts_per_revolution,
 chassis_params.pwm_limit,
 chassis_params.integral_limit,
 },
 wheel_gains);
 }

 // LSRI composition root. The mission uses both ends because it contains the
 // simulator and the autonomy loop in one process. A hardware runner gives
 // the platform end to a native driver or bridge instead.
 sim::runtime::RuntimeChannelConfig runtime_config;
 runtime_config.instance_id = robot_definition_.id;
 runtime_config.capabilities.chassis = chassis_caps_;
 runtime_config.capabilities.max_forward_speed_mps = motor_vmax;
 runtime_config.capabilities.max_lateral_speed_mps =
   chassis_caps_.command_space == sim::ChassisCommandSpace::HolonomicTwist ? motor_vmax : 0.0;
 if (differential_wheel_controller_) {
 runtime_config.capabilities.command_level =
 sim::runtime::PlatformCommandLevel::Actuator;
 runtime_config.capabilities.actuator_channels = {
 {"left", sim::runtime::ActuatorCommandMode::NormalizedDuty,
 -chassis_params.pwm_limit, chassis_params.pwm_limit, 0.0},
 {"right", sim::runtime::ActuatorCommandMode::NormalizedDuty,
 -chassis_params.pwm_limit, chassis_params.pwm_limit, 0.0},
 };
 }
 runtime_config.maximum_command_ttl_s = 0.25;
 runtime_channel_ = runtime_registry_.create(runtime_provider_id_, runtime_config);
 runtime_channel_->configure();
 if (!custom_global_planner_) {
  planning_system_ = autonomy::GlobalPlanningSystem(
    chassis_caps_.command_space == sim::ChassisCommandSpace::SteeringAngle
      ? autonomy::ackermann_global_planner(chassis_caps_.geometry.steering_wheel_base_m,
                                           chassis_caps_.geometry.max_steer_rad,
                                           map_definition_.grid_resolution_m)
      : autonomy::default_global_planner());
 }
 // The global costmap reserves the physical envelope plus a small numerical
 // margin. Policy clearance belongs to the local planner, which can trade it
 // against narrow passages and moving objects continuously. Baking the whole
 // policy distance into a coarse binary grid erased valid goal regions and
 // made a nominally cautious build unable to dock.
 const double planning_clearance = robot_definition_.collision_radius_m + 0.20 +
                                   chassis_caps_.planning_clearance_margin_m +
                                   level_.planning_localization_margin_m;
 planning_clearance_m_ = planning_clearance;
 planning_map_ = std::make_unique<sim::GridMap>(truth_map_->inflated(planning_clearance));
 const auto* mapping_lidar = resolved_build_.lidar();
 const double mapping_period_s = mapping_lidar ? 1.0 / mapping_lidar->scan_hz : 0.1;
 autonomy_pipeline_ = std::make_unique<sim::AutonomyPipeline>(
 *truth_map_, width, height, map_definition_.grid_resolution_m,
 effective_seed() ^ 0x9e3779b9U,
 sim::ObservedObstacleLayerConfig{
 std::max(0.30, 3.0 * mapping_period_s),
 mapping_period_s,
 });
 // Heading loop auto-tune: gain normalized by speed and wheelbase so the same
 // rule serves every DIY assembly. Kp = w_max / (2 * cruise * base) reproduces
 // the validated RO-1 feel; the integral/derivative ratios are method constants
 // (slow integrator kills steady-state error, short derivative damps).
 const double cruise = cruise_speed_mps();
 const double heading_kp =
   yaw_cap_rads_ / (2.0 * std::max(0.2, cruise) * std::max(0.1, robot_definition_.wheel_base_m));
 // PID output units follow the chassis (follow.hpp contract): yaw rate for
 // differential, steer angle for ackermann. Same derivation, capped by what
 // the actuator takes.
 // Differential and tracked share yaw-rate output units; their caps differ
 // (geometry vs vendor spin spec) and both already sit in yaw_cap_rads_.
 const double heading_out_lim = chassis_caps_.turn_command_limit();
 route_tracking_system_ =
   std::make_unique<sim::RouteTrackingSystem>(heading_kp, heading_kp / 125.0, heading_kp * 0.05, heading_out_lim,
                                   robot_definition_.steering.integral_limit);
 heading_kp_ = heading_kp;

 agents_.clear();
 dormant_agents_.clear();
 for (const auto& actor : map_definition_.actors) {
  if (!active_in_level(actor.active_in, level_id_) || actor.path.size() < 2) {
   continue;
  }
  SimAgent agent;
  agent.id = actor.id;
  agent.kind = actor.kind;
  agent.social = actor.collision_social;
  agent.behavior.points.clear();
  for (const auto& point : actor.path)
   agent.behavior.points.push_back(point);
  agent.behavior.waits = actor.waits;
  agent.behavior.speed_mps = actor.speed_mps;
  agent.behavior.radius_m = actor.radius_m;
  // Phase seed rides free on the id hash (no content edits): wander ignores
  // it, seeded-wander desynchronizes shared loops with it.
  agent.behavior.seed = mission::hash_actor_id(actor.id);
  agent.visual_kind = actor.visual_kind;
  agent.visual_model = actor.visual_model;
  agent.visual_ref_radius_m = actor.visual_ref_radius_m;
  if (agent.behavior.points.size() > 1) {
   const sim::Vec2 delta{agent.behavior.points[1].x - agent.behavior.points[0].x,
                         agent.behavior.points[1].y - agent.behavior.points[0].y};
   agent.display_heading_rad = std::atan2(delta.y, delta.x);
  }
  if (actor.dormant_until_summoned) {
   dormant_agents_.push_back(agent);
  } else {
   agents_.push_back(agent);
  }
 }
 actual_actors_.assign(agents_.size(), {});
}

void ScenarioSession::plan_route() {
 planned_route_.clear();
 entry_governor_system_.reset();
 crossing_behavior_system_.reset();
 route_feasible_ = true;
 return_reserve_failure_ = false;
 if (snapshot_.status == MissionStatus::Ready)
  result_.astar_expanded = 0;
 const auto& spawn = map_definition_.markers.at(level_.objective.spawn);
 const Pose start = snapshot_.status == MissionStatus::Ready
                      ? spawn
                      : Pose{snapshot_.robot.x, snapshot_.robot.y, snapshot_.robot.theta};
 const auto* equipped_motor = resolved_build_.motor();
 const double stall_force =
   equipped_motor ? equipped_motor->stall_force_N : robot_definition_.motor.stall_force_N;
 const double drive_units = chassis_caps_.drive_force_units;
 const double support_width = chassis_caps_.geometry.support_width_m;
 terrain_capability_ = {};
 terrain_capability_.mass_kg = total_mass_kg();
 terrain_capability_.available_drive_force_N = stall_force * drive_units;
 terrain_capability_.friction_mu = resolved_build_.tires().friction_mu;
 terrain_capability_.max_cross_slope_rad = std::atan2(support_width * 0.5, combined_center_height_m()) * 0.98;

 select_next_goal(start);
 const GoalStep* next = active_goal();
 if (!next) {
  route_feasible_ = false;
  return;
 }
 const auto active_index = goal_system_.active_goal_index();
 snapshot_.active_goal_index = active_index.value_or(std::numeric_limits<std::size_t>::max());
 snapshot_.active_goal_id = next->id.empty() ? next->marker : next->id;
 const auto& goal = map_definition_.markers.at(next->marker);
 std::vector<autonomy::PlanningPose> waypoints{{as_vec(start), start.theta},
                                               {as_vec(goal), goal.theta, next->radius_m}};
 auto plan = planning_system_.plan(*planning_map_, waypoints, [this](sim::Vec2 from, sim::Vec2 to) {
  return navigation_traversal_cost(from, to);
 });
 result_.astar_expanded += plan.expanded;
 if (!plan.ok) {
  route_feasible_ = false;
  return;
 }
 planned_route_ = std::move(plan.path);
 route_tracking_system_->reset();
}

void ScenarioSession::select_next_goal(const Pose& start) {
 if (goal_system_.active_goal_index())
  return;
 const auto eligible = goal_system_.eligible_goals(level_.objective);
 if (eligible.empty())
  return;

 std::optional<std::size_t> best_index;
 double best_score = std::numeric_limits<double>::infinity();
 std::string best_id;
 std::vector<std::size_t> reserve_rejected;
 bool terminal_out_of_time = false;
 const auto terminal = std::find_if(level_.objective.goals.begin(), level_.objective.goals.end(),
                                    [](const GoalStep& goal) { return goal.action == "complete"; });
 struct RouteCost {
  double distance_m{0.0};
  double traversal_cost{0.0};
  double worst_multiplier{1.0};
 };
 const auto route_cost = [this](const std::vector<sim::RoutePoint>& path) {
  RouteCost cost;
  for (std::size_t point = 1; point < path.size(); ++point) {
   const auto from = path[point - 1].position;
   const auto to = path[point].position;
   const double multiplier = navigation_traversal_cost(from, to);
   const double segment_distance = distance(from, to);
   cost.distance_m += segment_distance;
   cost.traversal_cost += segment_distance * multiplier;
   cost.worst_multiplier = std::max(cost.worst_multiplier, multiplier);
  }
  return cost;
 };
 const auto ordering_algorithms = autonomy::TaskOrderingRegistry::with_builtins();
 const auto& ordering_policy = catalog_.task_policy(level_.objective.task_policy);
 for (const std::size_t index : eligible) {
  const auto& candidate = level_.objective.goals[index];
  const auto& marker = map_definition_.markers.at(candidate.marker);
  const auto plan = planning_system_.plan(
    *planning_map_, {{as_vec(start), start.theta}, {as_vec(marker), marker.theta, candidate.radius_m}},
    [this](sim::Vec2 from, sim::Vec2 to) { return navigation_traversal_cost(from, to); });
  result_.astar_expanded += plan.expanded;
  if (!plan.ok || plan.path.empty())
   continue;
  const auto candidate_cost = route_cost(plan.path);
 // Traversal cost already includes terrain slowdown. Applying another 25%
 // speed discount double-counts the same loss and can abandon feasible work
 // while more than the authored return reserve remains. The reserve itself
 // carries schedule uncertainty; convert weighted route length at nominal
 // cruise here.
 const double planning_speed = std::max(0.3, cruise_speed_mps());
  if (candidate.action != "complete" && level_.objective.return_reserve_s > 0.0 &&
      terminal != level_.objective.goals.end()) {
   const auto& home = map_definition_.markers.at(terminal->marker);
   const auto return_plan = planning_system_.plan(
     *planning_map_, {{as_vec(marker), marker.theta}, {as_vec(home), home.theta, terminal->radius_m}},
     [this](sim::Vec2 from, sim::Vec2 to) { return navigation_traversal_cost(from, to); });
   result_.astar_expanded += return_plan.expanded;
   if (!return_plan.ok || return_plan.path.empty())
    continue;
   const double return_cost = route_cost(return_plan.path).traversal_cost;
   const double expected_finish_s = snapshot_.time_s +
                                    (candidate_cost.traversal_cost + return_cost) / planning_speed +
                                    level_.objective.return_reserve_s;
   if (expected_finish_s > level_.objective.time_limit_s) {
    reserve_rejected.push_back(index);
    continue;
   }
  }
  if (candidate.action == "complete" && level_.objective.return_reserve_s > 0.0 &&
      snapshot_.time_s + candidate_cost.traversal_cost / planning_speed > level_.objective.time_limit_s) {
   terminal_out_of_time = true;
   continue;
  }
  double score = candidate_cost.traversal_cost;
  if (candidate.action != "complete") {
   score = ordering_algorithms.score(ordering_policy.algorithm, ordering_policy.parameters,
                                     {candidate_cost.distance_m, candidate_cost.traversal_cost,
                                      candidate_cost.worst_multiplier, candidate.item_value});
  }
  const std::string id = candidate.id.empty() ? candidate.marker : candidate.id;
  if (!best_index || score < best_score - 1e-9 || (std::abs(score - best_score) <= 1e-9 && id < best_id)) {
   best_index = index;
   best_score = score;
   best_id = id;
  }
 }
 if (best_index)
  goal_system_.activate(level_.objective, *best_index);
 else if (!reserve_rejected.empty()) {
  for (const std::size_t index : reserve_rejected) {
   if (!goal_system_.abandon(level_.objective, index))
    continue;
   const auto& goal = level_.objective.goals[index];
   record_event("task-abandoned", catalog_.trf("event.task_abandoned", {goal.label}));
  }
  snapshot_.abandoned_goal_ids = goal_system_.abandoned_goal_ids(level_.objective);
  select_next_goal(start);
 } else if (terminal_out_of_time) {
  return_reserve_failure_ = true;
 }
}

void ScenarioSession::finish_route_failure() {
 if (return_reserve_failure_) {
  finish_failure("return-reserve", catalog_.tr("event.return_reserve_failure"));
 } else {
  finish_failure("no-feasible-route", catalog_.tr("event.no_feasible_route"));
 }
}

void ScenarioSession::start() {
 if (snapshot_.status != MissionStatus::Ready) {
  return;
 }
 if (!runtime_channel_ || runtime_channel_->state() != sim::runtime::LifecycleState::Inactive) {
  throw std::logic_error("mission runtime channel is not ready for activation");
 }
 runtime_channel_->activate();
 snapshot_.status = MissionStatus::Running;
 result_.status = MissionStatus::Running;
 DecisionEvidence initial_decision;
 initial_decision.observation_time_s = snapshot_.time_s;
 initial_decision.constraint_code = "none";
 initial_decision.reason_code = "mission-start";
 initial_decision.action_code = "plan-route";
 publish_decision(std::move(initial_decision));
 if (endless_mode_)
  refresh_endless_route();
 record_event(endless_mode_ ? "field-session-start" : "mission-start",
              endless_mode_ ? catalog_.tr("event.field_session_start") : catalog_.tr("event.mission_start"));
 if (!route_feasible_ && !endless_mode_) {
  finish_route_failure();
 }
}

void ScenarioSession::set_endless_mode(bool enabled) {
 if (snapshot_.status != MissionStatus::Ready)
  return;
 endless_mode_ = enabled;
 if (enabled) {
  planned_route_.clear();
  route_feasible_ = true;
  held_command_ = {};
  manual_command_ = {};
  manual_control_active_ = false;
  endless_target_key_ = "uninitialized";
  if (route_tracking_system_)
   route_tracking_system_->reset();
 }
}

bool ScenarioSession::set_field_operator_command(double world_x, double world_y) {
 if (!endless_mode_)
  return false;
 field_operator_system_.set_move_command({world_x, world_y});
 return true;
}

bool ScenarioSession::issue_field_intent(FieldIntentKind kind, std::optional<sim::Vec2> target) {
 if (!endless_mode_ || !field_operator_system_.issue_intent(kind, target))
  return false;
 manual_control_active_ = false;
 manual_command_ = {};
 endless_target_key_ = "uninitialized";
 refresh_endless_route();
 return true;
}

void ScenarioSession::restore_endless_world(const WorldActivityCheckpoint& checkpoint) {
 if (!endless_mode_ || snapshot_.status != MissionStatus::Ready)
  throw std::logic_error("regional world can only be restored before an endless session starts");
 world_activity_system_.restore(checkpoint);
 snapshot_.world_activity = world_activity_system_.state();
 // Refresh now so a persisted road incident participates in the first
 // Bullet/LiDAR frame instead of appearing one simulation tick late.
 update_actors();
 if (!replay_.empty())
  replay_.back() = snapshot_;
}

WorldActivityCheckpoint ScenarioSession::endless_world_checkpoint() const {
 if (!endless_mode_)
  throw std::logic_error("regional world checkpoint requested outside endless mode");
 return world_activity_system_.checkpoint(snapshot_.time_s);
}

void ScenarioSession::restore_field_expedition(const FieldExpeditionCheckpoint& checkpoint) {
 if (!endless_mode_ || snapshot_.status != MissionStatus::Ready)
  throw std::logic_error("field expedition can only be restored before an endless session starts");
 if (!checkpoint.level_id.empty() && checkpoint.level_id != level_id_)
  throw std::logic_error("field expedition checkpoint targets a different level");
 field_operator_system_.restore(checkpoint, map_definition_, level_id_);
 endless_target_key_ = "uninitialized";
}

FieldExpeditionCheckpoint ScenarioSession::field_expedition_checkpoint() const {
 if (!endless_mode_)
  throw std::logic_error("field expedition checkpoint requested outside endless mode");
 auto checkpoint = field_operator_system_.checkpoint();
 checkpoint.level_id = level_id_;
 return checkpoint;
}

void ScenarioSession::run_to_completion() {
 if (endless_mode_) {
  throw std::logic_error("endless simulation has no completion condition");
 }
 start();
 while (snapshot_.status == MissionStatus::Running) {
  step();
 }
}

void ScenarioSession::set_manual_command(double forward, double turn) {
 if (endless_mode_ || level_.control_mode != "manual_calibration")
  return;
 manual_control_active_ = true;
 const double cruise = cruise_speed_mps();
 manual_command_.target_v = std::clamp(forward, -0.65, 1.0) * cruise;
 manual_command_.target_w = std::clamp(turn, -1.0, 1.0) * chassis_caps_.turn_command_limit();
}

void ScenarioSession::step() {
 if (snapshot_.status != MissionStatus::Running) {
  return;
 }
 if (endless_mode_)
  field_operator_system_.update(fixed_step_s, map_definition_, level_id_);
 update_actors();
 if (scheduler_.scan_due(snapshot_.time_s)) {
  scan_environment();
 }
 deliver_ready_lidar_observations();
 if (manual_control_active_) {
  held_command_ = manual_command_;
 } else if (scheduler_.decision_due(snapshot_.time_s)) {
  decide_control();
 }
 if (!manual_control_active_ && !endless_mode_)
  apply_goal_stop_constraint();
 apply_motion();
 snapshot_.time_s += fixed_step_s;
 result_.time_s = snapshot_.time_s;

 const std::size_t previous_environment_stage = snapshot_.environment.keyframe_index;
 update_environment();
 if (!endless_mode_ && snapshot_.environment.keyframe_index != previous_environment_stage) {
  const auto& stage = level_.environment_keyframes[snapshot_.environment.keyframe_index];
  record_event("environment-changed", catalog_.trf("event.environment_changed", {stage.label}));
  plan_route();
  if (!route_feasible_) {
   finish_route_failure();
   return;
  }
 }

 if (endless_mode_) {
  const auto world_transitions =
    world_activity_system_.update(snapshot_.time_s, fixed_step_s, map_definition_,
                                  {snapshot_.robot.x, snapshot_.robot.y}, snapshot_.robot.v);
  snapshot_.world_activity = world_activity_system_.state();
  for (const auto& transition : world_transitions) {
   switch (transition.kind) {
   case WorldActivityTransitionKind::WeatherChanged:
    record_event("weather-changed",
                 catalog_.trf("event.weather_changed", {weather_label(catalog_, transition.to)}));
    break;
   case WorldActivityTransitionKind::IncidentOpened:
    record_event("incident-opened",
                 catalog_.trf("event.incident_opened", {incident_label(catalog_, transition.incident)}));
    break;
   case WorldActivityTransitionKind::IncidentResolved:
    record_event("incident-resolved",
                 catalog_.trf("event.incident_resolved", {incident_label(catalog_, transition.incident)}));
    break;
   }
  }
  const auto transitions =
    field_dispatch_system_.update(snapshot_.time_s, fixed_step_s, map_definition_,
                                  {snapshot_.robot.x, snapshot_.robot.y}, snapshot_.robot.v);
  for (const auto& transition : transitions) {
   switch (transition.kind) {
   case FieldDispatchTransitionKind::Opened:
    record_event(
      "field-dispatch-opened",
      catalog_.trf("event.field_dispatch_opened", {field_kind_label(catalog_, transition.dispatch_kind)}));
    break;
   case FieldDispatchTransitionKind::PickedUp:
    record_event("field-dispatch-pickup", catalog_.tr("event.field_dispatch_pickup"));
    break;
   case FieldDispatchTransitionKind::Completed:
    record_event(
      "field-dispatch-complete",
      catalog_.trf("event.field_dispatch_complete", {field_kind_label(catalog_, transition.dispatch_kind)}));
    break;
   }
  }
  refresh_endless_route();
 }

 const bool completion_allowed =
   level_.objective.type != "observe_and_stop" || snapshot_.static_obstacle_detected;
 const auto arrival =
   endless_mode_
     ? std::optional<GoalArrival>{}
 : goal_system_.update(level_.objective, {snapshot_.robot.x, snapshot_.robot.y},
                           snapshot_.robot.v, completion_allowed, fixed_step_s);
 if (arrival) {
  const auto& goal = arrival->goal;
  snapshot_.goals_reached = goal_system_.goals_reached();
  snapshot_.completed_goal_ids = goal_system_.completed_goal_ids(level_.objective);
  snapshot_.abandoned_goal_ids = goal_system_.abandoned_goal_ids(level_.objective);
  const auto active_index = goal_system_.active_goal_index();
  snapshot_.active_goal_index = active_index.value_or(std::numeric_limits<std::size_t>::max());
  snapshot_.active_goal_id = active_index ? (level_.objective.goals[*active_index].id.empty()
                                               ? level_.objective.goals[*active_index].marker
                                               : level_.objective.goals[*active_index].id)
                                          : std::string{};
  if (arrival->mission_complete) {
   finish_success();
  } else if (goal.action == "pickup") {
   snapshot_.via_reached = true;
   result_.via_reached = true;
   const GoalStep* next_goal = active_goal();
   const std::string next_label = next_goal ? next_goal->label : level_.objective.goal_label;
   record_event("via-reached", catalog_.trf("event.via_reached", {goal.label, next_label}));
  } else if (goal.action == "collect") {
   collect_payload(goal);
   record_event("item-collected",
                catalog_.trf("event.item_collected", {goal.label, std::to_string(goal.item_count)}));
  } else if (goal.action == "recharge") {
   // Pass-through goal: top up the battery account, keep driving. Net energy
   // never goes negative; the top-up amount is content, the accounting here.
   result_.energy_wh = std::max(0.0, result_.energy_wh - goal.recharge_wh);
   record_event("recharged", catalog_.trf("event.recharged", {goal.label}));
  } else if (goal.action == "summon") {
   // Pass-through goal: wake a dormant actor; its clock starts now, so the
   // encounter reproduces from this moment on every same-seed run.
   bool woke = false;
   for (std::size_t index = 0; index < dormant_agents_.size(); ++index) {
    if (dormant_agents_[index].id == goal.summon_actor) {
     SimAgent agent = dormant_agents_[index];
     agent.t0 = snapshot_.time_s;
     agents_.push_back(agent);
     dormant_agents_.erase(dormant_agents_.begin() + (std::ptrdiff_t)index);
     woke = true;
     break;
    }
   }
   // Full-literal trf keys (no dynamic key building): the content gate greps
   // string literals, constructed keys would slip past it.
   if (woke) {
    record_event("summoned", catalog_.trf("event.summoned_ok", {goal.label}));
   } else {
    record_event("summon-missed", catalog_.trf("event.summoned_miss", {goal.label}));
   }
  } else if (goal.action == "inspect") {
   record_event("inspection-complete", catalog_.trf("event.inspection_complete", {goal.label}));
  }
  if (!arrival->mission_complete && snapshot_.status == MissionStatus::Running) {
   plan_route();
   if (!route_feasible_)
    finish_route_failure();
  }
 }
 if (!endless_mode_ && snapshot_.status == MissionStatus::Running &&
     snapshot_.time_s >= level_.objective.time_limit_s) {
  finish_failure("timeout", catalog_.tr("event.timeout_failure"));
 }
 if (scheduler_.replay_due(snapshot_.time_s)) {
  record_replay();
  // The fixed simulation tick is 20 ms. A 40 ms replay cadence keeps every
  // seek gap within the UI's 50 ms contract without fabricating interpolated
  // states (50 ms would alias into occasional 60 ms gaps).
 }
}

void ScenarioSession::apply_goal_stop_constraint() {
 const GoalStep* goal = active_goal();
 if (!goal || goal->action != "inspect")
  return;
 const auto marker = map_definition_.markers.find(goal->marker);
 if (marker == map_definition_.markers.end())
  return;
 const double remaining = std::max(
   0.0, distance({snapshot_.robot.x, snapshot_.robot.y}, as_vec(marker->second)) - goal->radius_m * 0.55);
 const double deceleration = std::max(0.35, resolved_build_.tires().brake_decel_mps2 * 0.65);
 const double stop_cap = std::sqrt(2.0 * deceleration * remaining);
 if (remaining <= 0.04) {
  held_command_.target_v = 0.0;
  held_command_.target_w = 0.0;
 } else {
  held_command_.target_v = std::clamp(held_command_.target_v, -stop_cap, stop_cap);
 }
}

void ScenarioSession::update_actors() {
 // Rebuilt every step (agents are few; summoning changes the count mid-run,
 // so index-stable incremental writes would misalign after a summon).
 snapshot_.actors.clear();
 actual_actors_.assign(agents_.size(), {});
 for (std::size_t index = 0; index < agents_.size(); ++index) {
  auto& agent = agents_[index];
  // Behavior comes from the registry (string-dispatched pure functions);
  // adding a kind touches the registry, never this loop.
  const sim::DynamicObjectState state =
    mission::ActorBehaviorRegistry::evaluate(agent.kind, agent.behavior, snapshot_.time_s - agent.t0);
  actual_actors_[index] = state;
  // A two-point patrol represents a vehicle lane. Keep its authored lane
  // heading while it reverses, instead of snapping the model by 180 degrees
  // at each endpoint. Other agents face their true motion direction and
  // retain it while dwelling.
  if (agent.kind == "patrol" && agent.behavior.points.size() > 1) {
   const sim::Vec2 delta{agent.behavior.points[1].x - agent.behavior.points[0].x,
                         agent.behavior.points[1].y - agent.behavior.points[0].y};
   agent.display_heading_rad = std::atan2(delta.y, delta.x);
  } else if (std::hypot(state.v.x, state.v.y) > 1e-5) {
   agent.display_heading_rad = std::atan2(state.v.y, state.v.x);
  }
  snapshot_.actors.push_back({
    agent.id,
    state.p,
    state.v,
    state.radius,
    false,
    agent.visual_kind,
    agent.display_heading_rad,
    agent.visual_model,
    agent.visual_ref_radius_m,
  });
 }
 if (platform_) {
  std::vector<sim::platform::PhysicsActorState> actors;
  actors.reserve(actual_actors_.size());
  for (std::size_t index = 0; index < actual_actors_.size(); ++index) {
   actors.push_back({index < agents_.size() ? agents_[index].id : std::to_string(index),
                     actual_actors_[index].p, actual_actors_[index].v, actual_actors_[index].radius});
  }
  if (endless_mode_ && snapshot_.world_activity.incident.active) {
   const auto& incident = snapshot_.world_activity.incident;
   actors.push_back({"field-incident", incident.position, {}, incident.radius_m});
  }
  platform_->sync_actors(actors);
 }
 refresh_dynamic_obstacles();
}

void ScenarioSession::scan_environment() {
 const auto* sensor = resolved_build_.lidar();
 if (!sensor || !sensor_system_)
  return;
 sim::WorldTruth truth;
 truth.robot = snapshot_.robot;
 truth.dynamic_objects = actual_actors_;
 sim::LidarConfig lidar_config;
 lidar_config.rays = sensor->rays;
 lidar_config.min_range_m = sensor->min_range_m;
 const double visibility = endless_mode_ ? snapshot_.world_activity.visibility : 1.0;
 lidar_config.range_m = sensor->range_m * (0.55 + 0.45 * visibility);
 lidar_config.scan_hz = sensor->scan_hz;
 lidar_config.latency_s = sensor->latency_s;
 lidar_config.projection_min_height_m = sensor->projection_min_height_m;
 lidar_config.projection_max_height_m = sensor->projection_max_height_m;
 lidar_config.projection_ground_seed_max_height_m =
 sensor->projection_ground_seed_max_height_m;
 lidar_config.projection_ground_max_slope_rad =
 sensor->projection_ground_max_slope_rad;
 if (sensor->vertical_channels > 1) {
 lidar_config.vertical_angles_rad = sensor->vertical_angles_rad;
 }
 lidar_config.noise = {
   sensor->noise_sigma_m,
   sensor->range_bias_m,
   sensor->range_resolution_m,
   sensor->dropout_probability,
 };
 lidar_config.returns = {
   sensor->terrain_reflectivity, sensor->obstacle_reflectivity,       sensor->actor_reflectivity, 0.50,
   sensor->incidence_floor,      sensor->distance_attenuation_per_m2,
 };
 if (endless_mode_) {
  lidar_config.noise.range_stddev_m += (1.0 - visibility) * 0.025;
  lidar_config.noise.dropout_probability =
    std::clamp(lidar_config.noise.dropout_probability + (1.0 - visibility) * 0.06, 0.0, 0.95);
 }
 const auto sensor_slot =
   std::find_if(robot_definition_.slots.begin(), robot_definition_.slots.end(),
                [](const auto& slot) { return slot.slot == "sensor" && !slot.mounts.empty(); });
 if (sensor_slot != robot_definition_.slots.end()) {
  const auto mount =
    std::find_if(robot_definition_.mounts.begin(), robot_definition_.mounts.end(),
                 [&](const auto& candidate) { return candidate.id == sensor_slot->mounts.front(); });
  if (mount != robot_definition_.mounts.end()) {
   lidar_config.frame_id = mount->frame_id;
   lidar_config.position_in_base = {mount->position_m.x, mount->position_m.y};
   lidar_config.yaw_in_base_rad = mount->rotation_deg.z * sim::kPi / 180.0;
  }
 }
 sim::LidarRaycastTruth lidar_truth;
 std::optional<sim::LidarPointCloudRaycastTruth> point_cloud_truth;
 if (platform_ && sensor->vertical_channels > 1) {
 point_cloud_truth = platform_->scan_lidar_point_cloud(
 sensor->rays, lidar_config.vertical_angles_rad, lidar_config.range_m,
 estimation_system_.state().position, estimation_system_.state().heading_rad,
 snapshot_.time_s, 1.0 / sensor->scan_hz, lidar_config.returns);
 } else if (platform_) {
 lidar_truth = platform_->scan_lidar(
 sensor->rays, lidar_config.range_m, estimation_system_.state().position,
 estimation_system_.state().heading_rad, snapshot_.time_s,
 1.0 / sensor->scan_hz, lidar_config.returns);
 } else {
 if (sensor->vertical_channels > 1) {
 throw std::runtime_error(
 "multi-line lidar requires a 3D platform provider");
 }
 const double cosine = std::cos(truth.robot.theta);
 const double sine = std::sin(truth.robot.theta);
 const sim::Vec2 sensor_origin{
 truth.robot.x + cosine * lidar_config.position_in_base.x -
 sine * lidar_config.position_in_base.y,
 truth.robot.y + sine * lidar_config.position_in_base.x +
 cosine * lidar_config.position_in_base.y,
 };
 lidar_truth = sim::planar_lidar_truth(
 *truth_map_, sensor_origin,
 truth.robot.theta + lidar_config.yaw_in_base_rad,
 sensor->rays, lidar_config.range_m, truth.dynamic_objects);
 }
 auto produced_observation = point_cloud_truth
 ? sensor_system_->capture_lidar_point_cloud(
 truth, snapshot_.time_s, lidar_config, *point_cloud_truth)
 : sensor_system_->capture_lidar(
 truth, snapshot_.time_s, lidar_config, lidar_truth);
 std::optional<sim::physics::RigidTransform> world_from_sensor_at_capture;
 if (produced_observation.lidar_point_cloud && platform_) {
 world_from_sensor_at_capture = platform_->lookup_transform(
 "map", produced_observation.lidar_point_cloud->header.frame_id);
 }
 const double available_time_s = produced_observation.time_s + lidar_config.latency_s;
 if (!lidar_delivery_queue_.enqueue(available_time_s,
 {std::move(produced_observation), lidar_config,
 std::move(world_from_sensor_at_capture)})) {
 throw std::runtime_error("simulated lidar delivery queue is full");
 }
}

void ScenarioSession::deliver_ready_lidar_observations() {
 while (auto pending = lidar_delivery_queue_.pop_ready(snapshot_.time_s)) {
 if (!runtime_channel_ ||
 !runtime_channel_->platform().publish_observation(std::move(pending->observation))) {
 throw std::runtime_error("runtime channel rejected the simulated lidar observation");
 }
 const auto delivered_observation = runtime_channel_->autonomy().read_observation();
 if (!delivered_observation) {
 throw std::runtime_error("runtime channel did not deliver the simulated lidar observation");
 }
 consume_lidar_observation(*delivered_observation, pending->config,
 pending->world_from_sensor_at_capture);
 }
}

void ScenarioSession::consume_lidar_observation(
 const sim::ObservationFrame& observation, const sim::LidarConfig& lidar_config,
 const std::optional<sim::physics::RigidTransform>& world_from_sensor_at_capture) {
 snapshot_.lidar_point_cloud.reset();
 if (observation.lidar_point_cloud && world_from_sensor_at_capture) {
 constexpr std::size_t kPresentationPointLimit = 1200;
 const std::size_t stride = std::max<std::size_t>(
 1, (observation.lidar_point_cloud->points.size() +
 kPresentationPointLimit - 1) / kPresentationPointLimit);
 auto presentation_cloud = std::make_shared<LidarPointCloudSnapshot>();
 presentation_cloud->reserve(
 std::min(kPresentationPointLimit,
 observation.lidar_point_cloud->points.size()));
 for (std::size_t index = 0;
 index < observation.lidar_point_cloud->points.size(); index += stride) {
 const auto& point = observation.lidar_point_cloud->points[index];
 const auto world_point = sim::physics::transform_point(
 *world_from_sensor_at_capture, {point.x_m, point.y_m, point.z_m});
 presentation_cloud->push_back({
 {world_point.x, world_point.y, world_point.z},
 point.intensity,
 point.ring,
 });
 }
 snapshot_.lidar_point_cloud = std::move(presentation_cloud);
 }
 if (observation.absolute_pose) {
  estimation_system_.correct(*observation.absolute_pose, observation.time_s);
  snapshot_.estimated_position = estimation_system_.state().position;
  snapshot_.estimated_heading = estimation_system_.state().heading_rad;
  update_frame_tree();
 }
 if (!observation.lidar || !observation.lidar_static_reference) {
 throw std::runtime_error("lidar observation is missing raw or static-reference scan");
 }
 const auto& estimate = estimation_system_.state();
 const auto perception = autonomy_pipeline_->process_lidar(
 *observation.lidar, *observation.lidar_static_reference,
 {estimate.position, estimate.heading_rad, estimate.linear_velocity_mps,
 estimate.angular_velocity_radps},
 {lidar_config.position_in_base, lidar_config.yaw_in_base_rad,
 lidar_config.noise.range_stddev_m,
 std::max(0.45, level_.planning_localization_margin_m)},
 track_max_age_s());
 const bool first_static_detection = !snapshot_.static_obstacle_detected;
 if (level_.objective.type == "observe_and_stop" && perception.static_obstacle_near) {
  snapshot_.static_obstacle_detected = true;
 }
 snapshot_.lidar_hits = perception.hits;

 if (first_static_detection && snapshot_.static_obstacle_detected &&
     level_.objective.type == "observe_and_stop") {
  record_event("obstacle-detected", catalog_.tr("event.obstacle_detected"));
 }

 bool first_detection = !snapshot_.dynamic_object_detected;
 if (perception.dynamic_object_detected) {
  snapshot_.dynamic_object_detected = true;
 }
 refresh_dynamic_obstacles();
 if (first_detection && snapshot_.dynamic_object_detected) {
  double first_range = 0.0;
  for (const auto& obstacle : snapshot_.dynamic_obstacles) {
   const double range = std::hypot(obstacle.estimated_position.x - snapshot_.robot.x,
                                   obstacle.estimated_position.y - snapshot_.robot.y);
   if (first_range <= 0.0 || range < first_range) {
    first_range = range;
   }
  }
  std::ostringstream detected;
  detected << std::fixed << std::setprecision(1) << first_range;
  record_event("dynamic-detected", catalog_.trf("event.dynamic_detected", {detected.str()}));
 }
}

void ScenarioSession::refresh_dynamic_obstacles() {
 snapshot_.dynamic_obstacles.clear();
 if (!autonomy_pipeline_) {
  return;
 }
 const auto tracks =
 autonomy_pipeline_->dynamic_obstacles().active_tracks(snapshot_.time_s, track_max_age_s());
 for (const auto& track : tracks) {
 if (track.motion_class == sim::TrackMotionClass::Unconfirmed)
 continue;
  std::size_t nearest_actor = actual_actors_.size();
  double nearest_distance = 1.5;
  for (std::size_t index = 0; index < actual_actors_.size(); ++index) {
   const double candidate_distance = distance(track.state.p, actual_actors_[index].p);
   if (candidate_distance < nearest_distance) {
    nearest_distance = candidate_distance;
    nearest_actor = index;
   }
  }
  ++result_.assoc_total;
  if (nearest_actor < actual_actors_.size()) {
   ++result_.assoc_agree;
  }
  const std::string id =
    nearest_actor < agents_.size() ? agents_[nearest_actor].id : "track." + std::to_string(track.object_id);
  snapshot_.dynamic_obstacles.push_back({
    id,
    track.state.p,
    track.state.v,
    track.state.radius,
    track.last_observed_s,
 track.learned_endpoint_dwell_s,
    nearest_actor < agents_.size() ? agents_[nearest_actor].visual_kind : "vehicle",
    nearest_actor < agents_.size() ? agents_[nearest_actor].display_heading_rad
                                   : std::atan2(track.state.v.y, track.state.v.x),
  });
  if (nearest_actor < snapshot_.actors.size()) {
   snapshot_.actors[nearest_actor].detected = true;
  }
 }
}

sim::Vec2 ScenarioSession::current_navigation_target() {
 const double lookahead = property(behavior_, "lookahead_m");
 const auto& estimate = estimation_system_.state();
 sim::Vec2 fallback;
 const auto& field_intent = field_operator_system_.intent();
 if (endless_mode_ && field_intent.active) {
  fallback = field_intent.target;
 } else if (endless_mode_ &&
            dispatch_has_priority(field_dispatch_system_.state(), snapshot_.world_activity.incident)) {
  fallback = field_dispatch_system_.state().target;
 } else if (endless_mode_ && snapshot_.world_activity.incident.active) {
  fallback = snapshot_.world_activity.incident.position;
 } else {
  const GoalStep* active = active_goal();
  fallback = active ? as_vec(map_definition_.markers.at(active->marker))
                    : as_vec(map_definition_.markers.at(level_.objective.goal));
 }
 return route_tracking_system_->navigation_target(planned_route_, estimate, lookahead, *planning_map_, fallback);
}

void ScenarioSession::refresh_endless_route() {
 const auto& field_intent = field_operator_system_.intent();
 if (!endless_mode_ || !planning_map_ || !route_tracking_system_)
  return;
 std::string target_key;
 sim::Vec2 target;
 bool explicit_intent = field_intent.active;
 bool follow_hold = false;
 if (explicit_intent) {
  target = field_intent.target;
  target_key =
    "intent:" + std::to_string(field_intent.revision) + ":" + field_intent_kind_id(field_intent.kind);
  if (field_intent.kind == FieldIntentKind::Follow) {
   const sim::Vec2 robot{snapshot_.robot.x, snapshot_.robot.y};
   const double separation = distance(robot, target);
   constexpr double kCompanionDistanceM = 1.15;
   constexpr double kCompanionHoldRadiusM = 1.35;
   if (separation <= kCompanionHoldRadiusM) {
    follow_hold = true;
   } else {
    const double scale = (separation - kCompanionDistanceM) / separation;
    target = {robot.x + (target.x - robot.x) * scale, robot.y + (target.y - robot.y) * scale};
   }
   target_key += ":" + std::to_string(static_cast<long long>(std::llround(target.x * 2.0))) + ":" +
                 std::to_string(static_cast<long long>(std::llround(target.y * 2.0)));
  }
 } else if (dispatch_has_priority(field_dispatch_system_.state(), snapshot_.world_activity.incident)) {
  const auto& dispatch = field_dispatch_system_.state();
  target_key = "dispatch:" + dispatch.marker_id + ":" + std::to_string(dispatch.stage);
  target = dispatch.target;
 } else if (snapshot_.world_activity.incident.active) {
  const auto& incident = snapshot_.world_activity.incident;
  target_key = "incident:" + incident.marker_id + ":" + world_incident_kind_id(incident.kind);
  target = incident.position;
 }
 if (target_key == endless_target_key_)
  return;
 endless_target_key_ = target_key;
 planned_route_.clear();
 route_tracking_system_->reset();
 held_command_ = {};
 route_feasible_ = true;
 if (explicit_intent && (field_intent.kind == FieldIntentKind::Wait || follow_hold)) {
  field_operator_system_.acknowledge(field_intent.revision, true);
  return;
 }
 if (target_key.empty())
  return;

 const sim::Vec2 start{snapshot_.robot.x, snapshot_.robot.y};
 const double target_heading = std::atan2(target.y - start.y, target.x - start.x);
 const std::vector<autonomy::PlanningPose> waypoints{
   {start, snapshot_.robot.theta},
   {target, target_heading, 0.85},
 };
 auto plan = planning_system_.plan(*planning_map_, waypoints, [this](sim::Vec2 from, sim::Vec2 to) {
  return navigation_traversal_cost(from, to);
 });
 result_.astar_expanded += plan.expanded;
 route_feasible_ = plan.ok;
 if (plan.ok) {
  planned_route_ = std::move(plan.path);
 }
 if (explicit_intent)
  field_operator_system_.acknowledge(field_intent.revision, plan.ok);
}

void ScenarioSession::apply_motion() {
 if (!runtime_channel_) {
  throw std::logic_error("mission has no active runtime channel");
 }
 const auto runtime_now = sim::runtime::TimePoint::from_seconds(snapshot_.time_s);
 sim::DiffCmd applied_command = held_command_;
 snapshot_.control.target_v_mps = applied_command.target_v;
 snapshot_.control.target_w_radps = applied_command.target_w;
 // NOTE (FF verdict): a shadow feedforward computation lived here in stage 3
 // and was removed when stage 4 measured net-negative tracking benefit with
 // saturation up. The pure mapping (src/robot/control/feedforward.*) and its evidence
 // (test_feedforward, test_ff_sensitivity) stay parked with the verdict; no
 // per-step shadow compute remains in this hot path.
 const auto before = snapshot_.robot;
 const auto ground_contact = wheel_ground({before.x, before.y});
 sim::DiffState domain_candidate;
 if (differential_wheel_controller_) {
 if (!platform_) {
 throw std::logic_error("actuator-level differential control requires a platform provider");
 }

 // The platform owns the joints. Publish their measured state through LSRI
 // before the deployable ROBOT controller sees it; simulator truth must not
 // cross this boundary through an in-process shortcut.
 const auto provider_feedback = platform_->actuator_feedback();
 const auto* provider_left = provider_feedback.find("left");
 const auto* provider_right = provider_feedback.find("right");
 if (!provider_left || !provider_right) {
 throw std::runtime_error("RO-1 platform did not provide both wheel joint measurements");
 }
 sim::ObservationFrame feedback_frame;
 feedback_frame.time_s = snapshot_.time_s;
 feedback_frame.actuator_feedback.emplace();
 feedback_frame.actuator_feedback->channels = {
 {"left", provider_left->position_rad, provider_left->velocity_radps},
 {"right", provider_right->position_rad, provider_right->velocity_radps},
 };
 feedback_frame.actuator_feedback->header = {
 snapshot_.time_s, actuator_feedback_sequence_++, "base_link", 0.0,
 sim::kMeasurementSchemaVersion, "sim.actuator-feedback"};
 if (!runtime_channel_->platform().publish_observation(std::move(feedback_frame))) {
 throw std::runtime_error("runtime channel rejected the actuator feedback observation");
 }
 const auto delivered_feedback = runtime_channel_->autonomy().read_observation();
 if (!delivered_feedback || !delivered_feedback->actuator_feedback) {
 throw std::runtime_error("runtime channel did not deliver actuator feedback");
 }
 const auto* left = delivered_feedback->actuator_feedback->find("left");
 const auto* right = delivered_feedback->actuator_feedback->find("right");
 if (!left || !right) {
 throw std::runtime_error("delivered actuator feedback is missing a RO-1 wheel channel");
 }
 const sim::DifferentialWheelFeedback wheel_feedback{
 left->position_rad,
 right->position_rad,
 left->velocity_radps,
 right->velocity_radps,
 true,
 };
 auto wheel_control = differential_wheel_controller_->update(
 applied_command, wheel_feedback, fixed_step_s);

 sim::runtime::ActuatorCommand outbound_command;
 outbound_command.header = {
 sim::runtime::kMessageSchemaVersion,
 runtime_now,
 runtime_now,
 command_sequence_++,
 "base_link",
 "controller.ro1",
 };
 outbound_command.valid_until = sim::runtime::TimePoint::from_seconds(
 snapshot_.time_s + fixed_step_s * 2.0);
 outbound_command.setpoints = {
 {"left", sim::runtime::ActuatorCommandMode::NormalizedDuty,
 wheel_control.left.normalized_duty},
 {"right", sim::runtime::ActuatorCommandMode::NormalizedDuty,
 wheel_control.right.normalized_duty},
 };
 if (runtime_channel_->autonomy().write_actuators(outbound_command) !=
 sim::runtime::CommandWriteResult::Accepted) {
 throw std::runtime_error("runtime channel rejected the RO-1 actuator command");
 }
 const auto delivered_command = runtime_channel_->platform().read_actuators(runtime_now);
 const auto delivered_duty = [&](const std::string& channel) {
 const auto found = std::find_if(
 delivered_command.setpoints.begin(), delivered_command.setpoints.end(),
 [&](const auto& setpoint) { return setpoint.channel_id == channel; });
 if (found == delivered_command.setpoints.end() ||
 found->mode != sim::runtime::ActuatorCommandMode::NormalizedDuty) {
 throw std::runtime_error("delivered actuator command is missing a normalized-duty channel");
 }
 return found->value;
 };
 wheel_control.left.normalized_duty = delivered_duty("left");
 wheel_control.right.normalized_duty = delivered_duty("right");

 // The SIM plant receives only the delivered actuator command and its own
 // provider-side joint state. Motor curves, joint effort, contact, and rigid
 // body integration remain below the LSRI boundary.
 chassis_->set_actuator_feedback(provider_feedback);
 domain_candidate = chassis_->step_normalized_duty(
 before, applied_command, wheel_control, fixed_step_s, ground_contact);
 } else {
 const auto outbound_command = sim::runtime::make_motion_command(
 held_command_, chassis_caps_, runtime_now, command_sequence_++, fixed_step_s * 2.0);
 if (runtime_channel_->autonomy().write_motion(outbound_command) !=
 sim::runtime::CommandWriteResult::Accepted) {
 throw std::runtime_error("runtime channel rejected the autonomous motion command");
 }
 applied_command = sim::runtime::to_diff_command(
 runtime_channel_->platform().read_motion(runtime_now));
 snapshot_.control.target_v_mps = applied_command.target_v;
 snapshot_.control.target_w_radps = applied_command.target_w;
 if (platform_)
 chassis_->set_actuator_feedback(platform_->actuator_feedback());
 domain_candidate = chassis_->step(
 before, applied_command, fixed_step_s, ground_contact);
 }
 const auto terrain_before = terrain_.sample({before.x, before.y});
 const double forward_grade = sim::longitudinal_grade(terrain_before, before.theta);
 if (!terrain_.empty()) {
  const double grade_accel = -9.81 * forward_grade / std::sqrt(1.0 + forward_grade * forward_grade);
  domain_candidate.v += grade_accel * fixed_step_s;
  const bool commanding_uphill = applied_command.target_v * forward_grade > 0.0;
  const bool gravity_reversed_motion = domain_candidate.v * applied_command.target_v < 0.0;
  if (commanding_uphill && gravity_reversed_motion && std::abs(before.v) < 0.08) {
   domain_candidate.v = 0.0;
  }
  domain_candidate.x = before.x + (domain_candidate.v * std::cos(domain_candidate.theta) -
                                   domain_candidate.lateral_v * std::sin(domain_candidate.theta)) *
                                    fixed_step_s;
  domain_candidate.y = before.y + (domain_candidate.v * std::sin(domain_candidate.theta) +
                                   domain_candidate.lateral_v * std::cos(domain_candidate.theta)) *
                                    fixed_step_s;
 }
 const auto& motors = chassis_->telemetry();
 sim::platform::RigidVehicleState rigid_state;
 bool have_rigid_state = false;
 if (platform_) {
  rigid_state = platform_->step(motors, ground_contact, fixed_step_s);
  have_rigid_state = true;
  snapshot_.rigid_contact_count = rigid_state.contact_count;
  snapshot_.ground_contact_links = rigid_state.ground_contact_links;
  snapshot_.max_drive_slip_ratio = rigid_state.max_drive_slip_ratio;
  result_.max_drive_slip_ratio = std::max(result_.max_drive_slip_ratio, rigid_state.max_drive_slip_ratio);
  result_.rigid_contacts += rigid_state.contact_count;
  if (physics_mode_ == PhysicsMode::BulletShadow) {
   snapshot_.shadow_position_error_m =
     distance({domain_candidate.x, domain_candidate.y}, {rigid_state.planar.x, rigid_state.planar.y});
   snapshot_.shadow_heading_error_rad =
     std::abs(sim::wrap_angle(domain_candidate.theta - rigid_state.planar.theta));
   const double domain_height = terrain_.sample({domain_candidate.x, domain_candidate.y}).height_m +
                                robot_definition_.base_link_height_m;
   snapshot_.shadow_height_error_m = std::abs(domain_height - rigid_state.height_m);
   result_.max_shadow_position_error_m =
     std::max(result_.max_shadow_position_error_m, snapshot_.shadow_position_error_m);
   result_.max_shadow_heading_error_rad =
     std::max(result_.max_shadow_heading_error_rad, snapshot_.shadow_heading_error_rad);
   result_.max_shadow_height_error_m =
     std::max(result_.max_shadow_height_error_m, snapshot_.shadow_height_error_m);
  }
 }
 auto candidate = physics_mode_ == PhysicsMode::Bullet ? rigid_state.planar : domain_candidate;
 // Telemetry shape follows the chassis (telemetry_.differential selects the
 // active side). Ackermann mirrors the single drive unit into the left_*
 // fields and reports the servo below; the right_* fields stay zero rather
 // than carrying fiction.
 const auto& drive_unit =
   motors.differential ? motors.left : (motors.holonomic ? motors.modules[0] : motors.drive);
 snapshot_.control.left_target_mps = drive_unit.target_mps;
 snapshot_.control.right_target_mps =
   motors.differential ? motors.right.target_mps : (motors.holonomic ? motors.modules[1].target_mps : 0.0);
 snapshot_.control.left_measured_mps = drive_unit.measured_mps;
 snapshot_.control.right_measured_mps = motors.differential
                                          ? motors.right.measured_mps
                                          : (motors.holonomic ? motors.modules[1].measured_mps : 0.0);
 snapshot_.control.left_pwm = drive_unit.pwm;
 snapshot_.control.right_pwm =
   motors.differential ? motors.right.pwm : (motors.holonomic ? motors.modules[1].pwm : 0.0);
 snapshot_.control.left_speed_pid_p = drive_unit.pid_p;
 snapshot_.control.left_speed_pid_i = drive_unit.pid_i;
 snapshot_.control.left_speed_pid_d = drive_unit.pid_d;
 snapshot_.control.right_speed_pid_p = motors.differential ? motors.right.pid_p : 0.0;
 snapshot_.control.right_speed_pid_i = motors.differential ? motors.right.pid_i : 0.0;
 snapshot_.control.right_speed_pid_d = motors.differential ? motors.right.pid_d : 0.0;
 // Onboard-observable slip estimate. Articulated differential encoders now
 // read physical joint motion, so both they and tracked drives have tread
 // state independent of body motion. Candidate is the post-step body state;
 // comparing against the previous snapshot keeps both accelerations on the
 // same sample interval.
 if (chassis_caps_.tread_slip_observable) {
  const double tread_v = (snapshot_.control.left_measured_mps + snapshot_.control.right_measured_mps) / 2.0;
  if (slip_have_prev_) {
   const double encoder_accel = (tread_v - slip_prev_tread_v_) / fixed_step_s;
   const double body_accel = (candidate.v - slip_prev_body_v_) / fixed_step_s;
   slip_detector_.step(encoder_accel, body_accel, fixed_step_s);
   if (slip_detector_.episodes() > static_cast<int>(slip_episode_times_.size())) {
    slip_episode_times_.push_back(snapshot_.time_s);
   }
  }
  slip_prev_tread_v_ = tread_v;
  slip_prev_body_v_ = candidate.v;
  slip_have_prev_ = true;
 }
 snapshot_.traction_slip_observed = slip_detector_.detected();
 snapshot_.control.steer_target_rad = motors.differential ? 0.0 : motors.steer.target_rad;
 snapshot_.control.steer_angle_rad = motors.differential ? 0.0 : motors.steer.angle_rad;
 snapshot_.control.steer_pwm = motors.differential ? 0.0 : motors.steer.pwm;
 snapshot_.control.steering_servo_present = motors.steering_servo_present;
 snapshot_.control.mount_actuations.clear();
 const auto find_mount = [&](const std::string& id) -> const MountDefinition* {
  const auto mount = std::find_if(robot_definition_.mounts.begin(), robot_definition_.mounts.end(),
                                  [&](const auto& candidate_mount) { return candidate_mount.id == id; });
  return mount == robot_definition_.mounts.end() ? nullptr : &*mount;
 };
 const auto tire_slot = std::find_if(robot_definition_.slots.begin(), robot_definition_.slots.end(),
                                     [](const auto& slot) { return slot.slot == "tires"; });
 if (tire_slot != robot_definition_.slots.end()) {
  const std::size_t count = tire_slot->mounts.size();
  snapshot_.control.mount_actuations.reserve(count + 2 * robot_definition_.passive_supports.size());
  for (std::size_t index = 0; index < count; ++index) {
   double speed_mps = 0.0;
   if (motors.holonomic && index < motors.modules.size()) {
    speed_mps = motors.modules[index].actual_mps;
   } else if (robot_definition_.drive_type == "ackermann") {
    speed_mps = motors.drive.actual_mps;
   } else if (index == 0) {
    speed_mps = motors.left.actual_mps;
   } else if (index == 1) {
    speed_mps = motors.right.actual_mps;
   }
   const auto* mount = find_mount(tire_slot->mounts[index]);
   auto& roll = mount_roll_rad_[tire_slot->mounts[index]];
   const auto physical_joint = physics_mode_ == PhysicsMode::Bullet && platform_ && mount
                                 ? platform_->joint_state_for_child(mount->frame_id)
                                 : std::nullopt;
   if (physical_joint) {
    roll = std::remainder(physical_joint->position_rad, 2.0 * sim::kPi);
   } else if (mount && mount->rolling_radius_m > 0.0) {
    roll = std::remainder(roll + speed_mps / mount->rolling_radius_m * fixed_step_s, 2.0 * sim::kPi);
   }
   const double steer = index < motors.wheel_steer_count ? motors.wheel_steer_rad[index] : 0.0;
   snapshot_.control.mount_actuations.push_back({tire_slot->mounts[index], steer, roll, std::nullopt});
  }
 }
 for (const auto& support : robot_definition_.passive_supports) {
  const auto* steer_mount = find_mount(support.steer_mount);
  const auto* roll_mount = find_mount(support.roll_mount);
  double steer = 0.0;
  auto& roll = mount_roll_rad_[support.roll_mount];
  const auto physical_steer = physics_mode_ == PhysicsMode::Bullet && platform_ && steer_mount
                                ? platform_->joint_state_for_child(steer_mount->frame_id)
                                : std::nullopt;
  const auto physical_roll = physics_mode_ == PhysicsMode::Bullet && platform_ && roll_mount
                               ? platform_->joint_state_for_child(roll_mount->frame_id)
                               : std::nullopt;
  if (physical_steer)
   steer = std::remainder(physical_steer->position_rad, 2.0 * sim::kPi);
  if (physical_roll) {
   roll = std::remainder(physical_roll->position_rad, 2.0 * sim::kPi);
  }
  snapshot_.control.mount_actuations.push_back({
    support.steer_mount,
    steer,
    0.0,
    std::nullopt,
  });
  snapshot_.control.mount_actuations.push_back({
    support.roll_mount,
    steer,
    roll,
    std::nullopt,
  });
 }
 if (physics_mode_ == PhysicsMode::Bullet && platform_) {
  for (auto& actuation : snapshot_.control.mount_actuations) {
   const auto* mount = find_mount(actuation.mount_id);
   if (mount)
    actuation.base_from_link = platform_->link_transform_in_base(mount->frame_id);
  }
 }
 const double meters_per_revolution = 2.0 * sim::kPi * robot_definition_.wheel_radius_m;
 snapshot_.control.left_target_rpm = snapshot_.control.left_target_mps * 60.0 / meters_per_revolution;
 snapshot_.control.right_target_rpm = snapshot_.control.right_target_mps * 60.0 / meters_per_revolution;
 snapshot_.control.left_measured_rpm = snapshot_.control.left_measured_mps * 60.0 / meters_per_revolution;
 snapshot_.control.right_measured_rpm = snapshot_.control.right_measured_mps * 60.0 / meters_per_revolution;
 const sim::Vec2 candidate_position{candidate.x, candidate.y};
 const auto terrain_state = terrain_.sample(candidate_position);
 const double candidate_grade = sim::longitudinal_grade(terrain_state, candidate.theta);
 const double candidate_cross_grade = sim::lateral_grade(terrain_state, candidate.theta);
 if (physics_mode_ == PhysicsMode::Bullet && have_rigid_state) {
  snapshot_.robot_height_m = rigid_state.height_m;
  snapshot_.robot_pitch_rad = rigid_state.pitch_rad;
  snapshot_.robot_roll_rad = rigid_state.roll_rad;
 } else {
  snapshot_.robot_height_m = terrain_state.height_m + robot_definition_.base_link_height_m;
  snapshot_.robot_pitch_rad = -std::atan(candidate_grade);
  snapshot_.robot_roll_rad = std::atan(candidate_cross_grade);
 }
 snapshot_.terrain_grade = candidate_grade;
 snapshot_.cross_slope = candidate_cross_grade;
 const double support_width = chassis_caps_.geometry.support_width_m;
 const double center_height = combined_center_height_m();
 const double bump_accel_mps2 =
   sim::roughness_lateral_acceleration(terrain_state.roughness, candidate.v, support_width);
 const double downhill_sign = candidate_cross_grade < 0.0 ? -1.0 : 1.0;
 const double lateral_accel_mps2 = candidate.v * candidate.w + downhill_sign * bump_accel_mps2;
 const double analytic_rollover_risk =
   sim::dynamic_rollover_risk(candidate_cross_grade, lateral_accel_mps2, support_width, center_height);
 snapshot_.rollover_risk = physics_mode_ == PhysicsMode::Bullet
                             ? std::abs(snapshot_.robot_roll_rad) /
                                 std::max(0.05, sim::static_rollover_angle(support_width, center_height))
                             : analytic_rollover_risk;
 result_.max_grade = std::max(result_.max_grade, std::abs(candidate_grade));
 result_.max_abs_roll_rad = std::max(result_.max_abs_roll_rad, std::abs(snapshot_.robot_roll_rad));
 result_.max_rollover_risk = std::max(result_.max_rollover_risk, snapshot_.rollover_risk);
 if (!update_payload_integrity(before, candidate, lateral_accel_mps2,
                               have_rigid_state ? rigid_state.max_contact_impulse_Ns : 0.0)) {
  if (!endless_mode_)
   finish_failure("payload-damage", catalog_.tr("event.payload_damage"));
  return;
 }

 if (physics_mode_ == PhysicsMode::Bullet) {
  if (snapshot_.rollover_risk >= 1.0)
   rigid_rollover_exposure_s_ += fixed_step_s;
  else
   rigid_rollover_exposure_s_ = std::max(0.0, rigid_rollover_exposure_s_ - 2.0 * fixed_step_s);
 }
 const bool rollover = physics_mode_ == PhysicsMode::Bullet
                         ? (snapshot_.rollover_risk >= 1.25 || rigid_rollover_exposure_s_ >= 0.40)
                         : snapshot_.rollover_risk >= 1.0;
 if (rollover) {
  ++result_.collisions;
  if (!endless_mode_)
   finish_failure("rollover", catalog_.tr("event.rollover_failure"));
  return;
 }

 CollisionObservation collision_observation;
 collision_observation.position = candidate_position;
 collision_observation.time_s = snapshot_.time_s;
 collision_observation.dt_s = fixed_step_s;
 collision_observation.backend = physics_mode_ == PhysicsMode::Bullet
 ? CollisionBackend::RigidBody
 : CollisionBackend::Occupancy;
 collision_observation.rigid_static_contact =
 have_rigid_state && rigid_state.static_collision;
 collision_observation.occupancy = collision_map_.get();
 collision_observation.actors.reserve(actual_actors_.size());
 for (std::size_t index = 0; index < actual_actors_.size(); ++index) {
 const bool rigid_contact = physics_mode_ == PhysicsMode::Bullet && have_rigid_state &&
 index < agents_.size() &&
 std::find(rigid_state.actor_collisions.begin(), rigid_state.actor_collisions.end(),
 agents_[index].id) != rigid_state.actor_collisions.end();
 collision_observation.actors.push_back({
 actual_actors_[index],
 index < agents_.size() && agents_[index].social,
 rigid_contact,
 });
 }
 const auto collision = collision_monitor_->assess(collision_observation);
 if (collision.static_clearance_m < result_.min_static_clearance_m) {
 result_.min_static_clearance_m = collision.static_clearance_m;
 result_.min_static_clearance_time_s = snapshot_.time_s;
 result_.min_static_robot_position = candidate_position;
 }
 result_.min_clearance_m = std::min(result_.min_clearance_m,
 collision.static_clearance_m);
 if (collision.dynamic_clearance_m < result_.min_dynamic_clearance_m) {
 result_.min_dynamic_clearance_m = collision.dynamic_clearance_m;
 result_.min_dynamic_clearance_time_s = snapshot_.time_s;
 result_.min_dynamic_robot_position = candidate_position;
 result_.min_dynamic_actor_position = collision.nearest_actor_position;
 }
 result_.min_clearance_m = std::min(result_.min_clearance_m,
 collision.dynamic_clearance_m);
 result_.pedestrian_contacts += collision.social_contacts;
 if (collision.static_collision) {
  ++result_.collisions;
  if (!endless_mode_)
   finish_failure("collision-static", catalog_.tr("event.collision_static"));
  return;
 }
 if (collision.dynamic_collision) {
   ++result_.collisions;
   if (!endless_mode_)
    finish_failure("collision-dynamic", catalog_.tr("event.collision_dynamic"));
   return;
  }

 snapshot_.robot = candidate;
 result_.path_length_m += distance({before.x, before.y}, candidate_position);
 const double motor_power = robot_definition_.energy_base_w +
                            robot_definition_.energy_per_v_w * std::hypot(candidate.v, candidate.lateral_v) +
                            robot_definition_.energy_per_w_w * std::abs(candidate.w);
 const auto* equipped_motor = resolved_build_.motor();
 const auto* sensor = resolved_build_.lidar();
 const auto* computer = resolved_build_.computer();
 const auto* imu = resolved_build_.imu();
 const double motor_scale = equipped_motor ? equipped_motor->power_scale : 1.0;
 const double grade_power = std::max(0.0, total_mass_kg() * 9.81 * candidate.v * candidate_grade /
                                            std::sqrt(1.0 + candidate_grade * candidate_grade));
 const double total_power = motor_power * resolved_build_.tires().power_scale * motor_scale + grade_power +
                            (sensor ? sensor->power_w : 0.0) + (computer ? computer->power_w : 0.0) +
                            (imu ? imu->power_w : 0.0);
 result_.energy_wh += total_power * fixed_step_s / 3600.0;

 double slip = resolved_build_.tires().slip_scale;
 const double traction = surface_traction(candidate_position);
 if (traction < 1.0) {
  slip /= traction;
 }
 // Traction saturation (model-side contact state, like motor force: used for
 // odometry degradation, never displayed as a sensed reading). Rising edge
 // counts a slip episode for the debrief.
 const bool saturated = motors.traction_saturated || (physics_mode_ == PhysicsMode::Bullet &&
                                                      have_rigid_state && rigid_state.drive_slipping);
 snapshot_.traction_saturated = saturated;
 if (saturated && !was_saturated_) {
  ++result_.slip_events;
 }
 was_saturated_ = saturated;
 if (saturated) {
  slip *= 3.0;
 }
 const double encoder_v = motors.measured_forward_mps;
 const double encoder_w = motors.measured_yaw_radps;
 sim::ObservationFrame motion_observation;
 motion_observation.time_s = snapshot_.time_s + fixed_step_s;
 motion_observation.encoders.emplace();
 motion_observation.encoders->linear_velocity_mps = encoder_v;
 motion_observation.encoders->angular_velocity_radps = encoder_w;
 motion_observation.encoders->slip_scale = slip;
 motion_observation.encoders->header = {
 motion_observation.time_s, motion_sequence_++, "base_link", 0.0,
 sim::kMeasurementSchemaVersion, "sim.encoder"};
 if (imu) {
  if (!sensor_system_)
   throw std::logic_error("configured IMU requires the hardware sensor system");
  const sim::ImuTruth truth{
    physics_mode_ == PhysicsMode::Bullet && have_rigid_state ? rigid_state.imu_angular_velocity_radps.x : 0.0,
    physics_mode_ == PhysicsMode::Bullet && have_rigid_state ? rigid_state.imu_angular_velocity_radps.y : 0.0,
    physics_mode_ == PhysicsMode::Bullet && have_rigid_state ? rigid_state.imu_angular_velocity_radps.z
                                                             : candidate.w,
    physics_mode_ == PhysicsMode::Bullet && have_rigid_state ? rigid_state.imu_specific_force_mps2.x
                                                             : (candidate.v - before.v) / fixed_step_s,
    physics_mode_ == PhysicsMode::Bullet && have_rigid_state
      ? rigid_state.imu_specific_force_mps2.y
      : (candidate.lateral_v - before.lateral_v) / fixed_step_s + candidate.v * candidate.w,
    physics_mode_ == PhysicsMode::Bullet && have_rigid_state ? rigid_state.imu_specific_force_mps2.z : 9.81,
  };
  const sim::ImuConfig config{
    imu->gyro_noise_radps,
    imu->gyro_bias_stddev_radps,
    imu->gyro_bias_random_walk_radps_sqrt_s,
    imu->accel_noise_mps2,
    "imu_link",
    0.0,
  };
  motion_observation.imu =
    sensor_system_->capture_imu(truth, motion_observation.time_s, fixed_step_s, config);
 }
 if (level_.localization_mode == "fused" && sensor_system_) {
  sim::WorldTruth truth;
  truth.robot = candidate;
  motion_observation.absolute_pose = sensor_system_->capture_pose(truth, motion_observation.time_s);
 }
 if (!runtime_channel_->platform().publish_observation(std::move(motion_observation))) {
  throw std::runtime_error("runtime channel rejected the simulated motion observation");
 }
 const auto delivered_motion = runtime_channel_->autonomy().read_observation();
 if (!delivered_motion) {
  throw std::runtime_error("runtime channel did not deliver the simulated motion observation");
 }
 estimation_system_.update(*delivered_motion, fixed_step_s);
 if (delivered_motion->absolute_pose) {
  estimation_system_.correct(*delivered_motion->absolute_pose, delivered_motion->time_s);
 }
 snapshot_.estimated_position = estimation_system_.state().position;
 snapshot_.estimated_heading = estimation_system_.state().heading_rad;
 update_frame_tree();
 result_.odometry_error_m = estimation_system_.position_error(candidate_position);
 result_.max_odometry_error_m = std::max(result_.max_odometry_error_m, result_.odometry_error_m);
 result_.odometry_heading_error_rad = estimation_system_.heading_error_rad(candidate.theta);
 result_.max_odometry_heading_error_rad =
   std::max(result_.max_odometry_heading_error_rad, result_.odometry_heading_error_rad);
 result_.fused_pose_error_m = estimation_system_.fused_position_error(candidate_position);
 result_.max_fused_pose_error_m =
 std::max(result_.max_fused_pose_error_m, result_.fused_pose_error_m);
 result_.fused_heading_error_rad = estimation_system_.fused_heading_error_rad(candidate.theta);
 result_.max_fused_heading_error_rad =
 std::max(result_.max_fused_heading_error_rad, result_.fused_heading_error_rad);
}

double ScenarioSession::surface_traction(sim::Vec2 position) const {
 for (const auto& surface : map_definition_.surfaces) {
  if (active_in_level(surface.active_in, level_id_) && inside(position, surface)) {
   return surface.traction_scale * environment_grip_multiplier(surface.material);
  }
 }
 return 1.0;
}

void ScenarioSession::finish_success() {
 held_command_ = {};
 snapshot_.waiting_for_clearance = false;
 update_shadow_episode();
 snapshot_.status = MissionStatus::Succeeded;
 snapshot_.robot.v = 0.0;
 snapshot_.robot.w = 0.0;
 snapshot_.control.target_v_mps = 0.0;
 snapshot_.control.target_w_radps = 0.0;
 snapshot_.control.left_target_mps = 0.0;
 snapshot_.control.right_target_mps = 0.0;
 snapshot_.control.left_target_rpm = 0.0;
 snapshot_.control.right_target_rpm = 0.0;
 snapshot_.control.left_measured_mps = 0.0;
 snapshot_.control.right_measured_mps = 0.0;
 snapshot_.control.left_measured_rpm = 0.0;
 snapshot_.control.right_measured_rpm = 0.0;
 snapshot_.control.left_pwm = 0.0;
 snapshot_.control.right_pwm = 0.0;
 result_.status = MissionStatus::Succeeded;
 if (level_.safe_clearance_scope == "static") {
  result_.scored_clearance_m = result_.min_static_clearance_m;
 } else if (level_.safe_clearance_scope == "dynamic") {
  result_.scored_clearance_m = result_.min_dynamic_clearance_m;
 } else {
  result_.scored_clearance_m = result_.min_clearance_m;
 }
 result_.safe_score = result_.collisions == 0 && result_.scored_clearance_m >= level_.safe_min_clearance_m &&
 result_.max_fused_pose_error_m <= level_.max_localization_error_m;
 if (active_contract_) {
  result_.cargo_integrity = std::min(
    result_.cargo_integrity,
    std::clamp(1.0 - active_contract_->fragility *
                       (0.45 * result_.collisions + 0.5 * std::max(0.0, result_.max_rollover_risk - 0.55)),
               0.0, 1.0));
  result_.safe_score = result_.safe_score && result_.cargo_integrity >= 0.75;
 }
 result_.fast_score = result_.time_s <= level_.fast_time_s;
 result_.efficient_score = result_.energy_wh <= level_.efficient_energy_wh;
 close_decision("mission-succeeded");
 record_event("mission-complete", catalog_.trf("event.mission_complete", {level_.objective.goal_label}));
 record_replay();
 if (runtime_channel_ && runtime_channel_->state() == sim::runtime::LifecycleState::Active) {
  runtime_channel_->deactivate();
 }
}

void ScenarioSession::finish_failure(std::string reason, std::string message) {
 held_command_ = {};
 snapshot_.status = MissionStatus::Failed;
 snapshot_.robot.v = 0.0;
 snapshot_.robot.w = 0.0;
 snapshot_.control.target_v_mps = 0.0;
 snapshot_.control.target_w_radps = 0.0;
 snapshot_.control.left_target_mps = 0.0;
 snapshot_.control.right_target_mps = 0.0;
 snapshot_.control.left_target_rpm = 0.0;
 snapshot_.control.right_target_rpm = 0.0;
 snapshot_.control.left_measured_mps = 0.0;
 snapshot_.control.right_measured_mps = 0.0;
 snapshot_.control.left_measured_rpm = 0.0;
 snapshot_.control.right_measured_rpm = 0.0;
 snapshot_.control.left_pwm = 0.0;
 snapshot_.control.right_pwm = 0.0;
 result_.status = MissionStatus::Failed;
 result_.failure_reason = std::move(reason);
 if (level_.safe_clearance_scope == "static") {
  result_.scored_clearance_m = result_.min_static_clearance_m;
 } else if (level_.safe_clearance_scope == "dynamic") {
  result_.scored_clearance_m = result_.min_dynamic_clearance_m;
 } else {
  result_.scored_clearance_m = result_.min_clearance_m;
 }
 result_.safe_score = false;
 result_.fast_score = false;
 result_.efficient_score = false;
 close_decision("mission-failed");
 record_event(result_.failure_reason, std::move(message));
 record_replay();
 if (runtime_channel_ && runtime_channel_->state() == sim::runtime::LifecycleState::Active) {
  runtime_channel_->deactivate();
 }
}

void ScenarioSession::record_event(std::string code, std::string message, double throttle_s) {
 if (code == last_event_code_ && snapshot_.time_s - last_event_time_s_ < throttle_s) {
  return;
 }
 last_event_code_ = code;
 last_event_time_s_ = snapshot_.time_s;
 events_.push_back({snapshot_.time_s, std::move(code), std::move(message)});
}

void ScenarioSession::publish_decision(DecisionEvidence decision) {
 snapshot_.decision = decision_evidence_system_.publish(
 std::move(decision), snapshot_.time_s);
}

void ScenarioSession::close_decision(std::string outcome_code) {
 if (const auto* closed = decision_evidence_system_.close(
 std::move(outcome_code), snapshot_.time_s)) {
 snapshot_.decision = *closed;
 }
}

void ScenarioSession::update_shadow_episode() {
 if (snapshot_.waiting_for_clearance) {
  if (!shadow_episode_open_) {
   shadow_episode_open_ = true;
   shadow_episode_start_s_ = snapshot_.time_s;
   shadow_episode_first_s_ = snapshot_.control.shadow_depart_s;
   shadow_episode_first_reason_ = snapshot_.control.shadow_reason;
   shadow_episode_last_s_ = shadow_episode_first_s_;
   shadow_episode_last_reason_ = shadow_episode_first_reason_;
   shadow_episode_entry_speed_ = shadow_open_speed_;
   shadow_episode_entry_nearest_ = shadow_open_nearest_;
  }
  if (snapshot_.control.shadow_assessed) {
   shadow_episode_last_s_ = snapshot_.control.shadow_depart_s;
   shadow_episode_last_reason_ = snapshot_.control.shadow_reason;
  }
  return;
 }
 if (shadow_episode_open_) {
  shadow_episode_open_ = false;
  ShadowEpisode episode;
  episode.start_s = shadow_episode_start_s_;
  episode.first_pred_s = shadow_episode_first_s_;
  episode.first_reason = shadow_episode_first_reason_;
  episode.entry_threat_speed_mps = shadow_episode_entry_speed_;
  episode.entry_nearest_m = shadow_episode_entry_nearest_;
  episode.end_s = snapshot_.time_s;
  episode.last_pred_s = shadow_episode_last_s_;
  episode.last_reason = shadow_episode_last_reason_;
  episode.waited_s = snapshot_.time_s - shadow_episode_start_s_;
  shadow_episodes_.push_back(episode);
  const auto fmt1 = [](double value) {
   std::ostringstream out;
   out << std::fixed << std::setprecision(1) << value;
   return out.str();
  };
  record_event(
    "shadow_wait",
    catalog_.trf("event.shadow_wait",
                 {fmt1(shadow_episode_first_s_),
                  sim::depart_reason_name(static_cast<sim::DepartReason>(shadow_episode_first_reason_)),
                  fmt1(shadow_episode_last_s_),
                  sim::depart_reason_name(static_cast<sim::DepartReason>(shadow_episode_last_reason_)),
                  fmt1(snapshot_.time_s - shadow_episode_start_s_)}));
 }
}

void ScenarioSession::record_replay() {
 MissionSnapshot frame = snapshot_;
 // Candidate fans are useful live but would dominate a 20 Hz replay. The chosen
 // local path and all control values remain available for deterministic review.
 frame.control.dwa_candidates.clear();
 replay_.push_back(std::move(frame));
 // Endless practice must have bounded memory. Keep roughly the latest five
 // minutes while erasing in chunks so a long session does not shift the
 // vector on every 40 ms replay sample.
 if (endless_mode_ && replay_.size() > 8000) {
  replay_.erase(replay_.begin(), replay_.begin() + 1000);
 }
}

void ScenarioSession::seek_replay(double time_s) {
 if (replay_.empty()) {
  return;
 }
 const auto found = std::lower_bound(
   replay_.begin(), replay_.end(), time_s,
   [](const MissionSnapshot& frame, double requested_time_s) { return frame.time_s < requested_time_s; });
 snapshot_ = found == replay_.end() ? replay_.back() : *found;
}

// Corner preview cap: the tightest bend within braking + reaction preview
// sets the speed ceiling via lateral physics (see follow.hpp). Grip is the
// minimum tire×surface mu over the preview — wet pavement ahead slows the
// car before it arrives, not after it slides.
double ScenarioSession::corner_speed_cap(double nav_x, double nav_y) const {
 const double cruise = cruise_speed_mps();
 const double lookahead = property(behavior_, "lookahead_m");
 const double speed = std::abs(snapshot_.robot.v);
 const double decel = resolved_build_.tires().brake_decel_mps2;
 const double preview =
   std::max(lookahead, speed * speed / (2.0 * std::max(0.5, decel)) + speed * 2.0 / decision_hz());
 double grip = wheel_ground({nav_x, nav_y}).mu_eff;
 const double support_width = chassis_caps_.geometry.support_width_m;
 const double center_height = combined_center_height_m();
 const double static_limit = sim::static_rollover_angle(support_width, center_height);
 double lateral_accel_limit = 0.5 * grip * 9.81;
 double along = 0.0;
 for (size_t i = route_tracking_system_->path_index() + 1; i < planned_route_.size(); ++i) {
  along += distance(planned_route_[i].position, planned_route_[i - 1].position);
  if (along > preview) {
   break;
  }
  grip = std::min(grip, wheel_ground(planned_route_[i].position).mu_eff);
  const double route_heading = planned_route_[i].heading_rad;
  const double cross_grade = sim::lateral_grade(terrain_.sample(planned_route_[i].position), route_heading);
  const double surface_angle = std::atan(std::abs(cross_grade));
  const double rollover_margin =
    9.81 * std::cos(surface_angle) * std::tan(static_limit) - 9.81 * std::sin(surface_angle);
  lateral_accel_limit = std::min(lateral_accel_limit, std::max(0.05, 0.45 * rollover_margin));
 }
 lateral_accel_limit = std::min(lateral_accel_limit, 0.5 * grip * 9.81);
 return sim::corner_speed_cap(planned_route_, route_tracking_system_->path_index(), cruise, lateral_accel_limit,
                              preview);
}

double ScenarioSession::decision_hz() const {
 const auto* computer = resolved_build_.computer();
 return computer ? computer->decision_hz : kBuiltInDecisionHz;
}

double ScenarioSession::cruise_speed_mps() const {
 const double requested = property(behavior_, "cruise_mps");
 const double platform_limited = max_forward_speed_mps_ > 0.0
 ? std::min(requested, max_forward_speed_mps_)
 : requested;
 return platform_limited *
 (physics_mode_ == PhysicsMode::Bullet ? chassis_caps_.rigid_body_cruise_scale : 1.0);
}

double ScenarioSession::track_max_age_s() const {
 // Dynamic-track memory lives on the computer; computer-less levels use the
 // documented engine default.
 const auto* computer = resolved_build_.computer();
 return computer ? computer->track_max_age_s : kDynamicTrackMaxAgeS;
}

sim::GroundContact ScenarioSession::wheel_ground(sim::Vec2 position) const {
 // Traction is always tire tread combined with ground material: neither alone
 // decides grip. mu_ref anchors the product so concrete + standard behaves as
 // the long-validated reference.
 constexpr double kMuReference = 0.8;
 const double tire_mu = resolved_build_.tires().friction_mu;
 const double tire_rr = resolved_build_.tires().rolling_resistance;
 const double tire_stiffness = resolved_build_.tires().normal_stiffness_Npm;
 const double tire_damping = resolved_build_.tires().normal_damping_Nspm;
 double ground_mu = 1.0;
 double ground_rr = 0.0;
 double ground_stiffness = 1.0e9;
 double ground_damping = 0.0;
 double traction = 1.0;
 std::string ground_material;
 for (const auto& surface : map_definition_.surfaces) {
  if (active_in_level(surface.active_in, level_id_) && inside(position, surface)) {
   ground_mu = surface.friction_mu;
   ground_rr = surface.rolling_drag;
   ground_stiffness = surface.normal_stiffness_Npm;
   ground_damping = surface.normal_damping_Nspm;
   traction = surface.traction_scale;
   ground_material = surface.material;
   break;
  }
 }
 const double environment_grip = environment_grip_multiplier(ground_material);
 ground_mu *= environment_grip;
 traction *= environment_grip;
 sim::GroundContact contact;
 contact.mu_eff = tire_mu * ground_mu / kMuReference;
 if (endless_mode_) {
  // Regional wetness affects the same contact contract consumed by chassis
  // dynamics and Bullet forces. It is not a renderer-only weather effect.
  contact.mu_eff *= 1.0 - 0.32 * snapshot_.world_activity.wetness;
  traction *= 1.0 - 0.28 * snapshot_.world_activity.wetness;
 }
 contact.rolling_resist = tire_rr + ground_rr;
 contact.accel_limit_mps2 = std::min(robot_definition_.motor.max_wheel_accel_mps2,
                                     resolved_build_.tires().brake_decel_mps2 * (0.5 + 0.5 * traction));
 contact.normal_stiffness_Npm = 1.0 / (1.0 / tire_stiffness + 1.0 / ground_stiffness);
 // The dynamic profile treats the material value as additional pair
 // dissipation. Identified profiles replace this reduced convention with a
 // fitted contact-pair damping value.
 contact.normal_damping_Nspm = tire_damping + ground_damping;
 return contact;
}

double ScenarioSession::navigation_traversal_cost(sim::Vec2 from, sim::Vec2 to) const {
 const double segment_length = distance(from, to);
 if (segment_length < 1e-9) {
  return 1.0;
 }
 const sim::Vec2 midpoint{(from.x + to.x) * 0.5, (from.y + to.y) * 0.5};
 const auto contact = wheel_ground(midpoint);
 auto capability = terrain_capability_;
 capability.friction_mu = contact.mu_eff;
 const double terrain_cost = terrain_.traversal_cost(from, to, capability);
 if (!std::isfinite(terrain_cost)) {
  return terrain_cost;
 }

 const auto from_sample = terrain_.sample(from);
 const auto to_sample = terrain_.sample(to);
 const double signed_grade = (to_sample.height_m - from_sample.height_m) / segment_length;
 const double slope = std::atan(std::abs(signed_grade));
 const double downhill_gravity_mps2 = signed_grade < 0.0 ? 9.81 * std::sin(slope) : 0.0;
 const double grip_brake_mps2 = std::max(0.0, contact.mu_eff) * 9.81 * std::cos(slope);
 const double available_brake_mps2 = std::min(resolved_build_.tires().brake_decel_mps2, grip_brake_mps2);
 if (downhill_gravity_mps2 > available_brake_mps2 + 1e-9) {
  return std::numeric_limits<double>::infinity();
 }
 const double brake_usage = downhill_gravity_mps2 / std::max(0.1, available_brake_mps2);
 const double dry_tire_mu = resolved_build_.tires().friction_mu / 0.8;
 const double low_grip_penalty = std::max(0.0, dry_tire_mu / std::max(0.08, contact.mu_eff) - 1.0);
 double extra_ground_drag = 0.0;
 for (const auto& surface : map_definition_.surfaces) {
  if (active_in_level(surface.active_in, level_id_) && inside(midpoint, surface)) {
   extra_ground_drag = std::max(0.0, surface.rolling_drag);
   break;
  }
 }
 const double rolling_penalty = extra_ground_drag * 5.0;
 return terrain_cost * (1.0 + 1.8 * brake_usage * brake_usage + 0.35 * low_grip_penalty + rolling_penalty);
}

} // namespace mission
