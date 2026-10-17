#include "mission_simulation.hpp"
#include "autonomy/planning/task/task_ordering.hpp"

#include "autonomy/planning/global/astar.hpp"
#include "mission/world/actor_behavior_registry.hpp"
#include "hw/chassis/chassis_factory.hpp"
#include "autonomy/localization/slip_detect.hpp"

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

bool dispatch_has_priority(const FieldDispatchState& dispatch,
 const WorldIncidentState& incident) {
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

const std::string& MissionSimulation::level_robot_id(const ContentCatalog& catalog,
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

MissionSimulation::MissionSimulation(const ContentCatalog& catalog, std::string level_id,
 BuildSelection build, const MissionSimulationOptions& options)
 : MissionSimulation(catalog, std::move(level_id), std::move(build), options.seed_override,
 options.owned_abilities, options.component_registry, options.global_planner,
 options.physics_mode, options.high_level_policy, options.contract_id,
 options.runtime_provider_id, options.runtime_registry) {}

MissionSimulation::MissionSimulation(const ContentCatalog& catalog, std::string level_id,
                                     BuildSelection build, std::optional<uint64_t> seed_override,
                                     const std::vector<std::string>& owned_abilities,
                                     const ComponentRegistry& component_registry,
 std::shared_ptr<const autonomy::IGlobalPlanner> global_planner, PhysicsMode physics_mode,
 std::shared_ptr<sim::IHighLevelPolicy> high_level_policy,
 std::string contract_id, std::string runtime_provider_id,
 sim::runtime::RuntimeRegistry runtime_registry)
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
 resolved_build_ = component_registry.resolve(catalog_, build_);
 reset();
}

void MissionSimulation::reset() {
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
 events_.clear();
 decisions_.clear();
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
 goal_system_.reset(level_.objective);
 snapshot_.goals_reached = goal_system_.goals_reached();
 snapshot_.completed_goal_ids = goal_system_.completed_goal_ids(level_.objective);
 snapshot_.abandoned_goal_ids = goal_system_.abandoned_goal_ids(level_.objective);
 snapshot_.payload = payload_;
 field_dispatch_system_.reset(effective_seed() ^ 0xbb67ae85U);
 world_activity_system_.reset(effective_seed() ^ 0x510e527fU);
 snapshot_.world_activity = world_activity_system_.state();
 next_scan_s_ = 0.0;
 next_decision_s_ = 0.0;
 next_replay_s_ = 0.0;
 motion_sequence_ = 0;
 command_sequence_ = 0;
 decision_sequence_ = 0;
 last_event_time_s_ = -1000.0;
 last_event_code_.clear();
 governor_state_ = {};
 was_saturated_ = false;
 rigid_rollover_exposure_s_ = 0.0;
 rigid_static_contact_s_ = 0.0;
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
 update_actors();
 slip_detector_.reset();
 slip_episode_times_.clear();
 slip_prev_tread_v_ = 0.0;
 slip_prev_body_v_ = 0.0;
 slip_have_prev_ = false;
 record_replay();
}

void MissionSimulation::reset_payload() {
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
 payload_.cog_height_m = payload_.mass_kg > 1e-9
 ? payload_.first_moment_kgm / payload_.mass_kg
 : 0.0;
}

void MissionSimulation::update_environment() {
 const auto& keyframes = level_.environment_keyframes;
 if (keyframes.empty()) {
 snapshot_.environment = {};
 return;
 }
 const auto upper = std::upper_bound(
 keyframes.begin(), keyframes.end(), snapshot_.time_s,
 [](double time_s, const EnvironmentKeyframeDefinition& keyframe) {
 return time_s < keyframe.time_s;
 });
 const std::size_t next = upper == keyframes.end()
 ? keyframes.size() - 1
 : static_cast<std::size_t>(upper - keyframes.begin());
 const std::size_t current = upper == keyframes.end() ? next : (next == 0 ? 0 : next - 1);
 const auto& from = keyframes[current];
 const auto& to = keyframes[next];
 const double duration_s = to.time_s - from.time_s;
 const double blend = duration_s > 1e-9
 ? std::clamp((snapshot_.time_s - from.time_s) / duration_s, 0.0, 1.0)
 : 0.0;
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

double MissionSimulation::environment_grip_multiplier(const std::string& material) const {
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

double MissionSimulation::total_mass_kg() const {
 return robot_definition_.mass_kg + payload_.mass_kg;
}

double MissionSimulation::combined_center_height_m() const {
 return std::max(0.05,
 (robot_definition_.mass_kg * robot_definition_.center_of_mass_height_m +
 payload_.first_moment_kgm) /
 std::max(0.5, total_mass_kg()));
}

double MissionSimulation::combined_yaw_inertia_kgm2() const {
 return robot_definition_.yaw_inertia_kgm2 +
 payload_.mass_kg * robot_definition_.collision_radius_m * robot_definition_.collision_radius_m;
}

void MissionSimulation::synchronize_payload_dynamics() {
 const sim::ChassisBodyDynamics dynamics{total_mass_kg(), combined_yaw_inertia_kgm2()};
 if (chassis_)
 chassis_->set_body_dynamics(dynamics);
 if (platform_)
 platform_->update_load(dynamics.mass_kg, combined_center_height_m(),
 dynamics.yaw_inertia_kgm2);
 terrain_capability_.mass_kg = dynamics.mass_kg;
 const double support_width = chassis_caps_.geometry.support_width_m;
 terrain_capability_.max_cross_slope_rad =
 std::atan2(support_width * 0.5, combined_center_height_m()) * 0.98;
 snapshot_.payload = payload_;
 result_.collected_items = payload_.item_count;
 result_.collected_value = payload_.value;
 result_.final_payload_mass_kg = payload_.mass_kg;
 result_.final_payload_cog_height_m = payload_.cog_height_m;
}

void MissionSimulation::collect_payload(const GoalStep& goal) {
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
 payload_.history.push_back({id, snapshot_.time_s, goal.item_count, goal.item_mass_kg,
 goal.item_cog_height_m, goal.item_value, goal.item_fragility});
 synchronize_payload_dynamics();
}

bool MissionSimulation::update_payload_integrity(const sim::DiffState& before,
 const sim::DiffState& after, double lateral_accel_mps2, double contact_impulse_Ns) {
 if (payload_.collected_mass_kg <= 1e-9)
 return true;
 const double forward_accel_mps2 = (after.v - before.v) / fixed_step_s;
 const double raw_shock_mps2 = std::hypot(forward_accel_mps2, lateral_accel_mps2);
 // Fruit sits in a tray rather than at an ideal rigid-body point. A 120 ms
 // first-order mount response rejects fixed-step contact chatter while keeping
 // braking, bumps and collisions in the measured cargo acceleration.
 const double filter_alpha = 1.0 - std::exp(-fixed_step_s / 0.12);
 filtered_payload_shock_mps2_ +=
 (raw_shock_mps2 - filtered_payload_shock_mps2_) * filter_alpha;
 const double shock_mps2 = filtered_payload_shock_mps2_;
 const double jerk_mps3 = payload_shock_initialized_
 ? std::abs(shock_mps2 - previous_payload_shock_mps2_) / fixed_step_s
 : 0.0;
 previous_payload_shock_mps2_ = shock_mps2;
 payload_shock_initialized_ = true;
 payload_.max_shock_mps2 = std::max(payload_.max_shock_mps2, shock_mps2);
 payload_.max_jerk_mps3 = std::max(payload_.max_jerk_mps3, jerk_mps3);

 const double fragility = std::clamp(
 payload_.fragility_mass_sum / payload_.collected_mass_kg, 0.0, 1.0);
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

const char* MissionSimulation::physics_backend_name() const {
 return physics_mode_name(physics_mode_);
}

void MissionSimulation::initialize_rigid_physics() {
 platform_.reset();
 if (physics_mode_ == PhysicsMode::Domain) {
 return;
 }
 const auto& spawn = map_definition_.markers.at(level_.objective.spawn);
 platform_ = std::make_unique<sim::platform::SimulatedPlatform>(map_definition_, level_, robot_definition_,
 chassis_caps_, terrain_, sim::Vec2{spawn.x, spawn.y}, spawn.theta,
 total_mass_kg(), combined_center_height_m(), combined_yaw_inertia_kgm2());
}

void MissionSimulation::initialize_frame_tree() {
 frame_tree_.clear();
 constexpr double kDegreesToRadians = sim::kPi / 180.0;
 for (const auto& mount : robot_definition_.mounts) {
 frame_tree_.set_transform("base_link", mount.frame_id, {
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

void MissionSimulation::update_frame_tree() {
 const auto map_from_odom = estimation_system_.map_from_odom_translation();
 frame_tree_.set_transform("map", "odom", {
 {map_from_odom.x, map_from_odom.y, 0.0},
 sim::physics::quaternion_from_euler(0.0, 0.0,
 estimation_system_.map_from_odom_heading_rad()),
 });
 const auto odom_position = estimation_system_.odometry_position();
 const auto estimated_ground = terrain_.sample(estimation_system_.state().position);
 frame_tree_.set_transform("odom", "base_footprint", {
 {odom_position.x, odom_position.y, estimated_ground.height_m},
 sim::physics::quaternion_from_euler(0.0, 0.0,
 estimation_system_.odometry_heading_rad()),
 });
 const double base_height = snapshot_.robot_height_m - estimated_ground.height_m;
 frame_tree_.set_transform("base_footprint", "base_link", {
 {0.0, 0.0, base_height},
 sim::physics::quaternion_from_euler(snapshot_.robot_roll_rad,
 snapshot_.robot_pitch_rad, 0.0),
 });

 constexpr double kDegreesToRadians = sim::kPi / 180.0;
 for (const auto& actuation : snapshot_.control.mount_actuations) {
 const auto mount = std::find_if(robot_definition_.mounts.begin(), robot_definition_.mounts.end(),
 [&](const auto& candidate) { return candidate.id == actuation.mount_id; });
 if (mount == robot_definition_.mounts.end())
 continue;
 frame_tree_.set_transform("base_link", mount->frame_id, {
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

void MissionSimulation::build_world() {
 const int width = static_cast<int>(std::ceil(map_definition_.size_m.x / map_definition_.grid_resolution_m));
 const int height = static_cast<int>(std::ceil(map_definition_.size_m.y / map_definition_.grid_resolution_m));
 truth_map_ = std::make_unique<sim::GridMap>(width, height, map_definition_.grid_resolution_m);
 for (const auto& obstacle : map_definition_.obstacles) {
  if (active_in_level(obstacle.active_in, level_id_)) {
   truth_map_->addBox(obstacle.min.x, obstacle.min.y, obstacle.max.x, obstacle.max.y);
  }
 }
 sensor_system_ =
   std::make_unique<sim::SensorSystem>(*truth_map_, effective_seed(), effective_seed() ^ 0x9e3779b9U);
 collision_map_ = std::make_unique<sim::GridMap>(truth_map_->inflated(robot_definition_.collision_radius_m));
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

 sim::DiffParams params;
 params.v_max = std::max(1.5, property(behavior_, "cruise_mps") * 1.25);
 params.accel_max = resolved_build_.tires().brake_decel_mps2;
 params.wheel_base = robot_definition_.wheel_base_m;
 drive_ = std::make_unique<sim::DiffDrive>(params);
 // Chassis dispatch: drive.type selects the actuator model. Unknown types
 // fail loudly in content, never silently fall back to another chassis.
 // The equipped motor component overrides the robot default; an empty motor
 // slot (early levels) runs the robot stock motor. New motors stay plug-and-play
 // because PID gains are derived from these same numbers.
 const auto* equipped_motor = resolved_build_.motor();
 const double motor_vmax =
   equipped_motor ? equipped_motor->max_wheel_speed_mps : robot_definition_.motor.max_wheel_speed_mps;
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
 const double differential_mode_mass = 2.0 * combined_yaw_inertia_kgm2() /
 (robot_definition_.wheel_base_m * robot_definition_.wheel_base_m);
 wheel_mass += std::max(common_mode_mass, differential_mode_mass);
 }
 const sim::WheelPidGains wheel_gains =
 sim::tune_wheel_pid(wheel_mass, motor_stall, motor_free, 0.5 * motor_vmax,
 controller_design);
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
 chassis_params.viscous_drag_N_per_mps = equipped_motor
 ? equipped_motor->viscous_drag_N_per_mps
 : robot_definition_.motor.viscous_drag_N_per_mps;
 chassis_params.encoder_counts_per_revolution = robot_definition_.encoder.counts_per_revolution;
 chassis_params.mass_kg = simulation_mass;
 chassis_params.yaw_inertia_kgm2 = combined_yaw_inertia_kgm2();
 chassis_params.max_steer_rad = robot_definition_.steering.max_steer_rad;
 chassis_params.max_steer_rate_radps = robot_definition_.steering.max_steer_rate_radps;
 chassis_params.servo_response_s = robot_definition_.steering.servo_response_s;
 chassis_params.steering_integral_limit = robot_definition_.steering.integral_limit;
 chassis_params.tracked_max_yaw_rate_rads = robot_definition_.tracked_max_yaw_rate_rads;
 chassis_params.behavior_w_max = params.w_max;
 chassis_params.behavior_accel_max = params.accel_max;
 chassis_params.max_yaw_accel_radps2 = robot_definition_.max_yaw_accel_radps2;
 // Resolve the support footprint from the assembled rolling contacts. Robot
 // data is the source of truth for where wheels touch; collision radius is a
 // body envelope and must not masquerade as a stability footprint.
 std::vector<const MountDefinition*> rolling_contacts;
 const auto tire_slot_definition = std::find_if(
 robot_definition_.slots.begin(), robot_definition_.slots.end(),
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

 // LSRI composition root. The mission uses both ends because it contains the
 // simulator and the autonomy loop in one process. A hardware runner gives
 // the platform end to a native driver or bridge instead.
 sim::runtime::RuntimeChannelConfig runtime_config;
 runtime_config.instance_id = robot_definition_.id;
 runtime_config.capabilities.chassis = chassis_caps_;
 runtime_config.capabilities.max_forward_speed_mps = motor_vmax;
 runtime_config.capabilities.max_lateral_speed_mps =
 chassis_caps_.command_space == sim::ChassisCommandSpace::HolonomicTwist ? motor_vmax : 0.0;
 runtime_config.maximum_command_ttl_s = 0.25;
 runtime_channel_ = runtime_registry_.create(runtime_provider_id_, runtime_config);
 runtime_channel_->configure();
 if (!custom_global_planner_) {
 planning_system_ = autonomy::GlobalPlanningSystem(
 chassis_caps_.command_space == sim::ChassisCommandSpace::SteeringAngle
 ? autonomy::ackermann_global_planner(chassis_caps_.geometry.steering_wheel_base_m,
 chassis_caps_.geometry.max_steer_rad, map_definition_.grid_resolution_m)
 : autonomy::default_global_planner());
 }
 // The global costmap reserves the physical envelope plus a small numerical
 // margin. Policy clearance belongs to the local planner, which can trade it
 // against narrow passages and moving objects continuously. Baking the whole
 // policy distance into a coarse binary grid erased valid goal regions and
 // made a nominally cautious build unable to dock.
 const double planning_clearance = robot_definition_.collision_radius_m + 0.20 +
 chassis_caps_.planning_clearance_margin_m + level_.planning_localization_margin_m;
 planning_map_ = std::make_unique<sim::GridMap>(truth_map_->inflated(planning_clearance));
 // Heading loop auto-tune: gain normalized by speed and wheelbase so the same
 // rule serves every DIY assembly. Kp = w_max / (2 * cruise * base) reproduces
 // the validated RO-1 feel; the integral/derivative ratios are method constants
 // (slow integrator kills steady-state error, short derivative damps).
 const double cruise = property(behavior_, "cruise_mps");
 const double heading_kp =
   yaw_cap_rads_ / (2.0 * std::max(0.2, cruise) * std::max(0.1, robot_definition_.wheel_base_m));
 // PID output units follow the chassis (follow.hpp contract): yaw rate for
 // differential, steer angle for ackermann. Same derivation, capped by what
 // the actuator takes.
 // Differential and tracked share yaw-rate output units; their caps differ
 // (geometry vs vendor spin spec) and both already sit in yaw_cap_rads_.
 const double heading_out_lim = chassis_caps_.turn_command_limit();
 control_system_ =
   std::make_unique<ControlSystem>(heading_kp, heading_kp / 125.0, heading_kp * 0.05, heading_out_lim,
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

void MissionSimulation::plan_route() {
 planned_route_.clear();
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
 terrain_capability_.max_cross_slope_rad =
 std::atan2(support_width * 0.5, combined_center_height_m()) * 0.98;

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
 std::vector<autonomy::PlanningPose> waypoints{
 {as_vec(start), start.theta}, {as_vec(goal), goal.theta, next->radius_m}};
 auto plan = planning_system_.plan(*planning_map_, waypoints,
 [this](sim::Vec2 from, sim::Vec2 to) { return navigation_traversal_cost(from, to); });
 result_.astar_expanded += plan.expanded;
 if (!plan.ok) {
 route_feasible_ = false;
 return;
 }
 planned_route_ = std::move(plan.path);
 control_system_->reset();
}

void MissionSimulation::select_next_goal(const Pose& start) {
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
 const auto plan = planning_system_.plan(*planning_map_,
 {{as_vec(start), start.theta}, {as_vec(marker), marker.theta, candidate.radius_m}},
 [this](sim::Vec2 from, sim::Vec2 to) { return navigation_traversal_cost(from, to); });
 result_.astar_expanded += plan.expanded;
 if (!plan.ok || plan.path.empty())
 continue;
 const auto candidate_cost = route_cost(plan.path);
 const double planning_speed = std::max(0.3, property(behavior_, "cruise_mps") * 0.75);
 if (candidate.action != "complete" && level_.objective.return_reserve_s > 0.0 &&
 terminal != level_.objective.goals.end()) {
 const auto& home = map_definition_.markers.at(terminal->marker);
 const auto return_plan = planning_system_.plan(*planning_map_,
 {{as_vec(marker), marker.theta}, {as_vec(home), home.theta, terminal->radius_m}},
 [this](sim::Vec2 from, sim::Vec2 to) { return navigation_traversal_cost(from, to); });
 result_.astar_expanded += return_plan.expanded;
 if (!return_plan.ok || return_plan.path.empty())
 continue;
 const double return_cost = route_cost(return_plan.path).traversal_cost;
 const double expected_finish_s = snapshot_.time_s +
 (candidate_cost.traversal_cost + return_cost) / planning_speed + level_.objective.return_reserve_s;
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
 if (!best_index || score < best_score - 1e-9 ||
 (std::abs(score - best_score) <= 1e-9 && id < best_id)) {
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
 record_event("task-abandoned",
 catalog_.trf("event.task_abandoned", {goal.label}));
 }
 snapshot_.abandoned_goal_ids = goal_system_.abandoned_goal_ids(level_.objective);
 select_next_goal(start);
 } else if (terminal_out_of_time) {
 return_reserve_failure_ = true;
 }
}

void MissionSimulation::finish_route_failure() {
 if (return_reserve_failure_) {
 finish_failure("return-reserve", catalog_.tr("event.return_reserve_failure"));
 } else {
 finish_failure("no-feasible-route", catalog_.tr("event.no_feasible_route"));
 }
}

void MissionSimulation::start() {
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

void MissionSimulation::set_endless_mode(bool enabled) {
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
 if (control_system_)
 control_system_->reset();
 }
}

bool MissionSimulation::set_field_operator_command(double world_x, double world_y) {
 if (!endless_mode_)
 return false;
 field_operator_system_.set_move_command({world_x, world_y});
 return true;
}

bool MissionSimulation::issue_field_intent(FieldIntentKind kind,
 std::optional<sim::Vec2> target) {
 if (!endless_mode_ || !field_operator_system_.issue_intent(kind, target))
 return false;
 manual_control_active_ = false;
 manual_command_ = {};
 endless_target_key_ = "uninitialized";
 refresh_endless_route();
 return true;
}

void MissionSimulation::restore_endless_world(const WorldActivityCheckpoint& checkpoint) {
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

WorldActivityCheckpoint MissionSimulation::endless_world_checkpoint() const {
 if (!endless_mode_)
 throw std::logic_error("regional world checkpoint requested outside endless mode");
 return world_activity_system_.checkpoint(snapshot_.time_s);
}

void MissionSimulation::restore_field_expedition(
 const FieldExpeditionCheckpoint& checkpoint) {
 if (!endless_mode_ || snapshot_.status != MissionStatus::Ready)
 throw std::logic_error("field expedition can only be restored before an endless session starts");
 if (!checkpoint.level_id.empty() && checkpoint.level_id != level_id_)
 throw std::logic_error("field expedition checkpoint targets a different level");
 field_operator_system_.restore(checkpoint, map_definition_, level_id_);
 endless_target_key_ = "uninitialized";
}

FieldExpeditionCheckpoint MissionSimulation::field_expedition_checkpoint() const {
 if (!endless_mode_)
 throw std::logic_error("field expedition checkpoint requested outside endless mode");
 auto checkpoint = field_operator_system_.checkpoint();
 checkpoint.level_id = level_id_;
 return checkpoint;
}

void MissionSimulation::run_to_completion() {
 if (endless_mode_) {
 throw std::logic_error("endless simulation has no completion condition");
 }
 start();
 while (snapshot_.status == MissionStatus::Running) {
  step();
 }
}

void MissionSimulation::set_manual_command(double forward, double turn) {
 if (endless_mode_ || level_.control_mode != "manual_calibration")
  return;
 manual_control_active_ = true;
 const double cruise = property(behavior_, "cruise_mps");
 manual_command_.target_v = std::clamp(forward, -0.65, 1.0) * cruise;
 manual_command_.target_w = std::clamp(turn, -1.0, 1.0) * chassis_caps_.turn_command_limit();
}

void MissionSimulation::step() {
 if (snapshot_.status != MissionStatus::Running) {
  return;
 }
 if (endless_mode_)
 field_operator_system_.update(fixed_step_s, map_definition_, level_id_);
 update_actors();
 if (const auto* sensor = resolved_build_.lidar(); sensor && snapshot_.time_s + 1e-9 >= next_scan_s_) {
  scan_environment();
  next_scan_s_ += 1.0 / sensor->scan_hz;
 }
 if (manual_control_active_) {
  held_command_ = manual_command_;
 } else if (snapshot_.time_s + 1e-9 >= next_decision_s_) {
  decide_control();
  next_decision_s_ += 1.0 / decision_hz();
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
 record_event("environment-changed",
 catalog_.trf("event.environment_changed", {stage.label}));
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
 const auto transitions = field_dispatch_system_.update(snapshot_.time_s, fixed_step_s,
 map_definition_, {snapshot_.robot.x, snapshot_.robot.y}, snapshot_.robot.v);
 for (const auto& transition : transitions) {
 switch (transition.kind) {
 case FieldDispatchTransitionKind::Opened:
 record_event("field-dispatch-opened",
 catalog_.trf("event.field_dispatch_opened",
 {field_kind_label(catalog_, transition.dispatch_kind)}));
 break;
 case FieldDispatchTransitionKind::PickedUp:
 record_event("field-dispatch-pickup", catalog_.tr("event.field_dispatch_pickup"));
 break;
 case FieldDispatchTransitionKind::Completed:
 record_event("field-dispatch-complete",
 catalog_.trf("event.field_dispatch_complete",
 {field_kind_label(catalog_, transition.dispatch_kind)}));
 break;
 }
 }
 refresh_endless_route();
 }

 const bool completion_allowed =
   level_.objective.type != "observe_and_stop" || snapshot_.static_obstacle_detected;
 const auto arrival = endless_mode_
 ? std::optional<GoalArrival>{}
 : goal_system_.update(level_.objective, map_definition_,
 {snapshot_.robot.x, snapshot_.robot.y}, snapshot_.robot.v, completion_allowed,
 fixed_step_s);
 if (arrival) {
  const auto& goal = arrival->goal;
  snapshot_.goals_reached = goal_system_.goals_reached();
 snapshot_.completed_goal_ids = goal_system_.completed_goal_ids(level_.objective);
 snapshot_.abandoned_goal_ids = goal_system_.abandoned_goal_ids(level_.objective);
 const auto active_index = goal_system_.active_goal_index();
 snapshot_.active_goal_index = active_index.value_or(std::numeric_limits<std::size_t>::max());
 snapshot_.active_goal_id = active_index
 ? (level_.objective.goals[*active_index].id.empty()
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
 record_event("item-collected", catalog_.trf("event.item_collected",
 {goal.label, std::to_string(goal.item_count)}));
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
 record_event("inspection-complete",
 catalog_.trf("event.inspection_complete", {goal.label}));
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
 if (snapshot_.time_s + 1e-9 >= next_replay_s_) {
  record_replay();
 // The fixed simulation tick is 20 ms. A 40 ms replay cadence keeps every
 // seek gap within the UI's 50 ms contract without fabricating interpolated
 // states (50 ms would alias into occasional 60 ms gaps).
 next_replay_s_ += 0.04;
 }
}

void MissionSimulation::apply_goal_stop_constraint() {
 const GoalStep* goal = active_goal();
 if (!goal || goal->action != "inspect")
 return;
 const auto marker = map_definition_.markers.find(goal->marker);
 if (marker == map_definition_.markers.end())
 return;
 const double remaining =
 std::max(0.0, distance({snapshot_.robot.x, snapshot_.robot.y}, as_vec(marker->second)) -
 goal->radius_m * 0.55);
 const double deceleration = std::max(0.35, resolved_build_.tires().brake_decel_mps2 * 0.65);
 const double stop_cap = std::sqrt(2.0 * deceleration * remaining);
 if (remaining <= 0.04) {
 held_command_.target_v = 0.0;
 held_command_.target_w = 0.0;
 } else {
 held_command_.target_v = std::clamp(held_command_.target_v, -stop_cap, stop_cap);
 }
}

void MissionSimulation::update_actors() {
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

void MissionSimulation::scan_environment() {
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
 lidar_config.noise = {
 sensor->noise_sigma_m,
 sensor->range_bias_m,
 sensor->range_resolution_m,
 sensor->dropout_probability,
 };
 lidar_config.returns = {
 sensor->terrain_reflectivity,
 sensor->obstacle_reflectivity,
 sensor->actor_reflectivity,
 0.50,
 sensor->incidence_floor,
 sensor->distance_attenuation_per_m2,
 };
 if (endless_mode_) {
 lidar_config.noise.range_stddev_m += (1.0 - visibility) * 0.025;
 lidar_config.noise.dropout_probability = std::clamp(
 lidar_config.noise.dropout_probability + (1.0 - visibility) * 0.06, 0.0, 0.95);
 }
 const auto sensor_slot = std::find_if(robot_definition_.slots.begin(), robot_definition_.slots.end(),
 [](const auto& slot) { return slot.slot == "sensor" && !slot.mounts.empty(); });
 if (sensor_slot != robot_definition_.slots.end()) {
 const auto mount = std::find_if(robot_definition_.mounts.begin(), robot_definition_.mounts.end(),
 [&](const auto& candidate) { return candidate.id == sensor_slot->mounts.front(); });
 if (mount != robot_definition_.mounts.end()) {
 lidar_config.frame_id = mount->frame_id;
 lidar_config.position_in_base = {mount->position_m.x, mount->position_m.y};
 lidar_config.yaw_in_base_rad = mount->rotation_deg.z * sim::kPi / 180.0;
 }
 }
 auto produced_observation = platform_
 ? sensor_system_->capture_lidar(
 truth, snapshot_.time_s, lidar_config,
 platform_->scan_lidar(sensor->rays, lidar_config.range_m,
 estimation_system_.state().position, estimation_system_.state().heading_rad,
 snapshot_.time_s, 1.0 / sensor->scan_hz, lidar_config.returns))
 : sensor_system_->capture_lidar(truth, snapshot_.time_s, lidar_config);
 if (!runtime_channel_ ||
 !runtime_channel_->platform().publish_observation(std::move(produced_observation))) {
 throw std::runtime_error("runtime channel rejected the simulated lidar observation");
 }
 const auto delivered_observation = runtime_channel_->autonomy().read_observation();
 if (!delivered_observation) {
 throw std::runtime_error("runtime channel did not deliver the simulated lidar observation");
 }
 const auto& observation = *delivered_observation;
 if (observation.absolute_pose) {
  estimation_system_.correct(*observation.absolute_pose, observation.time_s);
  snapshot_.estimated_position = estimation_system_.state().position;
  snapshot_.estimated_heading = estimation_system_.state().heading_rad;
 update_frame_tree();
 }
 const auto perception =
   sensor_system_->process_lidar(observation, estimation_system_.state(), track_max_age_s());
 const bool first_static_detection = !snapshot_.static_obstacle_detected;
 if (level_.objective.type == "observe_and_stop" && perception.static_obstacle_near) {
  snapshot_.static_obstacle_detected = true;
 }
 snapshot_.lidar_hits = perception.lidar_hits;

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

void MissionSimulation::refresh_dynamic_obstacles() {
 snapshot_.dynamic_obstacles.clear();
 if (!sensor_system_) {
  return;
 }
 const auto tracks = sensor_system_->dynamic_obstacles().active_tracks(snapshot_.time_s, track_max_age_s());
 for (const auto& track : tracks) {
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
  nearest_actor < agents_.size() ? agents_[nearest_actor].visual_kind : "vehicle",
  nearest_actor < agents_.size() ? agents_[nearest_actor].display_heading_rad
                                : std::atan2(track.state.v.y, track.state.v.x),
  });
  if (nearest_actor < snapshot_.actors.size()) {
   snapshot_.actors[nearest_actor].detected = true;
  }
 }
}

sim::Vec2 MissionSimulation::current_navigation_target() {
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
 fallback = active
 ? as_vec(map_definition_.markers.at(active->marker))
 : as_vec(map_definition_.markers.at(level_.objective.goal));
 }
 return control_system_->navigation_target(
 planned_route_, estimate, lookahead, *planning_map_, fallback);
}

void MissionSimulation::refresh_endless_route() {
 const auto& field_intent = field_operator_system_.intent();
 if (!endless_mode_ || !planning_map_ || !control_system_)
 return;
 std::string target_key;
 sim::Vec2 target;
 bool explicit_intent = field_intent.active;
 bool follow_hold = false;
 if (explicit_intent) {
 target = field_intent.target;
 target_key = "intent:" + std::to_string(field_intent.revision) + ":" +
 field_intent_kind_id(field_intent.kind);
 if (field_intent.kind == FieldIntentKind::Follow) {
 const sim::Vec2 robot{snapshot_.robot.x, snapshot_.robot.y};
 const double separation = distance(robot, target);
 constexpr double kCompanionDistanceM = 1.15;
 constexpr double kCompanionHoldRadiusM = 1.35;
 if (separation <= kCompanionHoldRadiusM) {
 follow_hold = true;
 } else {
 const double scale = (separation - kCompanionDistanceM) / separation;
 target = {robot.x + (target.x - robot.x) * scale,
 robot.y + (target.y - robot.y) * scale};
 }
 target_key += ":" + std::to_string(static_cast<long long>(std::llround(target.x * 2.0))) +
 ":" + std::to_string(static_cast<long long>(std::llround(target.y * 2.0)));
 }
 } else if (dispatch_has_priority(
 field_dispatch_system_.state(), snapshot_.world_activity.incident)) {
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
 control_system_->reset();
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
 auto plan = planning_system_.plan(*planning_map_, waypoints,
 [this](sim::Vec2 from, sim::Vec2 to) { return navigation_traversal_cost(from, to); });
 result_.astar_expanded += plan.expanded;
 route_feasible_ = plan.ok;
 if (plan.ok) {
 planned_route_ = std::move(plan.path);
 }
 if (explicit_intent)
 field_operator_system_.acknowledge(field_intent.revision, plan.ok);
}

void MissionSimulation::decide_control() {
 ++result_.local_decisions;
 snapshot_.control = {};
 if (control_system_->path_index() >= planned_route_.size()) {
 if (planned_route_.empty()) {
   held_command_ = {0.0, 0.0};
 DecisionEvidence decision;
 decision.observation_time_s = snapshot_.time_s;
 decision.constraint_code = "route";
 decision.reason_code = "no-route";
 decision.action_code = "wait";
 publish_decision(std::move(decision));
   return;
  }
  // Docking hold: waypoint bookkeeping exhausted (resync/accept ran ahead of
  // a lagging chassis) but arrival tolerance unmet — success is measured by
  // distance, not by idx. Hold the last waypoint so normal tracking (reverse
  // hysteresis, watchdog, DWA arbitration below) docks the robot instead of
  // parking short with tv=0 until timeout.
 control_system_->set_path_index(planned_route_.size() - 1);
 }
 auto target = current_navigation_target();
 const auto& estimate = estimation_system_.state();
 const double nav_x = estimate.position.x;
 const double nav_y = estimate.position.y;
 const double nav_heading = estimate.heading_rad;
 const auto publish = [&](std::string constraint_code, std::string reason_code,
 std::string action_code, double observation_time_s,
 std::vector<std::string> observation_refs, double prediction_horizon_s,
 double measured_separation_m, double predicted_separation_m) {
 DecisionEvidence decision;
 decision.observation_time_s = observation_time_s;
 decision.observation_refs = std::move(observation_refs);
 decision.prediction_horizon_s = prediction_horizon_s;
 decision.measured_separation_m = measured_separation_m;
 decision.predicted_separation_m = predicted_separation_m;
 decision.constraint_code = std::move(constraint_code);
 decision.reason_code = std::move(reason_code);
 decision.action_code = std::move(action_code);
 decision.target_speed_mps = held_command_.target_v;
 decision.target_yaw_rate_radps = held_command_.target_w;
 publish_decision(std::move(decision));
 };
 sim::StrategyObservation strategy_observation;
 strategy_observation.stamp_s = estimate.timestamp_s;
 strategy_observation.ego = estimate;
 strategy_observation.route_target = target;
 const GoalStep* strategy_goal = active_goal();
 strategy_observation.goal = strategy_goal
 ? as_vec(map_definition_.markers.at(strategy_goal->marker))
 : as_vec(map_definition_.markers.at(level_.objective.goal));
 strategy_observation.rollover_risk = snapshot_.rollover_risk;
strategy_observation.traction_slip_observed = snapshot_.traction_slip_observed;
 strategy_observation.goals_reached = static_cast<int>(snapshot_.goals_reached);
 for (const auto& hit : snapshot_.lidar_hits) {
 strategy_observation.visible_clearance_m = std::min(
 strategy_observation.visible_clearance_m,
 distance(estimate.position, hit) - robot_definition_.collision_radius_m);
 }
 sim::StrategySafetyContext strategy_context;
 strategy_context.fallback_target = target;
 strategy_context.map_size = map_definition_.size_m;
 strategy_context.clearance_hold_enabled = custom_high_level_policy_;
 const auto strategy_terrain = terrain_.sample({nav_x, nav_y});
 const bool material_terrain_load =
 std::abs(sim::longitudinal_grade(strategy_terrain, nav_heading)) >= 0.12 ||
 std::abs(sim::lateral_grade(strategy_terrain, nav_heading)) >= 0.12;
 // A custom upper policy gets the complete safety shield. The validated
 // route baseline already owns flat-ground speed and obstacle arbitration;
 // its residual-based relief is reserved for a mapped slope, where reducing
 // translation preserves differential steering authority under motor load.
 strategy_context.traction_relief_enabled = custom_high_level_policy_ || material_terrain_load;
 strategy_context.subgoal_is_safe = [this, nav_x, nav_y](sim::Vec2 subgoal) {
 const double length = distance({nav_x, nav_y}, subgoal);
 const int samples = std::max(1, static_cast<int>(std::ceil(length / 0.25)));
 sim::Vec2 previous{nav_x, nav_y};
 for (int index = 1; index <= samples; ++index) {
 const double alpha = static_cast<double>(index) / samples;
 const sim::Vec2 point{
 nav_x + (subgoal.x - nav_x) * alpha,
 nav_y + (subgoal.y - nav_y) * alpha,
 };
 if (!planning_map_->isFree(point.x, point.y) ||
 !std::isfinite(navigation_traversal_cost(previous, point))) {
 return false;
 }
 previous = point;
 }
 return true;
 };
 const auto strategy_request = high_level_policy_->decide(strategy_observation);
 // Speed and stability constraints belong to the strategy boundary,
 // regardless of who produced the request. Context flags avoid duplicating
 // obstacle arbitration already owned by the validated local planner.
 snapshot_.strategy =
 sim::apply_strategy_safety(strategy_request, strategy_observation, strategy_context);
 snapshot_.strategy_policy = high_level_policy_->name();
 if (snapshot_.strategy.applied.has_subgoal)
 target = snapshot_.strategy.applied.subgoal;
 const double chassis_cruise = property(behavior_, "cruise_mps") *
 (physics_mode_ == PhysicsMode::Bullet ? chassis_caps_.rigid_body_cruise_scale : 1.0);
 auto route_control = control_system_->follow_route(
 planned_route_, estimate, property(behavior_, "lookahead_m"), chassis_cruise,
 1.0 / decision_hz(), *planning_map_, chassis_caps_);
 if (snapshot_.strategy.applied.has_subgoal) {
 const double target_distance = distance({nav_x, nav_y}, target);
 const double subgoal_error = sim::heading_err(target, nav_x, nav_y, nav_heading);
 route_control.tv = std::min(chassis_cruise, target_distance);
 route_control.steer = control_system_->heading_command(subgoal_error, 1.0 / decision_hz());
 route_control.err = subgoal_error;
 route_control.reached = target_distance < 0.25;
 }
 snapshot_.control.local_target = target;
 snapshot_.control.heading_error_rad = route_control.err;
 snapshot_.control.heading_pid_p = control_system_->pid_p();
 snapshot_.control.heading_pid_i = control_system_->pid_i();
 snapshot_.control.heading_pid_d = control_system_->pid_d();
 // Docking controller: guide_step reports reached when waypoint bookkeeping
 // exhausts (accept/resync ran ahead of a lagging chassis), but mission
 // success is distance-to-goal. If the goal is still beyond tolerance yet
 // close, dock with the same PID instead of parking short until timeout.
 // Structure mirrors guide_step (reverse hysteresis, spin capability branch)
 // so behavior stays consistent across the tracking/docking seam.
 {
 const GoalStep* active = active_goal();
 sim::Vec2 goal = active
 ? as_vec(map_definition_.markers.at(active->marker))
 : as_vec(map_definition_.markers.at(level_.objective.goal));
 double arrival_radius = active ? active->radius_m : level_.objective.arrive_tolerance_m;
 if (endless_mode_) {
 const auto& intent = field_operator_system_.intent();
 if (intent.active) {
 goal = intent.target;
 arrival_radius = 0.25;
 } else if (dispatch_has_priority(
 field_dispatch_system_.state(), snapshot_.world_activity.incident)) {
 goal = field_dispatch_system_.state().target;
 arrival_radius = 0.85;
 } else if (snapshot_.world_activity.incident.active) {
 goal = snapshot_.world_activity.incident.position;
 arrival_radius = std::max(0.25, snapshot_.world_activity.incident.radius_m);
 }
 }
  const double dist_goal = distance({nav_x, nav_y}, goal);
 const double tracking_arrival_radius = std::min(arrival_radius, 0.25);
  // Bounded: far exhaustion means the follower lost the path, not that the
  // goal is near; beelining across the map without waypoints risks walls.
  // (An unbounded trial stranded swift tracks in a reversing cycle instead
  // of docking — evidence for curation, not for a longer leash.)
 if (route_control.reached && dist_goal > tracking_arrival_radius && dist_goal < 3.0) {
   const double dock_err = sim::heading_err(goal, nav_x, nav_y, nav_heading);
   if (!control_system_->reversing() && std::abs(dock_err) > 1.4)
    control_system_->set_reversing(true);
   else if (control_system_->reversing() && std::abs(dock_err) < 0.7)
    control_system_->set_reversing(false);
   const double dock_dt = 1.0 / decision_hz();
   if (!control_system_->reversing()) {
    route_control.tv = std::min(0.3, dist_goal);
    route_control.steer = control_system_->heading_command(dock_err, dock_dt);
   } else if (chassis_->caps().can_spin) {
    control_system_->set_reversing(false);
    route_control.tv = 0.0;
    route_control.steer = control_system_->heading_command(dock_err, dock_dt);
   } else {
    route_control.tv = -0.3;
    route_control.steer = -control_system_->heading_command(dock_err, dock_dt);
   }
   route_control.err = dock_err;
   route_control.reached = false;
  }
 }
 route_control.tv *= snapshot_.strategy.applied.speed_scale;
 const double unavailable = std::numeric_limits<double>::quiet_NaN();
 if (snapshot_.strategy.applied.mode == sim::StrategyMode::Hold) {
 held_command_ = {};
 snapshot_.waiting_for_clearance = true;
 snapshot_.control.local_target = target;
 snapshot_.control.heading_error_rad = route_control.err;
 publish("high-level-intent", "strategy-hold", "wait", estimate.timestamp_s, {}, unavailable,
 unavailable, unavailable);
 update_shadow_episode();
 return;
 }
 if (actual_actors_.empty()) {
  const double preview_cap = corner_speed_cap(nav_x, nav_y);
  held_command_ = {route_control.tv > 0.0 ? std::min(route_control.tv, preview_cap) : route_control.tv,
                   route_control.steer};
  snapshot_.waiting_for_clearance = false;
  update_shadow_episode();
  snapshot_.control.shadow_assessed = false;
  snapshot_.control.shadow_pocket_verdict = -1;
  snapshot_.control.shadow_pocket_feasible = 0;
  snapshot_.control.shadow_pocket_total = 0;
  for (int& bin : snapshot_.control.shadow_pocket_reasons) {
   bin = 0;
  }
  const auto governor_idle = sim::govern_entry(governor_state_, sim::GovernorInput{});
  snapshot_.control.governor_hold = governor_idle.hold_active;
  snapshot_.control.governor_stop = governor_idle.stop_point;
 publish("none", "route-clear", "follow-route", estimate.timestamp_s, {}, unavailable, unavailable,
 unavailable);
  return;
 }

 const auto known_tracks =
   sensor_system_->dynamic_obstacles().active_tracks(snapshot_.time_s, track_max_age_s());

 std::vector<std::string> observation_refs;
 double observation_time_s = std::numeric_limits<double>::quiet_NaN();
 observation_refs.reserve(known_tracks.size());
 for (const auto& track : known_tracks) {
 observation_refs.push_back("track." + std::to_string(track.object_id));
 if (!std::isfinite(observation_time_s) || track.last_observed_s > observation_time_s)
 observation_time_s = track.last_observed_s;
 }

 double nearest_actor = std::numeric_limits<double>::infinity();
 for (const auto& track : known_tracks) {
  const auto& actor = track.state;
  nearest_actor = std::min(nearest_actor, distance({nav_x, nav_y}, actor.p) - actor.radius -
                                            robot_definition_.collision_radius_m);
 }

 const auto* computer = resolved_build_.computer();
 const double prediction_horizon = computer ? computer->prediction_horizon_s : 1.2;
 const auto* active_lidar = resolved_build_.lidar();
 const double scan_motion_margin = active_lidar
 ? 2.0 * (property(behavior_, "cruise_mps") +
 yaw_cap_rads_ * robot_definition_.collision_radius_m) / active_lidar->scan_hz
 : 0.0;
 // A rotating scan is up to one frame old and the tracker consumes it on the
 // next decision. Reserve the distance the robot footprint can sweep across
 // those two periods. This belongs to dynamic avoidance; inflating the static
 // global map with it would incorrectly close narrow but stationary routes.
 const double requested_clearance =
 property(behavior_, "safe_clearance_m") + scan_motion_margin;
 const double risk_distance =
   requested_clearance * requested_clearance * 2.5 + property(behavior_, "cruise_mps") * 0.6 + 0.1;
 if (known_tracks.empty() || nearest_actor > risk_distance) {
  const double preview_cap = corner_speed_cap(nav_x, nav_y);
  held_command_ = {route_control.tv > 0.0 ? std::min(route_control.tv, preview_cap) : route_control.tv,
                   route_control.steer};
  snapshot_.waiting_for_clearance = false;
  update_shadow_episode();
  snapshot_.control.shadow_assessed = false;
  snapshot_.control.shadow_pocket_verdict = -1;
  snapshot_.control.shadow_pocket_feasible = 0;
  snapshot_.control.shadow_pocket_total = 0;
  for (int& bin : snapshot_.control.shadow_pocket_reasons) {
   bin = 0;
  }
  const auto governor_idle = sim::govern_entry(governor_state_, sim::GovernorInput{});
  snapshot_.control.governor_hold = governor_idle.hold_active;
  snapshot_.control.governor_stop = governor_idle.stop_point;
 publish("none", known_tracks.empty() ? "no-dynamic-track" : "dynamic-track-clear", "follow-route",
 observation_time_s, std::move(observation_refs), prediction_horizon,
 std::isfinite(nearest_actor) ? nearest_actor : unavailable, unavailable);
  return;
 }

 const double heading_error = sim::heading_err(target, nav_x, nav_y, nav_heading);
 double speed_cap = std::min(std::max(0.20, sim::sched_v(heading_error, property(behavior_, "cruise_mps"))),
                             corner_speed_cap(nav_x, nav_y));
 speed_cap *= snapshot_.strategy.applied.speed_scale;
 const sim::DwaWeights weights{1.0, 1.3 + requested_clearance * 1.8, 0.25};
 constexpr double prediction_step_s = 0.1;
 // Predictor module (src/autonomy/predictor.*) owns future frames; the tracker
 // owns observations. known_tracks (age-advanced, for display) must not
 // feed prediction: map.predict() snapshots raw tracker state internally.
 const auto predicted_frames = sensor_system_->dynamic_obstacles().predict(
   snapshot_.time_s, prediction_horizon, prediction_step_s, track_max_age_s());
 // Shadow departure assessment (gate 3): observe only. This block reads
 // tracker state and writes snapshot shadow fields; held_command_ (the actual
 // control output) is untouched by everything in this block.
 std::vector<sim::Vec2> shadow_remaining{{nav_x, nav_y}};
 for (std::size_t i = control_system_->path_index(); i < planned_route_.size(); ++i) {
 shadow_remaining.push_back(planned_route_[i].position);
 }
 const double shadow_cruise = std::max(1e-6, property(behavior_, "cruise_mps"));
 {
  double shadow_length = 0.0;
  for (std::size_t i = 0; i + 1 < shadow_remaining.size(); ++i) {
   shadow_length += distance(shadow_remaining[i + 1], shadow_remaining[i]);
  }
  // Wait slack = the computer's own prediction horizon (existing content, not
  // a new knob): waits beyond what the computer predicts are unassessable by
  // construction (kHorizonShort) — which is itself shadow data.
  const double shadow_horizon = shadow_length / shadow_cruise + prediction_horizon;
  const auto shadow_frames = sensor_system_->dynamic_obstacles().predict(
    snapshot_.time_s, shadow_horizon, prediction_step_s, track_max_age_s());
  const auto assessment =
    sim::assess_departure(shadow_remaining, 0, shadow_cruise, shadow_frames, prediction_step_s,
                          robot_definition_.collision_radius_m, requested_clearance);
  snapshot_.control.shadow_assessed = true;
  snapshot_.control.shadow_depart_s = assessment.wait_s;
  snapshot_.control.shadow_reason = static_cast<int>(assessment.reason);
 }
 // Spatial pocket shadow (Gate B observation): same read-only contract, only
 // while waiting (last cycle's flag) — bounded cost, and waits are exactly
 // what Gate B must accumulate. Observation budget (window/offsets/cap), not
 // tuned: it cannot affect behavior by construction.
 snapshot_.control.shadow_pocket_verdict = -1;
 snapshot_.control.shadow_pocket_feasible = 0;
 snapshot_.control.shadow_pocket_total = 0;
 for (int& bin : snapshot_.control.shadow_pocket_reasons) {
  bin = 0;
 }
 if (snapshot_.waiting_for_clearance) {
  sim::PocketSampleParams pocket_params;
  pocket_params.lateral_offsets_m = {1.5, -1.5};
  pocket_params.window_m = 6.0;
  pocket_params.vertex_stride_m = 1.5;
  pocket_params.min_downstream_m = 3.0;
  pocket_params.min_pocket_dist_m = 0.3;
  pocket_params.max_candidates = 12;
  double shadow_length = 0.0;
  for (std::size_t i = 0; i + 1 < shadow_remaining.size(); ++i) {
   shadow_length += distance(shadow_remaining[i + 1], shadow_remaining[i]);
  }
  const double pocket_horizon = shadow_length / shadow_cruise + prediction_horizon;
  const auto pocket_frames = sensor_system_->dynamic_obstacles().predict(
    snapshot_.time_s, pocket_horizon, prediction_step_s, track_max_age_s());
  const auto search = sim::sample_pockets(
    *planning_map_, shadow_remaining, {nav_x, nav_y}, nav_heading, shadow_cruise,
 sim::ChassisExecCaps{chassis_caps_.can_spin, chassis_caps_.max_yaw_rate_radps,
 chassis_caps_.geometry.steering_wheel_base_m,
 chassis_caps_.geometry.max_steer_rad,
 chassis_caps_.geometry.max_steer_rate_radps,
 chassis_caps_.geometry.steering_response_s},
    prediction_horizon, pocket_params, pocket_frames, prediction_step_s, robot_definition_.collision_radius_m,
    requested_clearance);
  int pocket_feasible = 0;
  for (const auto& candidate : search.candidates) {
   if (candidate.assessment.overall == sim::Feasibility::Verdict::kFeasible) {
    ++pocket_feasible;
   }
   ++snapshot_.control.shadow_pocket_reasons[static_cast<int>(candidate.assessment.transit.reason)];
   ++snapshot_.control.shadow_pocket_reasons[static_cast<int>(candidate.assessment.hold.reason)];
   ++snapshot_.control.shadow_pocket_reasons[static_cast<int>(candidate.assessment.rejoin.reason)];
  }
  snapshot_.control.shadow_pocket_verdict = static_cast<int>(search.verdict);
  snapshot_.control.shadow_pocket_feasible = pocket_feasible;
  snapshot_.control.shadow_pocket_total = static_cast<int>(search.candidates.size());
 }
 // Entry governor (Phase-2, Behavior one-sided brake): fresh fatal tracks
 // feed the window shadow; Occupied engages HOLD_AT now, sustained Clear or
 // prolonged silence releases. Social tracks never govern (proximity already
 // eats clearance for them, contact never fails — holding would be pure
 // loss). Stopped ego is assessed at a 0.5 m/s departure floor
 // (conservative: slower than any cruise, so the window is pessimistic).
 // Stop geometry uses PREDICTED threat occupancy (all forecast frames), not
 // current positions: parking next to the incoming threat's future path is
 // how the first wiring collided on signal_hold (39.4s hold -> rammed while
 // stopped). The stop point must clear the whole swept band.
 sim::GovernorOutput governor_out;
 {
  const sim::Vec2 ego{nav_x, nav_y};
  const sim::Vec2 dir{std::cos(nav_heading), std::sin(nav_heading)};
  const double ego_speed = std::abs(estimate.linear_velocity_mps);
  const double assess_speed = ego_speed >= 0.15 ? ego_speed : 0.5;
  sim::GovernorInput governor_input;
  for (const auto& track : known_tracks) {
   if (track.object_id >= agents_.size() || agents_[track.object_id].social) {
    continue;
   }
   const double age = snapshot_.time_s - track.last_observed_s;
   if (age < 0.0 || age > sim::kEntryStaleBudgetS) {
    continue;
   }
   const sim::Vec2 tp{track.state.p.x, track.state.p.y};
   const sim::Vec2 tv{track.state.v.x, track.state.v.y};
   const auto center = sim::governor_conflict_center(ego, dir, tp, tv);
   if (std::hypot(center.x - ego.x, center.y - ego.y) > 10.0) {
    continue;
   }
   const double half = 0.45 + track.state.radius + 0.2;
   sim::WindowThreat threat;
   threat.valid = true;
   threat.dist_to_zone_m = std::hypot(tp.x - center.x, tp.y - center.y) - half;
   const double to_center = std::max(1e-6, std::hypot(center.x - tp.x, center.y - tp.y));
   const double toward = (tv.x * (center.x - tp.x) + tv.y * (center.y - tp.y)) / to_center;
   threat.speed_mps = (toward > 0.0) ? std::hypot(tv.x, tv.y) : 0.0;
   threat.age_s = age;
   const double ego_dist = std::max(0.0, std::hypot(center.x - ego.x, center.y - ego.y) - half);
   const auto assessment =
     sim::assess_entry_window(ego_dist, assess_speed, half, threat, sim::kEntryStaleBudgetS);
   if (assessment.verdict == sim::EntryVerdict::kObservedOccupied) {
    governor_input.occupied_fresh = true;
   } else if (assessment.verdict == sim::EntryVerdict::kObservedClear) {
    governor_input.clear_fresh = true;
   }
  }
  if (governor_input.occupied_fresh) {
   governor_input.clear_fresh = false; // occupied wins mixed frames.
  }
  // Corridor entry against every forecast frame: the first remaining-route
  // point inside any predicted swept disc, walked back brake + margin.
  if (governor_input.occupied_fresh || governor_input.clear_fresh) {
   auto near_predicted = [&](sim::Vec2 p) {
    for (const auto& frame : predicted_frames) {
     for (const auto& future : frame) {
      if (std::hypot(p.x - future.p.x, p.y - future.p.y) < 0.9) {
       return true;
      }
     }
    }
    return false;
   };
   std::size_t entry = shadow_remaining.size();
   for (std::size_t i = 0; i < shadow_remaining.size(); ++i) {
    if (near_predicted(shadow_remaining[i])) {
     entry = i;
     break;
    }
   }
   if (entry < shadow_remaining.size()) {
    const double cruise = property(behavior_, "cruise_mps");
    const double brake = resolved_build_.tires().brake_decel_mps2;
    const double need = cruise * cruise / (2.0 * brake) + 1.0;
    double walked = 0.0;
    std::size_t stop = entry;
    while (stop > 0 && walked < need) {
     walked += std::hypot(shadow_remaining[stop].x - shadow_remaining[stop - 1].x,
                          shadow_remaining[stop].y - shadow_remaining[stop - 1].y);
     --stop;
    }
    governor_input.stop_point = shadow_remaining[stop];
    governor_input.stop_valid = true;
   }
  }
  const bool was_hold = governor_state_.hold_active;
  governor_out = sim::govern_entry(governor_state_, governor_input);
  snapshot_.control.governor_hold = governor_out.hold_active;
  snapshot_.control.governor_stop = governor_out.stop_point;
  if (governor_out.hold_active && !was_hold) {
   record_event("governor_hold", catalog_.tr("event.governor_hold"));
  }
 }
 // Virtual wall (Phase-2 consumption): while the governor holds, cap speed
 // by stopping physics to the stop point. DWA still owns steering and the
 // release decision; this clamp only withholds speed, never commands it.
 if (governor_out.hold_active && governor_out.stop_valid) {
  const double brake = resolved_build_.tires().brake_decel_mps2;
  const double dist = std::hypot(nav_x - governor_out.stop_point.x, nav_y - governor_out.stop_point.y);
  const double v_allow = dist <= 0.05 ? 0.0 : std::sqrt(2.0 * brake * dist);
  speed_cap = std::min(speed_cap, v_allow);
 }
 sim::DwaResult dwa;
 if (chassis_->caps().command_space == sim::ChassisCommandSpace::HolonomicTwist) {
 sim::HolonomicLocalConfig local_config;
 local_config.max_speed_mps = speed_cap;
 local_config.max_yaw_radps = yaw_cap_rads_;
 local_config.max_accel_mps2 = resolved_build_.tires().brake_decel_mps2;
 local_config.max_yaw_accel_radps2 = robot_definition_.max_yaw_accel_radps2;
 local_config.predict_time_s = prediction_horizon;
 local_config.dt_s = prediction_step_s;
 local_config.frame_dt_s = prediction_step_s;
 local_config.safety_margin_m = requested_clearance;
 local_config.robot_radius_m = robot_definition_.collision_radius_m;
 dwa = sim::holonomic_local_compute(
 {nav_x, nav_y}, nav_heading, estimate.linear_velocity_mps, snapshot_.robot.lateral_v,
 estimate.angular_velocity_radps, target, *planning_map_, weights, predicted_frames,
 local_config, [this](sim::Vec2 from, sim::Vec2 to) {
 return std::isfinite(navigation_traversal_cost(from, to));
 });
 } else if (chassis_caps_.command_space == sim::ChassisCommandSpace::SteeringAngle) {
 sim::AckermannLocalConfig local_config;
 local_config.wheel_base_m = chassis_caps_.geometry.steering_wheel_base_m;
 local_config.max_steer_rad = chassis_caps_.geometry.max_steer_rad;
 local_config.max_steer_rate_radps = chassis_caps_.geometry.max_steer_rate_radps;
 local_config.max_accel_mps2 = resolved_build_.tires().brake_decel_mps2;
 local_config.max_speed_mps = speed_cap;
 local_config.preferred_direction =
 control_system_->travel_direction(planned_route_);
 local_config.predict_time_s = prediction_horizon;
 local_config.dt_s = prediction_step_s;
 local_config.frame_dt_s = prediction_step_s;
 local_config.safety_margin_m = requested_clearance;
 local_config.robot_radius_m = robot_definition_.collision_radius_m;
 dwa = sim::ackermann_local_compute(
 {nav_x, nav_y}, nav_heading, estimate.linear_velocity_mps, chassis_->telemetry().steer.angle_rad,
 target, *planning_map_, weights, predicted_frames, local_config,
 [this](sim::Vec2 from, sim::Vec2 to) {
 return std::isfinite(navigation_traversal_cost(from, to));
 });
 } else {
 dwa = sim::dwa_compute_trajectory(
 {nav_x, nav_y}, nav_heading, estimate.linear_velocity_mps, estimate.angular_velocity_radps, target,
 *planning_map_, weights, predicted_frames, speed_cap, yaw_cap_rads_,
 resolved_build_.tires().brake_decel_mps2, prediction_horizon, prediction_step_s, prediction_step_s,
 requested_clearance, robot_definition_.collision_radius_m, robot_definition_.max_yaw_accel_radps2,
 [this](sim::Vec2 from, sim::Vec2 to) {
 return std::isfinite(navigation_traversal_cost(from, to));
 });
 }
 snapshot_.control.dwa_active = true;
 snapshot_.control.dwa_selected = dwa.best_path;
 double predicted_separation_m = std::numeric_limits<double>::quiet_NaN();
 for (std::size_t frame_index = 0;
 frame_index < predicted_frames.size() && frame_index < dwa.best_path.size(); ++frame_index) {
 for (const auto& future : predicted_frames[frame_index]) {
 const double separation = distance(dwa.best_path[frame_index], future.p) - future.radius -
 robot_definition_.collision_radius_m;
 if (!std::isfinite(predicted_separation_m) || separation < predicted_separation_m)
 predicted_separation_m = separation;
 }
 }
 constexpr std::size_t kVisibleDwaCandidates = 24;
 const std::size_t candidate_stride =
   std::max<std::size_t>(1, (dwa.rollouts.size() + kVisibleDwaCandidates - 1) / kVisibleDwaCandidates);
 for (std::size_t index = 0; index < dwa.rollouts.size(); index += candidate_stride) {
  const auto& rollout = dwa.rollouts[index];
  snapshot_.control.dwa_candidates.push_back({
    rollout.points,
    rollout.collision,
    rollout.command.score,
  });
 }
 if (!dwa.best.valid) {
  held_command_ = {0.0, 0.0};
 // A local-planner failure enters the zero-speed safety state. A recovery
 // planner needs its own dynamically validated trajectory and termination
 // contract; an unverified steering heuristic must never bypass DWA.
  if (!snapshot_.waiting_for_clearance) {
   std::ostringstream waiting;
   waiting << std::fixed << std::setprecision(1) << nearest_actor;
   record_event("waiting", catalog_.trf("event.waiting", {waiting.str()}));
  }
  snapshot_.waiting_for_clearance = true;
  shadow_open_speed_ = 0.0;
  for (const auto& track : known_tracks) {
   shadow_open_speed_ = std::max(shadow_open_speed_, std::hypot(track.state.v.x, track.state.v.y));
  }
  shadow_open_nearest_ = nearest_actor;
  update_shadow_episode();
 } else {
  // Arbitration without magic gains: while the avoidance speed is healthy, DWA
  // owns both speed and turn; at creep speed the robot holds the route heading
  // with the same derived proportional gain (P-only: integrator and derivative
  // states are meaningless at standstill and would wind up or kick).
  constexpr double kCreepSpeedMps = 0.15;
  const double arbitrated_turn = std::abs(dwa.best.v) < kCreepSpeedMps
 ? (chassis_caps_.command_space == sim::ChassisCommandSpace::SteeringAngle
 ? route_control.steer
 : std::clamp(heading_kp_ * heading_error, -yaw_cap_rads_, yaw_cap_rads_))
                                   : dwa.best.w;
  held_command_ = {dwa.best.v < 0.0 ? dwa.best.v
                                    : std::min({std::max(0.0, route_control.tv), dwa.best.v,
                                                corner_speed_cap(nav_x, nav_y)}),
                   arbitrated_turn};
 held_command_.target_lateral_v = dwa.best.lateral_v;
  snapshot_.waiting_for_clearance = std::abs(dwa.best.v) < kCreepSpeedMps;
  if (snapshot_.waiting_for_clearance) {
   std::ostringstream waiting;
   waiting << std::fixed << std::setprecision(1) << nearest_actor;
   record_event("waiting", catalog_.trf("event.waiting", {waiting.str()}), 2.0);
  }
  shadow_open_speed_ = 0.0;
  for (const auto& track : known_tracks) {
   shadow_open_speed_ = std::max(shadow_open_speed_, std::hypot(track.state.v.x, track.state.v.y));
  }
  shadow_open_nearest_ = nearest_actor;
  update_shadow_episode();
 }
 const bool minimum_risk_escape = !dwa.best.valid &&
 (std::abs(held_command_.target_v) > 1e-9 || std::abs(held_command_.target_w) > 1e-9);
 const std::string action_code = minimum_risk_escape
 ? "minimum-risk-escape"
 : (snapshot_.waiting_for_clearance ? "wait" : (dwa.best.valid ? "avoid" : "wait"));
 const std::string reason_code = !dwa.best.valid
 ? "no-safe-local-trajectory"
 : (governor_out.hold_active
 ? "entry-window-occupied"
 : (snapshot_.waiting_for_clearance ? "predicted-path-conflict" : "safe-local-trajectory"));
 publish(reason_code == "safe-local-trajectory" ? "none" : "dynamic-clearance", reason_code,
 action_code, observation_time_s, std::move(observation_refs), prediction_horizon,
 std::isfinite(nearest_actor) ? nearest_actor : unavailable, predicted_separation_m);
}

void MissionSimulation::apply_motion() {
 if (!runtime_channel_) {
 throw std::logic_error("mission has no active runtime channel");
 }
 const auto runtime_now = sim::runtime::TimePoint::from_seconds(snapshot_.time_s);
 const auto outbound_command = sim::runtime::make_motion_command(held_command_, chassis_caps_, runtime_now,
 command_sequence_++, fixed_step_s * 2.0);
 if (runtime_channel_->autonomy().write_motion(outbound_command) !=
 sim::runtime::CommandWriteResult::Accepted) {
 throw std::runtime_error("runtime channel rejected the autonomous motion command");
 }
 const sim::DiffCmd applied_command =
 sim::runtime::to_diff_command(runtime_channel_->platform().read_motion(runtime_now));
 snapshot_.control.target_v_mps = applied_command.target_v;
 snapshot_.control.target_w_radps = applied_command.target_w;
 // NOTE (FF verdict): a shadow feedforward computation lived here in stage 3
 // and was removed when stage 4 measured net-negative tracking benefit with
 // saturation up. The pure mapping (src/hw/feedforward.*) and its evidence
 // (test_feedforward, test_ff_sensitivity) stay parked with the verdict; no
 // per-step shadow compute remains in this hot path.
 const auto before = snapshot_.robot;
 const auto ground_contact = wheel_ground({before.x, before.y});
 if (platform_)
 chassis_->set_actuator_feedback(platform_->actuator_feedback());
 auto domain_candidate = chassis_->step(before, applied_command, fixed_step_s, ground_contact);
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
 domain_candidate.x = before.x +
 (domain_candidate.v * std::cos(domain_candidate.theta) -
 domain_candidate.lateral_v * std::sin(domain_candidate.theta)) * fixed_step_s;
 domain_candidate.y = before.y +
 (domain_candidate.v * std::sin(domain_candidate.theta) +
 domain_candidate.lateral_v * std::cos(domain_candidate.theta)) * fixed_step_s;
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
 result_.max_drive_slip_ratio = std::max(result_.max_drive_slip_ratio,
 rigid_state.max_drive_slip_ratio);
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
 const auto& drive_unit = motors.differential ? motors.left
 : (motors.holonomic ? motors.modules[0] : motors.drive);
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
 snapshot_.control.mount_actuations.reserve(
 count + 2 * robot_definition_.passive_supports.size());
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
 roll = std::remainder(roll + speed_mps / mount->rolling_radius_m * fixed_step_s,
 2.0 * sim::kPi);
 }
 const double steer = index < motors.wheel_steer_count
 ? motors.wheel_steer_rad[index]
 : 0.0;
 snapshot_.control.mount_actuations.push_back(
 {tire_slot->mounts[index], steer, roll, std::nullopt});
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

 const bool outside_world = candidate.x < 0.0 || candidate.y < 0.0 ||
 candidate.x >= map_definition_.size_m.x || candidate.y >= map_definition_.size_m.y;
 if (physics_mode_ == PhysicsMode::Bullet) {
 if (have_rigid_state && rigid_state.static_collision)
 rigid_static_contact_s_ += fixed_step_s;
 else
 rigid_static_contact_s_ = 0.0;
 }
 const bool static_collision = physics_mode_ == PhysicsMode::Bullet
 ? (outside_world || rigid_static_contact_s_ >= 0.25)
 : !collision_map_->isFree(candidate.x, candidate.y);
 if (static_collision) {
  ++result_.collisions;
 if (!endless_mode_)
 finish_failure("collision-static", catalog_.tr("event.collision_static"));
  return;
 }
 for (std::size_t index = 0; index < actual_actors_.size(); ++index) {
  const auto& actor = actual_actors_[index];
  const double clearance =
    distance(candidate_position, actor.p) - actor.radius - robot_definition_.collision_radius_m;
  result_.min_clearance_m = std::min(result_.min_clearance_m, clearance);
  if (clearance < result_.min_dynamic_clearance_m) {
   result_.min_dynamic_clearance_m = clearance;
   result_.min_dynamic_clearance_time_s = snapshot_.time_s;
   result_.min_dynamic_robot_position = candidate_position;
   result_.min_dynamic_actor_position = actor.p;
  }
  const bool social = index < agents_.size() && agents_[index].social;
 const bool rigid_actor_collision = physics_mode_ == PhysicsMode::Bullet && have_rigid_state &&
 std::find(rigid_state.actor_collisions.begin(), rigid_state.actor_collisions.end(), agents_[index].id) !=
 rigid_state.actor_collisions.end();
 const bool actor_collision = physics_mode_ == PhysicsMode::Bullet ? rigid_actor_collision
 : clearance < 0.0;
 if (actor_collision && social) {
   // Social contact: proximity already ate the clearance score above; a
   // pedestrian never ends the run. Counted as evidence, not failure.
   ++result_.pedestrian_contacts;
   continue;
  }
 if (actor_collision) {
   ++result_.collisions;
 if (!endless_mode_)
 finish_failure("collision-dynamic", catalog_.tr("event.collision_dynamic"));
   return;
  }
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
 const double grade_power =
 std::max(0.0, total_mass_kg() * 9.81 * candidate.v * candidate_grade /
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
 const bool saturated = motors.traction_saturated ||
 (physics_mode_ == PhysicsMode::Bullet && have_rigid_state && rigid_state.drive_slipping);
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
 motion_observation.encoders->header =
 {motion_observation.time_s, motion_sequence_++, "base_link", 0.0, 1, "sim.encoder"};
 if (imu) {
 if (!sensor_system_)
 throw std::logic_error("configured IMU requires the hardware sensor system");
 const sim::ImuTruth truth{
 physics_mode_ == PhysicsMode::Bullet && have_rigid_state
 ? rigid_state.imu_angular_velocity_radps.x : 0.0,
 physics_mode_ == PhysicsMode::Bullet && have_rigid_state
 ? rigid_state.imu_angular_velocity_radps.y : 0.0,
 physics_mode_ == PhysicsMode::Bullet && have_rigid_state
 ? rigid_state.imu_angular_velocity_radps.z : candidate.w,
 physics_mode_ == PhysicsMode::Bullet && have_rigid_state
 ? rigid_state.imu_specific_force_mps2.x : (candidate.v - before.v) / fixed_step_s,
 physics_mode_ == PhysicsMode::Bullet && have_rigid_state
 ? rigid_state.imu_specific_force_mps2.y
 : (candidate.lateral_v - before.lateral_v) / fixed_step_s + candidate.v * candidate.w,
 physics_mode_ == PhysicsMode::Bullet && have_rigid_state
 ? rigid_state.imu_specific_force_mps2.z : 9.81,
 };
 const sim::ImuConfig config{
 imu->gyro_noise_radps,
 imu->gyro_bias_stddev_radps,
 imu->gyro_bias_random_walk_radps_sqrt_s,
 imu->accel_noise_mps2,
 "imu_link",
 0.0,
 };
 motion_observation.imu = sensor_system_->capture_imu(
 truth, motion_observation.time_s, fixed_step_s, config);
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
 const double fixed_clearance = static_clearance(candidate_position);
 if (fixed_clearance < result_.min_static_clearance_m) {
  result_.min_static_clearance_m = fixed_clearance;
  result_.min_static_clearance_time_s = snapshot_.time_s;
  result_.min_static_robot_position = candidate_position;
 }
 result_.min_clearance_m = std::min(result_.min_clearance_m, fixed_clearance);
}

double MissionSimulation::surface_traction(sim::Vec2 position) const {
 for (const auto& surface : map_definition_.surfaces) {
  if (active_in_level(surface.active_in, level_id_) && inside(position, surface)) {
 return surface.traction_scale * environment_grip_multiplier(surface.material);
  }
 }
 return 1.0;
}

double MissionSimulation::static_clearance(sim::Vec2 position) const {
 double minimum = std::min({position.x, position.y, map_definition_.size_m.x - position.x,
                            map_definition_.size_m.y - position.y}) -
                  robot_definition_.collision_radius_m;
 for (const auto& obstacle : map_definition_.obstacles) {
  if (!active_in_level(obstacle.active_in, level_id_)) {
   continue;
  }
  const double dx = std::max({obstacle.min.x - position.x, 0.0, position.x - obstacle.max.x});
  const double dy = std::max({obstacle.min.y - position.y, 0.0, position.y - obstacle.max.y});
  minimum = std::min(minimum, std::hypot(dx, dy) - robot_definition_.collision_radius_m);
 }
 return std::max(0.0, minimum);
}

void MissionSimulation::finish_success() {
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
                      result_.max_odometry_error_m <= level_.max_localization_error_m;
 if (active_contract_) {
 result_.cargo_integrity = std::min(result_.cargo_integrity, std::clamp(
 1.0 - active_contract_->fragility *
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

void MissionSimulation::finish_failure(std::string reason, std::string message) {
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

void MissionSimulation::record_event(std::string code, std::string message, double throttle_s) {
 if (code == last_event_code_ && snapshot_.time_s - last_event_time_s_ < throttle_s) {
  return;
 }
 last_event_code_ = code;
 last_event_time_s_ = snapshot_.time_s;
 events_.push_back({snapshot_.time_s, std::move(code), std::move(message)});
}

void MissionSimulation::publish_decision(DecisionEvidence decision) {
 const auto same_decision = [&] {
 if (decisions_.empty())
 return false;
 const auto& previous = decisions_.back();
 return previous.constraint_code == decision.constraint_code &&
 previous.reason_code == decision.reason_code && previous.action_code == decision.action_code &&
 previous.outcome_code == "active";
 }();

 decision.outcome_code = "active";
 decision.end_time_s = -1.0;
 decision.superseded_by = 0;
 if (same_decision) {
 decision.sequence = decisions_.back().sequence;
 decision.decision_time_s = decisions_.back().decision_time_s;
 decisions_.back() = decision;
 } else {
 const std::uint64_t next_sequence = ++decision_sequence_;
 if (!decisions_.empty() && decisions_.back().outcome_code == "active") {
 decisions_.back().end_time_s = snapshot_.time_s;
 decisions_.back().outcome_code = "superseded";
 decisions_.back().superseded_by = next_sequence;
 }
 decision.sequence = next_sequence;
 decision.decision_time_s = snapshot_.time_s;
 decisions_.push_back(decision);
 }
 snapshot_.decision = decision;
}

void MissionSimulation::close_decision(std::string outcome_code) {
 if (decisions_.empty())
 return;
 decisions_.back().end_time_s = snapshot_.time_s;
 decisions_.back().outcome_code = std::move(outcome_code);
 decisions_.back().superseded_by = 0;
 snapshot_.decision = decisions_.back();
}

void MissionSimulation::update_shadow_episode() {
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

void MissionSimulation::record_replay() {
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

void MissionSimulation::seek_replay(double time_s) {
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
double MissionSimulation::corner_speed_cap(double nav_x, double nav_y) const {
 const double cruise = property(behavior_, "cruise_mps");
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
 for (size_t i = control_system_->path_index() + 1; i < planned_route_.size(); ++i) {
 along += distance(planned_route_[i].position, planned_route_[i - 1].position);
  if (along > preview) {
   break;
  }
 grip = std::min(grip, wheel_ground(planned_route_[i].position).mu_eff);
 const double route_heading = planned_route_[i].heading_rad;
 const double cross_grade =
 sim::lateral_grade(terrain_.sample(planned_route_[i].position), route_heading);
 const double surface_angle = std::atan(std::abs(cross_grade));
 const double rollover_margin = 9.81 * std::cos(surface_angle) * std::tan(static_limit) -
 9.81 * std::sin(surface_angle);
 lateral_accel_limit = std::min(lateral_accel_limit, std::max(0.05, 0.45 * rollover_margin));
 }
 lateral_accel_limit = std::min(lateral_accel_limit, 0.5 * grip * 9.81);
 return sim::corner_speed_cap(planned_route_, control_system_->path_index(), cruise, lateral_accel_limit,
                              preview);
}

double MissionSimulation::decision_hz() const {
 const auto* computer = resolved_build_.computer();
 return computer ? computer->decision_hz : kBuiltInDecisionHz;
}

double MissionSimulation::track_max_age_s() const {
 // Dynamic-track memory lives on the computer; computer-less levels use the
 // documented engine default.
 const auto* computer = resolved_build_.computer();
 return computer ? computer->track_max_age_s : kDynamicTrackMaxAgeS;
}

sim::GroundContact MissionSimulation::wheel_ground(sim::Vec2 position) const {
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
 contact.normal_stiffness_Npm =
 1.0 / (1.0 / tire_stiffness + 1.0 / ground_stiffness);
 // The dynamic profile treats the material value as additional pair
 // dissipation. Identified profiles replace this reduced convention with a
 // fitted contact-pair damping value.
 contact.normal_damping_Nspm = tire_damping + ground_damping;
 return contact;
}

double MissionSimulation::navigation_traversal_cost(sim::Vec2 from, sim::Vec2 to) const {
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
 const double downhill_gravity_mps2 =
 signed_grade < 0.0 ? 9.81 * std::sin(slope) : 0.0;
 const double grip_brake_mps2 = std::max(0.0, contact.mu_eff) * 9.81 * std::cos(slope);
 const double available_brake_mps2 =
 std::min(resolved_build_.tires().brake_decel_mps2, grip_brake_mps2);
 if (downhill_gravity_mps2 > available_brake_mps2 + 1e-9) {
 return std::numeric_limits<double>::infinity();
 }
 const double brake_usage = downhill_gravity_mps2 / std::max(0.1, available_brake_mps2);
 const double dry_tire_mu = resolved_build_.tires().friction_mu / 0.8;
 const double low_grip_penalty =
 std::max(0.0, dry_tire_mu / std::max(0.08, contact.mu_eff) - 1.0);
 double extra_ground_drag = 0.0;
 for (const auto& surface : map_definition_.surfaces) {
 if (active_in_level(surface.active_in, level_id_) && inside(midpoint, surface)) {
 extra_ground_drag = std::max(0.0, surface.rolling_drag);
 break;
 }
 }
 const double rolling_penalty = extra_ground_drag * 5.0;
 return terrain_cost * (1.0 + 1.8 * brake_usage * brake_usage + 0.35 * low_grip_penalty +
 rolling_penalty);
}

} // namespace mission
