#pragma once

#include "robot/core/math.hpp"

#include <array>
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

namespace mission {

using Properties = std::unordered_map<std::string, double>;

struct Vec3 {
 double x{0.0};
 double y{0.0};
 double z{0.0};
};

struct VisualDefinition {
 std::string primitive_2d;
 Vec3 size_m;
 std::array<unsigned char, 4> color{255, 255, 255, 255};
 std::string model_3d;
};

struct Pose {
 double x{0.0};
 double y{0.0};
 double theta{0.0};
};

struct BoxDefinition {
 std::string id;
 sim::Vec2 min;
 sim::Vec2 max;
 std::vector<std::string> active_in;
};

struct SurfaceMaterialDefinition {
 std::string id;
 std::string title;
 std::string description;
 // Albedo texture for ground patches (empty = flat boxes, same as before).
 std::string texture_albedo;
 double friction_mu{1.0};
 double rolling_drag{0.0};
 double traction_scale{1.0};
 double normal_stiffness_Npm{1.0e9};
 double normal_damping_Nspm{0.0};
 double lidar_reflectivity{0.30};
 std::string drive_effect{"none"};
 std::array<unsigned char, 4> effect_color{255, 255, 255, 255};
 double effect_intensity{0.0};
};

struct SurfaceDefinition {
 std::string id;
 sim::Vec2 min;
 sim::Vec2 max;
 // Numbers resolved from the material library at load; rects carry only the
 // material id. Empty active_in = always on (wet_floor predates the gate).
 std::string material;
 std::vector<std::string> active_in;
 std::string label;
 double traction_scale{1.0};
 double friction_mu{0.8};
 double rolling_drag{0.0};
 double normal_stiffness_Npm{1.0e9};
 double normal_damping_Nspm{0.0};
 double lidar_reflectivity{0.30};
};

struct TerrainPatchDefinition {
 std::string id;
 sim::Vec2 min;
 sim::Vec2 max;
 double base_height_m{0.0};
 double grade_x{0.0};
 double grade_y{0.0};
 double roughness{0.0};
 std::vector<std::string> active_in;
};

// Authoritative authored height used by Bullet mesh generation and Robot Map
// export. Keeping one function prevents the physical world and 2.5D map from
// disagreeing about deterministic rough relief.
inline double terrain_patch_height(const TerrainPatchDefinition& patch,
 double x, double y) {
 const double plane = patch.base_height_m + patch.grade_x * (x - patch.min.x) +
 patch.grade_y * (y - patch.min.y);
 if (patch.roughness <= 0.0)
 return plane;
 const double width = std::max(0.1, patch.max.x - patch.min.x);
 const double depth = std::max(0.1, patch.max.y - patch.min.y);
 const double u = std::clamp((x - patch.min.x) / width, 0.0, 1.0);
 const double v = std::clamp((y - patch.min.y) / depth, 0.0, 1.0);
 const double edge_fade = std::sin(sim::kPi * u) * std::sin(sim::kPi * v);
 const double wave = std::sin(2.0 * sim::kPi * (x - patch.min.x) / 1.35) *
 std::sin(2.0 * sim::kPi * (y - patch.min.y) / 0.95);
 return plane + patch.roughness * edge_fade * wave;
}

struct ActorDefinition {
 std::string id;
 // Motion pattern (ping-pong patrol vs waypoint-loop wander), collision
 // semantics (fatal ends the run, social only eats clearance), and visual
 // (model path + the reference radius it was authored for) are all content:
 // the framework interprets, never branches on names.
 std::string kind{"patrol"};
 bool collision_social{false};
 std::vector<double> waits;
 std::string visual_model;
 // Rendering class is explicit content data. Motion behavior and collision
 // policy do not imply appearance: a social delivery cart is still a
 // vehicle, while a fatal moving hazard need not be one.
 std::string visual_kind{"vehicle"};
 double visual_ref_radius_m{0.45};
 std::vector<sim::Vec2> path;
 double speed_mps{0.0};
 double radius_m{0.0};
 std::vector<std::string> active_in;
 // Summon semantics: dormant actors are invisible and non-collidable until a
 // "summon" goal names them; their patrol clock starts at summon time, so the
 // encounter is reproducible from the summon moment, not wall-clock.
 bool dormant_until_summoned{false};
};

struct SceneShotDefinition {
 Vec3 offset_m;
 double target_height_m{0.45};
 double fov_degrees{41.0};
 double robot_focus_weight{1.0};
 double actor_focus_weight{0.0};
 double goal_focus_weight{0.0};
};

struct SceneCameraDefinition {
 Vec3 offset_m;
 double target_height_m{0.45};
 double fov_degrees{41.0};
 double robot_focus_weight{0.52};
 double actor_focus_weight{0.30};
 double goal_focus_weight{0.18};
 SceneShotDefinition event_shot;
 SceneShotDefinition success_shot;
 double transition_s{0.65};
};

struct SceneEffectsDefinition {
 double robot_scale{1.12};
 double bob_amplitude_m{0.012};
 double bob_frequency_hz{1.6};
 double eye_height_m{0.25};
 double scan_radius_m{1.35};
 double perception_link_height_m{0.42};
 double prediction_horizon_s{2.4};
 double success_ring_radius_m{0.72};
 double trail_duration_s{1.6};
 int surface_particle_count{18};
 double surface_particle_scale{1.0};
 double risk_indicator_radius_m{0.82};
};

struct SceneLightingDefinition {
 Vec3 direction;
 std::array<unsigned char, 4> directional_color{255, 255, 255, 255};
 double directional_intensity{1.0};
 std::array<unsigned char, 4> ambient_color{64, 64, 64, 255};
 std::array<unsigned char, 4> fog_color{8, 12, 12, 255};
 double fog_start_m{12.0};
 double fog_end_m{31.0};
 std::array<unsigned char, 4> robot_active_color{30, 220, 190, 255};
 std::array<unsigned char, 4> robot_waiting_color{232, 151, 52, 255};
 std::array<unsigned char, 4> goal_color{40, 220, 210, 255};
 int shadow_map_resolution{2048};
 double shadow_extent_m{28.0};
 double shadow_strength{0.62};
 double exposure{1.05};
 double bloom_threshold{0.82};
 double bloom_strength{0.18};
 double vignette_strength{0.14};
};

struct SceneFloorDefinition {
 std::string albedo_texture;
 std::array<unsigned char, 4> base_color{24, 28, 27, 255};
 std::array<unsigned char, 4> tile_color{66, 67, 62, 255};
 std::array<unsigned char, 4> boundary_color{46, 48, 46, 255};
 double tile_size_m{4.0};
 double seam_m{0.015};
 int color_variation{9};
 double boundary_height_m{0.24};
 double boundary_width_m{0.22};
};

struct ScenePartDefinition {
 std::string primitive;
 std::string model_3d;
 Vec3 position_m;
 Vec3 size_m{1.0, 1.0, 1.0};
 double yaw_degrees{0.0};
 std::array<unsigned char, 4> color{255, 255, 255, 255};
};

struct ScenePrefabDefinition {
 std::string id;
 std::vector<ScenePartDefinition> parts;
};

struct SceneGoalVisualDefinition {
 std::string id;
 // available, active, completed, abandoned. A missing state draws nothing.
 std::unordered_map<std::string, std::string> state_prefabs;
};

struct ScenePayloadVisualDefinition {
 std::string mount;
 std::string container_prefab;
 std::string item_prefab;
 std::string damaged_item_prefab;
 Vec3 item_origin_m;
 // Column spacing (+x), row spacing (+y), and layer spacing (+z) in the
 // robot frame. The item prefabs stay presentation-only; payload truth comes
 // from PayloadState and is replayed independently of this layout.
 Vec3 item_spacing_m;
 int grid_columns{1};
 int grid_rows{1};
 int items_per_visual{1};
 int max_visible_items{1};
 double damaged_below_integrity{0.75};
 double pickup_effect_duration_s{0.0};
 int pickup_effect_particles{0};
 std::array<unsigned char, 4> pickup_effect_color{255, 255, 255, 255};
 std::array<unsigned char, 4> pickup_effect_accent_color{255, 255, 255, 255};
 double pickup_effect_radius_m{0.0};
 double pickup_effect_rise_m{0.0};
 double damage_warning_below_integrity{0.0};
 std::array<unsigned char, 4> damage_warning_color{255, 255, 255, 255};
 double damage_warning_height_m{0.0};
};

struct SceneInstanceDefinition {
 std::string id;
 std::string prefab;
 Vec3 position_m;
 Vec3 scale{1.0, 1.0, 1.0};
 double yaw_degrees{0.0};
 std::vector<std::string> active_in;
};

struct SceneDefinition {
 std::string id;
 SceneCameraDefinition camera;
 SceneLightingDefinition lighting;
 SceneEffectsDefinition effects;
 SceneFloorDefinition floor;
 std::unordered_map<std::string, ScenePrefabDefinition> prefabs;
 std::unordered_map<std::string, SceneGoalVisualDefinition> goal_visuals;
 ScenePayloadVisualDefinition payload_visual;
 std::vector<SceneInstanceDefinition> instances;
};

// Authored map semantics for a region where independently moving traffic can
// conflict with the robot route. The behavior layer derives route entry,
// stop, and exit points from this box; actor paths and collision truth never
// enter autonomy.
struct ConflictZoneDefinition {
 std::string id;
 sim::Vec2 min;
 sim::Vec2 max;
 // "shadow" evaluates semantics without constraining motion; "control"
 // allows the autonomy behavior and trajectory validator to constrain motion.
 std::string authority{"shadow"};
 std::vector<std::string> active_in;
};

struct MapDefinition {
 std::string id;
 std::string scene_3d;
 sim::Vec2 size_m;
 double grid_resolution_m{0.5};
 std::vector<BoxDefinition> obstacles;
 std::vector<SurfaceDefinition> surfaces;
 std::vector<TerrainPatchDefinition> terrain;
 std::vector<ConflictZoneDefinition> conflict_zones;
 // Ordered: the editor saves maps back to JSON, and marker file order must
 // survive the round trip. Lookups only, never order-dependent logic.
 std::map<std::string, Pose> markers;
 std::vector<ActorDefinition> actors;
};

struct GoalStep {
 // Unified arrival logic: every GOAL is a marker plus what happens on arrival.
 // reach = [complete]; deliver_via/haul = [pickup, complete]; summon and
 // recharge are further on_arrive actions, same list, no special cases.
 std::string marker;
 std::string label;
 std::string action{"complete"};
 double radius_m{0.62};
 std::string summon_actor;
 double recharge_wh{0.0};
 // Optional stopped dwell for inspect actions. The goal system owns the
 // timer, while mission control only supplies a physically reachable stop.
 double dwell_s{0.0};
 // Stable task identity. Marker is the default for existing authored goals;
 // selectable task sets should name it explicitly so visit order can change
 // without changing result or replay identity.
 std::string id;
 // Optional scene-owned visual id. The goal framework never names themed art.
 std::string visual;
 int item_count{0};
 double item_mass_kg{0.0};
 double item_cog_height_m{0.0};
 double item_value{0.0};
 double item_fragility{0.0};
};

struct ObjectiveDefinition {
 std::string type;
 std::string spawn;
 std::string via;
 std::string goal;
 std::string via_label;
 std::string goal_label;
 std::string goal_visual;
 double time_limit_s{60.0};
 int max_collisions{0};
 double arrive_tolerance_m{0.62};

 double max_arrival_speed_mps{999.0};

 double via_radius_m{1.25};
 // Derived ordered goals (catalog fills from legacy fields when the JSON has
 // no explicit "goals"): mission consumes this list, never the legacy fields.
 std::vector<GoalStep> goals;
 // authored: visit array order. autonomous: select unfinished non-terminal
 // tasks at runtime, then activate the terminal complete goal.
 std::string goal_order{"authored"};
 std::string task_policy{"task.balanced"};
 double return_reserve_s{0.0};
};

struct EnvironmentKeyframeDefinition {
 std::string id;
 std::string label;
 double time_s{0.0};
 double wetness{0.0};
 double visibility{1.0};
 double daylight{1.0};
 // Material id -> multiplier applied to the shared tire/ground contact.
 // Missing materials remain at 1.0. Values interpolate on simulation time.
 std::unordered_map<std::string, double> material_grip;
};

struct ManeuverDefinition {
 std::string id;
 std::string title;
 std::string description;
 std::string spawn;
 std::string goal;
 // manual leaves authority with the operator; autonomous executes the same
 // deployable planning/control stack used by click-to-drive.
 std::string control{"autonomous"};
 std::vector<std::string> metrics;
};

struct ChapterDefinition {
 int id{1};
 std::string title;
 std::string blurb;
};

struct LevelDefinition {
 std::string id;
 int order{0};
 int chapter{1};
 std::string mode{"experiment"};
 std::string title;
 std::string map;
 std::string control_mode;
 std::string localization_mode;
 std::vector<std::string> prerequisites;
 unsigned seed{0};
 ObjectiveDefinition objective;
 // "campaign" applies inventory and ability progression. "open" exposes
 // the level-authored hardware matrix without granting any persistent item.
 std::string loadout_access{"campaign"};
 int budget{0};
 std::vector<std::string> required_slots;
 std::string concept_id;
 std::string prediction_prompt;
 std::string knowledge_card;
 double safe_min_clearance_m{0.0};
 std::string safe_clearance_scope{"all"};
 double max_localization_error_m{999.0};
 // Extra global-costmap inflation for known localization uncertainty. It is
 // content because a dead-reckoning lesson and a fused-pose lesson can use
 // the same robot and geometry with different safety envelopes.
 double planning_localization_margin_m{0.0};
 double fast_time_s{0.0};
 double efficient_energy_wh{0.0};
 std::vector<std::string> rewards;
 double payload_mass_kg{0.0};
 double payload_cog_height_m{0.0};
 std::vector<EnvironmentKeyframeDefinition> environment_keyframes;
 std::vector<ManeuverDefinition> maneuvers;
};

struct AbilityDefinition {
 std::string id;
 std::string title;
 std::string description;
};

// Contract board job (content, weekly board files): a level ridden for a
// cargo on one score axis. The earn rule lives in record_completion
// (axis flag achieved -> claim once); the garage spends reputation on parts.
struct ContractJob {
 std::string id;
 std::string level;
 std::string cargo;
 std::string axis; // fast / efficient / safe, matches MissionResult *score.
 int rep{1};
 double mass_kg{0.0};
 double cog_height_m{0.0};
 double fragility{0.0};
 std::string blurb;
};

// Non-localized provenance for a family of component parameters. A typical
// profile is a documented engineering default. Identified is reserved for a
// profile fitted to a named physical unit and validation data.
struct ModelProfileDefinition {
 std::string fidelity;
 std::string basis;
 std::vector<std::string> source_refs;
};

struct ComponentDefinition {
 std::string id;
 std::string slot;
 std::string title;
 std::string description;
 int cost{0};
 Properties behavior;
 VisualDefinition visual;
 ModelProfileDefinition model_profile;
};

struct MountDefinition {
 std::string id;
 Vec3 position_m;
 Vec3 rotation_deg;
 // Assembly-authored visual motion. Simulation binds actuator channels to
 // mounts; renderers interpret the role without naming a chassis or mount.
 std::string motion{"fixed"};
 // Physical rolling radius for wheel/caster motion. It lives on the mount so
 // passive supports and mixed wheel assemblies animate from their own
 // geometry rather than borrowing one chassis-wide visual constant.
 double rolling_radius_m{0.0};
 // Stable robotics frame name. Empty remains supported for player packs and
 // resolves to <mount-id>_link; official robots publish explicit names.
 std::string frame_id;
};

struct RobotPartDefinition {
 std::string id;
 std::string mount;
 VisualDefinition visual;
};

struct RobotSlotDefinition {
 std::string slot;
 std::vector<std::string> mounts;
};

struct PassiveSupportDefinition {
 // The fork swivels with caster heading while the wheel also rolls. Keeping
 // these as separate authored mounts avoids rotating the fork with the tire.
 std::string steer_mount;
 std::string roll_mount;
};

struct SteeringDefinition {
 double integral_limit{1.2};
 // Ackermann hardware. Unused by differential cars; required content for
 // ackermann robots (the servo derivation and yaw capability come from these).
 double max_steer_rad{0.6};
 double max_steer_rate_radps{2.5};
 double servo_response_s{0.4};
 double stall_torque_Nm{8.0};
};
struct MotorDefinition {
 double max_wheel_speed_mps{0.0};
 double max_wheel_accel_mps2{0.0};
 double stall_force_N{0.0};
 double free_speed_mps{0.0};
 double pwm_limit{1.0};
 double integral_limit{2.0};
 // Per-wheel equivalent translational mass (wheel + tire + reflected rotor).
 // Hardware data: the contact patch integrates the tread against this.
 double wheel_equiv_mass_kg{3.0};
 // Controller and low-speed drivetrain assumptions are authored data, not
 // hidden tuning constants in the simulation backend.
 double controller_bandwidth_radps{6.0};
 double controller_integral_time_s{5.0};
 double controller_derivative_time_s{0.005};
 double coulomb_smoothing_speed_mps{0.05};
 double viscous_drag_N_per_mps{0.6};
};

struct EncoderDefinition {
 int counts_per_revolution{0};
};

struct ServiceBrakeDefinition {
 bool enabled{false};
 double maximum_torque_Nm{0.0};
 double apply_time_s{0.12};
 double release_time_s{0.08};
 double low_speed_smoothing_radps{0.6};
};

struct RigidLinkDefinition {
 std::string id;
 std::string shape;
 Vec3 size_m;
 double radius_m{0.0};
 double length_m{0.0};
 Vec3 position_m;
 Vec3 rotation_deg;
 double mass_kg{0.0};
 Vec3 inertia_diagonal_kgm2;
 double friction{0.8};
 // Finite-patch support moment divided by normal load, in metres. This is
 // distinct from the tire component's dimensionless rolling resistance.
 double rolling_support_moment_arm_m{0.0};
 double spinning_resistance{0.0};
 double contact_stiffness_Npm{0.0};
 double contact_damping_Nspm{0.0};
 Vec3 contact_axle_axis;
};

struct RigidJointDefinition {
 std::string id;
 std::string type;
 std::string parent;
 std::string child;
 Vec3 pivot_in_parent_m;
 Vec3 pivot_in_child_m;
 Vec3 axis_in_parent;
 Vec3 axis_in_child;
 bool has_limits{false};
 double lower_limit_rad{0.0};
 double upper_limit_rad{0.0};
};

struct JointActuatorDefinition {
 // Chassis-neutral assembly data binds a named controller output channel to
 // a joint. The differential controller currently publishes left / right.
 std::string channel;
 std::string joint;
 double effort_limit_Nm{0.0};
 // Motor/gearbox inertia reflected to the driven joint axis. This is an
 // actuator property, separate from the physical wheel-link inertia.
 double armature_inertia_kgm2{0.0};
};

struct ArticulationDefinition {
 std::string root_link;
 std::vector<RigidLinkDefinition> links;
 std::vector<RigidJointDefinition> joints;
 std::vector<JointActuatorDefinition> actuators;
};

struct RobotDefinition {
 std::string id;
 std::string title;
 ModelProfileDefinition model_profile;
 std::string drive_type;
 std::string drive_label_key;
 double wheel_radius_m{0.0};
 double wheel_base_m{0.0};
 double mass_kg{0.0};
 double yaw_inertia_kgm2{0.0};
 double max_yaw_accel_radps2{8.0};
 // Tracked spin capability (vendor spec). Only read by tracked robots;
 // wheeled chassis derive yaw authority from geometry instead.
 double tracked_max_yaw_rate_rads{1.5};
 // Track contact length (vendor spec). Only read by tracked robots.
 double track_length_m{0.7};
 // Axle track width (vendor spec). Only read by ackermann robots (linkage).
 double track_width_m{0.58};
 // Unlock gate for player chassis choice (content data, not code): empty =
 // starter (always available), otherwise an ability.* id the garage checks.
 // The mission trusts validated builds; ownership lives at the UI layer.
 std::string unlock_ability;
 MotorDefinition motor;
 SteeringDefinition steering;
 double energy_base_w{0.0};
 double energy_per_v_w{0.0};
 double energy_per_w_w{0.0};
 EncoderDefinition encoder;
 ServiceBrakeDefinition service_brake;
 ArticulationDefinition articulation;
 // Orientation-independent envelope consumed by grid and local planners.
 double collision_radius_m{0.0};
 // Authored rigid hull consumed by Bullet. It is deliberately separate from
 // the planning envelope and from the wheel/caster support polygon.
 std::string rigid_body_shape;
 Vec3 rigid_body_size_m;
 double collision_height_m{0.0};
 // Rest height of the physical base_link origin above the support surface and
 // height of the complete unloaded assembly centre of mass. They are derived
 // from articulated link data when present; neither is a visual-mesh offset.
 double base_link_height_m{0.0};
 double center_of_mass_height_m{0.0};
 std::vector<MountDefinition> mounts;
 // Passive rolling supports belong to the chassis assembly, not to the
 // powered tire component slot. Array order is the telemetry channel order.
 std::vector<PassiveSupportDefinition> passive_supports;
 std::vector<RobotPartDefinition> base_parts;
 std::vector<RobotSlotDefinition> slots;
};

struct PolicyDefinition {
 std::string id;
 std::string title;
 std::string description;
 double style_anchor{0.5};
 Properties behavior;
};

struct TaskPolicyDefinition {
 std::string id;
 std::string title;
 std::string description;
 // Registered autonomy implementation plus data-only tuning parameters.
 std::string algorithm;
 Properties parameters;
};

struct BuildSelection {
 // slot -> component id. New hardware slots need no C++ changes: loader,
 // validation, cost, UI, renderers and saves all iterate this map.
 std::map<std::string, std::string> parts;
 double style{0.5};
 // Empty keeps the level-authored doctrine. Autonomous task sets may choose
 // References a catalog task policy without changing driving control.
 std::string task_policy;
 // Player-requested chassis (robot id, empty = presentation default).
 // Honored only when listed in the level's chassis_options and validated
 // (ownership checked by the caller holding progress, see validate_build).
 std::string chassis_robot;

 const std::string& part(const std::string& slot) const {
  static const std::string kEmpty;
  const auto found = parts.find(slot);
  return found == parts.end() ? kEmpty : found->second;
 }
};

struct ComponentColumn {
 std::string slot;
 std::string title;
 std::vector<std::string> options;
 bool player_choice{true};
};

struct PredictionChoice {
 std::string id;
 std::string text;
 std::string feedback;
};

struct PresentationDefinition {
 std::string id;
 bool is_default{false};
 std::string level;
 // Vehicle for this level. Empty = campaign companion (all existing levels).
 // A non-empty id must name a robot in assets/robots.
 std::string robot_id;
 // Player-selectable chassis for this level (opt-in; absent/empty = fixed as
 // today, zero behavior drift). Entries must name robots; the mission honors
 // a validated build request, the garage owns the UI, abilities own unlock.
 std::vector<std::string> chassis_options;
 // Ordered allow-list for autonomous task-ordering algorithms.
 std::vector<std::string> task_policies;
 std::string garage_heading;
 std::string garage_brief;
 std::vector<ComponentColumn> component_columns;
 std::string policy_title;
 std::vector<std::string> policies;
 BuildSelection default_build;
 BuildSelection recommended_build;
 std::string prediction_heading;
 std::string prediction_hint;
 std::vector<PredictionChoice> prediction_choices;
 std::string running_heading;
 std::string normal_line;
 std::string detected_line;
 // Ordered generic measurement ids shown while this presentation runs.
 // Content selects evidence; the UI resolves each id from typed snapshots.
 std::vector<std::string> evidence_fields;
 // Optional authored line for each ordered-goal index. This lets a multi-beat
 // mission describe its current intent without teaching the UI mission ids.
 std::vector<std::string> goal_lines;
 std::string success_title;
 std::string success_summary;
 std::string failure_summary;
 std::string evidence_heading;
 std::string evidence_caption;
 std::string knowledge_title;
 std::unordered_map<std::string, std::string> failure_titles;
};

struct StoryBeatDefinition {
 std::string id;
 std::string level;
 std::string when;
 std::string speaker;
 std::string text;
};

struct CampaignDefinition {
 std::string id;
 std::string title;
 std::string content_version;
 std::string companion_code;
 std::string companion_name;
 std::string companion_robot;
 std::string starting_level;
 // Guided Campaign order. Optional systemic experiences are listed
 // separately so adding a playground never changes chapter progression.
 std::vector<std::string> levels;
 std::vector<std::string> playgrounds;
 std::vector<std::string> starting_inventory;
 std::vector<std::string> starting_policies;
};

double property(const Properties& properties, const std::string& name);
bool active_in_level(const std::vector<std::string>& levels, const std::string& level_id);

} // namespace mission
