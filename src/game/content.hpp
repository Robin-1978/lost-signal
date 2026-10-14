#pragma once

#include "algo/math.hpp"

#include <array>
#include <filesystem>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

namespace game {

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
};

struct ActorDefinition {
 std::string id;
 std::vector<sim::Vec2> path;
 double speed_mps{0.0};
 double radius_m{0.0};
 std::vector<std::string> active_in;
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
 std::vector<SceneInstanceDefinition> instances;
};

struct MapDefinition {
 std::string id;
 std::string scene_3d;
 sim::Vec2 size_m;
 double grid_resolution_m{0.5};
 std::vector<BoxDefinition> obstacles;
 std::vector<SurfaceDefinition> surfaces;
 // Ordered: the editor saves maps back to JSON, and marker file order must
 // survive the round trip. Lookups only, never order-dependent logic.
 std::map<std::string, Pose> markers;
 std::vector<ActorDefinition> actors;
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
 double via_radius_m{1.25};
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
 std::string title;
 std::string map;
 std::string control_mode;
 std::string localization_mode;
 std::vector<std::string> prerequisites;
 unsigned seed{0};
 ObjectiveDefinition objective;
 int budget{0};
 std::vector<std::string> required_slots;
 std::string concept;
 std::string prediction_prompt;
 std::string knowledge_card;
 double safe_min_clearance_m{0.0};
 double max_localization_error_m{999.0};
 double fast_time_s{0.0};
 double efficient_energy_wh{0.0};
 std::vector<std::string> rewards;
};

struct AbilityDefinition {
 std::string id;
 std::string title;
 std::string description;
};

struct ComponentDefinition {
 std::string id;
 std::string slot;
 std::string title;
 std::string description;
 int cost{0};
 Properties behavior;
 VisualDefinition visual;
};

struct MountDefinition {
 std::string id;
 Vec3 position_m;
 Vec3 rotation_deg;
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

struct SteeringDefinition {
 double integral_limit{1.2};
 // Ackermann hardware. Unused by differential cars; required content for
 // ackermann robots (the servo derivation and yaw capability come from these).
 double max_steer_rad{0.6};
 double max_steer_rate_radps{2.5};
 double servo_response_s{0.4};
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
};

struct EncoderDefinition {
 int counts_per_revolution{0};
};

struct RobotDefinition {
 std::string id;
 std::string title;
 std::string drive_type;
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
 double collision_radius_m{0.0};
 double collision_height_m{0.0};
 std::vector<MountDefinition> mounts;
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

struct BuildSelection {
 // slot -> component id. New hardware slots need no C++ changes: loader,
 // validation, cost, UI, renderers and saves all iterate this map.
 std::map<std::string, std::string> parts;
 double style{0.5};
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
 std::vector<std::string> levels;
 std::vector<std::string> starting_inventory;
 std::vector<std::string> starting_policies;
};

double property(const Properties& properties, const std::string& name);
bool active_in_level(const std::vector<std::string>& levels, const std::string& level_id);

} // namespace game
