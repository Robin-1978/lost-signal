#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <raylib.h>

#include "sim/viewer/asset_manifest.hpp"
#include "sim/viewer/actor_render_policy.hpp"
#include "sim/viewer/assembly_registry.hpp"
#include "sim/viewer/frame.hpp"
#include "sim/viewer/frame_builder.hpp"
#include "sim/viewer/particle_system.hpp"
#include "sim/viewer/scene_manifest.hpp"
#include "sim/viewer/view_transform.hpp"
#include "sim/world/world.hpp"

namespace lost_signal::sim::viewer::render {

enum class RenderQuality { kFast, kHigh };

struct VisualAssets {
  struct EnvironmentModel {
    std::string asset_id;
    Model model{};
    const AssetDefinition* definition = nullptr;
  };
  struct FloorSurface {
    std::string material_id;
    Texture2D texture{};
    Model model{};
  };
  Texture2D dust{};
  Shader lighting{};
  Shader bloom_extract{};
  Shader bloom_blur{};
  Shader composite{};
  RenderTexture2D scene_target{};
  RenderTexture2D bloom_target_a{};
  RenderTexture2D bloom_target_b{};
  RenderTexture2D shadow_map{};
  int shadow_map_resolution = 0;
  int target_width = 0;
  int target_height = 0;
  int bloom_threshold_loc = -1;
  int blur_texel_size_loc = -1;
  int blur_direction_loc = -1;
  int composite_bloom_texture_loc = -1;
  int composite_exposure_loc = -1;
  int composite_bloom_strength_loc = -1;
  int composite_vignette_strength_loc = -1;
  int light_view_projection_loc = -1;
  int shadow_map_loc = -1;
  int shadow_strength_loc = -1;
  int shadow_resolution_loc = -1;
  int floor_texture_tiling_loc = -1;
  AssetManifest manifest;
  Model chassis{};
  const AssetDefinition* chassis_definition = nullptr;
  Model wheel{};
  const AssetDefinition* wheel_definition = nullptr;
  Model wheel_right{};
  const AssetDefinition* wheel_right_definition = nullptr;
  Model knuckle_left{};
  const AssetDefinition* knuckle_left_definition = nullptr;
  Model knuckle_right{};
  const AssetDefinition* knuckle_right_definition = nullptr;
  Model arm_turret{};
  const AssetDefinition* arm_turret_definition = nullptr;
  Model arm_upper{};
  const AssetDefinition* arm_upper_definition = nullptr;
  Model arm_forearm{};
  const AssetDefinition* arm_forearm_definition = nullptr;
  Model arm_tool{};
  const AssetDefinition* arm_tool_definition = nullptr;
  Model lidar{};
  const AssetDefinition* lidar_definition = nullptr;
  Model caster_fork{};
  const AssetDefinition* caster_fork_definition = nullptr;
  Model caster_wheel{};
  const AssetDefinition* caster_wheel_definition = nullptr;
  Model track{};
  const AssetDefinition* track_definition = nullptr;
  Model forklift{};
  const AssetDefinition* forklift_definition = nullptr;
  Model forklift_base{};
  const AssetDefinition* forklift_base_definition = nullptr;
  Model forklift_drive_wheel{};
  const AssetDefinition* forklift_drive_wheel_definition = nullptr;
  Model forklift_steer_wheel{};
  const AssetDefinition* forklift_steer_wheel_definition = nullptr;
  Model forklift_mast{};
  const AssetDefinition* forklift_mast_definition = nullptr;
  Model forklift_carriage{};
  const AssetDefinition* forklift_carriage_definition = nullptr;
  Model forklift_fork{};
  const AssetDefinition* forklift_fork_definition = nullptr;
  Model worker{};
  const AssetDefinition* worker_definition = nullptr;
  ModelAnimation* worker_animations = nullptr;
  int worker_animation_count = 0;
  std::vector<EnvironmentModel> environment;
  std::vector<FloorSurface> floor_surfaces;
  bool loaded = false;
};

struct CameraState {
  double target_x = 2.0;
  double target_y = 1.0;
  double target_z = 0.0;
  float yaw_deg = 35.0f;
  float pitch_deg = 50.0f;
  float dist_m = 12.0f;
};

struct WalkaroundView {
  Camera3D camera;
  Vector3 focus;
};

const char* render_quality_name(RenderQuality quality);
Color scene_background(const SceneManifest* scene);

void load_visual_assets(VisualAssets& assets, const std::string& root,
                        const std::string& robot_id,
                        const std::string& chassis_kind,
                        const AssemblySelection* assembly = nullptr);
void unload_visual_assets(VisualAssets& assets);

void map_bounds(const lost_signal::sim::world::World& world, double& x0,
                double& y0, double& x1, double& y1);

void draw_frame_2d(const Frame& frame, const ViewTransform& view,
                   bool show_lidar, bool show_estimate,
                   bool show_truth_debug);
void draw_baseline_comparison_ghost_2d(const Frame& baseline,
                                       const ViewTransform& view,
                                       bool event_aligned);
void draw_baseline_comparison_ghost_3d(const Frame& baseline,
                                       const Camera3D& camera,
                                       bool event_aligned);

inline Vector3 to_raylib(double x_m, double y_m, double z_m) {
  return {static_cast<float>(x_m), static_cast<float>(z_m),
          static_cast<float>(-y_m)};
}

Camera3D camera_from_state(const CameraState& state);
WalkaroundView walkaround_view(const Frame& frame,
                               const std::string& component_id,
                               CameraState state);
Camera3D blend_camera(Camera3D from, Camera3D to, float amount);
bool screen_to_ground_3d(const Camera3D& camera, Vector2 mouse, double& x_m,
                         double& y_m);

void draw_frame_3d(
    const Frame& frame, const lost_signal::sim::world::World& world,
    const Camera3D& camera, const ParticleSystem& particles,
    const SurfaceTraceSystem& surface_traces, bool show_lidar,
    bool show_estimate, bool show_actor_routes, bool show_places,
    const VisualAssets* assets = nullptr,
    const SceneManifest* scene = nullptr,
    const LidarMount* lidar_mount = nullptr, bool show_physics_overlay = false,
    bool presentation_effects = true, bool draw_fast_shadow_discs = false);

bool draw_frame_3d_postprocessed(
    const Frame& frame, const lost_signal::sim::world::World& world,
    const Camera3D& camera, const ParticleSystem& particles,
    const SurfaceTraceSystem& surface_traces, bool show_lidar,
    bool show_estimate, bool show_actor_routes, bool show_places,
    bool show_physics_overlay, VisualAssets& assets,
    const SceneManifest* scene, const LidarMount* lidar_mount);

void draw_frame_3d_direct(
    const Frame& frame, const lost_signal::sim::world::World& world,
    const Camera3D& camera, const ParticleSystem& particles,
    const SurfaceTraceSystem& surface_traces, bool show_lidar,
    bool show_estimate, bool show_actor_routes, bool show_places,
    bool show_physics_overlay, VisualAssets& assets,
    const SceneManifest* scene, const LidarMount* lidar_mount);

}  // namespace lost_signal::sim::viewer::render
