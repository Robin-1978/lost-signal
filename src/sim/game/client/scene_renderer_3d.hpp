#pragma once

#include "sim/game/mission/content_catalog.hpp"
#include "sim/game/mission/presentation_view.hpp"

#include <raylib.h>

#include <filesystem>
#include <optional>
#include <string>
#include <unordered_map>

namespace client {

class SceneRenderer3D {
 public:
 SceneRenderer3D(const std::filesystem::path& assets_root, const mission::ContentCatalog& catalog);
 ~SceneRenderer3D();

 SceneRenderer3D(const SceneRenderer3D&) = delete;
 SceneRenderer3D& operator=(const SceneRenderer3D&) = delete;

 void draw(const mission::PresentationFrameView& mission, int screen_width, int screen_height,
 Rectangle input_viewport, bool assembly_view = false);
 const Camera3D& camera() const { return camera_; }
 std::optional<sim::Vec2> screen_to_ground(Vector2 screen) const;
 std::optional<std::size_t> goal_at_screen(
 const mission::PresentationFrameView& mission, Vector2 screen, float tolerance_px = 46.0f) const;

 private:
 void bind_shader(Model& model) const;
 void ensure_shadow_map(int resolution);
 void ensure_post_targets(int width, int height);
 void render_shadow_map(const mission::PresentationFrameView& mission, const mission::SceneDefinition& scene,
                        Vector3 light_direction);
 void render_scene_target(const mission::PresentationFrameView& mission, const mission::SceneDefinition& scene);
 void composite_scene(const mission::SceneDefinition& scene) const;
 void draw_scene(const mission::PresentationFrameView& mission) const;
 void draw_floor(const mission::MapDefinition& map, const mission::SceneDefinition& scene) const;
 void draw_terrain(const mission::PresentationFrameView& mission) const;
 void draw_constructed_supports(const mission::PresentationFrameView& mission) const;
 void draw_shelf(const mission::BoxDefinition& obstacle) const;
 void draw_surface(const mission::PresentationFrameView& mission, const mission::SurfaceDefinition& surface) const;
 void draw_markers(const mission::PresentationFrameView& mission) const;
 void draw_field_operator(const mission::PresentationFrameView& mission) const;
 void draw_robot(const mission::PresentationFrameView& mission) const;
 void draw_actor(const mission::ActorSnapshot& actor, bool waiting, double time_s) const;
 void draw_environment(const mission::PresentationFrameView& mission, const mission::SceneDefinition& scene) const;
 void draw_accents(const mission::PresentationFrameView& mission) const;
 void draw_motion_effects(const mission::PresentationFrameView& mission) const;
 void draw_risk_effects(const mission::PresentationFrameView& mission) const;
 void draw_box(Vector3 position, Vector3 size, float yaw_degrees, Color color) const;
 void draw_prefab(const mission::ScenePrefabDefinition& prefab, const mission::Vec3& position,
 const mission::Vec3& scale, double yaw_degrees) const;
 void draw_attached_prefab(const mission::ScenePrefabDefinition& prefab, Vector3 body_origin,
 Quaternion body_rotation, const mission::Vec3& local_origin) const;
 const Model* model(const std::string& path) const;

 const mission::ContentCatalog& catalog_;
 std::filesystem::path assets_root_;
 std::unordered_map<std::string, Model> models_;
 std::unordered_map<std::string, Model> floor_models_;
 std::unordered_map<std::string, Model> surface_models_;
 std::unordered_map<std::string, Texture2D> textures_;
 Model cube_model_{};
 Shader shader_{};
 Shader bloom_extract_shader_{};
 Shader bloom_blur_shader_{};
 Shader composite_shader_{};
 Camera3D camera_{};
 Vector3 camera_focus_{};
 Vector3 camera_offset_{};
 float camera_fov_{41.0f};
 double camera_time_s_{-1.0};
 std::string camera_level_;
 bool camera_initialized_{false};
 bool camera_assembly_view_{false};
 Rectangle input_viewport_{};
 bool manual_camera_{false};
 Vector3 camera_pan_{};
 float camera_orbit_yaw_{0.0f};
 float camera_orbit_pitch_{0.65f};
 float camera_orbit_distance_{10.0f};
 RenderTexture2D shadow_map_{};
 RenderTexture2D scene_target_{};
 RenderTexture2D bloom_target_a_{};
 RenderTexture2D bloom_target_b_{};
 int shadow_map_resolution_{0};
 int target_width_{0};
 int target_height_{0};
 int view_position_loc_{-1};
 int point_light_0_position_loc_{-1};
 int point_light_0_color_loc_{-1};
 int point_light_1_position_loc_{-1};
 int point_light_1_color_loc_{-1};
 int light_view_projection_loc_{-1};
 int shadow_map_loc_{-1};
 int shadow_strength_loc_{-1};
 int shadow_resolution_loc_{-1};
 int bloom_threshold_loc_{-1};
 int blur_texel_size_loc_{-1};
 int blur_direction_loc_{-1};
 int composite_bloom_texture_loc_{-1};
 int composite_exposure_loc_{-1};
 int composite_bloom_strength_loc_{-1};
 int composite_vignette_strength_loc_{-1};
};

} // namespace client
