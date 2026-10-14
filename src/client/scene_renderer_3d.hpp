#pragma once

#include "game/content_catalog.hpp"
#include "game/mission_simulation.hpp"

#include <raylib.h>

#include <filesystem>
#include <string>
#include <unordered_map>

namespace client {

class SceneRenderer3D {
 public:
 SceneRenderer3D(const std::filesystem::path& assets_root, const game::ContentCatalog& catalog);
 ~SceneRenderer3D();

 SceneRenderer3D(const SceneRenderer3D&) = delete;
 SceneRenderer3D& operator=(const SceneRenderer3D&) = delete;

 void draw(const game::MissionSimulation& mission, int screen_width, int screen_height);
 const Camera3D& camera() const { return camera_; }

 private:
 void bind_shader(Model& model) const;
 void ensure_shadow_map(int resolution);
 void ensure_post_targets(int width, int height);
 void render_shadow_map(const game::MissionSimulation& mission, const game::SceneDefinition& scene,
                        Vector3 light_direction);
 void render_scene_target(const game::MissionSimulation& mission, const game::SceneDefinition& scene);
 void composite_scene(const game::SceneDefinition& scene) const;
 void draw_scene(const game::MissionSimulation& mission) const;
 void draw_floor(const game::MapDefinition& map, const game::SceneDefinition& scene) const;
 void draw_shelf(const game::BoxDefinition& obstacle) const;
 void draw_surface(const game::SurfaceDefinition& surface) const;
 void draw_markers(const game::MissionSimulation& mission) const;
 void draw_robot(const game::MissionSimulation& mission) const;
 void draw_forklift(const game::ActorSnapshot& actor, bool waiting) const;
 void draw_environment(const game::MissionSimulation& mission, const game::SceneDefinition& scene) const;
 void draw_accents(const game::MissionSimulation& mission) const;
 void draw_box(Vector3 position, Vector3 size, float yaw_degrees, Color color) const;
 const Model* model(const std::string& path) const;

 const game::ContentCatalog& catalog_;
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
