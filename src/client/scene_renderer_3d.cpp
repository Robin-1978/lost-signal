#include "scene_renderer_3d.hpp"

#include <raymath.h>
#include <rlgl.h>

#include <algorithm>
#include <cmath>

namespace client {
namespace {

constexpr float kRadToDeg = 57.29577951308232f;
constexpr Color kMetal{85, 84, 75, 255};
constexpr Color kMetalEdge{137, 128, 103, 255};
constexpr Color kAmber{232, 151, 52, 255};
constexpr Color kCyan{68, 226, 203, 255};
constexpr Color kDanger{235, 83, 76, 255};

float hash01(int x, int y, int salt = 0) {
 unsigned value = static_cast<unsigned>(x) * 374761393U + static_cast<unsigned>(y) * 668265263U +
                  static_cast<unsigned>(salt) * 1442695041U;
 value = (value ^ (value >> 13U)) * 1274126177U;
 return static_cast<float>((value ^ (value >> 16U)) & 0xffffU) / 65535.0f;
}

Vector3 world(sim::Vec2 point, float height = 0.0f) {
 return {static_cast<float>(point.x), height, static_cast<float>(-point.y)};
}

Vector3 world(const game::Pose& pose, float height = 0.0f) {
 return {static_cast<float>(pose.x), height, static_cast<float>(-pose.y)};
}

Vector3 world(const game::Vec3& point) {
 return {static_cast<float>(point.x), static_cast<float>(point.z), static_cast<float>(-point.y)};
}

Color color(const std::array<unsigned char, 4>& value) {
 return {value[0], value[1], value[2], value[3]};
}

Vector3 shader_color(const std::array<unsigned char, 4>& value, double intensity = 1.0) {
 const float scale = static_cast<float>(intensity / 255.0);
 return {value[0] * scale, value[1] * scale, value[2] * scale};
}

game::Vec3 rotate_offset(const game::Vec3& value, double yaw_degrees) {
 const double radians = yaw_degrees / kRadToDeg;
 const double cosine = std::cos(radians);
 const double sine = std::sin(radians);
 return {
   value.x * cosine - value.y * sine,
   value.x * sine + value.y * cosine,
   value.z,
 };
}

Vector3 pose_offset(sim::Vec2 position, double heading, float forward, float left, float up) {
 const float cosine = std::cos(static_cast<float>(heading));
 const float sine = std::sin(static_cast<float>(heading));
 const float x = static_cast<float>(position.x) + cosine * forward - sine * left;
 const float y = static_cast<float>(position.y) + sine * forward + cosine * left;
 return {x, up, -y};
}

Color cargo_color(int index) {
 constexpr Color colors[] = {
   {132, 82, 47, 255},
   {61, 88, 78, 255},
   {111, 105, 72, 255},
   {84, 72, 61, 255},
 };
 return colors[index % 4];
}

RenderTexture2D load_shadow_map(int width, int height) {
 RenderTexture2D target{};
 target.id = rlLoadFramebuffer();
 target.texture.width = width;
 target.texture.height = height;
 if (target.id == 0)
  return target;

 rlEnableFramebuffer(target.id);
 target.depth.id = rlLoadTextureDepth(width, height, false);
 target.depth.width = width;
 target.depth.height = height;
 target.depth.format = 19;
 target.depth.mipmaps = 1;
 rlFramebufferAttach(target.id, target.depth.id, RL_ATTACHMENT_DEPTH, RL_ATTACHMENT_TEXTURE2D, 0);
 if (!rlFramebufferComplete(target.id)) {
  rlUnloadFramebuffer(target.id);
  return {};
 }
 return target;
}

RenderTexture2D load_hdr_target(int width, int height) {
 RenderTexture2D target{};
 target.id = rlLoadFramebuffer();
 if (target.id == 0)
  return target;

 rlEnableFramebuffer(target.id);
 target.texture.id = rlLoadTexture(nullptr, width, height, RL_PIXELFORMAT_UNCOMPRESSED_R16G16B16A16, 1);
 target.texture.width = width;
 target.texture.height = height;
 target.texture.format = PIXELFORMAT_UNCOMPRESSED_R16G16B16A16;
 target.texture.mipmaps = 1;
 target.depth.id = rlLoadTextureDepth(width, height, true);
 target.depth.width = width;
 target.depth.height = height;
 target.depth.format = 19;
 target.depth.mipmaps = 1;
 rlFramebufferAttach(target.id, target.texture.id, RL_ATTACHMENT_COLOR_CHANNEL0, RL_ATTACHMENT_TEXTURE2D, 0);
 rlFramebufferAttach(target.id, target.depth.id, RL_ATTACHMENT_DEPTH, RL_ATTACHMENT_RENDERBUFFER, 0);
 if (!rlFramebufferComplete(target.id)) {
  UnloadRenderTexture(target);
  return LoadRenderTexture(width, height);
 }
 SetTextureFilter(target.texture, TEXTURE_FILTER_BILINEAR);
 return target;
}

void draw_render_texture(const RenderTexture2D& target, float width, float height) {
 DrawTexturePro(
   target.texture,
   {0.0f, 0.0f, static_cast<float>(target.texture.width), -static_cast<float>(target.texture.height)},
   {0.0f, 0.0f, width, height}, {0.0f, 0.0f}, 0.0f, WHITE);
}

} // namespace

SceneRenderer3D::SceneRenderer3D(const std::filesystem::path& assets_root,
                                 const game::ContentCatalog& catalog)
    : catalog_(catalog), assets_root_(assets_root) {
 const auto vertex = assets_root_ / "shaders/scene_lighting.vs";
 const auto fragment = assets_root_ / "shaders/scene_lighting.fs";
 shader_ = LoadShader(vertex.string().c_str(), fragment.string().c_str());
 if (IsShaderValid(shader_)) {
  view_position_loc_ = GetShaderLocation(shader_, "viewPos");
  shader_.locs[SHADER_LOC_VECTOR_VIEW] = view_position_loc_;
  point_light_0_position_loc_ = GetShaderLocation(shader_, "pointLight0Position");
  point_light_0_color_loc_ = GetShaderLocation(shader_, "pointLight0Color");
  point_light_1_position_loc_ = GetShaderLocation(shader_, "pointLight1Position");
  point_light_1_color_loc_ = GetShaderLocation(shader_, "pointLight1Color");
  light_view_projection_loc_ = GetShaderLocation(shader_, "lightVP");
  shadow_map_loc_ = GetShaderLocation(shader_, "shadowMap");
  shadow_strength_loc_ = GetShaderLocation(shader_, "shadowStrength");
  shadow_resolution_loc_ = GetShaderLocation(shader_, "shadowMapResolution");
  shader_.locs[SHADER_LOC_MATRIX_NORMAL] = GetShaderLocation(shader_, "matNormal");
  shader_.locs[SHADER_LOC_COLOR_SPECULAR] = GetShaderLocation(shader_, "colSpecular");
  const Vector3 light_direction{-0.48f, -1.0f, -0.32f};
  const Vector3 light_color{1.08f, 0.98f, 0.82f};
  const Vector3 ambient{0.24f, 0.29f, 0.28f};
  const Vector3 fog_color{0.035f, 0.055f, 0.052f};
  const float fog_start = 12.0f;
  const float fog_end = 31.0f;
  SetShaderValue(shader_, GetShaderLocation(shader_, "lightDirection"), &light_direction,
                 SHADER_UNIFORM_VEC3);
  SetShaderValue(shader_, GetShaderLocation(shader_, "lightColor"), &light_color, SHADER_UNIFORM_VEC3);
  SetShaderValue(shader_, GetShaderLocation(shader_, "ambient"), &ambient, SHADER_UNIFORM_VEC3);
  SetShaderValue(shader_, GetShaderLocation(shader_, "fogColor"), &fog_color, SHADER_UNIFORM_VEC3);
  SetShaderValue(shader_, GetShaderLocation(shader_, "fogStart"), &fog_start, SHADER_UNIFORM_FLOAT);
  SetShaderValue(shader_, GetShaderLocation(shader_, "fogEnd"), &fog_end, SHADER_UNIFORM_FLOAT);
 }

 bloom_extract_shader_ = LoadShader(nullptr, (assets_root_ / "shaders/bloom_extract.fs").string().c_str());
 bloom_blur_shader_ = LoadShader(nullptr, (assets_root_ / "shaders/bloom_blur.fs").string().c_str());
 composite_shader_ = LoadShader(nullptr, (assets_root_ / "shaders/scene_composite.fs").string().c_str());
 if (IsShaderValid(bloom_extract_shader_)) {
  bloom_threshold_loc_ = GetShaderLocation(bloom_extract_shader_, "threshold");
 }
 if (IsShaderValid(bloom_blur_shader_)) {
  blur_texel_size_loc_ = GetShaderLocation(bloom_blur_shader_, "texelSize");
  blur_direction_loc_ = GetShaderLocation(bloom_blur_shader_, "direction");
 }
 if (IsShaderValid(composite_shader_)) {
  composite_bloom_texture_loc_ = GetShaderLocation(composite_shader_, "bloomTexture");
  composite_exposure_loc_ = GetShaderLocation(composite_shader_, "exposure");
  composite_bloom_strength_loc_ = GetShaderLocation(composite_shader_, "bloomStrength");
  composite_vignette_strength_loc_ = GetShaderLocation(composite_shader_, "vignetteStrength");
 }

 cube_model_ = LoadModelFromMesh(GenMeshCube(1.0f, 1.0f, 1.0f));
 cube_model_.materials[0].maps[MATERIAL_MAP_METALNESS].value = 0.24f;
 cube_model_.materials[0].maps[MATERIAL_MAP_ROUGHNESS].value = 0.68f;
 bind_shader(cube_model_);

 const auto load_model = [&](const std::string& path) {
  if (path.empty() || models_.contains(path))
   return;
  Model loaded = LoadModel((assets_root_ / path).string().c_str());
  if (loaded.meshCount <= 0)
   return;
  loaded.transform = MatrixRotateX(-PI / 2.0f);
  bind_shader(loaded);
  models_.emplace(path, loaded);
 };
 for (const auto& [id, robot] : catalog_.robots()) {
  (void)id;
  for (const auto& part : robot.base_parts)
   load_model(part.visual.model_3d);
 }
 for (const auto& [id, component] : catalog_.components()) {
  (void)id;
  load_model(component.visual.model_3d);
 }
 for (const auto& [id, material] : catalog_.surface_materials()) {
  (void)id;
  if (!material.texture_albedo.empty() && !surface_models_.contains(material.texture_albedo)) {
   Texture2D texture = LoadTexture((assets_root_ / material.texture_albedo).string().c_str());
   if (texture.id > 0) {
    GenTextureMipmaps(&texture);
    SetTextureFilter(texture, TEXTURE_FILTER_TRILINEAR);
    SetTextureWrap(texture, TEXTURE_WRAP_REPEAT);
    Model plane = LoadModelFromMesh(GenMeshPlane(1.0f, 1.0f, 1, 1));
    plane.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = texture;
    plane.materials[0].maps[MATERIAL_MAP_METALNESS].value = 0.04f;
    plane.materials[0].maps[MATERIAL_MAP_ROUGHNESS].value = 0.91f;
    bind_shader(plane);
    textures_.emplace(material.texture_albedo, texture);
    surface_models_.emplace(material.texture_albedo, plane);
   }
  }
 }
 for (const auto& [id, scene] : catalog_.scenes()) {
  (void)id;
  if (!scene.floor.albedo_texture.empty() && !floor_models_.contains(scene.floor.albedo_texture)) {
   Texture2D texture = LoadTexture((assets_root_ / scene.floor.albedo_texture).string().c_str());
   if (texture.id > 0) {
    GenTextureMipmaps(&texture);
    SetTextureFilter(texture, TEXTURE_FILTER_TRILINEAR);
    SetTextureWrap(texture, TEXTURE_WRAP_REPEAT);
    Model floor_model = LoadModelFromMesh(GenMeshCube(1.0f, 1.0f, 1.0f));
    floor_model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = texture;
    floor_model.materials[0].maps[MATERIAL_MAP_METALNESS].value = 0.04f;
    floor_model.materials[0].maps[MATERIAL_MAP_ROUGHNESS].value = 0.91f;
    bind_shader(floor_model);
    textures_.emplace(scene.floor.albedo_texture, texture);
    floor_models_.emplace(scene.floor.albedo_texture, floor_model);
   }
  }
  for (const auto& [prefab_id, prefab] : scene.prefabs) {
   (void)prefab_id;
   for (const auto& part : prefab.parts)
    load_model(part.model_3d);
  }
 }
}

SceneRenderer3D::~SceneRenderer3D() {
 if (shadow_map_.id > 0)
  rlUnloadFramebuffer(shadow_map_.id);
 if (scene_target_.id > 0)
  UnloadRenderTexture(scene_target_);
 if (bloom_target_a_.id > 0)
  UnloadRenderTexture(bloom_target_a_);
 if (bloom_target_b_.id > 0)
  UnloadRenderTexture(bloom_target_b_);
 for (auto& [path, loaded] : models_) {
  (void)path;
  UnloadModel(loaded);
 }
 for (auto& [path, loaded] : floor_models_) {
  (void)path;
  UnloadModel(loaded);
 }
 for (auto& [path, loaded] : textures_) {
  (void)path;
  UnloadTexture(loaded);
 }
 if (cube_model_.meshCount > 0)
  UnloadModel(cube_model_);
 if (IsShaderValid(shader_))
  UnloadShader(shader_);
 if (IsShaderValid(bloom_extract_shader_))
  UnloadShader(bloom_extract_shader_);
 if (IsShaderValid(bloom_blur_shader_))
  UnloadShader(bloom_blur_shader_);
 if (IsShaderValid(composite_shader_))
  UnloadShader(composite_shader_);
}

void SceneRenderer3D::bind_shader(Model& model_value) const {
 if (!IsShaderValid(shader_))
  return;
 for (int index = 0; index < model_value.materialCount; ++index) {
  auto& material = model_value.materials[index];
  const float metallic = std::clamp(material.maps[MATERIAL_MAP_METALNESS].value, 0.0f, 1.0f);
  const float roughness = std::clamp(material.maps[MATERIAL_MAP_ROUGHNESS].value, 0.04f, 1.0f);
  const auto emission = material.maps[MATERIAL_MAP_EMISSION].color;
  const float emissive = static_cast<float>(std::max({emission.r, emission.g, emission.b})) / 255.0f;
  material.maps[MATERIAL_MAP_METALNESS].color = {static_cast<unsigned char>(std::round(metallic * 255.0f)),
                                                 static_cast<unsigned char>(std::round(roughness * 255.0f)),
                                                 static_cast<unsigned char>(std::round(emissive * 255.0f)),
                                                 255};
  material.shader = shader_;
 }
}

void SceneRenderer3D::ensure_shadow_map(int resolution) {
 if (resolution == shadow_map_resolution_)
  return;
 if (shadow_map_.id > 0)
  rlUnloadFramebuffer(shadow_map_.id);
 shadow_map_ = {};
 shadow_map_resolution_ = 0;
 if (resolution <= 0)
  return;
 shadow_map_ = load_shadow_map(resolution, resolution);
 if (shadow_map_.id > 0)
  shadow_map_resolution_ = resolution;
}

void SceneRenderer3D::ensure_post_targets(int width, int height) {
 if (width == target_width_ && height == target_height_)
  return;
 if (scene_target_.id > 0)
  UnloadRenderTexture(scene_target_);
 if (bloom_target_a_.id > 0)
  UnloadRenderTexture(bloom_target_a_);
 if (bloom_target_b_.id > 0)
  UnloadRenderTexture(bloom_target_b_);
 scene_target_ = {};
 bloom_target_a_ = {};
 bloom_target_b_ = {};
 target_width_ = 0;
 target_height_ = 0;
 if (width <= 0 || height <= 0)
  return;

 scene_target_ = load_hdr_target(width, height);
 const int bloom_width = std::max(1, width / 2);
 const int bloom_height = std::max(1, height / 2);
 bloom_target_a_ = LoadRenderTexture(bloom_width, bloom_height);
 bloom_target_b_ = LoadRenderTexture(bloom_width, bloom_height);
 if (scene_target_.id > 0 && bloom_target_a_.id > 0 && bloom_target_b_.id > 0) {
  SetTextureFilter(bloom_target_a_.texture, TEXTURE_FILTER_BILINEAR);
  SetTextureFilter(bloom_target_b_.texture, TEXTURE_FILTER_BILINEAR);
  target_width_ = width;
  target_height_ = height;
 }
}

const Model* SceneRenderer3D::model(const std::string& path) const {
 const auto found = models_.find(path);
 return found == models_.end() ? nullptr : &found->second;
}

void SceneRenderer3D::draw_box(Vector3 position, Vector3 size, float yaw_degrees, Color color) const {
 DrawModelEx(cube_model_, position, {0, 1, 0}, yaw_degrees, size, color);
}

void SceneRenderer3D::draw(const game::MissionSimulation& mission, int screen_width, int screen_height) {
 const auto& state = mission.snapshot();
 const auto& scene = catalog_.scene(mission.map_definition().scene_3d);
 sim::Vec2 focus{state.robot.x, state.robot.y};
 const auto actor =
   std::find_if(state.actors.begin(), state.actors.end(), [](const auto& value) { return value.detected; });
 const auto goal = mission.map_definition().markers.find(mission.level().objective.goal);
 const bool success_shot =
   state.status == game::MissionStatus::Succeeded && goal != mission.map_definition().markers.end();
 const bool event_shot = !success_shot && actor != state.actors.end() && state.dynamic_object_detected;
 const game::SceneShotDefinition* shot =
   success_shot ? &scene.camera.success_shot : (event_shot ? &scene.camera.event_shot : nullptr);
 if (shot) {
  focus.x = state.robot.x * shot->robot_focus_weight;
  focus.y = state.robot.y * shot->robot_focus_weight;
  if (actor != state.actors.end()) {
   focus.x += actor->position.x * shot->actor_focus_weight;
   focus.y += actor->position.y * shot->actor_focus_weight;
  }
  if (goal != mission.map_definition().markers.end()) {
   focus.x += goal->second.x * shot->goal_focus_weight;
   focus.y += goal->second.y * shot->goal_focus_weight;
  }
 } else if (actor != state.actors.end()) {
  focus.x =
    state.robot.x * scene.camera.robot_focus_weight + actor->position.x * scene.camera.actor_focus_weight;
  focus.y =
    state.robot.y * scene.camera.robot_focus_weight + actor->position.y * scene.camera.actor_focus_weight;
  if (goal != mission.map_definition().markers.end()) {
   focus.x += goal->second.x * scene.camera.goal_focus_weight;
   focus.y += goal->second.y * scene.camera.goal_focus_weight;
  }
 }
 const Vector3 target{static_cast<float>(focus.x),
                      static_cast<float>(shot ? shot->target_height_m : scene.camera.target_height_m),
                      static_cast<float>(-focus.y)};
 const Vector3 offset = world(shot ? shot->offset_m : scene.camera.offset_m);
 const float fov = static_cast<float>(shot ? shot->fov_degrees : scene.camera.fov_degrees);
 const bool reset_camera =
   !camera_initialized_ || camera_level_ != mission.level().id || state.time_s + 1e-9 < camera_time_s_;
 if (reset_camera) {
  camera_focus_ = target;
  camera_offset_ = offset;
  camera_fov_ = fov;
  camera_initialized_ = true;
  camera_level_ = mission.level().id;
 } else {
  const double simulation_elapsed = std::max(0.0, state.time_s - camera_time_s_);
  const float elapsed = static_cast<float>(simulation_elapsed > 0.0 ? simulation_elapsed : 1.0 / 60.0);
  const float blend = 1.0f - std::exp(-elapsed * 4.6f / static_cast<float>(scene.camera.transition_s));
  camera_focus_ = Vector3Lerp(camera_focus_, target, blend);
  camera_offset_ = Vector3Lerp(camera_offset_, offset, blend);
  camera_fov_ += (fov - camera_fov_) * blend;
 }
 camera_time_s_ = state.time_s;
 camera_.target = camera_focus_;
 camera_.position = Vector3Add(camera_focus_, camera_offset_);
 camera_.up = {0, 1, 0};
 camera_.fovy = camera_fov_;
 camera_.projection = CAMERA_PERSPECTIVE;

 if (IsShaderValid(shader_) && view_position_loc_ >= 0) {
  SetShaderValue(shader_, view_position_loc_, &camera_.position, SHADER_UNIFORM_VEC3);
  const Vector3 light_direction = Vector3Normalize(world(scene.lighting.direction));
  const Vector3 light_color =
    shader_color(scene.lighting.directional_color, scene.lighting.directional_intensity);
  const Vector3 ambient = shader_color(scene.lighting.ambient_color);
  const Vector3 fog_color = shader_color(scene.lighting.fog_color);
  const float fog_start = static_cast<float>(scene.lighting.fog_start_m);
  const float fog_end = static_cast<float>(scene.lighting.fog_end_m);
  const float shadow_strength =
    scene.lighting.shadow_map_resolution > 0 ? static_cast<float>(scene.lighting.shadow_strength) : 0.0f;
  const int shadow_resolution = std::max(1, scene.lighting.shadow_map_resolution);
  SetShaderValue(shader_, GetShaderLocation(shader_, "lightDirection"), &light_direction,
                 SHADER_UNIFORM_VEC3);
  SetShaderValue(shader_, GetShaderLocation(shader_, "lightColor"), &light_color, SHADER_UNIFORM_VEC3);
  SetShaderValue(shader_, GetShaderLocation(shader_, "ambient"), &ambient, SHADER_UNIFORM_VEC3);
  SetShaderValue(shader_, GetShaderLocation(shader_, "fogColor"), &fog_color, SHADER_UNIFORM_VEC3);
  SetShaderValue(shader_, GetShaderLocation(shader_, "fogStart"), &fog_start, SHADER_UNIFORM_FLOAT);
  SetShaderValue(shader_, GetShaderLocation(shader_, "fogEnd"), &fog_end, SHADER_UNIFORM_FLOAT);
  SetShaderValue(shader_, shadow_strength_loc_, &shadow_strength, SHADER_UNIFORM_FLOAT);
  SetShaderValue(shader_, shadow_resolution_loc_, &shadow_resolution, SHADER_UNIFORM_INT);
  const Vector3 robot_light{static_cast<float>(state.robot.x), 1.15f, static_cast<float>(-state.robot.y)};
  const Vector3 robot_color = shader_color(state.waiting_for_clearance ? scene.lighting.robot_waiting_color
                                                                       : scene.lighting.robot_active_color);
  Vector3 signal_light{camera_focus_.x, 1.6f, camera_focus_.z};
  if (goal != mission.map_definition().markers.end()) {
   signal_light = world(goal->second, 1.6f);
  }
  const Vector3 signal_color = shader_color(scene.lighting.goal_color);
  SetShaderValue(shader_, point_light_0_position_loc_, &robot_light, SHADER_UNIFORM_VEC3);
  SetShaderValue(shader_, point_light_0_color_loc_, &robot_color, SHADER_UNIFORM_VEC3);
  SetShaderValue(shader_, point_light_1_position_loc_, &signal_light, SHADER_UNIFORM_VEC3);
  SetShaderValue(shader_, point_light_1_color_loc_, &signal_color, SHADER_UNIFORM_VEC3);
 }

 ensure_shadow_map(scene.lighting.shadow_map_resolution);
 ensure_post_targets(screen_width, screen_height);
 const Vector3 light_direction = Vector3Normalize(world(scene.lighting.direction));
 if (shadow_map_.id > 0)
  render_shadow_map(mission, scene, light_direction);
 const bool post_processing_available = IsShaderValid(bloom_extract_shader_) &&
                                        IsShaderValid(bloom_blur_shader_) && IsShaderValid(composite_shader_);
 if (scene_target_.id > 0 && post_processing_available) {
  render_scene_target(mission, scene);
  composite_scene(scene);
 } else {
  BeginMode3D(camera_);
  if (IsShaderValid(shader_))
   BeginShaderMode(shader_);
  draw_scene(mission);
  if (IsShaderValid(shader_))
   EndShaderMode();
  draw_accents(mission);
  EndMode3D();
 }
}

void SceneRenderer3D::render_shadow_map(const game::MissionSimulation& mission,
                                        const game::SceneDefinition& scene, Vector3 light_direction) {
 const auto& map = mission.map_definition();
 const Vector3 center{static_cast<float>(map.size_m.x * 0.5), 0.35f, static_cast<float>(-map.size_m.y * 0.5)};
 Camera3D light_camera{};
 light_camera.target = center;
 light_camera.position =
   Vector3Subtract(center, Vector3Scale(light_direction, static_cast<float>(scene.lighting.shadow_extent_m)));
 light_camera.up =
   std::abs(Vector3DotProduct(light_direction, {0, 1, 0})) > 0.96f ? Vector3{0, 0, -1} : Vector3{0, 1, 0};
 light_camera.fovy = static_cast<float>(scene.lighting.shadow_extent_m);
 light_camera.projection = CAMERA_ORTHOGRAPHIC;

 rlActiveTextureSlot(10);
 rlDisableTexture();
 rlActiveTextureSlot(0);
 BeginTextureMode(shadow_map_);
 ClearBackground(WHITE);
 BeginMode3D(light_camera);
 const Matrix light_view = rlGetMatrixModelview();
 const Matrix light_projection = rlGetMatrixProjection();
 if (IsShaderValid(shader_))
  BeginShaderMode(shader_);
 draw_scene(mission);
 if (IsShaderValid(shader_))
  EndShaderMode();
 EndMode3D();
 EndTextureMode();
 const Matrix light_view_projection = MatrixMultiply(light_view, light_projection);
 SetShaderValueMatrix(shader_, light_view_projection_loc_, light_view_projection);
}

void SceneRenderer3D::render_scene_target(const game::MissionSimulation& mission,
                                          const game::SceneDefinition& scene) {
 if (shadow_map_.id > 0) {
  const int shadow_texture_slot = 10;
  rlActiveTextureSlot(10);
  rlEnableTexture(shadow_map_.depth.id);
  rlSetUniform(shadow_map_loc_, &shadow_texture_slot, SHADER_UNIFORM_INT, 1);
  rlActiveTextureSlot(0);
 }
 BeginTextureMode(scene_target_);
 ClearBackground(color(scene.lighting.fog_color));
 BeginMode3D(camera_);
 if (IsShaderValid(shader_))
  BeginShaderMode(shader_);
 draw_scene(mission);
 if (IsShaderValid(shader_))
  EndShaderMode();
 draw_accents(mission);
 EndMode3D();
 EndTextureMode();
 if (shadow_map_.id > 0) {
  rlActiveTextureSlot(10);
  rlDisableTexture();
  rlActiveTextureSlot(0);
 }
}

void SceneRenderer3D::composite_scene(const game::SceneDefinition& scene) const {
 const float bloom_strength = static_cast<float>(scene.lighting.bloom_strength);
 if (bloom_strength > 0.001f) {
  const float bloom_threshold = static_cast<float>(scene.lighting.bloom_threshold);
  SetShaderValue(bloom_extract_shader_, bloom_threshold_loc_, &bloom_threshold, SHADER_UNIFORM_FLOAT);
  BeginTextureMode(bloom_target_a_);
  ClearBackground(BLACK);
  BeginShaderMode(bloom_extract_shader_);
  draw_render_texture(scene_target_, static_cast<float>(bloom_target_a_.texture.width),
                      static_cast<float>(bloom_target_a_.texture.height));
  EndShaderMode();
  EndTextureMode();

  const Vector2 texel_size{1.0f / static_cast<float>(bloom_target_a_.texture.width),
                           1.0f / static_cast<float>(bloom_target_a_.texture.height)};
  SetShaderValue(bloom_blur_shader_, blur_texel_size_loc_, &texel_size, SHADER_UNIFORM_VEC2);
  for (int pass = 0; pass < 2; ++pass) {
   const Vector2 horizontal{1.0f, 0.0f};
   SetShaderValue(bloom_blur_shader_, blur_direction_loc_, &horizontal, SHADER_UNIFORM_VEC2);
   BeginTextureMode(bloom_target_b_);
   ClearBackground(BLACK);
   BeginShaderMode(bloom_blur_shader_);
   draw_render_texture(bloom_target_a_, static_cast<float>(bloom_target_b_.texture.width),
                       static_cast<float>(bloom_target_b_.texture.height));
   EndShaderMode();
   EndTextureMode();

   const Vector2 vertical{0.0f, 1.0f};
   SetShaderValue(bloom_blur_shader_, blur_direction_loc_, &vertical, SHADER_UNIFORM_VEC2);
   BeginTextureMode(bloom_target_a_);
   ClearBackground(BLACK);
   BeginShaderMode(bloom_blur_shader_);
   draw_render_texture(bloom_target_b_, static_cast<float>(bloom_target_a_.texture.width),
                       static_cast<float>(bloom_target_a_.texture.height));
   EndShaderMode();
   EndTextureMode();
  }
 } else {
  BeginTextureMode(bloom_target_a_);
  ClearBackground(BLACK);
  EndTextureMode();
 }

 const float exposure = static_cast<float>(scene.lighting.exposure);
 const float vignette_strength = static_cast<float>(scene.lighting.vignette_strength);
 SetShaderValue(composite_shader_, composite_exposure_loc_, &exposure, SHADER_UNIFORM_FLOAT);
 SetShaderValue(composite_shader_, composite_bloom_strength_loc_, &bloom_strength, SHADER_UNIFORM_FLOAT);
 SetShaderValue(composite_shader_, composite_vignette_strength_loc_, &vignette_strength,
                SHADER_UNIFORM_FLOAT);
 SetShaderValueTexture(composite_shader_, composite_bloom_texture_loc_, bloom_target_a_.texture);
 BeginShaderMode(composite_shader_);
 draw_render_texture(scene_target_, static_cast<float>(target_width_), static_cast<float>(target_height_));
 EndShaderMode();
}

void SceneRenderer3D::draw_scene(const game::MissionSimulation& mission) const {
 const auto& map = mission.map_definition();
 const auto& scene = catalog_.scene(map.scene_3d);
 draw_floor(map, scene);
 draw_environment(mission, scene);
 for (const auto& surface : map.surfaces) {
  if (!game::active_in_level(surface.active_in, mission.level().id))
   continue;
  draw_surface(surface);
 }
 for (const auto& obstacle : map.obstacles) {
  if (game::active_in_level(obstacle.active_in, mission.level().id))
   draw_shelf(obstacle);
 }
 draw_markers(mission);
 for (const auto& actor : mission.snapshot().actors) {
  draw_forklift(actor, mission.snapshot().waiting_for_clearance);
 }
 draw_robot(mission);
}

void SceneRenderer3D::draw_floor(const game::MapDefinition& map, const game::SceneDefinition& scene) const {
 const float width = static_cast<float>(map.size_m.x);
 const float length = static_cast<float>(map.size_m.y);
 DrawPlane({width * 0.5f, -0.08f, -length * 0.5f}, {width + 10.0f, length + 10.0f},
           color(scene.floor.base_color));
 const Color tile_color = color(scene.floor.tile_color);
 const auto floor_model = floor_models_.find(scene.floor.albedo_texture);
 for (double x = 0.0; x < map.size_m.x; x += scene.floor.tile_size_m) {
  for (double y = 0.0; y < map.size_m.y; y += scene.floor.tile_size_m) {
   const float tile_width =
     std::min(static_cast<float>(scene.floor.tile_size_m), width - static_cast<float>(x));
   const float tile_length =
     std::min(static_cast<float>(scene.floor.tile_size_m), length - static_cast<float>(y));
   const int tile_x = static_cast<int>(std::round(x * 100.0));
   const int tile_y = static_cast<int>(std::round(y * 100.0));
   const int tone = static_cast<int>(hash01(tile_x, tile_y) * scene.floor.color_variation);
   const int rotation_steps = std::abs(tile_width - tile_length) < 0.001f
                                ? static_cast<int>(hash01(tile_x, tile_y, 17) * 4.0f) % 4
                                : static_cast<int>(hash01(tile_x, tile_y, 17) * 2.0f) % 2 * 2;
   const float tile_yaw = static_cast<float>(rotation_steps * 90);
   const Vector3 tile_position{static_cast<float>(x) + tile_width * 0.5f, -0.035f,
                               -(static_cast<float>(y) + tile_length * 0.5f)};
   const Vector3 tile_scale{tile_width - static_cast<float>(scene.floor.seam_m), 0.07f,
                            tile_length - static_cast<float>(scene.floor.seam_m)};
   const Color tile_tint{static_cast<unsigned char>(std::min(255, tile_color.r + tone)),
                         static_cast<unsigned char>(std::min(255, tile_color.g + tone)),
                         static_cast<unsigned char>(std::min(255, tile_color.b + tone)), tile_color.a};
   if (floor_model != floor_models_.end()) {
    DrawModelEx(floor_model->second, tile_position, {0, 1, 0}, tile_yaw, tile_scale, tile_tint);
   } else {
    draw_box(tile_position, tile_scale, tile_yaw, tile_tint);
   }
  }
 }
 const float boundary_height = static_cast<float>(scene.floor.boundary_height_m);
 const float boundary_width = static_cast<float>(scene.floor.boundary_width_m);
 const Color boundary = color(scene.floor.boundary_color);
 draw_box({width * 0.5f, boundary_height * 0.5f, 0.02f},
          {width + boundary_width, boundary_height, boundary_width}, 0, boundary);
 draw_box({width * 0.5f, boundary_height * 0.5f, -length - 0.02f},
          {width + boundary_width, boundary_height, boundary_width}, 0, boundary);
 draw_box({-0.02f, boundary_height * 0.5f, -length * 0.5f}, {boundary_width, boundary_height, length}, 0,
          boundary);
 draw_box({width + 0.02f, boundary_height * 0.5f, -length * 0.5f}, {boundary_width, boundary_height, length},
          0, boundary);
}

void SceneRenderer3D::draw_shelf(const game::BoxDefinition& obstacle) const {
 const float min_x = static_cast<float>(obstacle.min.x);
 const float max_x = static_cast<float>(obstacle.max.x);
 const float min_z = static_cast<float>(-obstacle.max.y);
 const float max_z = static_cast<float>(-obstacle.min.y);
 const float width = max_x - min_x;
 const float length = max_z - min_z;
 const float center_x = (min_x + max_x) * 0.5f;
 const float center_z = (min_z + max_z) * 0.5f;
 constexpr float height = 2.45f;
 constexpr float post = 0.10f;
 for (float x : {min_x + post, max_x - post}) {
  for (float z : {min_z + post, max_z - post}) {
   draw_box({x, height * 0.5f, z}, {post, height, post}, 0, kMetalEdge);
  }
 }
 for (int level = 0; level < 4; ++level) {
  const float y = 0.24f + level * 0.68f;
  draw_box({center_x, y, center_z}, {width, 0.09f, length}, 0, kMetal);
 }
 const bool along_z = length >= width;
 const float long_side = along_z ? length : width;
 const int bays = std::max(1, static_cast<int>(long_side / 1.15f));
 for (int bay = 0; bay < bays; ++bay) {
  for (int level = 0; level < 3; ++level) {
   if ((bay + level) % 5 == 3)
    continue;
   const float offset = -long_side * 0.5f + (bay + 0.5f) * long_side / bays;
   const Vector3 position = along_z ? Vector3{center_x, 0.53f + level * 0.68f, center_z + offset}
                                    : Vector3{center_x + offset, 0.53f + level * 0.68f, center_z};
   const Vector3 size = along_z ? Vector3{std::max(0.22f, width - 0.25f), 0.48f, long_side / bays * 0.70f}
                                : Vector3{long_side / bays * 0.70f, 0.48f, std::max(0.22f, length - 0.25f)};
   draw_box(position, size, 0, cargo_color(bay + level));
  }
 }
}

void SceneRenderer3D::draw_surface(const game::SurfaceDefinition& surface) const {
 const float width = static_cast<float>(surface.max.x - surface.min.x);
 const float length = static_cast<float>(surface.max.y - surface.min.y);
 const float center_x = static_cast<float>((surface.min.x + surface.max.x) * 0.5);
 const float center_z = static_cast<float>(-(surface.min.y + surface.max.y) * 0.5);
 // Material albedo quad (1px = 1 texel per meter tile): the ground reads as
 // its physics (sand looks sandy) instead of generic teal boxes. Empty
 // texture falls back to the old patches (same as before).
 const auto& material = catalog_.surface_material(surface.material);
 const auto plane = surface_models_.find(material.texture_albedo);
 if (!material.texture_albedo.empty() && plane != surface_models_.end()) {
  const int tiles_x = std::max(1, static_cast<int>(std::ceil(width)));
  const int tiles_z = std::max(1, static_cast<int>(std::ceil(length)));
  const float cell_w = width / static_cast<float>(tiles_x);
  const float cell_l = length / static_cast<float>(tiles_z);
  for (int ix = 0; ix < tiles_x; ++ix) {
   for (int iz = 0; iz < tiles_z; ++iz) {
    const float cx = static_cast<float>(surface.min.x) + (static_cast<float>(ix) + 0.5f) * cell_w;
    const float cz = static_cast<float>(surface.min.y) + (static_cast<float>(iz) + 0.5f) * cell_l;
    DrawModelEx(plane->second, {cx, 0.006f, -cz}, {0, 1, 0}, 0.0f, {cell_w, 1.0f, cell_l}, WHITE);
   }
  }
  return;
 }
 for (int patch = 0; patch < 5; ++patch) {
  const float x = center_x + (hash01(patch, 4, 1) - 0.5f) * width * 0.62f;
  const float z = center_z + (hash01(patch, 5, 2) - 0.5f) * length * 0.62f;
  draw_box(
    {x, 0.015f, z},
    {width * (0.20f + hash01(patch, 6, 3) * 0.19f), 0.018f, length * (0.18f + hash01(patch, 7, 4) * 0.18f)},
    hash01(patch, 8, 5) * 35.0f, {39, 104, 105, 150});
 }
}

void SceneRenderer3D::draw_markers(const game::MissionSimulation& mission) const {
 const auto& map = mission.map_definition();
 // Supply markers are INFORMATION (drive here), not furniture: a solid crate
 // on the marker forces the robot to drive through box art on collection
 // (via radius 1.25, arrival 0.62 < crate half-width + body). Flat ring plus
 // floating cube plus light beam read as a collection zone and never
 // overlap the car body.
 const auto draw_supply = [&](const game::Pose& pose) {
  const auto base = world(pose);
  DrawCylinderEx({base.x, 0.01f, base.z}, {base.x, 0.05f, base.z}, 0.68f, 0.68f, 40, Fade(kAmber, 0.30f));
  DrawCylinderEx({base.x, 0.055f, base.z}, {base.x, 0.075f, base.z}, 0.60f, 0.60f, 40, kAmber);
  DrawCylinderEx({base.x, 0.02f, base.z}, {base.x, 1.15f, base.z}, 0.045f, 0.045f, 12, Fade(kAmber, 0.30f));
  DrawModelEx(cube_model_, {base.x, 1.30f, base.z}, {0, 1, 0},
              static_cast<float>(mission.snapshot().time_s * 40.0), {0.16f, 0.16f, 0.16f}, kAmber);
 };
 if (!mission.level().objective.via.empty()) {
  if (const auto via = map.markers.find(mission.level().objective.via); via != map.markers.end()) {
   draw_supply(via->second);
  }
 }
 if (const auto goal = map.markers.find(mission.level().objective.goal); goal != map.markers.end()) {
  const auto base = world(goal->second);
  if (mission.level().objective.goal_visual == "supply_station") {
   draw_supply(goal->second);
  } else if (mission.level().objective.goal_visual == "signal_tower") {
   // Mast, not a pillar: arrival tolerance (0.62) is smaller than base art +
   // body art would allow, so the shaft stays slim and the dock boundary is
   // drawn as a ground ring. Same information-not-furniture rule as supply.
   DrawCylinderEx({base.x, 0.0f, base.z}, {base.x, 2.35f, base.z}, 0.16f, 0.10f, 16, {96, 103, 98, 255});
   // Dock boundary ring draws the actual arrival rule, not a fixed number.
   const float dock = static_cast<float>(mission.level().objective.arrive_tolerance_m);
   DrawCylinderEx({base.x, 0.01f, base.z}, {base.x, 0.05f, base.z}, dock, dock, 40, Fade(kAmber, 0.30f));
   DrawCylinderEx({base.x, 0.055f, base.z}, {base.x, 0.075f, base.z}, dock - 0.07f, dock - 0.07f, 40, kAmber);
   DrawCylinderEx({base.x, 0.24f, base.z}, {base.x, 0.34f, base.z}, 0.52f, 0.52f, 20, {65, 70, 67, 255});
   DrawCylinderEx({base.x, 1.62f, base.z}, {base.x, 1.72f, base.z}, 0.28f, 0.28f, 16, {126, 132, 122, 255});
  } else {
   const Color marker = mission.level().objective.goal_visual == "stop_zone" ? kAmber : kCyan;
   DrawCylinderEx({base.x, 0.01f, base.z}, {base.x, 0.055f, base.z}, 0.58f, 0.58f, 24, Fade(marker, 0.55f));
   DrawCylinderWiresEx({base.x, 0.06f, base.z}, {base.x, 0.075f, base.z}, 0.58f, 0.58f, 24, marker);
   if (mission.level().objective.goal_visual == "calibration_pad") {
    draw_box({base.x, 0.12f, base.z}, {0.52f, 0.12f, 0.52f}, 45.0f, {63, 103, 96, 255});
   }
  }
 }
}

void SceneRenderer3D::draw_robot(const game::MissionSimulation& mission) const {
 const auto& state = mission.snapshot();
 const auto& scene = catalog_.scene(mission.map_definition().scene_3d);
 const auto& robot = mission.robot();
 const auto find_mount = [&](const std::string& id) -> const game::MountDefinition* {
  const auto found =
    std::find_if(robot.mounts.begin(), robot.mounts.end(), [&](const auto& mount) { return mount.id == id; });
  return found == robot.mounts.end() ? nullptr : &*found;
 };
 const auto component_id = [&](const std::string& slot) -> const std::string& {
  return mission.build().part(slot);
 };
 const auto draw_part = [&](const game::VisualDefinition& visual, const game::MountDefinition& mount) {
  const Model* loaded = model(visual.model_3d);
  if (!loaded)
   return;
  auto position =
    pose_offset({state.robot.x, state.robot.y}, state.robot.theta, static_cast<float>(mount.position_m.x),
                static_cast<float>(mount.position_m.y), static_cast<float>(mount.position_m.z));
  if (mount.id != "wheel_left" && mount.id != "wheel_right" && mount.id != "front_left" &&
      mount.id != "front_right" && mount.id != "rear_left" && mount.id != "rear_right") {
   const float movement = std::min(1.0f, std::abs(static_cast<float>(state.robot.v)));
   const float phase = static_cast<float>(state.time_s * scene.effects.bob_frequency_hz * 2.0 * PI);
   position.y += std::sin(phase) * movement * static_cast<float>(scene.effects.bob_amplitude_m);
   if (state.waiting_for_clearance) {
    position.y += 0.008f + std::sin(phase * 0.45f) * static_cast<float>(scene.effects.bob_amplitude_m * 0.22);
   }
  }
  float yaw = static_cast<float>(state.robot.theta) * kRadToDeg;
  // Steered wheels (ackermann linkage output): same rotational sense as body
  // yaw, since both live in world-CCW. Inner/outer already resolved in physics.
  if (mount.id == "front_left" || mount.id == "front_right") {
   const double steer = mount.id == "front_left" ? mission.snapshot().control.steer_left_rad
                                                 : mission.snapshot().control.steer_right_rad;
   yaw += static_cast<float>(steer) * kRadToDeg;
  }
  if (mount.id == "sensor_top") {
   if (!mission.build().part("sensor").empty()) {
    const double scan_hz =
      game::property(catalog_.component(mission.build().part("sensor")).behavior, "scan_hz");
    yaw += static_cast<float>(std::fmod(state.time_s * scan_hz * 360.0, 360.0));
   }
  }
  DrawModelEx(*loaded, position, {0, 1, 0}, yaw,
              Vector3Scale({1, 1, 1}, static_cast<float>(scene.effects.robot_scale)), WHITE);
 };

 for (const auto& part : robot.base_parts) {
  if (const auto* mount = find_mount(part.mount))
   draw_part(part.visual, *mount);
 }
 for (const auto& slot : robot.slots) {
  const auto& id = component_id(slot.slot);
  if (id.empty())
   continue;
  const auto& visual = catalog_.component(id).visual;
  for (const auto& mount_id : slot.mounts) {
   if (const auto* mount = find_mount(mount_id))
    draw_part(visual, *mount);
  }
 }
}

void SceneRenderer3D::draw_forklift(const game::ActorSnapshot& actor, bool waiting) const {
 const float yaw = static_cast<float>(std::atan2(actor.velocity.y, actor.velocity.x)) * kRadToDeg;
 const sim::Vec2 position = actor.position;
 draw_box(pose_offset(position, yaw / kRadToDeg, 0, 0, 0.34f), {1.12f, 0.58f, 0.72f}, yaw,
          {211, 132, 39, 255});
 draw_box(pose_offset(position, yaw / kRadToDeg, -0.18f, 0, 0.72f), {0.48f, 0.48f, 0.58f}, yaw,
          {48, 59, 57, 255});
 draw_box(pose_offset(position, yaw / kRadToDeg, 0.58f, 0.24f, 0.15f), {0.78f, 0.08f, 0.10f}, yaw,
          {92, 90, 78, 255});
 draw_box(pose_offset(position, yaw / kRadToDeg, 0.58f, -0.24f, 0.15f), {0.78f, 0.08f, 0.10f}, yaw,
          {92, 90, 78, 255});
 for (float forward : {-0.33f, 0.33f}) {
  for (float left : {-0.38f, 0.38f}) {
   const auto wheel = pose_offset(position, yaw / kRadToDeg, forward, left, 0.19f);
   draw_box(wheel, {0.24f, 0.28f, 0.13f}, yaw, {24, 27, 25, 255});
  }
 }
 if (waiting) {
  const auto beacon = pose_offset(position, yaw / kRadToDeg, -0.18f, 0, 1.0f);
  DrawSphere(beacon, 0.09f, kDanger);
 }
}

void SceneRenderer3D::draw_environment(const game::MissionSimulation& mission,
                                       const game::SceneDefinition& scene) const {
 for (const auto& instance : scene.instances) {
  if (!game::active_in_level(instance.active_in, mission.level().id))
   continue;
  const auto prefab = scene.prefabs.find(instance.prefab);
  if (prefab == scene.prefabs.end())
   continue;
  for (const auto& part : prefab->second.parts) {
   const game::Vec3 scaled_offset{
     part.position_m.x * instance.scale.x,
     part.position_m.y * instance.scale.y,
     part.position_m.z * instance.scale.z,
   };
   const auto offset = rotate_offset(scaled_offset, instance.yaw_degrees);
   const game::Vec3 position{
     instance.position_m.x + offset.x,
     instance.position_m.y + offset.y,
     instance.position_m.z + offset.z,
   };
   const game::Vec3 size{
     part.size_m.x * instance.scale.x,
     part.size_m.y * instance.scale.y,
     part.size_m.z * instance.scale.z,
   };
   const float yaw = static_cast<float>(instance.yaw_degrees + part.yaw_degrees);
   const Color tint = color(part.color);
   if (!part.model_3d.empty()) {
    if (const auto* loaded = model(part.model_3d)) {
     DrawModelEx(*loaded, world(position), {0, 1, 0}, yaw,
                 {static_cast<float>(size.x), static_cast<float>(size.y), static_cast<float>(size.z)}, tint);
    }
   } else if (part.primitive == "box") {
    draw_box(world(position),
             {static_cast<float>(size.x), static_cast<float>(size.z), static_cast<float>(size.y)}, yaw, tint);
   } else if (part.primitive == "sphere") {
    const float radius = static_cast<float>((size.x + size.y + size.z) / 3.0);
    DrawSphere(world(position), radius, tint);
   } else if (part.primitive == "cylinder") {
    const auto base = world(game::Vec3{position.x, position.y, position.z - size.z * 0.5});
    const auto top = world(game::Vec3{position.x, position.y, position.z + size.z * 0.5});
    const float planar_scale = static_cast<float>((instance.scale.x + instance.scale.y) * 0.5);
    DrawCylinderEx(base, top, static_cast<float>(part.size_m.x) * planar_scale,
                   static_cast<float>(part.size_m.y) * planar_scale, 8, tint);
   }
  }
 }
}

void SceneRenderer3D::draw_accents(const game::MissionSimulation& mission) const {
 const auto& state = mission.snapshot();
 const auto& scene = catalog_.scene(mission.map_definition().scene_3d);
 const Color active_color = color(scene.lighting.robot_active_color);
 const Color waiting_color = color(scene.lighting.robot_waiting_color);
 const Color robot_color = state.waiting_for_clearance ? waiting_color : active_color;
 const float movement = std::min(1.0f, std::abs(static_cast<float>(state.robot.v)));
 const float bob_phase = static_cast<float>(state.time_s * scene.effects.bob_frequency_hz * 2.0 * PI);
 const float bob = std::sin(bob_phase) * movement * static_cast<float>(scene.effects.bob_amplitude_m);
 const auto center = world(sim::Vec2{state.robot.x, state.robot.y},
                           static_cast<float>(scene.effects.perception_link_height_m) + bob);

 for (float left : {-0.10f, 0.10f}) {
  const auto eye = pose_offset({state.robot.x, state.robot.y}, state.robot.theta, 0.365f, left,
                               static_cast<float>(scene.effects.eye_height_m) + bob);
  DrawSphere(eye, 0.040f, {19, 27, 25, 255});
  const float eye_pulse =
    state.waiting_for_clearance ? 0.021f + 0.003f * std::sin(static_cast<float>(state.time_s * 8.0)) : 0.024f;
  DrawSphere(eye, eye_pulse, robot_color);
 }

 if (movement > 0.04f && state.status == game::MissionStatus::Running) {
  for (float left : {-0.16f, 0.16f}) {
   const auto trail_start =
     pose_offset({state.robot.x, state.robot.y}, state.robot.theta, -0.28f, left, 0.10f);
   const auto trail_end =
     pose_offset({state.robot.x, state.robot.y}, state.robot.theta, -0.28f - 0.72f * movement, left, 0.065f);
   DrawLine3D(trail_start, trail_end, Fade(active_color, 0.20f * movement));
  }
 }
 if (!mission.build().part("sensor").empty() && state.status == game::MissionStatus::Running) {
  const double scan_hz =
    game::property(catalog_.component(mission.build().part("sensor")).behavior, "scan_hz");
  const float angle = static_cast<float>(state.robot.theta + state.time_s * scan_hz * 2.0 * PI);
  const float scan_radius = static_cast<float>(scene.effects.scan_radius_m);
  constexpr float half_width = 0.16f;
  const auto scan_point = [&](float offset, float radius) {
   return Vector3{center.x + std::cos(angle + offset) * radius, 0.026f,
                  center.z - std::sin(angle + offset) * radius};
  };
  const Vector3 origin{center.x, 0.026f, center.z};
  const Vector3 left = scan_point(-half_width, scan_radius);
  const Vector3 right = scan_point(half_width, scan_radius);
  DrawTriangle3D(origin, left, right, Fade(kCyan, 0.055f));
  DrawLine3D(origin, scan_point(0.0f, scan_radius), Fade(kCyan, 0.28f));
 }
 DrawCircle3D({center.x, 0.018f, center.z}, 0.72f, {1, 0, 0}, 90.0f, Fade(robot_color, 0.10f));
 DrawSphere(center, 0.095f, robot_color);
 if (state.waiting_for_clearance) {
  for (float left : {-0.15f, 0.15f}) {
   DrawSphere(pose_offset({state.robot.x, state.robot.y}, state.robot.theta, -0.31f, left, 0.25f), 0.04f,
              kAmber);
  }
 }
 if (const auto goal = mission.map_definition().markers.find(mission.level().objective.goal);
     goal != mission.map_definition().markers.end()) {
  const auto base = world(goal->second);
  const float pulse = 0.10f + 0.025f * std::sin(static_cast<float>(state.time_s * 4.0));
  DrawCircle3D({base.x, 0.02f, base.z}, 1.18f, {1, 0, 0}, 90.0f, Fade(kCyan, 0.09f));
  DrawSphere({base.x, 2.40f, base.z}, pulse, kCyan);
  DrawCylinderWiresEx({base.x, 2.30f, base.z}, {base.x, 2.36f, base.z}, 0.34f, 0.34f, 24, Fade(kCyan, 0.65f));
  if (state.status == game::MissionStatus::Succeeded) {
   for (int ring = 0; ring < 3; ++ring) {
    const float height = 0.42f + std::fmod(static_cast<float>(state.time_s * 0.72 + ring * 0.56), 1.74f);
    DrawCylinderWiresEx({base.x, height, base.z}, {base.x, height + 0.025f, base.z}, 0.38f, 0.38f, 24,
                        Fade(kCyan, 0.42f));
   }
  }
 }
 if (state.status == game::MissionStatus::Succeeded) {
  const float base_radius = static_cast<float>(scene.effects.success_ring_radius_m);
  for (int ring = 0; ring < 3; ++ring) {
   const float pulse = std::fmod(static_cast<float>(state.time_s * 0.7 + ring * 0.34), 1.0f);
   DrawCircle3D({center.x, 0.035f + pulse * 0.32f, center.z}, base_radius * (0.72f + pulse * 0.48f),
                {1, 0, 0}, 90.0f, Fade(active_color, 0.36f * (1.0f - pulse)));
  }
 }
 if (state.dynamic_object_detected) {
  for (const auto& obstacle : state.dynamic_obstacles) {
   const auto position = world(obstacle.estimated_position, 0.025f);
   const auto actor_center =
     world(obstacle.estimated_position, static_cast<float>(scene.effects.perception_link_height_m));
   const sim::Vec2 predicted{
     obstacle.estimated_position.x + obstacle.estimated_velocity.x * scene.effects.prediction_horizon_s,
     obstacle.estimated_position.y + obstacle.estimated_velocity.y * scene.effects.prediction_horizon_s,
   };
   const auto predicted_center = world(predicted, static_cast<float>(scene.effects.perception_link_height_m));
   const Color link_color = state.waiting_for_clearance ? waiting_color : active_color;
   DrawLine3D(center, actor_center, Fade(link_color, 0.34f));
   for (int marker = 0; marker < 4; ++marker) {
    const float progress = std::fmod(static_cast<float>(state.time_s * 0.72 + marker * 0.25), 1.0f);
    DrawSphere(Vector3Lerp(center, actor_center, progress), 0.025f, Fade(link_color, 0.72f));
   }
   DrawLine3D(actor_center, predicted_center, Fade(kDanger, 0.56f));
   for (int marker = 1; marker <= 4; ++marker) {
    DrawSphere(Vector3Lerp(actor_center, predicted_center, marker / 4.0f), 0.021f, Fade(kDanger, 0.70f));
   }
   DrawCircle3D({predicted_center.x, 0.028f, predicted_center.z},
                static_cast<float>(obstacle.radius_m + 0.12), {1, 0, 0}, 90.0f, Fade(kDanger, 0.34f));
   DrawCircle3D(position, static_cast<float>(obstacle.radius_m + 0.34), {1, 0, 0}, 90.0f,
                Fade(kDanger, 0.08f));
   DrawCylinderWires(position, static_cast<float>(obstacle.radius_m + 0.18),
                     static_cast<float>(obstacle.radius_m + 0.18), 0.035f, 28, Fade(kDanger, 0.72f));
  }
 }
}

} // namespace client
