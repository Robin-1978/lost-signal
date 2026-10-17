#include "scene_renderer_3d.hpp"

#include <raymath.h>
#include <rlgl.h>

#include <algorithm>
#include <array>
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

Vector3 world(const mission::Pose& pose, float height = 0.0f) {
 return {static_cast<float>(pose.x), height, static_cast<float>(-pose.y)};
}

Vector3 world(const mission::Vec3& point) {
 return {static_cast<float>(point.x), static_cast<float>(point.z), static_cast<float>(-point.y)};
}

Color color(const std::array<unsigned char, 4>& value) {
 return {value[0], value[1], value[2], value[3]};
}

Color mix(Color from, Color to, float amount) {
 const float value = std::clamp(amount, 0.0f, 1.0f);
 return {
   static_cast<unsigned char>(from.r + (to.r - from.r) * value),
   static_cast<unsigned char>(from.g + (to.g - from.g) * value),
   static_cast<unsigned char>(from.b + (to.b - from.b) * value),
   static_cast<unsigned char>(from.a + (to.a - from.a) * value),
 };
}

Vector3 shader_color(const std::array<unsigned char, 4>& value, double intensity = 1.0) {
 const float scale = static_cast<float>(intensity / 255.0);
 return {value[0] * scale, value[1] * scale, value[2] * scale};
}

mission::Vec3 rotate_offset(const mission::Vec3& value, double yaw_degrees) {
 const double radians = yaw_degrees / kRadToDeg;
 const double cosine = std::cos(radians);
 const double sine = std::sin(radians);
 return {
   value.x * cosine - value.y * sine,
   value.x * sine + value.y * cosine,
   value.z,
 };
}

Vector3 pose_offset(sim::Vec2 position, double heading, float forward, float left, float up,
                    float base_height = 0.0f) {
 const float cosine = std::cos(static_cast<float>(heading));
 const float sine = std::sin(static_cast<float>(heading));
 const float x = static_cast<float>(position.x) + cosine * forward - sine * left;
 const float y = static_cast<float>(position.y) + sine * forward + cosine * left;
 return {x, base_height + up, -y};
}

void draw_ground_ribbon(Vector3 from, Vector3 to, float width, Color tint) {
 const Vector3 direction = Vector3Subtract(to, from);
 const float planar_length = std::hypot(direction.x, direction.z);
 if (planar_length <= 1e-5f) {
  return;
 }
 const Vector3 side{direction.z * width / planar_length, 0.0f, -direction.x * width / planar_length};
 const Vector3 from_left = Vector3Add(from, side);
 const Vector3 from_right = Vector3Subtract(from, side);
 const Vector3 to_left = Vector3Add(to, side);
 const Vector3 to_right = Vector3Subtract(to, side);
 DrawTriangle3D(from_left, from_right, to_left, tint);
 DrawTriangle3D(from_right, to_right, to_left, tint);
}

void draw_ground_quad(Vector3 a, Vector3 b, Vector3 c, Vector3 d, Color tint) {
 DrawTriangle3D(a, b, c, tint);
 DrawTriangle3D(a, c, d, tint);
}

sim::TerrainSample terrain_at(const mission::PresentationFrameView& mission, sim::Vec2 position) {
 for (auto patch = mission.map_definition().terrain.rbegin();
      patch != mission.map_definition().terrain.rend(); ++patch) {
  if (!mission::active_in_level(patch->active_in, mission.level().id) || position.x < patch->min.x ||
      position.x >= patch->max.x || position.y < patch->min.y || position.y >= patch->max.y) {
   continue;
  }
  return {
    patch->base_height_m + patch->grade_x * (position.x - patch->min.x) +
      patch->grade_y * (position.y - patch->min.y),
    patch->grade_x,
    patch->grade_y,
    patch->roughness,
  };
 }
 return {};
}

const mission::SurfaceDefinition* surface_at(const mission::PresentationFrameView& mission,
                                             sim::Vec2 position) {
 for (auto surface = mission.map_definition().surfaces.rbegin();
      surface != mission.map_definition().surfaces.rend(); ++surface) {
  if (mission::active_in_level(surface->active_in, mission.level().id) && position.x >= surface->min.x &&
      position.x < surface->max.x && position.y >= surface->min.y && position.y < surface->max.y) {
   return &*surface;
  }
 }
 return nullptr;
}

struct GroundPose {
 Vector3 position;
 Vector3 axis{1.0f, 0.0f, 0.0f};
 float angle_degrees{0.0f};
};

GroundPose ground_pose(sim::Vec2 position, const sim::TerrainSample& terrain, float lift = 0.0f) {
 const Vector3 normal =
   Vector3Normalize({static_cast<float>(-terrain.grade_x), 1.0f, static_cast<float>(terrain.grade_y)});
 GroundPose pose;
 pose.position = world(position, static_cast<float>(terrain.height_m) + lift);
 pose.axis = Vector3CrossProduct({0.0f, 1.0f, 0.0f}, normal);
 const float axis_length = Vector3Length(pose.axis);
 if (axis_length > 1e-6f) {
  pose.axis = Vector3Scale(pose.axis, 1.0f / axis_length);
  pose.angle_degrees = std::acos(std::clamp(normal.y, -1.0f, 1.0f)) * kRadToDeg;
 }
 return pose;
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
                                 const mission::ContentCatalog& catalog)
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
 // Actor visuals come from the map actors' self-descriptions (content names
 // the model file); code never names one. Everything loads up front so a
 // missing file fails here at startup, not mid-run on first sighting.
 for (const auto& [map_id, map_value] : catalog_.maps()) {
  (void)map_id;
  for (const auto& actor : map_value.actors)
   load_model(actor.visual_model);
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

std::optional<sim::Vec2> SceneRenderer3D::screen_to_ground(Vector2 screen) const {
 if (!camera_initialized_ || input_viewport_.width <= 1.0f || input_viewport_.height <= 1.0f ||
     !CheckCollisionPointRec(screen, input_viewport_)) {
  return std::nullopt;
 }
 const Vector2 local{screen.x - input_viewport_.x, screen.y - input_viewport_.y};
 const Ray ray = GetScreenToWorldRayEx(local, camera_, static_cast<int>(input_viewport_.width),
                                       static_cast<int>(input_viewport_.height));
 if (std::abs(ray.direction.y) < 1e-6f) {
  return std::nullopt;
 }
 const float distance = -ray.position.y / ray.direction.y;
 if (distance < 0.0f) {
  return std::nullopt;
 }
 const Vector3 hit = Vector3Add(ray.position, Vector3Scale(ray.direction, distance));
 return sim::Vec2{hit.x, -hit.z};
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

void SceneRenderer3D::draw_prefab(const mission::ScenePrefabDefinition& prefab, const mission::Vec3& origin,
                                  const mission::Vec3& scale, double yaw_degrees) const {
 for (const auto& part : prefab.parts) {
  const mission::Vec3 scaled_offset{
    part.position_m.x * scale.x,
    part.position_m.y * scale.y,
    part.position_m.z * scale.z,
  };
  const auto offset = rotate_offset(scaled_offset, yaw_degrees);
  const mission::Vec3 position{
    origin.x + offset.x,
    origin.y + offset.y,
    origin.z + offset.z,
  };
  const mission::Vec3 size{
    part.size_m.x * scale.x,
    part.size_m.y * scale.y,
    part.size_m.z * scale.z,
  };
  const float yaw = static_cast<float>(yaw_degrees + part.yaw_degrees);
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
   const auto base = world(mission::Vec3{position.x, position.y, position.z - size.z * 0.5});
   const auto top = world(mission::Vec3{position.x, position.y, position.z + size.z * 0.5});
   const float planar_scale = static_cast<float>((scale.x + scale.y) * 0.5);
   DrawCylinderEx(base, top, static_cast<float>(part.size_m.x) * planar_scale,
                  static_cast<float>(part.size_m.y) * planar_scale, 8, tint);
  }
 }
}

void SceneRenderer3D::draw_attached_prefab(const mission::ScenePrefabDefinition& prefab, Vector3 body_origin,
                                           Quaternion body_rotation,
                                           const mission::Vec3& local_origin) const {
 for (const auto& part : prefab.parts) {
  const mission::Vec3 local{
    local_origin.x + part.position_m.x,
    local_origin.y + part.position_m.y,
    local_origin.z + part.position_m.z,
  };
  const Vector3 local_render{static_cast<float>(local.x), static_cast<float>(local.z),
                             static_cast<float>(-local.y)};
  const Vector3 position = Vector3Add(body_origin, Vector3RotateByQuaternion(local_render, body_rotation));
  const Quaternion part_rotation = QuaternionMultiply(
    body_rotation, QuaternionFromEuler(0.0f, static_cast<float>(part.yaw_degrees / kRadToDeg), 0.0f));
  Vector3 axis;
  float angle = 0.0f;
  QuaternionToAxisAngle(part_rotation, &axis, &angle);
  const Vector3 scale{static_cast<float>(part.size_m.x), static_cast<float>(part.size_m.z),
                      static_cast<float>(part.size_m.y)};
  const Color tint = color(part.color);
  if (!part.model_3d.empty()) {
   if (const auto* loaded = model(part.model_3d))
    DrawModelEx(*loaded, position, axis, angle * kRadToDeg, scale, tint);
  } else if (part.primitive == "box") {
   DrawModelEx(cube_model_, position, axis, angle * kRadToDeg, scale, tint);
  } else if (part.primitive == "sphere") {
   const float radius = static_cast<float>((part.size_m.x + part.size_m.y + part.size_m.z) / 3.0);
   DrawSphere(position, radius, tint);
  } else if (part.primitive == "cylinder") {
   const Vector3 half_axis =
     Vector3RotateByQuaternion({0.0f, static_cast<float>(part.size_m.z * 0.5), 0.0f}, body_rotation);
   const float radius = static_cast<float>((part.size_m.x + part.size_m.y) * 0.5);
   DrawCylinderEx(Vector3Subtract(position, half_axis), Vector3Add(position, half_axis), radius, radius, 8,
                  tint);
  }
 }
}

void SceneRenderer3D::draw(const mission::PresentationFrameView& mission, int screen_width, int screen_height,
                           Rectangle input_viewport, bool assembly_view) {
 input_viewport_ = input_viewport;
 const auto& state = mission.snapshot();
 const auto& scene = catalog_.scene(mission.map_definition().scene_3d);
 sim::Vec2 focus{state.robot.x, state.robot.y};
 const auto actor =
   std::find_if(state.actors.begin(), state.actors.end(), [](const auto& value) { return value.detected; });
 auto goal = mission.map_definition().markers.find(mission.level().objective.goal);
 if (const auto* active = mission.active_goal())
  goal = mission.map_definition().markers.find(active->marker);
 const bool success_shot =
   state.status == mission::MissionStatus::Succeeded && goal != mission.map_definition().markers.end();
 const bool event_shot = !success_shot && actor != state.actors.end() && state.dynamic_object_detected;
 const mission::SceneShotDefinition* shot =
   assembly_view
     ? nullptr
     : (success_shot ? &scene.camera.success_shot : (event_shot ? &scene.camera.event_shot : nullptr));
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
 } else if (!assembly_view && mission.endless_mode()) {
  const auto& field_operator = mission.field_operator();
  focus.x = state.robot.x * 0.52 + field_operator.position.x * 0.48;
  focus.y = state.robot.y * 0.52 + field_operator.position.y * 0.48;
 } else if (!assembly_view && actor != state.actors.end()) {
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
                      assembly_view
                        ? static_cast<float>(state.robot_height_m + 0.08)
                        : static_cast<float>(shot ? shot->target_height_m : scene.camera.target_height_m),
                      static_cast<float>(-focus.y)};
 const Vector3 offset =
   assembly_view ? Vector3{1.65f, 1.05f, 1.65f} : world(shot ? shot->offset_m : scene.camera.offset_m);
 const float fov =
   assembly_view ? 32.0f : static_cast<float>(shot ? shot->fov_degrees : scene.camera.fov_degrees);
 const bool reset_camera = !camera_initialized_ || camera_level_ != mission.level().id ||
                           camera_assembly_view_ != assembly_view || state.time_s + 1e-9 < camera_time_s_;
 if (reset_camera) {
  camera_focus_ = target;
  camera_offset_ = offset;
  camera_fov_ = fov;
  manual_camera_ = false;
  camera_pan_ = {};
  camera_initialized_ = true;
  camera_level_ = mission.level().id;
  camera_assembly_view_ = assembly_view;
 }

 const Vector2 mouse = GetMousePosition();
 const bool pointer_in_view = CheckCollisionPointRec(mouse, input_viewport);
 const float wheel = pointer_in_view ? GetMouseWheelMove() : 0.0f;
 const bool orbiting = pointer_in_view && IsMouseButtonDown(MOUSE_BUTTON_RIGHT);
 const bool panning = pointer_in_view && IsMouseButtonDown(MOUSE_BUTTON_MIDDLE);
 if (IsKeyPressed(KEY_C)) {
  manual_camera_ = false;
  camera_pan_ = {};
 }
 if (!manual_camera_ && (orbiting || panning || std::abs(wheel) > 1e-6f)) {
  manual_camera_ = true;
  camera_pan_ = Vector3Subtract(camera_focus_, target);
  camera_orbit_distance_ = std::max(0.001f, Vector3Length(camera_offset_));
  camera_orbit_yaw_ = std::atan2(camera_offset_.x, camera_offset_.z);
  camera_orbit_pitch_ = std::asin(std::clamp(camera_offset_.y / camera_orbit_distance_, -1.0f, 1.0f));
 }

 const double simulation_elapsed = std::max(0.0, state.time_s - camera_time_s_);
 const float elapsed = static_cast<float>(simulation_elapsed > 0.0 ? simulation_elapsed : 1.0 / 60.0);
 const float blend = 1.0f - std::exp(-elapsed * 4.6f / static_cast<float>(scene.camera.transition_s));
 if (manual_camera_) {
  const Vector2 mouse_delta = GetMouseDelta();
  if (orbiting) {
   camera_orbit_yaw_ -= mouse_delta.x * 0.006f;
   camera_orbit_pitch_ = std::clamp(camera_orbit_pitch_ - mouse_delta.y * 0.005f, 0.18f, 1.35f);
  }
  if (std::abs(wheel) > 1e-6f) {
   camera_orbit_distance_ = std::clamp(camera_orbit_distance_ * std::exp(-wheel * 0.13f), 2.6f, 28.0f);
  }
  if (panning) {
   const Vector3 horizontal_offset{std::sin(camera_orbit_yaw_), 0.0f, std::cos(camera_orbit_yaw_)};
   const Vector3 view_forward = Vector3Negate(horizontal_offset);
   const Vector3 view_right{view_forward.z, 0.0f, -view_forward.x};
   const float units_per_pixel = camera_orbit_distance_ * 0.0014f;
   camera_pan_ = Vector3Add(camera_pan_, Vector3Scale(view_right, -mouse_delta.x * units_per_pixel));
   camera_pan_ = Vector3Add(camera_pan_, Vector3Scale(view_forward, mouse_delta.y * units_per_pixel));
  }
  const float horizontal_distance = camera_orbit_distance_ * std::cos(camera_orbit_pitch_);
  camera_offset_ = {
    horizontal_distance * std::sin(camera_orbit_yaw_),
    camera_orbit_distance_ * std::sin(camera_orbit_pitch_),
    horizontal_distance * std::cos(camera_orbit_yaw_),
  };
  camera_focus_ = Vector3Lerp(camera_focus_, Vector3Add(target, camera_pan_), blend);
  camera_fov_ += (fov - camera_fov_) * blend;
 } else if (!reset_camera) {
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
  const bool scripted_environment = !mission.level().environment_keyframes.empty();
  const double daylight_level = scripted_environment
                                  ? state.environment.daylight
                                  : (mission.endless_mode() ? state.world_activity.daylight : 1.0);
  const double visibility = scripted_environment
                              ? state.environment.visibility
                              : (mission.endless_mode() ? state.world_activity.visibility : 1.0);
  const double daylight = 0.28 + 0.72 * daylight_level;
  const Vector3 light_color =
    shader_color(scene.lighting.directional_color, scene.lighting.directional_intensity * daylight);
  const Vector3 ambient = shader_color(scene.lighting.ambient_color, 0.52 + 0.48 * daylight);
  const Vector3 fog_color = shader_color(scene.lighting.fog_color);
  const float fog_start = static_cast<float>(scene.lighting.fog_start_m * (0.55 + 0.45 * visibility));
  const float fog_end = static_cast<float>(scene.lighting.fog_end_m * (0.40 + 0.60 * visibility));
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
  double signal_intensity = 1.0;
  if (mission.level().objective.goal_visual == "signal_tower") {
   const bool powered = state.status == mission::MissionStatus::Succeeded;
   signal_intensity = powered ? 1.15 + 0.18 * std::sin(state.time_s * 5.4) : 0.06;
  }
  const Vector3 signal_color = shader_color(scene.lighting.goal_color, signal_intensity);
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

void SceneRenderer3D::render_shadow_map(const mission::PresentationFrameView& mission,
                                        const mission::SceneDefinition& scene, Vector3 light_direction) {
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

void SceneRenderer3D::render_scene_target(const mission::PresentationFrameView& mission,
                                          const mission::SceneDefinition& scene) {
 if (shadow_map_.id > 0) {
  const int shadow_texture_slot = 10;
  rlActiveTextureSlot(10);
  rlEnableTexture(shadow_map_.depth.id);
  rlSetUniform(shadow_map_loc_, &shadow_texture_slot, SHADER_UNIFORM_INT, 1);
  rlActiveTextureSlot(0);
 }
 BeginTextureMode(scene_target_);
 Color background = color(scene.lighting.fog_color);
 if (mission.endless_mode()) {
  const auto& activity = mission.world_activity();
  const float night = static_cast<float>(1.0 - activity.daylight);
  const float haze = static_cast<float>(1.0 - activity.visibility);
  background = mix(background, {8, 16, 24, 255}, night * 0.72f);
  background = mix(background, {104, 116, 119, 255}, haze * 0.55f);
 }
 ClearBackground(background);
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

void SceneRenderer3D::composite_scene(const mission::SceneDefinition& scene) const {
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

void SceneRenderer3D::draw_scene(const mission::PresentationFrameView& mission) const {
 const auto& map = mission.map_definition();
 const auto& scene = catalog_.scene(map.scene_3d);
 draw_floor(map, scene);
 draw_terrain(mission);
 draw_environment(mission, scene);
 for (const auto& surface : map.surfaces) {
  if (!mission::active_in_level(surface.active_in, mission.level().id))
   continue;
  draw_surface(mission, surface);
 }
 for (const auto& obstacle : map.obstacles) {
  if (mission::active_in_level(obstacle.active_in, mission.level().id))
   draw_shelf(obstacle);
 }
 draw_markers(mission);
 (void)scene;
 for (const auto& actor : mission.snapshot().actors) {
  draw_actor(actor, mission.snapshot().waiting_for_clearance, mission.snapshot().time_s);
 }
 draw_field_operator(mission);
 draw_robot(mission);
 draw_lidar_point_cloud(mission);
}

void SceneRenderer3D::draw_lidar_point_cloud(
 const mission::PresentationFrameView& mission) const {
 static constexpr Color ring_colors[] = {
 {65, 220, 255, 210}, {77, 244, 190, 210},
 {255, 218, 92, 210}, {255, 116, 145, 210},
 };
 const auto& cloud = mission.snapshot().lidar_point_cloud;
 if (!cloud)
 return;
 for (const auto& point : *cloud) {
 const Color base = ring_colors[point.ring % 4];
 const float intensity = static_cast<float>(
 std::clamp(0.25 + point.intensity * 0.75, 0.0, 1.0));
 const Color color{
 static_cast<unsigned char>(base.r * intensity),
 static_cast<unsigned char>(base.g * intensity),
 static_cast<unsigned char>(base.b * intensity),
 base.a,
 };
 DrawPoint3D(world(point.position_m), color);
 }
}

void SceneRenderer3D::draw_floor(const mission::MapDefinition& map,
                                 const mission::SceneDefinition& scene) const {
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

void SceneRenderer3D::draw_terrain(const mission::PresentationFrameView& mission) const {
 for (const auto& patch : mission.map_definition().terrain) {
  if (!mission::active_in_level(patch.active_in, mission.level().id)) {
   continue;
  }
  const float width = static_cast<float>(patch.max.x - patch.min.x);
  const float length = static_cast<float>(patch.max.y - patch.min.y);
  const sim::Vec2 center{
    (patch.min.x + patch.max.x) * 0.5,
    (patch.min.y + patch.max.y) * 0.5,
  };
  const auto terrain = terrain_at(mission, center);
  const float grade = static_cast<float>(std::hypot(patch.grade_x, patch.grade_y));

  // Flat elevated patches are real high ground. Fill the volume down to the
  // base floor so they read as platforms instead of paper-thin planes hanging
  // in space. Sloped patches stay bridge-like and use piers below the deck.
  if (grade < 0.001f && terrain.height_m > 0.08) {
   const float support_height = static_cast<float>(terrain.height_m);
   draw_box({static_cast<float>(center.x), support_height * 0.5f - 0.03f, static_cast<float>(-center.y)},
            {width, support_height, length}, 0.0f, {43, 49, 46, 255});
  } else if (grade >= 0.001f) {
   for (float fraction : {0.34f, 0.70f}) {
    const bool road_runs_x = width >= length;
    const sim::Vec2 pier_position{
      road_runs_x ? patch.min.x + (patch.max.x - patch.min.x) * fraction : center.x,
      road_runs_x ? center.y : patch.min.y + (patch.max.y - patch.min.y) * fraction,
    };
    const auto pier_terrain = terrain_at(mission, pier_position);
    const float pier_height = static_cast<float>(pier_terrain.height_m);
    if (pier_height < 0.16f)
     continue;
    draw_box(
      {static_cast<float>(pier_position.x), pier_height * 0.5f - 0.02f, static_cast<float>(-pier_position.y)},
      road_runs_x ? Vector3{0.24f, pier_height, std::max(0.3f, length - 0.42f)}
                  : Vector3{std::max(0.3f, width - 0.42f), pier_height, 0.24f},
      0.0f, {48, 54, 51, 255});
   }
  }

  const Color tint = grade > 0.3f ? Color{83, 91, 80, 255} : Color{71, 80, 75, 255};
  // Use the authored corner heights directly. Rotating a width-sized box
  // foreshortens its horizontal span and creates a visible gap at a flat
  // highland even though the Bullet triangle meshes share an exact edge.
  const auto corner = [&](double x, double y) {
   const double height =
     patch.base_height_m + patch.grade_x * (x - patch.min.x) + patch.grade_y * (y - patch.min.y);
return world(sim::Vec2{x, y}, static_cast<float>(height) + 0.002f);
  };
  const Vector3 deck_a = corner(patch.min.x, patch.min.y);
  const Vector3 deck_b = corner(patch.max.x, patch.min.y);
  const Vector3 deck_c = corner(patch.max.x, patch.max.y);
  const Vector3 deck_d = corner(patch.min.x, patch.max.y);
  draw_ground_quad(deck_a, deck_b, deck_c, deck_d, tint);

  // Curbs, guard rails and posts expose the edge and height of the route from
  // the default three-quarter camera. They also give every terrain lane the
  // same visual grammar: amber means a drop or unstable shoulder.
  const bool road_runs_x = width >= length;
  const float inset = 0.10f;
  const float rail_lift = grade > 0.001f || terrain.height_m > 0.35 ? 0.52f : 0.12f;
  const auto draw_edge_beam = [&](sim::Vec2 edge_center, Vector3 beam_scale, float lift, Color tint_color) {
   const auto edge_pose = ground_pose(edge_center, terrain_at(mission, edge_center), lift);
   DrawModelEx(cube_model_, edge_pose.position, edge_pose.axis, edge_pose.angle_degrees, beam_scale,
               tint_color);
  };
  if (road_runs_x) {
   for (double y : {patch.min.y + inset, patch.max.y - inset}) {
    draw_edge_beam({center.x, y}, {width, 0.075f, 0.10f}, 0.075f, kAmber);
    if (rail_lift > 0.2f)
     draw_edge_beam({center.x, y}, {width, 0.055f, 0.055f}, rail_lift, {111, 119, 111, 255});
    for (double x = patch.min.x + 0.35; rail_lift > 0.2f && x < patch.max.x; x += 1.5) {
     const auto post_ground = terrain_at(mission, {x, y});
     draw_box({static_cast<float>(x), static_cast<float>(post_ground.height_m) + rail_lift * 0.5f,
               static_cast<float>(-y)},
              {0.055f, rail_lift, 0.055f}, 0.0f, {93, 102, 96, 255});
    }
   }
  } else {
   for (double x : {patch.min.x + inset, patch.max.x - inset}) {
    draw_edge_beam({x, center.y}, {0.10f, 0.075f, length}, 0.075f, kAmber);
    if (rail_lift > 0.2f)
     draw_edge_beam({x, center.y}, {0.055f, 0.055f, length}, rail_lift, {111, 119, 111, 255});
    for (double y = patch.min.y + 0.35; rail_lift > 0.2f && y < patch.max.y; y += 1.5) {
     const auto post_ground = terrain_at(mission, {x, y});
     draw_box({static_cast<float>(x), static_cast<float>(post_ground.height_m) + rail_lift * 0.5f,
               static_cast<float>(-y)},
              {0.055f, rail_lift, 0.055f}, 0.0f, {93, 102, 96, 255});
    }
   }
  }
 }
}

void SceneRenderer3D::draw_shelf(const mission::BoxDefinition& obstacle) const {
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

void SceneRenderer3D::draw_surface(const mission::PresentationFrameView& mission,
                                   const mission::SurfaceDefinition& surface) const {
 const float width = static_cast<float>(surface.max.x - surface.min.x);
 const float length = static_cast<float>(surface.max.y - surface.min.y);
 const float center_x = static_cast<float>((surface.min.x + surface.max.x) * 0.5);
 const float center_z = static_cast<float>(-(surface.min.y + surface.max.y) * 0.5);
 // Material albedo quad (1px = 1 texel per meter tile): the ground reads as
 // its physics (sand looks sandy) instead of generic teal boxes. Empty
 // texture falls back to the old patches (same as before).
 const auto& material = catalog_.surface_material(surface.material);
 const auto plane = surface_models_.find(material.texture_albedo);
 bool textured = false;
 if (!material.texture_albedo.empty() && plane != surface_models_.end()) {
  textured = true;
  const int tiles_x = std::max(1, static_cast<int>(std::ceil(width)));
  const int tiles_z = std::max(1, static_cast<int>(std::ceil(length)));
  const float cell_w = width / static_cast<float>(tiles_x);
  const float cell_l = length / static_cast<float>(tiles_z);
  for (int ix = 0; ix < tiles_x; ++ix) {
   for (int iz = 0; iz < tiles_z; ++iz) {
    const float cx = static_cast<float>(surface.min.x) + (static_cast<float>(ix) + 0.5f) * cell_w;
    const float cz = static_cast<float>(surface.min.y) + (static_cast<float>(iz) + 0.5f) * cell_l;
    const sim::Vec2 cell_center{cx, cz};
    const auto cell_pose = ground_pose(cell_center, terrain_at(mission, cell_center), 0.006f);
    DrawModelEx(plane->second, cell_pose.position, cell_pose.axis, cell_pose.angle_degrees,
                {cell_w, 1.0f, cell_l}, WHITE);
   }
  }
 }
 if (!textured) {
  for (int patch = 0; patch < 5; ++patch) {
   const float x = center_x + (hash01(patch, 4, 1) - 0.5f) * width * 0.62f;
   const float z = center_z + (hash01(patch, 5, 2) - 0.5f) * length * 0.62f;
   const sim::Vec2 patch_center{x, -z};
   const auto patch_pose = ground_pose(patch_center, terrain_at(mission, patch_center), 0.015f);
   const Vector3 patch_size{
     width * (0.20f + hash01(patch, 6, 3) * 0.19f),
     0.018f,
     length * (0.18f + hash01(patch, 7, 4) * 0.18f),
   };
   DrawModelEx(cube_model_, patch_pose.position, patch_pose.axis, patch_pose.angle_degrees, patch_size,
               {39, 104, 105, 150});
  }
 }

 // Water must read before the robot reaches it. Texture gives the body of
 // the puddle; deterministic animated ripples and glints make it visibly
 // alive without feeding any state back into the simulation.
 if (material.drive_effect == "spray") {
  const float time = static_cast<float>(mission.snapshot().time_s);
  const Color water = color(material.effect_color);
  for (int ripple = 0; ripple < 8; ++ripple) {
   const float x = static_cast<float>(surface.min.x) + (0.12f + hash01(ripple, 31) * 0.76f) * width;
   const float y = static_cast<float>(surface.min.y) + (0.12f + hash01(ripple, 47) * 0.76f) * length;
   const auto ground = terrain_at(mission, {x, y});
   const float phase = std::fmod(time * (0.24f + hash01(ripple, 59) * 0.12f) + hash01(ripple, 67), 1.0f);
   DrawCircle3D(world(sim::Vec2{x, y}, static_cast<float>(ground.height_m) + 0.022f), 0.06f + phase * 0.24f,
                {1, 0, 0}, 90.0f, Fade(water, 0.26f * (1.0f - phase)));
  }
  for (int glint = 0; glint < 5; ++glint) {
   const float x = static_cast<float>(surface.min.x) + (0.10f + hash01(glint, 73) * 0.80f) * width;
   const float y = static_cast<float>(surface.min.y) + (0.12f + hash01(glint, 79) * 0.76f) * length;
   const auto ground = terrain_at(mission, {x, y});
   const float pulse = 0.18f + 0.12f * std::sin(time * 1.7f + hash01(glint, 83) * 2.0f * PI);
   DrawLine3D(world(sim::Vec2{x - 0.18f, y}, static_cast<float>(ground.height_m) + 0.025f),
              world(sim::Vec2{x + 0.18f, y}, static_cast<float>(ground.height_m) + 0.025f),
              Fade(water, pulse));
  }
 }
}

void SceneRenderer3D::draw_markers(const mission::PresentationFrameView& mission) const {
 const auto& map = mission.map_definition();
 const auto& scene = catalog_.scene(map.scene_3d);
 // Supply markers are INFORMATION (drive here), not furniture: a solid crate
 // on the marker forces the robot to drive through box art on collection
 // (via radius 1.25, arrival 0.62 < crate half-width + body). Flat ring plus
 // floating cube plus light beam read as a collection zone and never
 // overlap the car body.
 const auto draw_supply = [&](const mission::Pose& pose) {
  const auto base = world(pose, static_cast<float>(terrain_at(mission, {pose.x, pose.y}).height_m));
  DrawCylinderEx({base.x, base.y + 0.01f, base.z}, {base.x, base.y + 0.05f, base.z}, 0.68f, 0.68f, 40,
                 Fade(kAmber, 0.30f));
  DrawCylinderEx({base.x, base.y + 0.055f, base.z}, {base.x, base.y + 0.075f, base.z}, 0.60f, 0.60f, 40,
                 kAmber);
  DrawCylinderEx({base.x, base.y + 0.02f, base.z}, {base.x, base.y + 1.15f, base.z}, 0.045f, 0.045f, 12,
                 Fade(kAmber, 0.30f));
  DrawModelEx(cube_model_, {base.x, base.y + 1.30f, base.z}, {0, 1, 0},
              static_cast<float>(mission.snapshot().time_s * 40.0), {0.16f, 0.16f, 0.16f}, kAmber);
 };
 const auto draw_power_cradle = [&](const mission::Pose& pose, bool module_present, bool active) {
  const auto base = world(pose, static_cast<float>(terrain_at(mission, {pose.x, pose.y}).height_m));
  const Color accent = active ? kAmber : (module_present ? Color{128, 105, 61, 255} : kCyan);
  DrawCylinderEx({base.x, base.y + 0.02f, base.z}, {base.x, base.y + 0.12f, base.z}, 0.52f, 0.44f, 20,
                 {45, 52, 50, 255});
  DrawCylinderWiresEx({base.x, base.y + 0.12f, base.z}, {base.x, base.y + 0.16f, base.z}, 0.45f, 0.45f, 24,
                      Fade(accent, 0.88f));
  for (int arm = 0; arm < 3; ++arm) {
   const float angle = static_cast<float>(arm) * 2.0f * PI / 3.0f;
   const Vector3 foot{base.x + std::cos(angle) * 0.34f, base.y + 0.14f, base.z + std::sin(angle) * 0.34f};
   const Vector3 shoulder{base.x + std::cos(angle) * 0.22f, base.y + 0.48f, base.z + std::sin(angle) * 0.22f};
   DrawCylinderEx(foot, shoulder, 0.035f, 0.025f, 8, {91, 98, 92, 255});
  }
  if (module_present) {
   const float hover =
     active ? 0.52f + 0.025f * std::sin(static_cast<float>(mission.snapshot().time_s * 4.0)) : 0.48f;
   DrawModelEx(cube_model_, {base.x, base.y + hover, base.z}, {0, 1, 0},
               active ? static_cast<float>(mission.snapshot().time_s * 34.0) : 45.0f, {0.20f, 0.20f, 0.20f},
               accent);
  } else {
   DrawSphere({base.x, base.y + 0.18f, base.z}, 0.045f, Fade(kCyan, 0.72f));
  }
  if (active)
   draw_supply(pose);
 };
 const auto draw_observation_point = [&](const mission::Pose& pose, bool active, bool complete) {
  const auto base = world(pose, static_cast<float>(terrain_at(mission, {pose.x, pose.y}).height_m));
  const Color accent = active ? kAmber : (complete ? kCyan : Color{90, 98, 94, 255});
  for (float side : {-0.42f, 0.42f}) {
   DrawCylinderEx({base.x + side, base.y + 0.02f, base.z}, {base.x + side, base.y + 0.52f, base.z}, 0.045f,
                  0.035f, 10, {66, 72, 69, 255});
   DrawSphere({base.x + side, base.y + 0.56f, base.z}, 0.07f, accent);
  }
  DrawLine3D({base.x - 0.34f, base.y + 0.035f, base.z - 0.34f}, {base.x, base.y + 0.035f, base.z},
             Fade(accent, 0.86f));
  DrawLine3D({base.x, base.y + 0.035f, base.z}, {base.x - 0.34f, base.y + 0.035f, base.z + 0.34f},
             Fade(accent, 0.86f));
  if (active) {
   const float pulse = 0.62f + 0.08f * std::sin(static_cast<float>(mission.snapshot().time_s * 5.0));
   DrawCylinderWiresEx({base.x, base.y + 0.02f, base.z}, {base.x, base.y + 0.05f, base.z}, pulse, pulse, 36,
                       Fade(kAmber, 0.72f));
  }
 };
 const auto draw_signal_tower = [&](const mission::Pose& pose, bool active, bool powered) {
  const auto base = world(pose, static_cast<float>(terrain_at(mission, {pose.x, pose.y}).height_m));
  const float pulse =
    powered ? 0.72f + 0.28f * std::sin(static_cast<float>(mission.snapshot().time_s * 5.4)) : 0.18f;
  const Color signal = powered ? kCyan : Color{61, 70, 67, 255};
  DrawCylinderEx({base.x, base.y + 0.02f, base.z}, {base.x, base.y + 0.24f, base.z}, 0.52f, 0.46f, 20,
                 {54, 60, 58, 255});
  DrawCylinderEx({base.x, base.y + 0.22f, base.z}, {base.x, base.y + 2.70f, base.z}, 0.13f, 0.07f, 16,
                 {92, 101, 96, 255});
  for (float height : {1.25f, 1.85f, 2.42f}) {
   DrawCylinderEx({base.x - 0.34f, base.y + height, base.z}, {base.x + 0.34f, base.y + height, base.z},
                  0.025f, 0.025f, 8, {112, 120, 113, 255});
   DrawSphere({base.x, base.y + height, base.z}, 0.055f, Fade(signal, pulse));
  }
  DrawSphere({base.x, base.y + 2.78f, base.z}, powered ? 0.11f + 0.025f * pulse : 0.075f,
             Fade(signal, powered ? pulse : 0.72f));
  const float dock = static_cast<float>(mission.level().objective.arrive_tolerance_m);
  DrawCylinderWiresEx({base.x, base.y + 0.015f, base.z}, {base.x, base.y + 0.045f, base.z}, dock, dock, 40,
                      Fade(active ? kAmber : signal, 0.74f));
  if (powered) {
   for (int ring = 0; ring < 3; ++ring) {
    const float phase = std::fmod(static_cast<float>(mission.snapshot().time_s * 0.55) + ring / 3.0f, 1.0f);
    DrawCircle3D({base.x, base.y + 2.78f, base.z}, 0.18f + phase * 0.92f, {1, 0, 0}, 90.0f,
                 Fade(kCyan, 0.38f * (1.0f - phase)));
   }
  }
 };
 if (mission.endless_mode()) {
  const auto& incident = mission.world_activity().incident;
  if (incident.active) {
   const auto base =
     world(incident.position, static_cast<float>(terrain_at(mission, incident.position).height_m));
   const float pulse = 0.5f + 0.5f * std::sin(static_cast<float>(mission.snapshot().time_s * 3.2));
   for (int beacon = -1; beacon <= 1; ++beacon) {
    const float offset = static_cast<float>(beacon) * 0.48f;
    DrawCylinderEx({base.x + offset, base.y + 0.02f, base.z - 0.52f},
                   {base.x + offset, base.y + 0.31f, base.z - 0.52f}, 0.028f, 0.028f, 8, {73, 79, 75, 255});
    DrawSphere({base.x + offset, base.y + 0.34f, base.z - 0.52f}, 0.045f + 0.008f * pulse, kDanger);
   }
   if (incident.kind == mission::WorldIncidentKind::RoadClosure) {
    draw_box({base.x, base.y + 0.32f, base.z}, {1.35f, 0.42f, 0.24f}, 12.0f, {207, 90, 55, 255});
    draw_box({base.x, base.y + 0.55f, base.z}, {1.05f, 0.09f, 0.12f}, 12.0f, kAmber);
   } else if (incident.kind == mission::WorldIncidentKind::FallenCargo) {
    draw_box({base.x - 0.22f, base.y + 0.20f, base.z}, {0.48f, 0.40f, 0.50f}, -18.0f, {164, 116, 64, 255});
    draw_box({base.x + 0.25f, base.y + 0.15f, base.z + 0.16f}, {0.40f, 0.30f, 0.38f}, 24.0f,
             {137, 92, 52, 255});
   } else {
    draw_box({base.x, base.y + 0.34f, base.z}, {0.54f, 0.68f, 0.46f}, 0.0f, {56, 69, 71, 255});
    DrawSphere({base.x, base.y + 0.64f, base.z - 0.24f}, 0.075f, kDanger);
   }
  }
  const auto& intent = mission.field_intent();
  if (intent.active && intent.kind != mission::FieldIntentKind::Follow &&
      intent.kind != mission::FieldIntentKind::Wait) {
   const auto base = world(intent.target, static_cast<float>(terrain_at(mission, intent.target).height_m));
   const float pulse = 0.5f + 0.5f * std::sin(static_cast<float>(mission.snapshot().time_s * 4.8));
   const Color intent_color = intent.acknowledgement == mission::FieldIntentAcknowledgement::Rejected
                                ? kDanger
                                : (intent.kind == mission::FieldIntentKind::Search ? kAmber : kCyan);
   DrawCylinderWiresEx({base.x, base.y + 0.035f, base.z}, {base.x, base.y + 0.075f, base.z},
                       0.74f + pulse * 0.08f, 0.74f + pulse * 0.08f, 48, intent_color);
   DrawCylinderEx({base.x, base.y + 0.02f, base.z}, {base.x, base.y + 0.045f, base.z}, 0.58f, 0.58f, 40,
                  Fade(intent_color, 0.30f));
   if (intent.kind == mission::FieldIntentKind::Search) {
    DrawCylinderWiresEx({base.x, base.y + 0.025f, base.z}, {base.x, base.y + 0.055f, base.z},
                        1.10f + pulse * 0.24f, 1.10f + pulse * 0.24f, 48, Fade(intent_color, 0.55f));
   }
  }
  const auto& dispatch = mission.field_dispatch();
  if (!dispatch.active)
   return;
  const mission::Pose target{dispatch.target.x, dispatch.target.y, 0.0};
  const auto base = world(target, static_cast<float>(terrain_at(mission, dispatch.target).height_m));
  const float pulse = 0.5f + 0.5f * std::sin(static_cast<float>(mission.snapshot().time_s * 4.0));
  DrawCylinderEx({base.x, base.y + 0.01f, base.z}, {base.x, base.y + 0.055f, base.z}, 0.72f + 0.06f * pulse,
                 0.72f + 0.06f * pulse, 40, Fade(kAmber, 0.42f));
  DrawCylinderWiresEx({base.x, base.y + 0.06f, base.z}, {base.x, base.y + 0.08f, base.z}, 0.68f, 0.68f, 40,
                      kAmber);
  DrawCylinderEx({base.x, base.y + 0.04f, base.z}, {base.x, base.y + 1.65f, base.z}, 0.035f, 0.035f, 10,
                 Fade(kAmber, 0.28f));
  if (dispatch.kind == "delivery") {
   DrawModelEx(cube_model_, {base.x, base.y + 1.42f, base.z}, {0, 1, 0},
               static_cast<float>(mission.snapshot().time_s * 34.0), {0.18f, 0.18f, 0.18f}, kAmber);
  } else {
   DrawSphere({base.x, base.y + 1.42f, base.z}, 0.16f + 0.025f * pulse, kAmber);
   DrawSphereWires({base.x, base.y + 1.42f, base.z}, 0.25f + 0.04f * pulse, 8, 12, Fade(kAmber, 0.65f));
  }
  return;
 }
 const auto* active = mission.active_goal();
 const std::size_t active_index = mission.snapshot().active_goal_index;
 for (std::size_t index = 0; index < mission.level().objective.goals.size(); ++index) {
  const auto& step = mission.level().objective.goals[index];
  const auto marker = map.markers.find(step.marker);
  if (marker == map.markers.end())
   continue;
  const bool is_active = index == active_index;
  const std::string step_id = step.id.empty() ? step.marker : step.id;
  const bool complete =
    std::find(mission.snapshot().completed_goal_ids.begin(), mission.snapshot().completed_goal_ids.end(),
              step_id) != mission.snapshot().completed_goal_ids.end();
  const bool abandoned =
    std::find(mission.snapshot().abandoned_goal_ids.begin(), mission.snapshot().abandoned_goal_ids.end(),
              step_id) != mission.snapshot().abandoned_goal_ids.end();
  if (!step.visual.empty()) {
   const auto visual = scene.goal_visuals.find(step.visual);
   if (visual != scene.goal_visuals.end()) {
    const std::string state =
      abandoned ? "abandoned" : (complete ? "completed" : (is_active ? "active" : "available"));
    const auto state_prefab = visual->second.state_prefabs.find(state);
    if (state_prefab != visual->second.state_prefabs.end()) {
     const auto prefab = scene.prefabs.find(state_prefab->second);
     if (prefab != scene.prefabs.end()) {
      const auto terrain = terrain_at(mission, {marker->second.x, marker->second.y});
      draw_prefab(prefab->second, {marker->second.x, marker->second.y, terrain.height_m}, {1.0, 1.0, 1.0},
                  marker->second.theta * kRadToDeg);
     }
    }
    if (is_active) {
     const auto base =
       world(marker->second,
             static_cast<float>(terrain_at(mission, {marker->second.x, marker->second.y}).height_m));
     const float pulse = 0.5f + 0.5f * std::sin(static_cast<float>(mission.snapshot().time_s * 4.2));
     DrawCylinderWiresEx({base.x, base.y + 0.035f, base.z}, {base.x, base.y + 0.075f, base.z},
                         0.68f + 0.06f * pulse, 0.68f + 0.06f * pulse, 36,
                         Fade(kAmber, 0.72f + 0.20f * pulse));
    }
    continue;
   }
  }
  if (step.action == "pickup")
   draw_power_cradle(marker->second, !complete, is_active);
  else if (step.action == "inspect")
   draw_observation_point(marker->second, is_active, complete);
  else if (step.action == "complete" && mission.level().objective.goal_visual == "signal_tower")
   draw_signal_tower(marker->second, is_active,
                     complete || mission.snapshot().status == mission::MissionStatus::Succeeded);
 }
 if (!active)
  return;
 if (!active->visual.empty())
  return;
 if (const auto goal = map.markers.find(active->marker); goal != map.markers.end()) {
  if (active->action == "pickup") {
   return;
  }
  const auto base =
    world(goal->second, static_cast<float>(terrain_at(mission, {goal->second.x, goal->second.y}).height_m));
  if (active->action == "complete" && mission.level().objective.goal_visual == "supply_station") {
   draw_supply(goal->second);
  } else if (active->action == "complete" && mission.level().objective.goal_visual == "signal_tower") {
   return;
  } else {
   const Color marker =
     active->action != "complete" || mission.level().objective.goal_visual == "stop_zone" ? kAmber : kCyan;
   DrawCylinderEx({base.x, base.y + 0.01f, base.z}, {base.x, base.y + 0.055f, base.z}, 0.58f, 0.58f, 24,
                  Fade(marker, 0.55f));
   DrawCylinderWiresEx({base.x, base.y + 0.06f, base.z}, {base.x, base.y + 0.075f, base.z}, 0.58f, 0.58f, 24,
                       marker);
   if (active->action == "complete" && mission.level().objective.goal_visual == "calibration_pad") {
    draw_box({base.x, base.y + 0.12f, base.z}, {0.52f, 0.12f, 0.52f}, 45.0f, {63, 103, 96, 255});
   }
  }
 }
}

void SceneRenderer3D::draw_field_operator(const mission::PresentationFrameView& mission) const {
 if (!mission.endless_mode())
  return;
 const auto& field_operator = mission.field_operator();
 const auto terrain = terrain_at(mission, field_operator.position);
 const float ground = static_cast<float>(terrain.height_m);
 const float heading = static_cast<float>(field_operator.heading_rad);
 const float stride =
   field_operator.moving ? 0.10f * std::sin(static_cast<float>(mission.snapshot().time_s * 11.0)) : 0.0f;
 const Color jacket{222, 146, 58, 255};
 const Color trousers{43, 58, 61, 255};
 const Color helmet{236, 196, 91, 255};

 const auto left_hip = pose_offset(field_operator.position, heading, 0.0f, 0.10f, 0.53f, ground);
 const auto right_hip = pose_offset(field_operator.position, heading, 0.0f, -0.10f, 0.53f, ground);
 const auto left_boot = pose_offset(field_operator.position, heading, stride, 0.10f, 0.08f, ground);
 const auto right_boot = pose_offset(field_operator.position, heading, -stride, -0.10f, 0.08f, ground);
 DrawCylinderEx(left_boot, left_hip, 0.065f, 0.085f, 10, trousers);
 DrawCylinderEx(right_boot, right_hip, 0.065f, 0.085f, 10, trousers);
 DrawCylinderEx(pose_offset(field_operator.position, heading, 0.0f, 0.0f, 0.48f, ground),
                pose_offset(field_operator.position, heading, 0.0f, 0.0f, 0.94f, ground), 0.20f, 0.16f, 14,
                jacket);
 DrawSphere(pose_offset(field_operator.position, heading, 0.015f, 0.0f, 1.10f, ground), 0.145f,
            {216, 174, 139, 255});
 DrawCylinderEx(pose_offset(field_operator.position, heading, 0.015f, 0.0f, 1.19f, ground),
                pose_offset(field_operator.position, heading, 0.015f, 0.0f, 1.25f, ground), 0.18f, 0.13f, 16,
                helmet);
 draw_box(pose_offset(field_operator.position, heading, -0.16f, 0.0f, 0.77f, ground), {0.18f, 0.34f, 0.30f},
          -heading * kRadToDeg, {47, 77, 74, 255});

 const auto ground_center = world(field_operator.position, ground + 0.025f);
 DrawCylinderWiresEx(ground_center, {ground_center.x, ground_center.y + 0.02f, ground_center.z}, 0.34f, 0.34f,
                     28, Fade(kAmber, 0.72f));

 const auto& intent = mission.field_intent();
 if (intent.active && intent.kind == mission::FieldIntentKind::Follow) {
  const auto robot = world(sim::Vec2{mission.snapshot().robot.x, mission.snapshot().robot.y},
                           static_cast<float>(mission.snapshot().robot_height_m * 0.32));
  const auto person = pose_offset(field_operator.position, heading, 0.0f, 0.0f, 0.20f, ground);
  for (int segment = 0; segment < 12; segment += 2) {
   const float from_t = static_cast<float>(segment) / 12.0f;
   const float to_t = static_cast<float>(segment + 1) / 12.0f;
   DrawLine3D(Vector3Lerp(robot, person, from_t), Vector3Lerp(robot, person, to_t), Fade(kCyan, 0.72f));
  }
 }
}

void SceneRenderer3D::draw_robot(const mission::PresentationFrameView& mission) const {
 const auto& state = mission.snapshot();
 const auto& scene = catalog_.scene(mission.map_definition().scene_3d);
 const auto& robot = mission.robot();
 const float body_height = static_cast<float>(state.robot_height_m);
 const Quaternion body_rotation =
   QuaternionFromEuler(static_cast<float>(state.robot_roll_rad), static_cast<float>(state.robot.theta),
                       static_cast<float>(-state.robot_pitch_rad));
 const auto find_mount = [&](const std::string& id) -> const mission::MountDefinition* {
  const auto found =
    std::find_if(robot.mounts.begin(), robot.mounts.end(), [&](const auto& mount) { return mount.id == id; });
  return found == robot.mounts.end() ? nullptr : &*found;
 };
 const auto component_id = [&](const std::string& slot) -> const std::string& {
  return mission.build().part(slot);
 };
 const auto mount_in_slot = [&](const mission::MountDefinition& mount, const std::string& slot_id) {
  const auto slot = std::find_if(robot.slots.begin(), robot.slots.end(),
                                 [&](const auto& candidate) { return candidate.slot == slot_id; });
  return slot != robot.slots.end() &&
         std::find(slot->mounts.begin(), slot->mounts.end(), mount.id) != slot->mounts.end();
 };
 const auto draw_part = [&](const mission::VisualDefinition& visual, const mission::MountDefinition& mount) {
  const Model* loaded = model(visual.model_3d);
  if (!loaded)
   return;
  const Vector3 local_position{static_cast<float>(mount.position_m.x), static_cast<float>(mount.position_m.z),
                               static_cast<float>(-mount.position_m.y)};
  auto position =
    Vector3Add({static_cast<float>(state.robot.x), body_height, static_cast<float>(-state.robot.y)},
               Vector3RotateByQuaternion(local_position, body_rotation));
  // Ready / garage / chapter-preview snapshots do not have a stepped
  // actuator frame yet. Their pose still comes from the authored mount TF;
  // omitting it was why all wheel meshes appeared as horizontal pucks in the
  // chapter selector even after live Bullet rendering used the right links.
  constexpr double kDegreesToRadians = sim::kPi / 180.0;
  const auto authored_mount_sim = sim::physics::quaternion_from_euler(
    mount.rotation_deg.x * kDegreesToRadians, mount.rotation_deg.y * kDegreesToRadians,
    mount.rotation_deg.z * kDegreesToRadians);
  const Quaternion authored_mount{
    static_cast<float>(authored_mount_sim.x),
    static_cast<float>(authored_mount_sim.z),
    static_cast<float>(-authored_mount_sim.y),
    static_cast<float>(authored_mount_sim.w),
  };
  Quaternion part_rotation = QuaternionMultiply(body_rotation, authored_mount);
  const auto actuation =
    std::find_if(state.control.mount_actuations.begin(), state.control.mount_actuations.end(),
                 [&](const auto& candidate) { return candidate.mount_id == mount.id; });
  if (actuation != state.control.mount_actuations.end()) {
   if (actuation->base_from_link) {
    const auto& transform = *actuation->base_from_link;
    const Vector3 physical_position{static_cast<float>(transform.translation_m.x),
                                    static_cast<float>(transform.translation_m.z),
                                    static_cast<float>(-transform.translation_m.y)};
    position =
      Vector3Add({static_cast<float>(state.robot.x), body_height, static_cast<float>(-state.robot.y)},
                 Vector3RotateByQuaternion(physical_position, body_rotation));
    // Simulator coordinates are x-forward/y-left/z-up; raylib uses
    // x-forward/y-up/z-back. Map both the rotation axis and translation.
    const Quaternion physical_rotation{
      static_cast<float>(transform.rotation.x),
      static_cast<float>(transform.rotation.z),
      static_cast<float>(-transform.rotation.y),
      static_cast<float>(transform.rotation.w),
    };
    part_rotation = QuaternionMultiply(body_rotation, physical_rotation);
   } else {
    // A visual is authored in its link frame. Reproduce the same
    // parent-yaw * authored-mount * local-joint-roll chain used by TF and by
    // the articulated Bullet model. Previously the wheel GLBs baked in the
    // mount's 90-degree rotation while physical rendering applied the link
    // transform again, laying every wheel flat.
    auto local_sim = sim::physics::quaternion_multiply(
      sim::physics::quaternion_from_euler(0.0, 0.0, actuation->yaw_rad),
      sim::physics::quaternion_from_euler(mount.rotation_deg.x * kDegreesToRadians,
                                          mount.rotation_deg.y * kDegreesToRadians,
                                          mount.rotation_deg.z * kDegreesToRadians));
    if (mount.motion == "wheel") {
     local_sim = sim::physics::quaternion_multiply(
       local_sim, sim::physics::quaternion_from_euler(0.0, 0.0, actuation->roll_rad));
    }
    const Quaternion local_actuation{
      static_cast<float>(local_sim.x),
      static_cast<float>(local_sim.z),
      static_cast<float>(-local_sim.y),
      static_cast<float>(local_sim.w),
    };
    part_rotation = QuaternionMultiply(body_rotation, local_actuation);
   }
  }
  if (mount_in_slot(mount, "sensor")) {
   if (!mission.build().part("sensor").empty()) {
    const double scan_hz =
      mission::property(catalog_.component(mission.build().part("sensor")).behavior, "scan_hz");
    const float scan_angle = static_cast<float>(std::fmod(state.time_s * scan_hz * 2.0 * PI, 2.0 * PI));
    part_rotation = QuaternionMultiply(body_rotation, QuaternionFromEuler(0.0f, scan_angle, 0.0f));
   }
  }
  Vector3 rotation_axis;
  float rotation_angle = 0.0f;
  QuaternionToAxisAngle(part_rotation, &rotation_axis, &rotation_angle);
  DrawModelEx(*loaded, position, rotation_axis, rotation_angle * kRadToDeg,
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

 // Attached cargo is scene-authored and derives only from authoritative
 // PayloadState. The renderer knows neither the item theme nor a level id.
 const auto& payload = mission.payload_state();
 const auto& cargo = scene.payload_visual;
 if (!payload.history.empty() && !cargo.mount.empty()) {
  const auto* mount = find_mount(cargo.mount);
  const auto container = scene.prefabs.find(cargo.container_prefab);
  const auto item = scene.prefabs.find(
    payload.integrity < cargo.damaged_below_integrity ? cargo.damaged_item_prefab : cargo.item_prefab);
  if (mount && container != scene.prefabs.end() && item != scene.prefabs.end()) {
   const Vector3 body_origin{static_cast<float>(state.robot.x), body_height,
                             static_cast<float>(-state.robot.y)};
   draw_attached_prefab(container->second, body_origin, body_rotation, mount->position_m);
   const int count = std::clamp((payload.item_count + cargo.items_per_visual - 1) / cargo.items_per_visual, 0,
                                cargo.max_visible_items);
   const int per_layer = cargo.grid_columns * cargo.grid_rows;
   for (int index = 0; index < count; ++index) {
    const int column = index % cargo.grid_columns;
    const int row = (index / cargo.grid_columns) % cargo.grid_rows;
    const int layer = index / per_layer;
    const mission::Vec3 local{
      mount->position_m.x + cargo.item_origin_m.x + column * cargo.item_spacing_m.x,
      mount->position_m.y + cargo.item_origin_m.y + row * cargo.item_spacing_m.y,
      mount->position_m.z + cargo.item_origin_m.z + layer * cargo.item_spacing_m.z,
    };
    draw_attached_prefab(item->second, body_origin, body_rotation, local);
   }
  }
 }
}

// Pedestrian stand-in: two stacked boxes (torso + head), teal so nobody reads
// it as a vehicle. Proportions scale with the actor's own radius; a proper
// character model replaces this, never new physics.
void SceneRenderer3D::draw_actor(const mission::ActorSnapshot& actor, bool waiting, double time_s) const {
 // One function for every agent: pose the model its registry entry names,
 // scaled linearly from the reference radius it was authored for (content
 // data). The waiting beacon stays procedural (it blinks; models don't) and
 // marks yielding-to-traffic for any agent kind.
 const sim::Vec2 position = actor.position;
 const float s =
   static_cast<float>(actor.radius_m) / std::max(1e-6f, static_cast<float>(actor.visual_ref_radius_m));
 const float yaw = static_cast<float>(actor.heading_rad) * kRadToDeg;
 if (const Model* body = model(actor.visual_model)) {
  DrawModelEx(*body, pose_offset(position, yaw / kRadToDeg, 0, 0, 0), {0, 1, 0}, yaw, {s, s, s}, WHITE);
 }
 const bool beacon_active =
   actor.visual_kind == "vehicle" && std::hypot(actor.velocity.x, actor.velocity.y) > 0.05;
 if (beacon_active) {
  const auto beacon = pose_offset(position, yaw / kRadToDeg, -0.18f * s, 0, 1.0f * s);
  const float pulse =
    0.72f +
    0.28f * std::sin(static_cast<float>(actor.position.x * 0.7 + actor.position.y * 0.9 + time_s * 8.0));
  const Color beacon_color = waiting ? kDanger : kAmber;
  DrawSphere(beacon, (0.065f + 0.018f * pulse) * s, Fade(beacon_color, pulse));
  if (actor.detected)
   DrawSphereWires(beacon, 0.13f * s, 6, 10, Fade(beacon_color, 0.42f * pulse));
 }
}

void SceneRenderer3D::draw_environment(const mission::PresentationFrameView& mission,
                                       const mission::SceneDefinition& scene) const {
 for (const auto& instance : scene.instances) {
  if (!mission::active_in_level(instance.active_in, mission.level().id))
   continue;
  const auto prefab = scene.prefabs.find(instance.prefab);
  if (prefab == scene.prefabs.end())
   continue;
  draw_prefab(prefab->second, instance.position_m, instance.scale, instance.yaw_degrees);
 }
 const double environment_wetness = !mission.level().environment_keyframes.empty()
                                      ? mission.snapshot().environment.wetness
                                      : (mission.endless_mode() ? mission.world_activity().wetness : 0.0);
 if (environment_wetness > 0.04) {
  // Wetness is shown as sparse, ground-hugging reflected glints. There are no
  // falling rain streaks: water reads from the road surface and puddle ripples.
  const float wetness = static_cast<float>(environment_wetness);
  const float time = static_cast<float>(mission.snapshot().time_s);
  const auto& map = mission.map_definition();
  for (int glint = 0; glint < 22; ++glint) {
   const sim::Vec2 p{
     0.4 + hash01(glint, 149) * std::max(0.1, map.size_m.x - 0.8),
     0.4 + hash01(glint, 157) * std::max(0.1, map.size_m.y - 0.8),
   };
   const auto ground = terrain_at(mission, p);
   const float length = 0.10f + 0.22f * hash01(glint, 163);
   const float pulse = 0.5f + 0.5f * std::sin(time * 0.8f + hash01(glint, 167) * 6.28318f);
   const Color reflected{
     151, 198, 205,
     static_cast<unsigned char>(std::clamp(12.0f + wetness * (20.0f + 25.0f * pulse), 0.0f, 255.0f))};
   DrawLine3D(world(sim::Vec2{p.x - length, p.y}, static_cast<float>(ground.height_m) + 0.018f),
              world(sim::Vec2{p.x + length, p.y}, static_cast<float>(ground.height_m) + 0.018f), reflected);
  }
 }
}

void SceneRenderer3D::draw_motion_effects(const mission::PresentationFrameView& mission) const {
 const auto& state = mission.snapshot();
 const auto& scene = catalog_.scene(mission.map_definition().scene_3d);
 const double oldest_time = state.time_s - scene.effects.trail_duration_s;
 const float half_track = static_cast<float>(mission.robot().wheel_base_m * 0.42);
 const mission::MissionSnapshot* previous = nullptr;
 std::vector<const mission::MissionSnapshot*> particle_frames;

 for (const auto& frame : mission.replay()) {
  if (frame.time_s + 1e-9 < oldest_time || frame.time_s > state.time_s + 1e-9) {
   continue;
  }
  const float age = static_cast<float>(state.time_s - frame.time_s);
  const float life = 1.0f - std::clamp(age / static_cast<float>(scene.effects.trail_duration_s), 0.0f, 1.0f);
  const float ground =
    static_cast<float>(terrain_at(mission, {frame.robot.x, frame.robot.y}).height_m + 0.018);
  if (previous && std::abs(frame.robot.v) > 0.04) {
   const float previous_ground =
     static_cast<float>(terrain_at(mission, {previous->robot.x, previous->robot.y}).height_m + 0.018);
   for (float left : {-half_track, half_track}) {
    const auto from = pose_offset({previous->robot.x, previous->robot.y}, previous->robot.theta, -0.18f, left,
                                  0.0f, previous_ground);
    const auto to =
      pose_offset({frame.robot.x, frame.robot.y}, frame.robot.theta, -0.18f, left, 0.0f, ground);
    const Color mark = frame.traction_saturated ? Color{42, 35, 28, 255} : Color{35, 39, 37, 255};
    const float width = frame.traction_saturated ? 0.065f : 0.042f;
    draw_ground_ribbon(from, to, width, Fade(mark, (frame.traction_saturated ? 0.76f : 0.34f) * life));
   }
  }
  if (std::abs(frame.robot.v) > 0.08 &&
      (frame.traction_saturated || surface_at(mission, {frame.robot.x, frame.robot.y}) != nullptr)) {
   particle_frames.push_back(&frame);
  }
  previous = &frame;
 }

 const int budget = scene.effects.surface_particle_count;
 if (budget <= 0 || particle_frames.empty()) {
  return;
 }
 const std::size_t stride = std::max<std::size_t>(
   1, (particle_frames.size() + static_cast<std::size_t>(budget) - 1) / static_cast<std::size_t>(budget));
 for (std::size_t index = 0; index < particle_frames.size(); index += stride) {
  const auto& frame = *particle_frames[index];
  const auto* surface = surface_at(mission, {frame.robot.x, frame.robot.y});
  const mission::SurfaceMaterialDefinition* material =
    surface ? &catalog_.surface_material(surface->material) : nullptr;
  const bool skid_smoke = frame.traction_saturated && (!material || material->drive_effect == "none");
  if (!skid_smoke && (!material || material->drive_effect == "none")) {
   continue;
  }
  const float age = static_cast<float>(state.time_s - frame.time_s);
  const float life = 1.0f - std::clamp(age / static_cast<float>(scene.effects.trail_duration_s), 0.0f, 1.0f);
  const int key = static_cast<int>(std::round(frame.time_s * 20.0));
  const float side = (hash01(key, 3) - 0.5f) * 0.62f;
  const float back = -0.20f - hash01(key, 7) * 0.28f - age * 0.12f;
  const float intensity = static_cast<float>(scene.effects.surface_particle_scale *
                                             (skid_smoke ? 0.8 : material->effect_intensity));
  const float speed = std::clamp(std::abs(static_cast<float>(frame.robot.v)) * 1.35f, 0.24f, 1.0f);
  const float rise = age * (skid_smoke || material->drive_effect == "dust" ? 0.24f : 0.38f);
  const float ground = static_cast<float>(terrain_at(mission, {frame.robot.x, frame.robot.y}).height_m);
  const auto base =
    pose_offset({frame.robot.x, frame.robot.y}, frame.robot.theta, back, side, 0.035f + rise, ground);
  const Color effect_color = skid_smoke ? Color{118, 112, 101, 255} : color(material->effect_color);
  const float alpha = life * speed * intensity;
  if (!skid_smoke && material->drive_effect == "spray") {
   // Thin standing water produces a ground wake. Vertical streaks and drops
   // read as rainfall, so wheel contact is shown only with expanding,
   // low-profile rings that remain attached to the road surface.
   const float wake = 0.07f + age * 0.32f;
   const Vector3 wake_center{base.x, ground + 0.025f, base.z};
   DrawCircle3D(wake_center, wake, {1, 0, 0}, 90.0f, Fade(effect_color, 0.46f * alpha * life));
   if (age > 0.08f) {
    DrawCircle3D(wake_center, wake * 0.62f, {1, 0, 0}, 90.0f, Fade(effect_color, 0.24f * alpha * life));
   }
  } else {
   const float radius = (0.035f + hash01(key, 17) * 0.055f) * intensity;
   DrawSphere(base, radius, Fade(effect_color, 0.46f * alpha));
  }
 }
}

void SceneRenderer3D::draw_risk_effects(const mission::PresentationFrameView& mission) const {
 const auto& state = mission.snapshot();
 const auto& scene = catalog_.scene(mission.map_definition().scene_3d);
 const float ground = static_cast<float>(terrain_at(mission, {state.robot.x, state.robot.y}).height_m);
 const auto center = world(sim::Vec2{state.robot.x, state.robot.y}, ground + 0.035f);

 if (state.rollover_risk >= 0.55) {
  const float severity = std::clamp(static_cast<float>((state.rollover_risk - 0.55) / 0.45), 0.0f, 1.0f);
  const Color warning = mix(kAmber, kDanger, severity);
  const float radius = static_cast<float>(scene.effects.risk_indicator_radius_m);
  const float downhill_side = state.cross_slope >= 0.0 ? -1.0f : 1.0f;
  const float pulse = 0.72f + 0.28f * std::sin(static_cast<float>(state.time_s * 9.0));
  // A risk warning belongs to the machine. Three lamps on the downhill
  // shoulder communicate urgency without painting animated debug rings onto
  // the physical world; the numeric/vector explanation remains in Tab view.
  for (float forward : {-0.17f, 0.0f, 0.17f}) {
   const auto lamp = pose_offset({state.robot.x, state.robot.y}, state.robot.theta, forward,
                                 downhill_side * radius * 0.34f, 0.27f, ground);
   DrawSphere(lamp, 0.025f + 0.012f * severity, Fade(warning, pulse));
  }
 }

 if (state.traction_saturated) {
  for (float left : {-0.18f, 0.18f}) {
   const auto wheel =
     pose_offset({state.robot.x, state.robot.y}, state.robot.theta, -0.18f, left, 0.055f, ground);
   DrawSphere(wheel, 0.045f, kAmber);
  }
 }

 if (state.status == mission::MissionStatus::Failed) {
  for (int ray = 0; ray < 8; ++ray) {
   const float angle = ray * 2.0f * PI / 8.0f;
   const float length = 0.18f + 0.16f * (ray % 3);
   const Vector3 end{center.x + std::cos(angle) * length, center.y + 0.10f + 0.03f * (ray % 2),
                     center.z + std::sin(angle) * length};
   DrawLine3D({center.x, center.y + 0.05f, center.z}, end, Fade(kAmber, 0.62f));
  }
 }
}

void SceneRenderer3D::draw_accents(const mission::PresentationFrameView& mission) const {
 const auto& state = mission.snapshot();
 const auto& scene = catalog_.scene(mission.map_definition().scene_3d);
 const Color active_color = color(scene.lighting.robot_active_color);
 const Color waiting_color = color(scene.lighting.robot_waiting_color);
 const Color robot_color = state.waiting_for_clearance ? waiting_color : active_color;
 const float movement = std::min(1.0f, std::abs(static_cast<float>(state.robot.v)));
 const float bob_phase = static_cast<float>(state.time_s * scene.effects.bob_frequency_hz * 2.0 * PI);
 const float bob = std::sin(bob_phase) * movement * static_cast<float>(scene.effects.bob_amplitude_m);
 const float ground_height = static_cast<float>(terrain_at(mission, {state.robot.x, state.robot.y}).height_m);
 const float body_height = static_cast<float>(state.robot_height_m);

 draw_motion_effects(mission);
 draw_risk_effects(mission);

 const auto& payload = mission.payload_state();
 const auto& cargo = scene.payload_visual;
 const auto cargo_mount = std::find_if(mission.robot().mounts.begin(), mission.robot().mounts.end(),
                                       [&](const auto& mount) { return mount.id == cargo.mount; });
 if (!payload.history.empty() && cargo_mount != mission.robot().mounts.end()) {
  const double pickup_age = state.time_s - payload.history.back().time_s;
  if (pickup_age >= 0.0 && pickup_age < cargo.pickup_effect_duration_s) {
   const float progress = static_cast<float>(pickup_age / cargo.pickup_effect_duration_s);
   const float alpha = 1.0f - progress;
   const auto center =
     pose_offset({state.robot.x, state.robot.y}, state.robot.theta,
                 static_cast<float>(cargo_mount->position_m.x), static_cast<float>(cargo_mount->position_m.y),
                 static_cast<float>(cargo_mount->position_m.z + cargo.item_origin_m.z), body_height);
   const Color pickup_color = color(cargo.pickup_effect_color);
   const Color pickup_accent = color(cargo.pickup_effect_accent_color);
   for (int mote = 0; mote < cargo.pickup_effect_particles; ++mote) {
    const float angle = static_cast<float>(mote) * 2.39996f + progress * 5.4f;
    const float radius = static_cast<float>(cargo.pickup_effect_radius_m) * (1.0f - progress * 0.7f) *
                         (0.72f + 0.25f * hash01(mote, 211));
    const Vector3 point{center.x + std::cos(angle) * radius,
                        center.y + 0.04f + progress * static_cast<float>(cargo.pickup_effect_rise_m) +
                          0.08f * hash01(mote, 223),
                        center.z + std::sin(angle) * radius};
    DrawSphere(point, 0.026f + 0.015f * hash01(mote, 227),
               Fade(mote % 3 == 0 ? pickup_accent : pickup_color, alpha));
   }
   DrawCircle3D({center.x, ground_height + 0.04f, center.z},
                static_cast<float>(cargo.pickup_effect_radius_m) * (0.5f + progress), {1, 0, 0}, 90.0f,
                Fade(pickup_accent, 0.55f * alpha));
  }
  if (payload.collected_mass_kg > 0.0 && payload.integrity < cargo.damage_warning_below_integrity) {
   const float severity =
     std::clamp(static_cast<float>((cargo.damage_warning_below_integrity - payload.integrity) /
                                   std::max(0.01, cargo.damage_warning_below_integrity)),
                0.0f, 1.0f);
   const float pulse = 0.55f + 0.45f * std::sin(static_cast<float>(state.time_s * 10.0));
   const auto warning =
     pose_offset({state.robot.x, state.robot.y}, state.robot.theta,
                 static_cast<float>(cargo_mount->position_m.x), static_cast<float>(cargo_mount->position_m.y),
                 static_cast<float>(cargo_mount->position_m.z + cargo.damage_warning_height_m), body_height);
   const Color warning_color = color(cargo.damage_warning_color);
   DrawSphere(warning, 0.035f + severity * 0.025f, Fade(warning_color, pulse));
   DrawSphereWires(warning, 0.09f + severity * 0.05f, 6, 10, Fade(warning_color, 0.58f * pulse));
  }
 }

 for (float left : {-0.10f, 0.10f}) {
  const auto eye = pose_offset({state.robot.x, state.robot.y}, state.robot.theta, 0.365f, left,
                               static_cast<float>(scene.effects.eye_height_m) + bob, ground_height);
  DrawSphere(eye, 0.040f, {19, 27, 25, 255});
  const float eye_pulse =
    state.waiting_for_clearance ? 0.021f + 0.003f * std::sin(static_cast<float>(state.time_s * 8.0)) : 0.024f;
  DrawSphere(eye, eye_pulse, robot_color);
 }

 if (state.waiting_for_clearance) {
  for (float left : {-0.15f, 0.15f}) {
   DrawSphere(
     pose_offset({state.robot.x, state.robot.y}, state.robot.theta, -0.31f, left, 0.25f, ground_height),
     0.04f, kAmber);
  }
 }
 if (state.status == mission::MissionStatus::Succeeded) {
  for (float side : {-0.13f, 0.13f}) {
   const auto lamp =
     pose_offset({state.robot.x, state.robot.y}, state.robot.theta, -0.29f, side, 0.24f, ground_height);
   DrawSphere(lamp, 0.034f, active_color);
  }
 }
}

} // namespace client
