#include "sim/viewer/scene_renderer.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <memory>
#include <utility>
#include <vector>

#include <raymath.h>
#include <rlgl.h>

#include "sim/viewer/collision_geom_wireframe.hpp"
#include "sim/world/actor_behavior.hpp"
#include "sim/world/terrain.hpp"

namespace lost_signal::sim::viewer::render {
using namespace lost_signal::sim::viewer;
using lost_signal::sim::world::ActorLinkKind;

const char* render_quality_name(RenderQuality quality) {
  return quality == RenderQuality::kHigh ? "HIGH" : "FAST";
}

bool model_ready(const Model& model) { return model.meshCount > 0; }

RenderTexture2D load_shadow_map(int width, int height) {
 RenderTexture2D target{};
 target.id = rlLoadFramebuffer();
 target.texture.width = width;
 target.texture.height = height;
 if (target.id == 0) return target;
 rlEnableFramebuffer(target.id);
 // Keep a real color attachment even though the lighting pass samples only
 // depth. BeginTextureMode() and several OpenGL drivers require a complete
 // draw/read framebuffer; a depth-only target was intermittently reported as
 // incomplete and produced missing or shimmering shadows.
 target.texture.id = rlLoadTexture(
  nullptr, width, height, RL_PIXELFORMAT_UNCOMPRESSED_R8G8B8A8, 1);
 target.texture.format = RL_PIXELFORMAT_UNCOMPRESSED_R8G8B8A8;
 target.texture.mipmaps = 1;
 rlFramebufferAttach(target.id, target.texture.id, RL_ATTACHMENT_COLOR_CHANNEL0,
  RL_ATTACHMENT_TEXTURE2D, 0);
 target.depth.id = rlLoadTextureDepth(width, height, false);
 target.depth.width = width;
 target.depth.height = height;
 target.depth.format = 19;
 target.depth.mipmaps = 1;
 rlFramebufferAttach(target.id, target.depth.id, RL_ATTACHMENT_DEPTH,
 RL_ATTACHMENT_TEXTURE2D, 0);
 if (!rlFramebufferComplete(target.id)) {
 rlDisableFramebuffer();
 rlUnloadFramebuffer(target.id);
 rlUnloadTexture(target.texture.id);
 return {};
 }
 // rlLoadFramebuffer leaves the new target bound.  Keep render-target
 // ownership local to this helper: callers may fall back to the direct
 // renderer before the first post-process pass, and an accidentally bound
 // shadow FBO would make the window appear blank until EndTextureMode().
 rlDisableFramebuffer();
 return target;
}

void release_shadow_map(VisualAssets& assets) {
 if (assets.shadow_map.id > 0) rlUnloadFramebuffer(assets.shadow_map.id);
 if (assets.shadow_map.texture.id > 0)
 rlUnloadTexture(assets.shadow_map.texture.id);
 assets.shadow_map = {};
 assets.shadow_map_resolution = 0;
}

bool ensure_shadow_map(VisualAssets& assets, int resolution) {
 if (resolution == assets.shadow_map_resolution &&
 (resolution == 0 || assets.shadow_map.id > 0)) {
 return resolution > 0;
 }
 release_shadow_map(assets);
 if (resolution <= 0) return false;
 assets.shadow_map = load_shadow_map(resolution, resolution);
 if (assets.shadow_map.id == 0) return false;
 assets.shadow_map_resolution = resolution;
 return true;
}

RenderTexture2D load_hdr_target(int width, int height) {
 RenderTexture2D target{};
 target.id = rlLoadFramebuffer();
 if (target.id == 0) return target;
 rlEnableFramebuffer(target.id);
 target.texture.id = rlLoadTexture(
 nullptr, width, height, RL_PIXELFORMAT_UNCOMPRESSED_R16G16B16A16, 1);
 target.texture.width = width;
 target.texture.height = height;
 target.texture.format = PIXELFORMAT_UNCOMPRESSED_R16G16B16A16;
 target.texture.mipmaps = 1;
 target.depth.id = rlLoadTextureDepth(width, height, true);
 target.depth.width = width;
 target.depth.height = height;
 target.depth.format = 19;
 target.depth.mipmaps = 1;
 rlFramebufferAttach(target.id, target.texture.id,
 RL_ATTACHMENT_COLOR_CHANNEL0, RL_ATTACHMENT_TEXTURE2D, 0);
 rlFramebufferAttach(target.id, target.depth.id,
 RL_ATTACHMENT_DEPTH, RL_ATTACHMENT_RENDERBUFFER, 0);
 if (!rlFramebufferComplete(target.id)) {
 UnloadRenderTexture(target);
 return LoadRenderTexture(width, height);
 }
 SetTextureFilter(target.texture, TEXTURE_FILTER_BILINEAR);
 return target;
}

void release_post_targets(VisualAssets& assets) {
 if (assets.scene_target.id > 0) UnloadRenderTexture(assets.scene_target);
 if (assets.bloom_target_a.id > 0)
 UnloadRenderTexture(assets.bloom_target_a);
 if (assets.bloom_target_b.id > 0)
 UnloadRenderTexture(assets.bloom_target_b);
 assets.scene_target = {};
 assets.bloom_target_a = {};
 assets.bloom_target_b = {};
 assets.target_width = 0;
 assets.target_height = 0;
}

bool ensure_post_targets(VisualAssets& assets, int width, int height) {
 if (width == assets.target_width && height == assets.target_height &&
 assets.scene_target.id > 0 && assets.bloom_target_a.id > 0 &&
 assets.bloom_target_b.id > 0) {
 return true;
 }
 release_post_targets(assets);
 if (width <= 0 || height <= 0) return false;
 assets.scene_target = load_hdr_target(width, height);
 const int bloom_width = std::max(1, width / 2);
 const int bloom_height = std::max(1, height / 2);
 assets.bloom_target_a = LoadRenderTexture(bloom_width, bloom_height);
 assets.bloom_target_b = LoadRenderTexture(bloom_width, bloom_height);
 if (assets.scene_target.id == 0 || assets.bloom_target_a.id == 0 ||
 assets.bloom_target_b.id == 0) {
 release_post_targets(assets);
 return false;
 }
 SetTextureFilter(assets.bloom_target_a.texture, TEXTURE_FILTER_BILINEAR);
 SetTextureFilter(assets.bloom_target_b.texture, TEXTURE_FILTER_BILINEAR);
 assets.target_width = width;
 assets.target_height = height;
 return true;
}

void draw_render_texture(const RenderTexture2D& target,
 float width, float height) {
 DrawTexturePro(target.texture,
 {0.0f, 0.0f, static_cast<float>(target.texture.width),
 -static_cast<float>(target.texture.height)},
 {0.0f, 0.0f, width, height}, {0.0f, 0.0f}, 0.0f, WHITE);
}

void log_model_bounds(const char* label, const Model& model) {
 if (!model_ready(model)) return;
 const BoundingBox b = GetModelBoundingBox(model);
 const Vector3 lo{std::min(b.min.x, b.max.x), std::min(b.min.y, b.max.y),
 std::min(b.min.z, b.max.z)};
 const Vector3 hi{std::max(b.min.x, b.max.x), std::max(b.min.y, b.max.y),
 std::max(b.min.z, b.max.z)};
 std::fprintf(stderr,
 "[viewer] %s meshes=%d bounds x=%.3f..%.3f y=%.3f..%.3f z=%.3f..%.3f\n",
 label, model.meshCount, lo.x, hi.x, lo.y, hi.y, lo.z, hi.z);
}

void bind_lighting(Model& model, Shader shader) {
 if (!model_ready(model) || !IsShaderValid(shader)) return;
 for (int i = 0; i < model.materialCount; ++i) {
 auto& material = model.materials[i];
 const float metallic = std::clamp(
 material.maps[MATERIAL_MAP_METALNESS].value, 0.0f, 1.0f);
 const float roughness = std::clamp(
 material.maps[MATERIAL_MAP_ROUGHNESS].value, 0.04f, 1.0f);
 const Color emission = material.maps[MATERIAL_MAP_EMISSION].color;
 const float emissive = static_cast<float>(
 std::max({emission.r, emission.g, emission.b})) / 255.0f;
 // raylib uploads MATERIAL_MAP_METALNESS.color through colSpecular. The
 // project shader uses those otherwise unused channels as compact PBR
 // material metadata; texture/color content remains authored in the GLB.
 material.maps[MATERIAL_MAP_METALNESS].color = {
 static_cast<unsigned char>(std::round(metallic * 255.0f)),
 static_cast<unsigned char>(std::round(roughness * 255.0f)),
 static_cast<unsigned char>(std::round(emissive * 255.0f)), 255};
 material.shader = shader;
 }
}

BoundingBox raylib_asset_bounds(const Model& model) {
 // raylib transforms only the minimum and maximum corners, which is invalid
 // for a rotated model. Transform all eight raw corners so axis-corrected
 // legacy assets retain exact fitted and grounding bounds.
 Model raw = model;
 raw.transform = MatrixIdentity();
 const BoundingBox b = GetModelBoundingBox(raw);
 BoundingBox result{{INFINITY, INFINITY, INFINITY},
 {-INFINITY, -INFINITY, -INFINITY}};
 for (int x = 0; x < 2; ++x) {
 for (int y = 0; y < 2; ++y) {
 for (int z = 0; z < 2; ++z) {
 const Vector3 corner{x == 0 ? b.min.x : b.max.x,
 y == 0 ? b.min.y : b.max.y,
 z == 0 ? b.min.z : b.max.z};
 const Vector3 point = Vector3Transform(corner, model.transform);
 result.min.x = std::min(result.min.x, point.x);
 result.min.y = std::min(result.min.y, point.y);
 result.min.z = std::min(result.min.z, point.z);
 result.max.x = std::max(result.max.x, point.x);
 result.max.y = std::max(result.max.y, point.y);
 result.max.z = std::max(result.max.z, point.z);
 }
 }
 }
 return result;
}

bool model_bounds_match_manifest(const Model& model,
 const AssetDefinition& definition) {
 Model raw = model;
 raw.transform = MatrixIdentity();
 const BoundingBox bounds = GetModelBoundingBox(raw);
 // The manifest records the coordinates stored in the file. Compare each
 // axis directly; sorting extents used to let a side-lying model pass.
 const std::array<double, 3> actual_min{
 bounds.min.x * definition.meters_per_unit,
 bounds.min.y * definition.meters_per_unit,
 bounds.min.z * definition.meters_per_unit};
 const std::array<double, 3> actual_max{
 bounds.max.x * definition.meters_per_unit,
 bounds.max.y * definition.meters_per_unit,
 bounds.max.z * definition.meters_per_unit};
 for (std::size_t axis = 0; axis < 3; ++axis) {
 const double expected_extent =
 definition.expected_bounds_m.maximum_m[axis] -
 definition.expected_bounds_m.minimum_m[axis];
 const double tolerance = std::max(0.002, expected_extent * 0.02);
 if (std::abs(actual_min[axis] -
 definition.expected_bounds_m.minimum_m[axis]) > tolerance ||
 std::abs(actual_max[axis] -
 definition.expected_bounds_m.maximum_m[axis]) > tolerance) {
 return false;
 }
 }
 return true;
}

Vector3 manifest_point_to_raylib(const AssetDefinition& definition,
 Vector3 asset_point) {
 const auto render = lost_signal::sim::viewer::asset_point_to_render(
 definition, {asset_point.x, asset_point.y, asset_point.z});
 return {static_cast<float>(render[0]), static_cast<float>(render[1]),
 static_cast<float>(render[2])};
}

Matrix manifest_model_transform(const AssetDefinition& definition) {
 const Vector3 origin = manifest_point_to_raylib(
 definition, {0.0f, 0.0f, 0.0f});
 const Vector3 x = manifest_point_to_raylib(
 definition, {1.0f, 0.0f, 0.0f});
 const Vector3 y = manifest_point_to_raylib(
 definition, {0.0f, 1.0f, 0.0f});
 const Vector3 z = manifest_point_to_raylib(
 definition, {0.0f, 0.0f, 1.0f});
 Matrix matrix = MatrixIdentity();
 matrix.m0 = x.x - origin.x;
 matrix.m1 = x.y - origin.y;
 matrix.m2 = x.z - origin.z;
 matrix.m4 = y.x - origin.x;
 matrix.m5 = y.y - origin.y;
 matrix.m6 = y.z - origin.z;
 matrix.m8 = z.x - origin.x;
 matrix.m9 = z.y - origin.y;
 matrix.m10 = z.z - origin.z;
 matrix.m12 = origin.x;
 matrix.m13 = origin.y;
 matrix.m14 = origin.z;
 return matrix;
}

Model load_manifest_model(const std::filesystem::path& models_root,
 const AssetManifest& manifest, const std::string& asset_id,
 const AssetDefinition*& definition_out) {
 definition_out = manifest.find(asset_id);
 if (definition_out == nullptr) {
 std::fprintf(stderr, "[viewer] asset id missing from manifest: %s\n",
 asset_id.c_str());
 return {};
 }
 const std::filesystem::path path =
 models_root / definition_out->relative_path;
 Model model = LoadModel(path.string().c_str());
 if (!model_ready(model)) return {};
 if (!model_bounds_match_manifest(model, *definition_out)) {
 std::fprintf(stderr,
 "[viewer] asset bounds differ from manifest, using fallback: %s\n",
 asset_id.c_str());
 UnloadModel(model);
 definition_out = nullptr;
 return {};
 }
 model.transform = manifest_model_transform(*definition_out);
 return model;
}

const VisualAssets::EnvironmentModel* find_environment_model(
 const VisualAssets& assets, const std::string& asset_id) {
 for (const auto& environment : assets.environment) {
 if (environment.asset_id == asset_id) return &environment;
 }
 return nullptr;
}

void load_visual_assets(VisualAssets& assets, const std::string& root,
 const std::string& robot_id, const std::string& chassis_kind,
 const AssemblySelection* assembly) {
 if (root.empty()) return;
 const std::filesystem::path base(root);
 const std::filesystem::path models_root = base / "models";
 std::string manifest_error;
 if (!lost_signal::sim::viewer::load_asset_manifest(
 (models_root / "manifest.json").string(), assets.manifest,
 &manifest_error)) {
 std::fprintf(stderr, "[viewer] asset manifest rejected: %s\n",
 manifest_error.c_str());
 return;
 }
 auto selection = lost_signal::sim::viewer::asset_selection_for_robot(
 robot_id, chassis_kind);
 if (assembly != nullptr && chassis_kind == "differential") {
   // The registry has already validated these IDs before runtime creation;
   // this only selects presentation assets and never changes ROBOT inputs.
   if (!assembly->wheel.empty()) {
     selection.wheel = assembly->wheel;
     selection.wheel_right = assembly->wheel;
   }
 }
 assets.chassis = load_manifest_model(models_root, assets.manifest,
 selection.chassis, assets.chassis_definition);
 assets.wheel = load_manifest_model(models_root, assets.manifest,
 selection.wheel, assets.wheel_definition);
 assets.wheel_right = load_manifest_model(models_root, assets.manifest,
 selection.wheel_right, assets.wheel_right_definition);
 if (!selection.steering_knuckle_left.empty()) {
 assets.knuckle_left = load_manifest_model(models_root, assets.manifest,
 selection.steering_knuckle_left, assets.knuckle_left_definition);
 assets.knuckle_right = load_manifest_model(models_root, assets.manifest,
 selection.steering_knuckle_right, assets.knuckle_right_definition);
 }
 if (!selection.arm_turret.empty()) {
 assets.arm_turret = load_manifest_model(models_root, assets.manifest,
 selection.arm_turret, assets.arm_turret_definition);
 assets.arm_upper = load_manifest_model(models_root, assets.manifest,
 selection.arm_upper, assets.arm_upper_definition);
 assets.arm_forearm = load_manifest_model(models_root, assets.manifest,
 selection.arm_forearm, assets.arm_forearm_definition);
 assets.arm_tool = load_manifest_model(models_root, assets.manifest,
 selection.arm_tool, assets.arm_tool_definition);
 }
 assets.lidar = load_manifest_model(
     models_root, assets.manifest,
     assembly != nullptr && !assembly->lidar.empty()
         ? assembly->lidar : "robots.ro1.lidar_standard",
     assets.lidar_definition);
 assets.caster_fork = load_manifest_model(models_root, assets.manifest,
 "robots.ro1.caster_fork", assets.caster_fork_definition);
 assets.caster_wheel = load_manifest_model(models_root, assets.manifest,
 "robots.ro1.caster_wheel", assets.caster_wheel_definition);
 assets.track = load_manifest_model(models_root, assets.manifest,
 "robots.ro3.track", assets.track_definition);
 assets.forklift = load_manifest_model(models_root, assets.manifest,
 "actors.forklift", assets.forklift_definition);
 assets.forklift_base = load_manifest_model(models_root, assets.manifest,
 "actors.forklift_base", assets.forklift_base_definition);
 assets.forklift_drive_wheel = load_manifest_model(
 models_root, assets.manifest, "actors.forklift_drive_wheel",
 assets.forklift_drive_wheel_definition);
 assets.forklift_steer_wheel = load_manifest_model(
 models_root, assets.manifest, "actors.forklift_steer_wheel",
 assets.forklift_steer_wheel_definition);
 assets.forklift_mast = load_manifest_model(models_root, assets.manifest,
 "actors.forklift_mast", assets.forklift_mast_definition);
 assets.forklift_carriage = load_manifest_model(
 models_root, assets.manifest, "actors.forklift_carriage",
 assets.forklift_carriage_definition);
 assets.forklift_fork = load_manifest_model(models_root, assets.manifest,
 "actors.forklift_fork", assets.forklift_fork_definition);
 assets.worker = load_manifest_model(models_root, assets.manifest,
 "actors.worker", assets.worker_definition);
 if (model_ready(assets.worker) && assets.worker_definition != nullptr &&
 !assets.worker_definition->animation_clips.empty()) {
 const std::filesystem::path worker_path =
 models_root / assets.worker_definition->relative_path;
 assets.worker_animations = LoadModelAnimations(
 worker_path.string().c_str(), &assets.worker_animation_count);
 if (assets.worker_animations == nullptr ||
 assets.worker_animation_count <= 0) {
 std::fprintf(stderr,
 "[viewer] worker animation clips declared but none could be loaded: %s\n",
 worker_path.string().c_str());
 if (assets.worker_animations != nullptr) {
 UnloadModelAnimations(assets.worker_animations,
 assets.worker_animation_count);
 }
 assets.worker_animations = nullptr;
 assets.worker_animation_count = 0;
 }
 }
 for (const auto& definition : assets.manifest.assets()) {
 if (definition.kind != "environment") continue;
 const AssetDefinition* loaded_definition = nullptr;
 Model model = load_manifest_model(models_root, assets.manifest,
 definition.asset_id, loaded_definition);
 if (model_ready(model) && loaded_definition != nullptr) {
 assets.environment.push_back(
 {definition.asset_id, model, loaded_definition});
 }
 }
 const auto texture_dir = base / "textures" / "environment";
 if (std::filesystem::is_regular_file(texture_dir / "dust_puff.png")) {
 assets.dust = LoadTexture((texture_dir / "dust_puff.png").string().c_str());
 if (assets.dust.id > 0) {
 SetTextureFilter(assets.dust, TEXTURE_FILTER_BILINEAR);
 }
 // Ground materials use a real textured XZ plane in the 3D renderer.  The
 // world still owns friction/contact truth; these surfaces are presentation
 // only and are selected by the scene floor material or a zone overlay.
 const std::array<std::pair<const char*, const char*>, 5> floor_texture_files{{
     {"paved", "concrete_floor.png"},
     {"gravel", "gravel_floor.png"},
     {"mud", "mud_floor.png"},
     {"sand", "sand_floor.png"},
     {"standing_water", "water_floor.png"},
 }};
 for (const auto& entry : floor_texture_files) {
   const auto path = texture_dir / entry.second;
   if (!std::filesystem::is_regular_file(path)) continue;
   Texture2D texture = LoadTexture(path.string().c_str());
   if (texture.id == 0) continue;
   SetTextureFilter(texture, TEXTURE_FILTER_BILINEAR);
   SetTextureWrap(texture, TEXTURE_WRAP_REPEAT);
   Model model = LoadModelFromMesh(GenMeshPlane(1.0f, 1.0f, 1, 1));
   if (!model_ready(model) || model.materialCount <= 0) {
     if (model_ready(model)) UnloadModel(model);
     UnloadTexture(texture);
     continue;
   }
   model.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = texture;
   assets.floor_surfaces.push_back({entry.first, texture, model});
 }
 }
 log_model_bounds("chassis", assets.chassis);
 log_model_bounds("wheel", assets.wheel);
 log_model_bounds("lidar", assets.lidar);
 log_model_bounds("caster_fork", assets.caster_fork);
 log_model_bounds("caster_wheel", assets.caster_wheel);
 log_model_bounds("track", assets.track);
 log_model_bounds("forklift", assets.forklift);
 log_model_bounds("forklift_base", assets.forklift_base);
 log_model_bounds("forklift_drive_wheel", assets.forklift_drive_wheel);
 log_model_bounds("forklift_steer_wheel", assets.forklift_steer_wheel);
 log_model_bounds("forklift_mast", assets.forklift_mast);
 log_model_bounds("forklift_carriage", assets.forklift_carriage);
 log_model_bounds("forklift_fork", assets.forklift_fork);
 log_model_bounds("worker", assets.worker);
 const auto shader_root = base / "shaders";
 if (std::filesystem::is_regular_file(shader_root / "scene_lighting.vs") &&
 std::filesystem::is_regular_file(shader_root / "scene_lighting.fs")) {
 assets.lighting = LoadShader(
 (shader_root / "scene_lighting.vs").string().c_str(),
 (shader_root / "scene_lighting.fs").string().c_str());
 }
 if (std::filesystem::is_regular_file(shader_root / "bloom_extract.fs")) {
 assets.bloom_extract = LoadShader(
 nullptr, (shader_root / "bloom_extract.fs").string().c_str());
 }
 if (std::filesystem::is_regular_file(shader_root / "bloom_blur.fs")) {
 assets.bloom_blur = LoadShader(
 nullptr, (shader_root / "bloom_blur.fs").string().c_str());
 }
 if (std::filesystem::is_regular_file(shader_root / "scene_composite.fs")) {
 assets.composite = LoadShader(
 nullptr, (shader_root / "scene_composite.fs").string().c_str());
 }
 if (IsShaderValid(assets.bloom_extract)) {
 assets.bloom_threshold_loc = GetShaderLocation(
 assets.bloom_extract, "threshold");
 }
 if (IsShaderValid(assets.bloom_blur)) {
 assets.blur_texel_size_loc = GetShaderLocation(
 assets.bloom_blur, "texelSize");
 assets.blur_direction_loc = GetShaderLocation(
 assets.bloom_blur, "direction");
 }
 if (IsShaderValid(assets.composite)) {
 assets.composite_bloom_texture_loc = GetShaderLocation(
 assets.composite, "bloomTexture");
 assets.composite_exposure_loc = GetShaderLocation(
 assets.composite, "exposure");
 assets.composite_bloom_strength_loc = GetShaderLocation(
 assets.composite, "bloomStrength");
 assets.composite_vignette_strength_loc = GetShaderLocation(
 assets.composite, "vignetteStrength");
 }
 if (IsShaderValid(assets.lighting)) {
 assets.lighting.locs[SHADER_LOC_VECTOR_VIEW] = GetShaderLocation(
 assets.lighting, "viewPos");
 assets.lighting.locs[SHADER_LOC_MATRIX_NORMAL] = GetShaderLocation(
 assets.lighting, "matNormal");
 assets.lighting.locs[SHADER_LOC_COLOR_SPECULAR] = GetShaderLocation(
 assets.lighting, "colSpecular");
 assets.light_view_projection_loc = GetShaderLocation(
 assets.lighting, "lightVP");
 assets.shadow_map_loc = GetShaderLocation(assets.lighting, "shadowMap");
 assets.shadow_strength_loc = GetShaderLocation(
 assets.lighting, "shadowStrength");
 assets.shadow_resolution_loc = GetShaderLocation(
 assets.lighting, "shadowMapResolution");
 assets.floor_texture_tiling_loc = GetShaderLocation(
 assets.lighting, "floorTextureTiling");
 const Vector3 light_dir{-0.48f, -1.0f, -0.32f};
 const Vector3 light_color{1.08f, 0.98f, 0.82f};
 const Vector3 ambient{0.24f, 0.29f, 0.28f};
 const Vector3 fog_color{0.035f, 0.055f, 0.052f};
 const Vector3 point0{4.0f, 5.0f, -3.0f};
 const Vector3 point1{-4.0f, 3.0f, 2.0f};
 const float fog_start = 12.0f, fog_end = 31.0f;
 const float shadow_strength = 0.0f;
 const int shadow_resolution = 1;
 const Matrix light_vp = MatrixIdentity();
 SetShaderValue(assets.lighting,
 GetShaderLocation(assets.lighting, "lightDirection"), &light_dir,
 SHADER_UNIFORM_VEC3);
 SetShaderValue(assets.lighting,
 GetShaderLocation(assets.lighting, "lightColor"), &light_color,
 SHADER_UNIFORM_VEC3);
 SetShaderValue(assets.lighting,
 GetShaderLocation(assets.lighting, "ambient"), &ambient,
 SHADER_UNIFORM_VEC3);
 SetShaderValue(assets.lighting,
 GetShaderLocation(assets.lighting, "fogColor"), &fog_color,
 SHADER_UNIFORM_VEC3);
 SetShaderValue(assets.lighting,
 GetShaderLocation(assets.lighting, "pointLight0Position"), &point0,
 SHADER_UNIFORM_VEC3);
 SetShaderValue(assets.lighting,
 GetShaderLocation(assets.lighting, "pointLight0Color"), &light_color,
 SHADER_UNIFORM_VEC3);
 SetShaderValue(assets.lighting,
 GetShaderLocation(assets.lighting, "pointLight1Position"), &point1,
 SHADER_UNIFORM_VEC3);
 SetShaderValue(assets.lighting,
 GetShaderLocation(assets.lighting, "pointLight1Color"), &ambient,
 SHADER_UNIFORM_VEC3);
 SetShaderValue(assets.lighting,
 GetShaderLocation(assets.lighting, "fogStart"), &fog_start,
 SHADER_UNIFORM_FLOAT);
 SetShaderValue(assets.lighting,
 GetShaderLocation(assets.lighting, "fogEnd"), &fog_end,
 SHADER_UNIFORM_FLOAT);
 SetShaderValue(assets.lighting,
 GetShaderLocation(assets.lighting, "shadowStrength"), &shadow_strength,
 SHADER_UNIFORM_FLOAT);
 SetShaderValue(assets.lighting,
 GetShaderLocation(assets.lighting, "shadowMapResolution"),
 &shadow_resolution, SHADER_UNIFORM_INT);
 SetShaderValueMatrix(assets.lighting,
 GetShaderLocation(assets.lighting, "lightVP"), light_vp);
 bind_lighting(assets.chassis, assets.lighting);
 bind_lighting(assets.wheel, assets.lighting);
 bind_lighting(assets.wheel_right, assets.lighting);
 bind_lighting(assets.knuckle_left, assets.lighting);
 bind_lighting(assets.knuckle_right, assets.lighting);
 bind_lighting(assets.arm_turret, assets.lighting);
 bind_lighting(assets.arm_upper, assets.lighting);
 bind_lighting(assets.arm_forearm, assets.lighting);
 bind_lighting(assets.arm_tool, assets.lighting);
 bind_lighting(assets.lidar, assets.lighting);
 bind_lighting(assets.caster_fork, assets.lighting);
 bind_lighting(assets.caster_wheel, assets.lighting);
 bind_lighting(assets.track, assets.lighting);
 bind_lighting(assets.forklift, assets.lighting);
 bind_lighting(assets.forklift_base, assets.lighting);
 bind_lighting(assets.forklift_drive_wheel, assets.lighting);
 bind_lighting(assets.forklift_steer_wheel, assets.lighting);
 bind_lighting(assets.forklift_mast, assets.lighting);
 bind_lighting(assets.forklift_carriage, assets.lighting);
 bind_lighting(assets.forklift_fork, assets.lighting);
 bind_lighting(assets.worker, assets.lighting);
 for (auto& environment : assets.environment) {
 bind_lighting(environment.model, assets.lighting);
 }
 for (auto& floor : assets.floor_surfaces) {
   bind_lighting(floor.model, assets.lighting);
 }
 }
 assets.loaded = model_ready(assets.chassis) || model_ready(assets.wheel) ||
 model_ready(assets.wheel_right) || model_ready(assets.knuckle_left) ||
 model_ready(assets.knuckle_right) ||
 model_ready(assets.arm_turret) || model_ready(assets.arm_upper) ||
 model_ready(assets.arm_forearm) || model_ready(assets.arm_tool) ||
 model_ready(assets.lidar) || model_ready(assets.caster_fork) ||
 model_ready(assets.caster_wheel) ||
 model_ready(assets.track) || model_ready(assets.forklift) ||
 model_ready(assets.forklift_base) ||
 model_ready(assets.forklift_drive_wheel) ||
 model_ready(assets.forklift_steer_wheel) ||
 model_ready(assets.forklift_mast) ||
 model_ready(assets.forklift_carriage) ||
 model_ready(assets.forklift_fork) ||
 model_ready(assets.worker) || !assets.environment.empty() ||
 !assets.floor_surfaces.empty();
}

void unload_visual_assets(VisualAssets& assets) {
 release_shadow_map(assets);
 release_post_targets(assets);
 if (assets.dust.id > 0) UnloadTexture(assets.dust);
 if (model_ready(assets.chassis)) UnloadModel(assets.chassis);
 if (model_ready(assets.wheel)) UnloadModel(assets.wheel);
 if (model_ready(assets.wheel_right)) UnloadModel(assets.wheel_right);
 if (model_ready(assets.knuckle_left)) UnloadModel(assets.knuckle_left);
 if (model_ready(assets.knuckle_right)) UnloadModel(assets.knuckle_right);
 if (model_ready(assets.arm_turret)) UnloadModel(assets.arm_turret);
 if (model_ready(assets.arm_upper)) UnloadModel(assets.arm_upper);
 if (model_ready(assets.arm_forearm)) UnloadModel(assets.arm_forearm);
 if (model_ready(assets.arm_tool)) UnloadModel(assets.arm_tool);
 if (model_ready(assets.lidar)) UnloadModel(assets.lidar);
 if (model_ready(assets.caster_fork)) UnloadModel(assets.caster_fork);
 if (model_ready(assets.caster_wheel)) UnloadModel(assets.caster_wheel);
 if (model_ready(assets.track)) UnloadModel(assets.track);
 if (model_ready(assets.forklift)) UnloadModel(assets.forklift);
 if (model_ready(assets.forklift_base)) UnloadModel(assets.forklift_base);
 if (model_ready(assets.forklift_drive_wheel))
 UnloadModel(assets.forklift_drive_wheel);
 if (model_ready(assets.forklift_steer_wheel))
 UnloadModel(assets.forklift_steer_wheel);
 if (model_ready(assets.forklift_mast)) UnloadModel(assets.forklift_mast);
 if (model_ready(assets.forklift_carriage))
 UnloadModel(assets.forklift_carriage);
 if (model_ready(assets.forklift_fork)) UnloadModel(assets.forklift_fork);
 if (model_ready(assets.worker)) UnloadModel(assets.worker);
 if (assets.worker_animations != nullptr) {
 UnloadModelAnimations(assets.worker_animations,
 assets.worker_animation_count);
 }
 for (auto& environment : assets.environment) {
 if (model_ready(environment.model)) UnloadModel(environment.model);
 }
 for (auto& floor : assets.floor_surfaces) {
   if (model_ready(floor.model)) UnloadModel(floor.model);
   if (floor.texture.id > 0) UnloadTexture(floor.texture);
 }
 if (IsShaderValid(assets.lighting)) UnloadShader(assets.lighting);
 if (IsShaderValid(assets.bloom_extract)) UnloadShader(assets.bloom_extract);
 if (IsShaderValid(assets.bloom_blur)) UnloadShader(assets.bloom_blur);
 if (IsShaderValid(assets.composite)) UnloadShader(assets.composite);
 assets = {};
}

void draw_fitted_model(const Model& model, Vector3 position, float radius,
 float height, Color tint, float rotation_deg = 0.0f) {
 if (!model_ready(model)) return;
 const BoundingBox bounds = raylib_asset_bounds(model);
 const Vector3 lo{bounds.min.x, bounds.min.y, bounds.min.z};
 const Vector3 hi{bounds.max.x, bounds.max.y, bounds.max.z};
 const float sx = std::max(1e-3f, hi.x - lo.x);
 const float sy = std::max(1e-3f, hi.y - lo.y);
 const float sz = std::max(1e-3f, hi.z - lo.z);
 const float scale = std::min({2.0f * radius / sx, height / sy,
 2.0f * radius / sz});
 const Vector3 center{(lo.x + hi.x) * 0.5f,
 (lo.y + hi.y) * 0.5f, (lo.z + hi.z) * 0.5f};
 const Vector3 centered{position.x - center.x * scale,
 position.y - center.y * scale, position.z - center.z * scale};
 DrawModelEx(model, centered, {0.0f, 1.0f, 0.0f}, rotation_deg,
 {scale, scale, scale}, tint);
}

// Actor GLBs are authored with their local origin on the ground and with a
// content reference radius (forklift=0.45 m, worker=0.25 m).  Keep that
// contract instead of fitting the model into a collision cylinder: fitting
// all three axes to the radius made the forklift half-height and visually
// detached its wheels from the chassis.  The actor pose remains the SIM
// kinematic pose; this helper only converts the asset's z-up coordinates to
// raylib's y-up frame and guarantees that its lowest vertex touches terrain.
void draw_actor_model_grounded(const Model& model, double x_m, double y_m,
 const AssetDefinition& definition, double ground_z_m, double radius_m,
 double heading_rad) {
 if (!model_ready(model)) return;
 const float reference_radius = static_cast<float>(
 definition.reference_radius_m);
 if (!(reference_radius > 0.0f)) return;
 const float scale = static_cast<float>(radius_m) /
 std::max(1e-3f, reference_radius);
 const BoundingBox bounds = raylib_asset_bounds(model);
 const float min_y = bounds.min.y;
 // Model origins in the asset pack are ground anchors.  The min-y correction
 // keeps that invariant true even when a future asset has a small negative
 // bevel below its authored origin.
 const Vector3 position{
 static_cast<float>(x_m),
 static_cast<float>(ground_z_m - min_y * scale),
 static_cast<float>(-y_m)};
 DrawModelEx(model, position, {0.0f, 1.0f, 0.0f},
 static_cast<float>(-heading_rad * RAD2DEG),
 {scale, scale, scale}, WHITE);
}

void draw_scene_instance(const SceneInstance& instance,
 const VisualAssets& assets, double ground_z_m) {
 const auto* loaded = find_environment_model(assets, instance.asset_id);
 if (loaded == nullptr || !model_ready(loaded->model)) return;
 // Scene scale is authored in canonical X-forward/Y-left/Z-up. Convert it
 // once at the asset boundary to raylib X-forward/Y-up/Z-right.
 const Vector3 scale{
 static_cast<float>(instance.scale[0]),
 static_cast<float>(instance.scale[2]),
 static_cast<float>(instance.scale[1])};
 const BoundingBox bounds = raylib_asset_bounds(loaded->model);
 const Vector3 position{
 static_cast<float>(instance.position_m[0]),
 static_cast<float>(ground_z_m + instance.position_m[2] -
 bounds.min.y * scale.y),
 static_cast<float>(-instance.position_m[1])};
 DrawModelEx(loaded->model, position, {0.0f, 1.0f, 0.0f},
 static_cast<float>(-instance.yaw_deg), scale, WHITE);
}

// Ground-material paint (master's drive_effect colors, SIM-side display
// only — physics never reads these; unknown ids fall back to asphalt).
Color material_color(const std::string& id, unsigned char alpha = 255) {
  if (id == "standing_water") return {0x7B, 0xCE, 0xD2, alpha};
  if (id == "sand") return {0xD8, 0xB9, 0x78, alpha};
  if (id == "gravel") return {0xB3, 0xA1, 0x7A, alpha};
  if (id == "mud") return {0x79, 0x60, 0x44, alpha};
  if (id == "paved") return {0x89, 0x91, 0x8B, alpha};
  if (id == "grip_grating") return {0x4E, 0xE7, 0xC5, alpha};
  if (id == "steel_deck") return {0xB8, 0xC4, 0xCA, alpha};
  if (id == "ice") return {0xC4, 0xE8, 0xF2, alpha};
  if (id == "asphalt") return {0x52, 0x64, 0x6A, alpha};
  return {0x52, 0x64, 0x6A, alpha};  // weathered_asphalt fallback
}

const VisualAssets::FloorSurface* floor_surface_for(
    const VisualAssets* assets, const std::string& material_id) {
  if (assets == nullptr) return nullptr;
  std::string canonical = material_id;
  if (material_id == "industrial_gravel") canonical = "gravel";
  else if (material_id == "loading_wet" || material_id == "standing_water" ||
           material_id == "ice") canonical = "standing_water";
  else if (material_id == "dry_asphalt" ||
           material_id == "weathered_asphalt" || material_id == "asphalt") {
    canonical = "paved";
  }
  auto find = [&](const std::string& id) -> const VisualAssets::FloorSurface* {
    for (const auto& surface : assets->floor_surfaces) {
      if (surface.material_id == id) return &surface;
    }
    return nullptr;
  };
  if (const auto* exact = find(canonical); exact != nullptr) return exact;
  if (const auto* paved = find("paved"); paved != nullptr) return paved;
  return find("standing_water");
}

Vector3 shader_rgb(const std::array<int, 4>& rgba, double intensity) {
 const float scale = static_cast<float>(intensity / 255.0);
 return {rgba[0] * scale, rgba[1] * scale, rgba[2] * scale};
}

Color scene_background(const SceneManifest* scene) {
 if (scene == nullptr) return Color{18, 31, 34, 255};
 const auto& color = scene->lighting.fog_rgba;
 return {static_cast<unsigned char>(color[0]),
 static_cast<unsigned char>(color[1]),
 static_cast<unsigned char>(color[2]), 255};
}

void configure_scene_lighting(VisualAssets& assets,
 const SceneManifest* scene, const Frame& frame, const Camera3D& camera) {
 if (!IsShaderValid(assets.lighting)) return;
 const lost_signal::sim::viewer::SceneLighting defaults;
 const auto& lighting = scene != nullptr ? scene->lighting : defaults;
 Vector3 light_direction{
 static_cast<float>(lighting.direction_xyz[0]),
 static_cast<float>(lighting.direction_xyz[2]),
 static_cast<float>(-lighting.direction_xyz[1])};
 if (Vector3Length(light_direction) > 1e-6f) {
 light_direction = Vector3Normalize(light_direction);
 }
 const Vector3 directional = shader_rgb(
 lighting.directional_rgba, lighting.directional_intensity);
 const Vector3 ambient = shader_rgb(
 lighting.ambient_rgba, lighting.ambient_intensity);
 const Vector3 fog = shader_rgb(lighting.fog_rgba, 1.0);
 const float fog_start = static_cast<float>(lighting.fog_start_m);
 const float fog_end = static_cast<float>(lighting.fog_end_m);
 const float shadow_strength = assets.shadow_map.id > 0
 ? static_cast<float>(lighting.shadow_strength) : 0.0f;
 const int shadow_resolution = std::max(1, assets.shadow_map_resolution);
 const Vector3 robot_light{
 static_cast<float>(frame.body.x_m),
 static_cast<float>(frame.body.z_m + 0.72),
 static_cast<float>(-frame.body.y_m)};
 Vector3 goal_light = robot_light;
 for (const auto& place : frame.places) {
 if (place.reached) continue;
 goal_light = {static_cast<float>(place.x_m), 1.15f,
 static_cast<float>(-place.y_m)};
 break;
 }
 const Vector3 robot_color{0.22f, 0.86f, 0.79f};
 const Vector3 goal_color{1.05f, 0.58f, 0.17f};
 SetShaderValue(assets.lighting,
 GetShaderLocation(assets.lighting, "viewPos"), &camera.position,
 SHADER_UNIFORM_VEC3);
 SetShaderValue(assets.lighting,
 GetShaderLocation(assets.lighting, "lightDirection"), &light_direction,
 SHADER_UNIFORM_VEC3);
 SetShaderValue(assets.lighting,
 GetShaderLocation(assets.lighting, "lightColor"), &directional,
 SHADER_UNIFORM_VEC3);
 SetShaderValue(assets.lighting,
 GetShaderLocation(assets.lighting, "ambient"), &ambient,
 SHADER_UNIFORM_VEC3);
 SetShaderValue(assets.lighting,
 GetShaderLocation(assets.lighting, "fogColor"), &fog,
 SHADER_UNIFORM_VEC3);
 SetShaderValue(assets.lighting,
 GetShaderLocation(assets.lighting, "fogStart"), &fog_start,
 SHADER_UNIFORM_FLOAT);
 SetShaderValue(assets.lighting,
 GetShaderLocation(assets.lighting, "fogEnd"), &fog_end,
 SHADER_UNIFORM_FLOAT);
 SetShaderValue(assets.lighting, assets.shadow_strength_loc,
 &shadow_strength, SHADER_UNIFORM_FLOAT);
 SetShaderValue(assets.lighting, assets.shadow_resolution_loc,
 &shadow_resolution, SHADER_UNIFORM_INT);
 SetShaderValue(assets.lighting,
 GetShaderLocation(assets.lighting, "pointLight0Position"), &robot_light,
 SHADER_UNIFORM_VEC3);
 SetShaderValue(assets.lighting,
 GetShaderLocation(assets.lighting, "pointLight0Color"), &robot_color,
 SHADER_UNIFORM_VEC3);
 SetShaderValue(assets.lighting,
 GetShaderLocation(assets.lighting, "pointLight1Position"), &goal_light,
 SHADER_UNIFORM_VEC3);
 SetShaderValue(assets.lighting,
 GetShaderLocation(assets.lighting, "pointLight1Color"), &goal_color,
 SHADER_UNIFORM_VEC3);
}

// Terrain height tint (low grass -> dry earth -> rock).
Color height_color(double h, double hmax) {
  const double t = hmax > 1e-6 ? std::min(1.0, std::max(0.0, h / hmax)) : 0.0;
  if (t < 0.5) {
    const float k = static_cast<float>(t * 2.0);
    return {static_cast<unsigned char>(126 + 110 * k),
            static_cast<unsigned char>(178 - 20 * k),
            static_cast<unsigned char>(120 - 30 * k), 255};
  }
  const float k = static_cast<float>((t - 0.5) * 2.0);
  return {static_cast<unsigned char>(236 - 60 * k),
          static_cast<unsigned char>(158 - 50 * k),
          static_cast<unsigned char>(90 - 10 * k), 255};
}

// Flat fake shadow: a dark translucent disc under bodies/props. No shadow
// map, no shader — 80% of the depth cue for 1% of the cost.
inline void blob_shadow_3d(double x_m, double y_m, double r_m,
 double ground_z_m = 0.0) {
 DrawCylinder({static_cast<float>(x_m), static_cast<float>(ground_z_m + 0.012),
                static_cast<float>(-y_m)},
               static_cast<float>(r_m), static_cast<float>(r_m), 0.012f, 20,
               {20, 20, 25, 50});
}

inline void box_shadow_3d(double x0_m, double y0_m, double x1_m,
 double y1_m, double ground_z_m = 0.0) {
 constexpr float kShadowMarginM = 0.06f;
 const Vector3 center{
 static_cast<float>((x0_m + x1_m) * 0.5),
 static_cast<float>(ground_z_m + 0.004),
 static_cast<float>(-(y0_m + y1_m) * 0.5)};
 const Vector3 size{
 static_cast<float>(std::abs(x1_m - x0_m)) + 2.0f * kShadowMarginM,
 0.008f,
 static_cast<float>(std::abs(y1_m - y0_m)) + 2.0f * kShadowMarginM};
 DrawCubeV(center, size, {20, 20, 25, 45});
}

void draw_model_quaternion(const Model& model, Vector3 position,
 Quaternion rotation, Vector3 scale, Color tint) {
 Vector3 axis{0.0f, 1.0f, 0.0f};
 float angle = 0.0f;
 QuaternionToAxisAngle(rotation, &axis, &angle);
 DrawModelEx(model, position, axis, angle * RAD2DEG, scale, tint);
}

Vector3 link_position(const FrameLinkPose& pose) {
 return {static_cast<float>(pose.x_m), static_cast<float>(pose.z_m),
 static_cast<float>(-pose.y_m)};
}

// MuJoCo links use x-forward/y-left/z-up. Model::transform converts each
// declared file frame into raylib's x-forward/y-up/z-right basis. Live link
// rotations therefore use Q*R_link*Q^-1 after that fixed conversion.
Quaternion link_rotation(const FrameLinkPose& pose, bool converted_asset) {
 const Quaternion q_link{static_cast<float>(pose.qx),
 static_cast<float>(pose.qy), static_cast<float>(pose.qz),
 static_cast<float>(pose.qw)};
 const Quaternion q_basis = QuaternionFromAxisAngle(
 {1.0f, 0.0f, 0.0f}, -PI * 0.5f);
 if (!converted_asset) return QuaternionMultiply(q_basis, q_link);
 return QuaternionMultiply(q_basis,
 QuaternionMultiply(q_link, QuaternionInvert(q_basis)));
}

void draw_actor_model_physical(const Model& model,
 const AssetDefinition& definition, const FrameLinkPose& world_pose,
 double radius_m) {
 if (!model_ready(model) || !world_pose.valid ||
 !(definition.reference_radius_m > 0.0)) return;
 const float scale = static_cast<float>(radius_m /
 std::max(1e-3, definition.reference_radius_m));
 const BoundingBox bounds = raylib_asset_bounds(model);
 const Vector3 position{
 static_cast<float>(world_pose.x_m),
 static_cast<float>(world_pose.z_m - bounds.min.y * scale),
 static_cast<float>(-world_pose.y_m)};
 draw_model_quaternion(model, position, link_rotation(world_pose, true),
 {scale, scale, scale}, WHITE);
}

// Component assets are authored in their MuJoCo link frame. Unlike a whole
// actor asset, their lowest vertex is not necessarily the ground contact, so
// grounding them by mesh bounds would move a chassis or mast by its visual
// clearance. The manifest pivot/translation and the live link pose are the
// complete placement contract.
void draw_component_model_physical(const Model& model,
 const AssetDefinition& definition, const FrameLinkPose& link_pose,
 double actor_radius_m) {
 if (!model_ready(model) || !link_pose.valid ||
 !(definition.reference_radius_m > 0.0)) return;
 const float scale = static_cast<float>(actor_radius_m /
 std::max(1e-3, definition.reference_radius_m));
 draw_model_quaternion(model, link_position(link_pose),
 link_rotation(link_pose, true), {scale, scale, scale}, WHITE);
}

struct ForkliftRenderParts {
 const Model* base = nullptr;
 const AssetDefinition* base_definition = nullptr;
 const Model* drive_wheel = nullptr;
 const AssetDefinition* drive_wheel_definition = nullptr;
 const Model* steer_wheel = nullptr;
 const AssetDefinition* steer_wheel_definition = nullptr;
 const Model* mast = nullptr;
 const AssetDefinition* mast_definition = nullptr;
 const Model* carriage = nullptr;
 const AssetDefinition* carriage_definition = nullptr;
 const Model* fork = nullptr;
 const AssetDefinition* fork_definition = nullptr;
};

ForkliftRenderParts forklift_render_parts(const VisualAssets& assets) {
 return {&assets.forklift_base, assets.forklift_base_definition,
  &assets.forklift_drive_wheel, assets.forklift_drive_wheel_definition,
  &assets.forklift_steer_wheel, assets.forklift_steer_wheel_definition,
  &assets.forklift_mast, assets.forklift_mast_definition,
  &assets.forklift_carriage, assets.forklift_carriage_definition,
  &assets.forklift_fork, assets.forklift_fork_definition};
}

std::pair<const Model*, const AssetDefinition*>
forklift_link_asset(const ForkliftRenderParts& parts, std::uint8_t raw_kind) {
 const std::pair<const Model*, const AssetDefinition*> kMissing{
 nullptr, nullptr};
 const ActorLinkKind kind = static_cast<ActorLinkKind>(raw_kind);
 switch (kind) {
 case ActorLinkKind::kForkliftDriveWheelLeft:
 case ActorLinkKind::kForkliftDriveWheelRight: {
 return {parts.drive_wheel, parts.drive_wheel_definition};
 }
 case ActorLinkKind::kForkliftSteerWheelLeft:
 case ActorLinkKind::kForkliftSteerWheelRight: {
 return {parts.steer_wheel, parts.steer_wheel_definition};
 }
 case ActorLinkKind::kForkliftMast: {
 return {parts.mast, parts.mast_definition};
 }
 case ActorLinkKind::kForkliftCarriage: {
 return {parts.carriage, parts.carriage_definition};
 }
 case ActorLinkKind::kForkliftForkLeft:
 case ActorLinkKind::kForkliftForkRight: {
 return {parts.fork, parts.fork_definition};
 }
 }
 return kMissing;
}

bool forklift_parts_ready(const ForkliftRenderParts& parts) {
 const auto ready = [](const Model* model,
  const AssetDefinition* definition) {
 return model != nullptr && definition != nullptr && model_ready(*model);
 };
 return ready(parts.base, parts.base_definition) &&
 ready(parts.drive_wheel, parts.drive_wheel_definition) &&
 ready(parts.steer_wheel, parts.steer_wheel_definition) &&
 ready(parts.mast, parts.mast_definition) &&
 ready(parts.carriage, parts.carriage_definition) &&
 ready(parts.fork, parts.fork_definition);
}

void draw_forklift_parts(const FrameActor& actor,
 const ForkliftRenderParts& parts, double ground_z_m) {
 if (!forklift_parts_ready(parts) || !actor.world_pose.valid ||
 actor.links.empty()) return;
 draw_component_model_physical(*parts.base, *parts.base_definition,
 actor.world_pose, actor.radius_m);
 for (const auto& link : actor.links) {
 if (!link.valid || !link.pose.valid) continue;
 const auto& asset = forklift_link_asset(parts, link.kind);
 if (asset.first == nullptr || asset.second == nullptr ||
 !model_ready(*asset.first)) continue;
 draw_component_model_physical(*asset.first, *asset.second, link.pose,
 actor.radius_m);
 }
 (void)ground_z_m;
}

// Wheel GLBs are authored at a reference size (RO-1 uses a 100 mm radius
// and the caster a 55 mm radius). Fit the radial and axle dimensions to the
// live plant configuration independently; a future profile therefore cannot
// put a decorative blackwall below the physical contact patch.
Vector3 wheel_asset_scale(const AssetDefinition* definition, double radius_m,
 double half_width_m) {
 if (definition == nullptr || !(definition->reference_radius_m > 0.0) ||
 !(definition->reference_half_width_m > 0.0)) {
 return {1.0f, 1.0f, 1.0f};
 }
 const float radial = static_cast<float>(
 radius_m / definition->reference_radius_m);
 const float axial = static_cast<float>(
 half_width_m / definition->reference_half_width_m);
 // Manifest wheels use X forward, Y axle, Z up. glTF/raylib maps them to
 // X forward, -Z axle, Y up, so the axial scale is Z.
 return {radial, radial, axial};
}

// Presentation companion to the MuJoCo caster tire: a full-radius centre rib
// with smaller shoulders. The same dimensions are used by the physical
// compound geoms, so a GLB wheel cannot hide a flat, high-scrub tire profile.
void draw_crowned_tire(double radius_m, double half_width_m, Color color) {
 const float radius = static_cast<float>(radius_m);
 const float centre_half = static_cast<float>(half_width_m * 0.5);
 const float shoulder_half = static_cast<float>(half_width_m * 0.25);
 const float shoulder_radius =
 std::max(0.001f, radius * 0.90f);
 const float shoulder_offset = centre_half + shoulder_half;
 // Raylib cylinders are Y-axis by default. The caster axle is local Z.
 rlPushMatrix();
 DrawCylinder({0.0f, 0.0f, 0.0f}, radius, radius,
 2.0f * centre_half, 20, color);
 rlTranslatef(0.0f, shoulder_offset, 0.0f);
 DrawCylinder({0.0f, 0.0f, 0.0f}, shoulder_radius, shoulder_radius,
 2.0f * shoulder_half, 20, color);
 rlTranslatef(0.0f, -2.0f * shoulder_offset, 0.0f);
 DrawCylinder({0.0f, 0.0f, 0.0f}, shoulder_radius, shoulder_radius,
 2.0f * shoulder_half, 20, color);
 rlPopMatrix();
}

// DrawModelEx post-multiplies the live rotation after Model::transform. For
// globally converted GLB parts (the caster fork, chassis attachments),
// A=Rx(-90) maps Z-up to raylib Y-up, so a world-frame yaw must be conjugated
// into the asset frame before it is composed with the authored mount.
Quaternion asset_frame_rotation(Quaternion world_rotation) {
 const Quaternion asset_to_world = QuaternionFromAxisAngle(
 {1.0f, 0.0f, 0.0f}, -PI * 0.5f);
 return QuaternionMultiply(
 QuaternionInvert(asset_to_world),
 QuaternionMultiply(world_rotation, asset_to_world));
}

inline Vector3 to_raylib(double x_m, double y_m, double z_m);

void draw_motion_particles_3d(const ParticleSystem& particles,
 const Camera3D& camera, const VisualAssets* assets) {
 BeginBlendMode(BLEND_ALPHA);
 for (const auto& particle : particles.particles()) {
 if (!particle.active || !(particle.lifetime_s > 0.0)) continue;
 const float phase = static_cast<float>(std::clamp(
 particle.age_s / particle.lifetime_s, 0.0, 1.0));
 const float size = static_cast<float>(
 particle.start_size_m +
 (particle.end_size_m - particle.start_size_m) * phase);
 const unsigned char alpha = static_cast<unsigned char>(
 std::clamp(190.0f * (1.0f - phase), 0.0f, 190.0f));
 if (particle.kind == ParticleKind::kImpactSpark) {
 const Vector3 head{
 static_cast<float>(particle.position_m[0]),
 static_cast<float>(particle.position_m[2]),
 static_cast<float>(-particle.position_m[1])};
 const Vector3 tail{
 head.x - static_cast<float>(particle.velocity_mps[0]) * 0.025f,
 head.y - static_cast<float>(particle.velocity_mps[2]) * 0.025f,
 head.z + static_cast<float>(particle.velocity_mps[1]) * 0.025f};
 DrawLine3D(tail, head, Color{255, 176, 54, alpha});
 DrawSphere(head, size, Color{255, 238, 170, alpha});
 continue;
 }
 const bool water = particle.kind == ParticleKind::kWater;
 const Color tint = water
 ? Color{150, 225, 245, alpha}
 : particle.kind == ParticleKind::kMud ? Color{105, 76, 48, alpha}
 : particle.kind == ParticleKind::kTireSmoke
 ? Color{175, 180, 184, alpha}
 : Color{198, 173, 121, alpha};
 const Vector3 position{
 static_cast<float>(particle.position_m[0]),
 static_cast<float>(particle.position_m[2]),
 static_cast<float>(-particle.position_m[1])};
 if (!water && assets != nullptr && assets->dust.id > 0) {
 DrawBillboard(camera, assets->dust, position, size * 2.0f, tint);
 } else if (water) {
 // A droplet's short streak follows its recorded world velocity. This makes
 // shallow splashes readable without inventing a wheel-relative spray axis.
 const Vector3 tail{
 position.x - static_cast<float>(particle.velocity_mps[0]) * 0.035f,
 position.y - static_cast<float>(particle.velocity_mps[2]) * 0.035f,
 position.z + static_cast<float>(particle.velocity_mps[1]) * 0.035f};
 DrawLine3D(tail, position, Color{185, 239, 252, alpha});
 DrawSphere(position, size * 0.60f, tint);
 } else {
 DrawSphere(position, size * 0.65f, tint);
 }
 }
 EndBlendMode();
}

void draw_surface_traces_3d(const SurfaceTraceSystem& traces) {
 BeginBlendMode(BLEND_ALPHA);
 for (const auto& trace : traces.traces()) {
 if (!trace.active || !(trace.lifetime_s > 0.0)) continue;
 const double dx = trace.end_m[0] - trace.start_m[0];
 const double dy = trace.end_m[1] - trace.start_m[1];
 const double length_m = std::hypot(dx, dy);
 if (!(length_m > 1e-9)) continue;
 const double side_x = -dy / length_m * trace.half_width_m;
 const double side_y = dx / length_m * trace.half_width_m;
 const float fade = static_cast<float>(std::clamp(
 1.0 - trace.age_s / trace.lifetime_s, 0.0, 1.0));
 const unsigned char alpha = static_cast<unsigned char>(std::clamp(
 190.0 * trace.intensity * fade, 0.0, 190.0));
 Color color{82, 78, 72, alpha};
 if (trace.kind == SurfaceTraceKind::kDust) {
 color = {156, 128, 82, static_cast<unsigned char>(alpha * 0.72)};
 } else if (trace.kind == SurfaceTraceKind::kMud) {
 color = {91, 60, 38, alpha};
 } else if (trace.kind == SurfaceTraceKind::kWater) {
 color = {78, 166, 188, static_cast<unsigned char>(alpha * 0.54)};
 }
 const Vector3 a = to_raylib(
 trace.start_m[0] + side_x, trace.start_m[1] + side_y,
 trace.start_m[2]);
 const Vector3 b = to_raylib(
 trace.start_m[0] - side_x, trace.start_m[1] - side_y,
 trace.start_m[2]);
 const Vector3 c = to_raylib(
 trace.end_m[0] - side_x, trace.end_m[1] - side_y,
 trace.end_m[2]);
 const Vector3 d = to_raylib(
 trace.end_m[0] + side_x, trace.end_m[1] + side_y,
 trace.end_m[2]);
 DrawTriangle3D(a, b, c, color);
 DrawTriangle3D(a, c, d, color);
 }
 EndBlendMode();
}

struct PlaybackHitRegions {
 std::array<Rectangle, 4> rates{};
 Rectangle turbo{};
};

void map_bounds(const lost_signal::sim::world::World& w, double& x0,
                double& y0, double& x1, double& y1) {
  x0 = y0 = 1e9;
  x1 = y1 = -1e9;
  auto eat = [&](double x, double y) {
    x0 = std::min(x0, x);
    y0 = std::min(y0, y);
    x1 = std::max(x1, x);
    y1 = std::max(y1, y);
  };
  eat(0.0, 0.0);
  for (const auto& b : w.static_boxes()) {
    eat(b.x0, b.y0);
    eat(b.x1, b.y1);
  }
  for (const auto& p : w.places()) eat(p.x_m, p.y_m);
  for (const auto& z : w.material_zones()) {
    eat(z.x0, z.y0);
    eat(z.x1, z.y1);
  }
  for (const auto& p : w.terrain_patches()) {
    eat(p.x0, p.y0);
    eat(p.x1, p.y1);
  }
  if (x1 - x0 < 8.0) {
    x0 -= (8.0 - (x1 - x0)) * 0.5;
    x1 += (8.0 - (x1 - x0)) * 0.5;
  }
  if (y1 - y0 < 8.0) {
    y0 -= (8.0 - (y1 - y0)) * 0.5;
    y1 += (8.0 - (y1 - y0)) * 0.5;
  }
}

Color box_color(const lost_signal::sim::viewer::FrameBox& b,
                bool collision_flash) {
  if (b.selected) return YELLOW;
  if (collision_flash) return RED;
  return DARKGRAY;
}

void draw_frame_2d(const Frame& f, const ViewTransform& view, bool show_lidar,
 bool show_estimate, bool show_truth_debug) {
  // Ground grid, 1 m spacing over the visible range.
  double wx0 = 0.0, wy0 = 0.0, wx1 = 0.0, wy1 = 0.0;
  if (view.screen_to_world(0, 0, wx0, wy0) &&
      view.screen_to_world(static_cast<float>(GetScreenWidth()),
                           static_cast<float>(GetScreenHeight()), wx1, wy1)) {
    const double gx0 = std::floor(std::min(wx0, wx1));
    const double gx1 = std::ceil(std::max(wx0, wx1));
    const double gy0 = std::floor(std::min(wy0, wy1));
    const double gy1 = std::ceil(std::max(wy0, wy1));
    for (double gx = gx0; gx <= gx1; gx += 1.0) {
      const auto a = view.world_to_screen(gx, gy0);
      const auto b = view.world_to_screen(gx, gy1);
      DrawLineV({a.x_px, a.y_px}, {b.x_px, b.y_px}, {220, 220, 220, 255});
    }
    for (double gy = gy0; gy <= gy1; gy += 1.0) {
      const auto a = view.world_to_screen(gx0, gy);
      const auto b = view.world_to_screen(gx1, gy);
      DrawLineV({a.x_px, a.y_px}, {b.x_px, b.y_px}, {220, 220, 220, 255});
    }
  }
  // Ground-material paint + terrain tint (SIM truth overlay, same colors
  // as the 3D frontend below).
  for (const auto& z : f.zones) {
    const auto a = view.world_to_screen(z.x0_m, z.y0_m);
    const auto c = view.world_to_screen(z.x1_m, z.y1_m);
    const float x = std::min(a.x_px, c.x_px), y = std::min(a.y_px, c.y_px);
    const float w = std::abs(c.x_px - a.x_px), h = std::abs(c.y_px - a.y_px);
    DrawRectangle(x, y, w, h, material_color(z.material_id, 70));
  }
  for (const auto& p : f.patches) {
    const auto a = view.world_to_screen(p.x0_m, p.y0_m);
    const auto c = view.world_to_screen(p.x1_m, p.y1_m);
    const float x = std::min(a.x_px, c.x_px), y = std::min(a.y_px, c.y_px);
    const float w = std::abs(c.x_px - a.x_px), h = std::abs(c.y_px - a.y_px);
    DrawRectangle(x, y, w, h, {190, 150, 100, 60});
    DrawRectangleLines(x, y, w, h, {150, 110, 70, 160});
  }
  // Trail (truth history, oldest first).
 if (show_estimate) for (std::size_t i = 1; i < f.trail.size(); ++i) {
    const auto a = view.world_to_screen(f.trail[i - 1].first,
                                        f.trail[i - 1].second);
    const auto b =
        view.world_to_screen(f.trail[i].first, f.trail[i].second);
    DrawLineV({a.x_px, a.y_px}, {b.x_px, b.y_px}, BLUE);
  }
  // Boxes.
  for (const auto& b : f.boxes) {
    const auto a = view.world_to_screen(b.x0_m, b.y0_m);
    const auto c = view.world_to_screen(b.x1_m, b.y1_m);
    const float x = std::min(a.x_px, c.x_px), y = std::min(a.y_px, c.y_px);
    const float w = std::abs(c.x_px - a.x_px), h = std::abs(c.y_px - a.y_px);
    DrawRectangle(x, y, w, h, box_color(b, f.static_collision));
    DrawRectangleLines(x, y, w, h, BLACK);
  }
  // Places (green when the objective driver reports done).
  for (const auto& p : f.places) {
    const auto s = view.world_to_screen(p.x_m, p.y_m);
    const Color col = p.reached ? GREEN : ORANGE;
    DrawCircleV({s.x_px, s.y_px}, 7.0f, col);
    DrawCircleLines(s.x_px, s.y_px, 7.0f, BLACK);
    const float hx = s.x_px + 12.0f * std::cos(static_cast<float>(p.yaw_rad));
    const float hy = s.y_px - 12.0f * std::sin(static_cast<float>(p.yaw_rad));
    DrawLineV({s.x_px, s.y_px}, {hx, hy}, BLACK);
    DrawText(p.place_id.c_str(), static_cast<int>(s.x_px) + 10,
             static_cast<int>(s.y_px) - 18, 12, DARKGRAY);
  }
  // Actors.
  for (const auto& a : f.actors) {
    const auto s = view.world_to_screen(a.x_m, a.y_m);
    DrawCircleV({s.x_px, s.y_px},
                static_cast<float>(a.radius_m * view.scale()), PURPLE);
    DrawCircleLines(s.x_px, s.y_px,
                    static_cast<float>(a.radius_m * view.scale()), BLACK);
  }
 // ROBOT engineering view: cyan is the current anonymous perception track;
 // purple is the same multi-hypothesis forecast used by planning and safety.
 // Ring radius includes the track's explicit uncertainty envelope.
 if (show_estimate) for (const auto& track : f.dynamic_tracks) {
 const Color current = track.prediction_ready
 ? Color{55, 218, 202, 225} : Color{55, 218, 202, 105};
 const Color predicted = track.prediction_ready
 ? Color{168, 85, 247, 185} : Color{168, 85, 247, 75};
 const auto center = view.world_to_screen(track.x_m, track.y_m);
 const float physical_radius = static_cast<float>(
 std::max(0.02, track.radius_m) * view.scale());
 const float occupied_radius = static_cast<float>(
 std::max(0.02, track.radius_m + track.uncertainty_m) * view.scale());
 DrawCircleLines(center.x_px, center.y_px, physical_radius, current);
 DrawCircleLines(center.x_px, center.y_px, occupied_radius,
 Color{current.r, current.g, current.b,
 static_cast<unsigned char>(current.a / 2)});
 const auto velocity = view.world_to_screen(
 track.x_m + track.vx_mps, track.y_m + track.vy_mps);
 DrawLineEx({center.x_px, center.y_px},
 {velocity.x_px, velocity.y_px}, 2.0f, current);
 for (const auto& hypothesis : track.hypotheses) {
 for (std::size_t index = 1; index < hypothesis.points.size(); ++index) {
 if (!track.prediction_ready && index % 2 == 0) continue;
 const auto& previous = hypothesis.points[index - 1];
 const auto& point = hypothesis.points[index];
 const auto a = view.world_to_screen(previous.x_m, previous.y_m);
 const auto b = view.world_to_screen(point.x_m, point.y_m);
 DrawLineEx({a.x_px, a.y_px}, {b.x_px, b.y_px}, 2.0f, predicted);
 if (index % 5 == 0 || index + 1 == hypothesis.points.size()) {
 const float envelope = static_cast<float>(
 std::max(0.02, track.radius_m + point.uncertainty_m) * view.scale());
 DrawCircleLines(b.x_px, b.y_px, envelope,
 Color{predicted.r, predicted.g, predicted.b,
 static_cast<unsigned char>(predicted.a / 2)});
 }
 }
 }
 }
  // Editor selection handles: box corners + selected place ring.
  for (const auto& b : f.boxes) {
    if (!b.selected) continue;
    const double cx[4] = {b.x0_m, b.x1_m, b.x1_m, b.x0_m};
    const double cy[4] = {b.y0_m, b.y0_m, b.y1_m, b.y1_m};
    for (int c = 0; c < 4; ++c) {
      const auto s = view.world_to_screen(cx[c], cy[c]);
      DrawCircleV({s.x_px, s.y_px}, c == b.selected_corner ? 7.0f : 5.0f,
                  c == b.selected_corner ? RED : YELLOW);
      DrawCircleLines(s.x_px, s.y_px, c == b.selected_corner ? 7.0f : 5.0f,
                      BLACK);
    }
  }
  for (const auto& p : f.places) {
    if (!p.selected) continue;
    const auto s = view.world_to_screen(p.x_m, p.y_m);
    DrawCircleLines(s.x_px, s.y_px, 12.0f, RED);
  }
 // Exact local-planner costmap exported by ROBOT. The viewer does not
 // reconstruct inflation policy: it only colors the bounded quantized cells
 // that DWA/TEB actually consumed.
 if (show_estimate && f.decision.local_costmap_resolution_m > 0.0 &&
 f.decision.local_costmap_width > 0 &&
 f.decision.local_costmap_height > 0 &&
 f.decision.local_costmap_cells.size() ==
 static_cast<std::size_t>(f.decision.local_costmap_width) *
 f.decision.local_costmap_height) {
 const double resolution = f.decision.local_costmap_resolution_m;
 for (std::uint32_t gy = 0; gy < f.decision.local_costmap_height; ++gy) {
 for (std::uint32_t gx = 0; gx < f.decision.local_costmap_width; ++gx) {
 const std::uint8_t cell = f.decision.local_costmap_cells[
 static_cast<std::size_t>(gy) * f.decision.local_costmap_width + gx];
 if (cell == 0) continue;
 Color color{};
 if (cell == 255) {
 color = {100, 116, 139, 18};
 } else if (cell == 254) {
 color = {239, 68, 68, 58};
 } else {
 color = {245, 158, 11,
 static_cast<unsigned char>(14 + 54 * cell / 253)};
 }
 const double x0 = f.decision.local_costmap_origin_x_m + gx * resolution;
 const double y0 = f.decision.local_costmap_origin_y_m + gy * resolution;
 const auto a = view.world_to_screen(x0, y0);
 const auto b = view.world_to_screen(x0 + resolution, y0 + resolution);
 DrawRectangleRec(
 {std::min(a.x_px, b.x_px), std::min(a.y_px, b.y_px),
 std::abs(b.x_px - a.x_px) + 0.5f,
 std::abs(b.y_px - a.y_px) + 0.5f},
 color);
 }
 }
 }
 // Lime is the exact final candidate accepted by the validator. It stays
 // distinct from the independent cyan/red reaction-and-braking safety sweep.
 if (show_estimate &&
 f.decision.validator_swept_envelope_radius_m > 0.0 &&
 !f.decision.validator_swept_centerline.empty()) {
 const Color fill{132, 204, 22, 32};
 const Color edge{163, 230, 53, 205};
 const float radius_px = static_cast<float>(
 f.decision.validator_swept_envelope_radius_m * view.scale());
 Vector2 previous{};
 bool have_previous = false;
 Vector2 first{};
 for (const auto& center : f.decision.validator_swept_centerline) {
 const auto projected = view.world_to_screen(center.first, center.second);
 const Vector2 point{projected.x_px, projected.y_px};
 if (!have_previous) first = point;
 if (have_previous) {
 DrawLineEx(previous, point, std::max(1.0f, 2.0f * radius_px), fill);
 DrawLineEx(previous, point, 1.5f, edge);
 }
 previous = point;
 have_previous = true;
 }
 if (have_previous) {
 DrawCircleV(first, radius_px, fill);
 DrawCircleLinesV(first, radius_px, edge);
 DrawCircleV(previous, radius_px, fill);
 DrawCircleLinesV(previous, radius_px, edge);
 }
 }
 // ROBOT scan projected by the recorded ROBOT estimate. Truth projection is
 // deliberately absent from the live overlay.
 if (show_estimate &&
 f.decision.safety_static_envelope_radius_m > 0.0 &&
 !f.decision.safety_static_sweep.empty()) {
 const bool veto = f.decision.safety_state != "NOMINAL";
 const Color fill = veto ? Color{236, 82, 72, 42}
 : Color{55, 218, 202, 32};
 const Color edge = veto ? Color{236, 82, 72, 205}
 : Color{55, 218, 202, 180};
 const float radius_px = static_cast<float>(
 f.decision.safety_static_envelope_radius_m * view.scale());
 Vector2 previous{};
 bool have_previous = false;
 Vector2 first{};
 for (const auto& center : f.decision.safety_static_sweep) {
 const auto projected = view.world_to_screen(center.first, center.second);
 const Vector2 point{projected.x_px, projected.y_px};
 if (!have_previous) first = point;
 if (have_previous) {
 DrawLineEx(previous, point, std::max(1.0f, 2.0f * radius_px), fill);
 DrawLineEx(previous, point, 1.5f, edge);
 }
 previous = point;
 have_previous = true;
 }
 if (have_previous) {
 DrawCircleV(first, radius_px, fill);
 DrawCircleLinesV(first, radius_px, edge);
 DrawCircleV(previous, radius_px, fill);
 DrawCircleLinesV(previous, radius_px, edge);
 }
 }
  // L is the physical sensor view: project the recorded rays from the
  // MuJoCo body pose. The estimate-projected fan is part of E below, so a
  // drifting estimator cannot make the real LiDAR appear to climb a ramp.
  if (show_lidar && f.truth_lidar_valid) {
    const auto o = view.world_to_screen(f.body.sens_x_m, f.body.sens_y_m);
    const float oldest = f.truth_lidar_time_offsets_s.empty() ? 0.0f
        : *std::min_element(f.truth_lidar_time_offsets_s.begin(),
                            f.truth_lidar_time_offsets_s.end());
    const float sweep_span = std::max(1e-6f, -oldest);
    for (std::size_t i = 0; i < f.truth_lidar_hits.size(); ++i) {
      const auto& hit = f.truth_lidar_hits[i];
      const float phase = i < f.truth_lidar_time_offsets_s.size()
          ? std::clamp((f.truth_lidar_time_offsets_s[i] - oldest) /
                           sweep_span,
                       0.0f, 1.0f)
          : 1.0f;
      const auto s = view.world_to_screen(hit.first, hit.second);
      DrawLineV({o.x_px, o.y_px}, {s.x_px, s.y_px},
                {64, 220, 225,
                 static_cast<unsigned char>(35.0f + 105.0f * phase)});
      DrawCircleV({s.x_px, s.y_px}, 1.0f + phase,
                  {95, 245, 235,
                   static_cast<unsigned char>(120.0f + 120.0f * phase)});
    }
  }
  if (show_estimate && f.lidar_valid) {
 const auto o = view.world_to_screen(
 f.observation.projected_sensor_x_m,
 f.observation.projected_sensor_y_m);
    DrawCircleV({o.x_px, o.y_px}, 4.0f, DARKGREEN);
 const float oldest = f.lidar_time_offsets_s.empty() ? 0.0f
 : *std::min_element(f.lidar_time_offsets_s.begin(),
 f.lidar_time_offsets_s.end());
 const float sweep_span = std::max(1e-6f, -oldest);
 for (std::size_t i = 0; i < f.lidar_hits.size(); ++i) {
 const auto& hit = f.lidar_hits[i];
 const float phase = i < f.lidar_time_offsets_s.size()
 ? std::clamp((f.lidar_time_offsets_s[i] - oldest) / sweep_span,
 0.0f, 1.0f)
 : 1.0f;
 const auto alpha = static_cast<unsigned char>(35.0f + 105.0f * phase);
      const auto s = view.world_to_screen(hit.first, hit.second);
 DrawLineV({o.x_px, o.y_px}, {s.x_px, s.y_px},
 {64, 220, 225, alpha});
 DrawCircleV({s.x_px, s.y_px}, 1.0f + phase,
 {95, 245, 235, static_cast<unsigned char>(120.0f + 120.0f * phase)});
    }
  }
  if (show_truth_debug && !show_lidar && f.truth_lidar_valid) {
 const auto o = view.world_to_screen(f.body.sens_x_m, f.body.sens_y_m);
 const float oldest = f.truth_lidar_time_offsets_s.empty() ? 0.0f
 : *std::min_element(f.truth_lidar_time_offsets_s.begin(),
 f.truth_lidar_time_offsets_s.end());
 const float sweep_span = std::max(1e-6f, -oldest);
 for (std::size_t i = 0; i < f.truth_lidar_hits.size(); ++i) {
 const auto& hit = f.truth_lidar_hits[i];
 const float phase = i < f.truth_lidar_time_offsets_s.size()
 ? std::clamp((f.truth_lidar_time_offsets_s[i] - oldest) / sweep_span,
 0.0f, 1.0f)
 : 1.0f;
 const auto s = view.world_to_screen(hit.first, hit.second);
 DrawLineV({o.x_px, o.y_px}, {s.x_px, s.y_px},
 {246, 169, 61, static_cast<unsigned char>(22.0f + 55.0f * phase)});
 DrawPixelV({s.x_px, s.y_px},
 Color{246, 169, 61,
 static_cast<unsigned char>(100.0f + 110.0f * phase)});
 }
 }
  // ROBOT estimate ghost (offline truth-vs-estimate overlay, live).
  if (show_estimate && f.body.estimate_valid) {
    const auto s = view.world_to_screen(f.body.est_x_m, f.body.est_y_m);
    DrawCircleLines(s.x_px, s.y_px, 9.0f, DARKGREEN);
    const float hx =
        s.x_px + 14.0f * std::cos(static_cast<float>(f.body.est_yaw_rad));
    const float hy =
        s.y_px - 14.0f * std::sin(static_cast<float>(f.body.est_yaw_rad));
    DrawLineV({s.x_px, s.y_px}, {hx, hy}, DARKGREEN);
  }
 if (show_estimate && f.estimate.correction_transition_valid) {
 const auto before = view.world_to_screen(
 f.estimate.before_correction_x_m,
 f.estimate.before_correction_y_m);
 const auto after = view.world_to_screen(
 f.estimate.after_correction_x_m,
 f.estimate.after_correction_y_m);
 const Color prior{246, 169, 61, 230};
 const Color corrected{55, 218, 202, 240};
 DrawCircleLinesV({before.x_px, before.y_px}, 11.0f, prior);
 DrawLineEx({before.x_px, before.y_px}, {after.x_px, after.y_px},
 2.5f, corrected);
 DrawCircleLinesV({after.x_px, after.y_px}, 13.0f, corrected);
 DrawText("ODOM PREDICTION", static_cast<int>(before.x_px + 14.0f),
 static_cast<int>(before.y_px - 18.0f), 11, prior);
 DrawText("MAP CORRECTION", static_cast<int>(after.x_px + 14.0f),
 static_cast<int>(after.y_px + 5.0f), 11, corrected);
 }
  // Body (truth rect + heading).
  {
    const auto s = view.world_to_screen(f.body.x_m, f.body.y_m);
    const float hl =
        static_cast<float>(f.body.half_length_m * view.scale());
    const float hw = static_cast<float>(f.body.half_width_m * view.scale());
    const float yaw = static_cast<float>(f.body.yaw_rad);
    const float c = std::cos(yaw), sn = std::sin(yaw);
    Vector2 corners[4];
    const float lx[4] = {hl, hl, -hl, -hl};
    const float ly[4] = {hw, -hw, -hw, hw};
    for (int i = 0; i < 4; ++i) {
      const float rx = lx[i] * c - ly[i] * sn;
      const float ry = lx[i] * sn + ly[i] * c;
      corners[i] = {s.x_px + rx, s.y_px - ry};
    }
    DrawTriangleFan(corners, 4, f.static_collision ? RED : SKYBLUE);
    for (int i = 0; i < 4; ++i) {
      DrawLineV(corners[i], corners[(i + 1) % 4], DARKBLUE);
    }
    const float hx = s.x_px + (hl + 10.0f) * c;
    const float hy = s.y_px - (hl + 10.0f) * sn;
    DrawLineV({s.x_px, s.y_px}, {hx, hy}, DARKBLUE);
  }
  // Wheels (simplified, default on): short segments at the module
  // positions, steered by the live joint (ackermann fronts read visibly),
 // red in explicit SIM truth debug while the engine flags traction
 // saturation (per side on tracks).
  // Screen is y-flipped: world angle t draws along (cos t, -sin t).
  {
    const float yaw = static_cast<float>(f.body.yaw_rad);
    const float c = std::cos(yaw), sn = std::sin(yaw);
    const float sc = static_cast<float>(view.scale());
    const auto bs = view.world_to_screen(f.body.x_m, f.body.y_m);
    auto to_screen = [&](double bx, double by) {
      return Vector2{bs.x_px + static_cast<float>(bx) * c * sc -
                         static_cast<float>(by) * sn * sc,
                     bs.y_px - (static_cast<float>(bx) * sn * sc +
                               static_cast<float>(by) * c * sc)};
    };
    if (f.body.tracked) {
      // Skid-steer slabs: side lines, red per side from the drive flags.
      for (int k = 0; k < 2; ++k) {
        const double wy = (k == 0 ? 1.0 : -1.0) * f.body.half_width_m;
        const Vector2 a = to_screen(-f.body.half_length_m, wy);
        const Vector2 b = to_screen(f.body.half_length_m, wy);
 const bool slip = show_truth_debug &&
 k < static_cast<int>(f.body.wheels.size()) &&
 f.body.wheels[static_cast<std::size_t>(k)].slipping;
        DrawLineEx(a, b, 3.0f, slip ? RED : DARKGRAY);
      }
    } else {
      for (const auto& w : f.body.wheels) {
        const Vector2 m = to_screen(w.x_m, w.y_m);
        const float t =
            yaw + static_cast<float>(w.steer_rad);  // steered heading
        const float half =
            static_cast<float>(w.radius_m * 2.0 * sc * 0.5);
        const Vector2 a{m.x - half * std::cos(t),
                        m.y + half * std::sin(t)};
        const Vector2 b{m.x + half * std::cos(t),
                        m.y - half * std::sin(t)};
 DrawLineEx(a, b, 3.0f,
 show_truth_debug && w.slipping ? RED : DARKGRAY);
      }
    }
  }
}

// 3D frontend over the same Frame model. World (x, y, z_up) maps to raylib
// (x, z_up, -y): +y north goes away from the default camera, right-handed.
Camera3D camera_from_state(const CameraState& st) {
  const float yaw = st.yaw_deg * DEG2RAD;
  const float pitch = st.pitch_deg * DEG2RAD;
  Vector3 pos{static_cast<float>(st.target_x + st.dist_m * std::cos(pitch) * std::cos(yaw)),
              static_cast<float>(st.dist_m * std::sin(pitch)),
              static_cast<float>(-(st.target_y + st.dist_m * std::cos(pitch) * std::sin(yaw)))};
  return {.position = pos,
 .target = to_raylib(st.target_x, st.target_y, st.target_z),
          .up = {0.0f, 1.0f, 0.0f},
          .fovy = 45.0f,
          .projection = CAMERA_PERSPECTIVE};
}

void draw_baseline_comparison_ghost_2d(
 const Frame& baseline, const ViewTransform& view, bool event_aligned) {
 const auto center = view.world_to_screen(
 baseline.body.x_m, baseline.body.y_m);
 const float radius = static_cast<float>(
 std::hypot(baseline.body.half_length_m, baseline.body.half_width_m) *
 view.scale());
 const Color amber{246, 169, 61, 220};
 DrawCircleLinesV({center.x_px, center.y_px}, radius, amber);
 const Vector2 heading{
 center.x_px + radius * std::cos(static_cast<float>(baseline.body.yaw_rad)),
 center.y_px - radius * std::sin(static_cast<float>(baseline.body.yaw_rad))};
 DrawLineEx({center.x_px, center.y_px}, heading, 2.0f, amber);
 const char* label = event_aligned
 ? "BASELINE SIM / SAME EVENT" : "BASELINE SIM / SAME TIME";
 DrawText(label, static_cast<int>(center.x_px + radius + 6.0f),
 static_cast<int>(center.y_px - 8.0f), 12, amber);
}

void draw_baseline_comparison_ghost_3d(
 const Frame& baseline, const Camera3D& camera, bool event_aligned) {
 const float radius = static_cast<float>(std::hypot(
 baseline.body.half_length_m, baseline.body.half_width_m));
 const Vector3 center = to_raylib(
 baseline.body.x_m, baseline.body.y_m, 0.025);
 const Vector3 heading = to_raylib(
 baseline.body.x_m + radius * std::cos(baseline.body.yaw_rad),
 baseline.body.y_m + radius * std::sin(baseline.body.yaw_rad), 0.025);
 const Color amber{246, 169, 61, 220};
 BeginMode3D(camera);
 DrawCircle3D(center, radius, {1.0f, 0.0f, 0.0f}, 90.0f, amber);
 DrawLine3D(center, heading, amber);
 DrawSphereWires(center, 0.045f, 8, 8, amber);
 EndMode3D();
 const Vector2 label = GetWorldToScreen(center, camera);
 const char* label_text = event_aligned
 ? "BASELINE SIM / SAME EVENT" : "BASELINE SIM / SAME TIME";
 DrawText(label_text, static_cast<int>(label.x + 8.0f),
 static_cast<int>(label.y - 10.0f), 12, amber);
}

Vector3 body_local_to_world(const FrameBody& body, const FrameLinkPose& local) {
 const double cr = std::cos(body.roll_rad), sr = std::sin(body.roll_rad);
 const double cp = std::cos(body.pitch_rad), sp = std::sin(body.pitch_rad);
 const double cy = std::cos(body.yaw_rad), sy = std::sin(body.yaw_rad);
 const double x = body.x_m + cy * cp * local.x_m +
 (cy * sp * sr - sy * cr) * local.y_m +
 (cy * sp * cr + sy * sr) * local.z_m;
 const double y = body.y_m + sy * cp * local.x_m +
 (sy * sp * sr + cy * cr) * local.y_m +
 (sy * sp * cr - cy * sr) * local.z_m;
 const double z = body.z_m - sp * local.x_m + cp * sr * local.y_m +
 cp * cr * local.z_m;
 return to_raylib(x, y, z);
}

WalkaroundView walkaround_view(const Frame& frame,
 const std::string& component_id, CameraState state) {
 double focus_x = frame.body.x_m;
 double focus_y = frame.body.y_m;
 double focus_z = frame.body.z_m + 0.08;
 state.dist_m = 1.55f;
 state.pitch_deg = 27.0f;
 if (component_id == "drive_wheel" && !frame.body.wheels.empty() &&
 frame.body.wheels.front().pose.valid) {
 state.yaw_deg = -100.0f;
 // Pick the driven wheel on the camera-facing side. Focusing the far-side
 // wheel projected the clickable hotspot through the chassis deck and taught
 // the wrong part even though the wheel pose itself was physically correct.
 const bool camera_on_positive_y = std::sin(state.yaw_deg * DEG2RAD) > 0.0;
 const FrameWheel* visible_wheel = nullptr;
 for (const auto& wheel : frame.body.wheels) {
 if (!wheel.driven || !wheel.pose.valid) continue;
 if (visible_wheel == nullptr ||
 (camera_on_positive_y
 ? wheel.pose.y_m > visible_wheel->pose.y_m
 : wheel.pose.y_m < visible_wheel->pose.y_m)) {
 visible_wheel = &wheel;
 }
 }
 if (visible_wheel == nullptr) visible_wheel = &frame.body.wheels.front();
 const Vector3 p = body_local_to_world(frame.body, visible_wheel->pose);
 focus_x = p.x;
 focus_y = -p.z;
 focus_z = p.y;
 // Show the selected tire together with its axle, chassis, rear support and
 // sensor mast. The previous close crop hid the mechanical relationships
 // that the lesson is meant to teach.
 state.dist_m = 1.75f;
 state.pitch_deg = 23.0f;
 } else if (component_id == "caster" && !frame.body.casters.empty() &&
 frame.body.casters.front().wheel_pose.valid) {
 const Vector3 p = body_local_to_world(
 frame.body, frame.body.casters.front().wheel_pose);
 focus_x = p.x;
 focus_y = -p.z;
 focus_z = p.y;
 state.dist_m = 0.82f;
 state.pitch_deg = 16.0f;
 state.yaw_deg = 20.0f;
 } else if (component_id == "lidar") {
 focus_x = frame.body.sens_x_m;
 focus_y = frame.body.sens_y_m;
 focus_z = frame.body.sens_z_m;
 state.dist_m = 0.82f;
 state.pitch_deg = 8.0f;
 state.yaw_deg = 35.0f;
  } else if (component_id.rfind("arm_joint_", 0) == 0) {
    // Arm links are relative to the MuJoCo base frame.  Resolve the
    // requested joint/link from the recorded pose table so the teaching
    // camera follows the same articulated state as the renderer.  Unknown
    // or unavailable links fall back to the chassis without inventing a
    // presentation pose.
    const std::string ordinal = component_id.substr(10);
    char* end = nullptr;
    const long requested = std::strtol(ordinal.c_str(), &end, 10);
    if (end != ordinal.c_str() && *end == '\0' && requested >= 1 &&
        requested <= static_cast<long>(frame.body.arm_links.size())) {
      const auto& link = frame.body.arm_links[
          static_cast<std::size_t>(requested - 1)];
      if (link.valid && link.pose.valid) {
        const Vector3 p = body_local_to_world(frame.body, link.pose);
        focus_x = p.x;
        focus_y = -p.z;
        focus_z = p.y;
        state.dist_m = 0.72f;
        state.pitch_deg = 18.0f;
        state.yaw_deg = 28.0f;
      }
    }
  } else if (component_id == "com" && frame.body.com_z_m != 0.0) {
    FrameLinkPose com;
    com.valid = true;
    com.x_m = frame.body.com_x_m;
    com.y_m = frame.body.com_y_m;
    com.z_m = frame.body.com_z_m;
    const Vector3 p = body_local_to_world(frame.body, com);
    focus_x = p.x;
    focus_y = -p.z;
    focus_z = p.y;
    state.dist_m = 0.95f;
    state.pitch_deg = 22.0f;
    state.yaw_deg = 34.0f;
 } else if (component_id == "mission") {
 const FramePlace* target = nullptr;
 for (const auto& place : frame.places) {
 if (!place.reached) {
 target = &place;
 break;
 }
 }
 if (target != nullptr) {
 const double route_span_m = std::hypot(
 target->x_m - frame.body.x_m, target->y_m - frame.body.y_m);
 focus_x = frame.body.x_m + 0.42 * (target->x_m - frame.body.x_m);
 focus_y = frame.body.y_m + 0.42 * (target->y_m - frame.body.y_m);
 focus_z = 0.18;
 state.dist_m = static_cast<float>(std::clamp(
 3.5 + 0.55 * route_span_m, 4.8, 8.5));
 state.pitch_deg = 31.0f;
 state.yaw_deg = 38.0f;
 }
 }
 state.target_x = focus_x;
 state.target_y = focus_y;
 state.target_z = focus_z;
 // Compose the highlighted part in the clear right-hand viewport.
 const double yaw_rad = state.yaw_deg * DEG2RAD;
 const double optical_offset_m = 0.18 * std::min(1.0f, state.dist_m);
 state.target_x += optical_offset_m * std::sin(yaw_rad);
 state.target_y -= optical_offset_m * std::cos(yaw_rad);
 return {camera_from_state(state), to_raylib(focus_x, focus_y, focus_z)};
}

Camera3D blend_camera(Camera3D from, Camera3D to, float amount) {
 const float t = std::clamp(amount, 0.0f, 1.0f);
 const float smooth = t * t * (3.0f - 2.0f * t);
 const auto blend = [smooth](float a, float b) {
 return a + (b - a) * smooth;
 };
 from.position = {
 blend(from.position.x, to.position.x),
 blend(from.position.y, to.position.y),
 blend(from.position.z, to.position.z)};
 from.target = {
 blend(from.target.x, to.target.x),
 blend(from.target.y, to.target.y),
 blend(from.target.z, to.target.z)};
 from.up = {
 blend(from.up.x, to.up.x),
 blend(from.up.y, to.up.y),
 blend(from.up.z, to.up.z)};
 from.fovy = blend(from.fovy, to.fovy);
 return from;
}

// Ground-plane pick for 3D edit mode: intersect the mouse ray with z_up=0.
bool screen_to_ground_3d(const Camera3D& cam, Vector2 mouse,
                                double& x_m, double& y_m) {
  const Ray ray = GetMouseRay(mouse, cam);
  if (std::abs(ray.direction.y) < 1e-6f) return false;
  const float t = -ray.position.y / ray.direction.y;
  if (t < 0.0f) return false;
  x_m = ray.position.x + t * ray.direction.x;
  y_m = -(ray.position.z + t * ray.direction.z);
  return true;
}

void draw_goal_beacon_3d(
 const lost_signal::sim::viewer::FramePlace& place, double ground_z_m,
 std::uint64_t step) {
 const double time_s = static_cast<double>(step) * 0.02;
 const float wave = 0.5f + 0.5f * static_cast<float>(
 std::sin(time_s * 2.8 + place.x_m * 0.37 + place.y_m * 0.19));
 const Color signal = place.reached
 ? Color{66, 224, 132, 255} : Color{255, 174, 62, 255};
 const Vector3 base = to_raylib(place.x_m, place.y_m, ground_z_m + 0.012);

 BeginBlendMode(BLEND_ALPHA);
 DrawCylinderEx(
 to_raylib(place.x_m, place.y_m, ground_z_m + 0.008),
 to_raylib(place.x_m, place.y_m, ground_z_m + 0.022),
 0.46f, 0.46f, 40, Color{18, 26, 27, 210});
 DrawCylinderEx(
 to_raylib(place.x_m, place.y_m, ground_z_m + 0.023),
 to_raylib(place.x_m, place.y_m, ground_z_m + 0.030),
 0.38f, 0.38f, 40, {signal.r, signal.g, signal.b, 82});
 DrawCircle3D(base, 0.46f, {1.0f, 0.0f, 0.0f}, 90.0f,
 {signal.r, signal.g, signal.b, 150});
 DrawCircle3D(base, 0.58f + wave * 0.10f, {1.0f, 0.0f, 0.0f}, 90.0f,
 {signal.r, signal.g, signal.b,
 static_cast<unsigned char>(90.0f * (1.0f - 0.45f * wave))});
 DrawCylinderEx(
 to_raylib(place.x_m, place.y_m, ground_z_m + 0.03),
 to_raylib(place.x_m, place.y_m,
 ground_z_m + (place.reached ? 0.72 : 1.28)),
 0.028f, 0.012f, 12,
 {signal.r, signal.g, signal.b,
 static_cast<unsigned char>(place.reached ? 52 : 78)});

 const double forward_x = std::cos(place.yaw_rad);
 const double forward_y = std::sin(place.yaw_rad);
 const double side_x = -forward_y;
 const double side_y = forward_x;
 const double arrow_z = ground_z_m + 0.026;
 const Vector3 tip = to_raylib(
 place.x_m + 0.57 * forward_x,
 place.y_m + 0.57 * forward_y, arrow_z);
 const Vector3 left = to_raylib(
 place.x_m - 0.10 * forward_x + 0.18 * side_x,
 place.y_m - 0.10 * forward_y + 0.18 * side_y, arrow_z);
 const Vector3 right = to_raylib(
 place.x_m - 0.10 * forward_x - 0.18 * side_x,
 place.y_m - 0.10 * forward_y - 0.18 * side_y, arrow_z);
 DrawTriangle3D(tip, right, left,
 {signal.r, signal.g, signal.b, 175});

 const float orb_z = static_cast<float>(
 ground_z_m + (place.reached ? 0.72 : 0.88) + wave * 0.08);
 DrawSphere({base.x, orb_z, base.z}, place.reached ? 0.09f : 0.12f,
 {signal.r, signal.g, signal.b, 225});
 DrawSphereWires({base.x, orb_z, base.z},
 0.16f + wave * 0.035f, 8, 12,
 {signal.r, signal.g, signal.b, 150});
 EndBlendMode();
}

// Engine collision geom is expanded in the same display basis as all other
// world geometry by the headless-tested pure helper.
void draw_collision_geom_wireframe(const FrameGeom& geom) {
 const Color tint{226, 74, 210, 200};
 const auto wires =
 lost_signal::sim::viewer::build_collision_geom_wireframe(geom);
 for (const auto& line : wires.segments) {
 DrawLine3D({line.start.x, line.start.y, line.start.z},
 {line.end.x, line.end.y, line.end.z}, tint);
 }
 for (const auto& sphere : wires.spheres) {
 DrawSphereWires({sphere.center.x, sphere.center.y, sphere.center.z},
 sphere.radius_m, 8, 8, tint);
 }
 for (const auto& cylinder : wires.cylinders) {
 DrawCylinderWiresEx(
 {cylinder.start.x, cylinder.start.y, cylinder.start.z},
 {cylinder.end.x, cylinder.end.y, cylinder.end.z},
 cylinder.radius_m, cylinder.radius_m, 12, tint);
 }
}

void draw_frame_3d(const Frame& f,
                   const lost_signal::sim::world::World& world,
 const Camera3D& camera, const ParticleSystem& particles,
 const SurfaceTraceSystem& surface_traces,
 bool show_lidar, bool show_estimate,
 bool show_actor_routes, bool show_places,
 const VisualAssets* assets,
 const SceneManifest* scene,
 const lost_signal::sim::viewer::LidarMount* lidar_mount,
 bool show_physics_overlay,
 bool presentation_effects,
 // Fast/direct rendering has no shadow map, so it may use the inexpensive
 // contact-disc fallback. High-quality rendering passes this explicitly as
 // false and owns all shadows through the physical shadow map.
 bool draw_fast_shadow_discs) {
  namespace world_ns = lost_signal::sim::world;
  // High quality rendering owns shadows through the MuJoCo driven shadow
  // map.  The old path also drew a translucent ground disc for every body;
  // that second projection is intentionally retained only for the direct
  // fast path, where no shadow map exists.  Drawing both makes the disc drift
  // away from the physical shadow whenever the light or body moves.
  const bool draw_contact_shadow_disc =
      presentation_effects && draw_fast_shadow_discs;
  // Ground base sized to the authored map (concrete apron under everything).
  double gx0, gy0, gx1, gy1;
  map_bounds(world, gx0, gy0, gx1, gy1);
  {
    const float w = static_cast<float>(gx1 - gx0 + 8.0);
    const float h = static_cast<float>(gy1 - gy0 + 8.0);
 const auto& rgba = scene != nullptr ? scene->floor.base_rgba
 : std::array<int, 4>{198, 203, 196, 255};
 const std::string floor_material =
     scene != nullptr ? scene->floor.material_id : "paved";
 const auto* floor_surface = floor_surface_for(assets, floor_material);
 if (floor_surface != nullptr && model_ready(floor_surface->model)) {
   // The floor mesh is deliberately unit-sized and scaled to the authored
   // map.  Drive UVs from world position so a 200 km-class map keeps the
   // same readable material grain as a small teaching scene.
   const float floor_texture_tiling[2] = {0.5f, 0.5f};
   if (assets != nullptr && assets->floor_texture_tiling_loc >= 0) {
     SetShaderValue(assets->lighting, assets->floor_texture_tiling_loc,
                    floor_texture_tiling, SHADER_UNIFORM_VEC2);
   }
   DrawModelEx(
       floor_surface->model,
       to_raylib((gx0 + gx1) * 0.5, (gy0 + gy1) * 0.5, -0.001),
       {0.0f, 1.0f, 0.0f}, 0.0f, {w, 1.0f, h}, WHITE);
   if (assets != nullptr && assets->floor_texture_tiling_loc >= 0) {
     const float authored_uvs[2] = {0.0f, 0.0f};
     SetShaderValue(assets->lighting, assets->floor_texture_tiling_loc,
                    authored_uvs, SHADER_UNIFORM_VEC2);
   }
 } else {
   DrawPlane(to_raylib((gx0 + gx1) * 0.5, (gy0 + gy1) * 0.5, -0.001),
             {w, h}, {static_cast<unsigned char>(rgba[0]),
                      static_cast<unsigned char>(rgba[1]),
                      static_cast<unsigned char>(rgba[2]),
                      static_cast<unsigned char>(rgba[3])});
 }
  }
 if (presentation_effects && scene == nullptr) {
 DrawGrid(static_cast<int>(std::max(gx1 - gx0, gy1 - gy0) + 8.0), 1.0f);
 } else if (presentation_effects) {
 const auto& rgba = scene->floor.grid_rgba;
 const Color grid{static_cast<unsigned char>(rgba[0]),
 static_cast<unsigned char>(rgba[1]), static_cast<unsigned char>(rgba[2]),
 static_cast<unsigned char>(rgba[3])};
 const double spacing = scene->floor.grid_spacing_m;
 const double first_x = std::floor((gx0 - 4.0) / spacing) * spacing;
 const double first_y = std::floor((gy0 - 4.0) / spacing) * spacing;
 for (double x = first_x; x <= gx1 + 4.0; x += spacing) {
 DrawLine3D(to_raylib(x, gy0 - 4.0, 0.002),
 to_raylib(x, gy1 + 4.0, 0.002), grid);
 }
 for (double y = first_y; y <= gy1 + 4.0; y += spacing) {
 DrawLine3D(to_raylib(gx0 - 4.0, y, 0.002),
 to_raylib(gx1 + 4.0, y, 0.002), grid);
 }
 }
  // Material zones as translucent paint quads (slick water reads blue,
  // sand/gravel warm — same table as the 2D overlay).
 if (presentation_effects) for (const auto& z : f.zones) {
 const double surface_z = z.surface_effect == "water"
 ? z.fluid_depth_m : 0.005;
    const Vector3 center = to_raylib((z.x0_m + z.x1_m) * 0.5,
 (z.y0_m + z.y1_m) * 0.5, surface_z);
    const Vector3 size{static_cast<float>(std::abs(z.x1_m - z.x0_m)), 0.006f,
                       static_cast<float>(std::abs(z.y1_m - z.y0_m))};
    DrawCubeV(center, size, material_color(z.material_id, 150));
  }
 // Surface-only props are explicit paint/decal authority. Solid props are
 // drawn in the collision-box pass below, replacing only their own proxy.
 if (scene != nullptr && assets != nullptr) {
 for (const auto& instance : scene->instances) {
 if (!instance.surface_only) continue;
 const double ground_z = world_ns::sample_terrain(
 world.terrain_patches(), world.support_slabs(), instance.position_m[0],
 instance.position_m[1]).height_m;
 draw_scene_instance(instance, *assets, ground_z);
 }
 }
  // Terrain patches as a height-tinted mesh sampling the SAME canonical
  // function the engine builds and the survey rasterizes — the slope you
  // see is the slope the robot climbs.
  {
 // Keep presentation work bounded for large worlds. The physics heightfield
 // remains at its authored/tile resolution; the viewer only needs a stable
 // visual mesh and must not turn a 200,000 m² training map into millions of
 // draw calls on every frame. Small patches retain the original 0.5 m detail,
 // while large patches adaptively decimate to at most this many cells per
 // axis. The endpoint is always sampled, so the visible edge still coincides
 // with the canonical terrain function.
 constexpr int kMaxTerrainRenderCellsPerAxis = 160;
 const auto render_steps = [](const auto& patch) {
 const double width = std::max(0.0, patch.x1_m - patch.x0_m);
 const double height = std::max(0.0, patch.y1_m - patch.y0_m);
 return std::pair<double, double>{
 std::max(0.5, width / kMaxTerrainRenderCellsPerAxis),
 std::max(0.5, height / kMaxTerrainRenderCellsPerAxis)};
 };
    double hmax = 0.2;
    for (const auto& p : f.patches) {
 const auto [step_x, step_y] = render_steps(p);
 for (double x = p.x0_m; x < p.x1_m; x += step_x) {
 for (double y = p.y0_m; y < p.y1_m; y += step_y) {
          const auto s = world_ns::sample_terrain(
              world.terrain_patches(), world.support_slabs(), x, y);
          hmax = std::max(hmax, s.height_m);
        }
      }
    }
    for (const auto& p : f.patches) {
 const auto [step_x, step_y] = render_steps(p);
 for (double x = p.x0_m; x < p.x1_m; x += step_x) {
 for (double y = p.y0_m; y < p.y1_m; y += step_y) {
 const double x2 = std::min(x + step_x, p.x1_m);
 const double y2 = std::min(y + step_y, p.y1_m);
          const double h00 = world_ns::sample_terrain(
                                 world.terrain_patches(),
                                 world.support_slabs(), x, y)
                                 .height_m;
          const double h10 = world_ns::sample_terrain(
                                 world.terrain_patches(),
                                 world.support_slabs(), x2, y)
                                 .height_m;
          const double h01 = world_ns::sample_terrain(
                                 world.terrain_patches(),
                                 world.support_slabs(), x, y2)
                                 .height_m;
          const double h11 = world_ns::sample_terrain(
                                 world.terrain_patches(),
                                 world.support_slabs(), x2, y2)
                                 .height_m;
 // Keep the visual surface within 1 mm of the engine's contact surface.
 // A former 12 mm lift made tire blackwalls visibly sink into ramps.
 const double hm = (h00 + h10 + h01 + h11) * 0.25 + 0.001;
          const Color col = height_color(hm, hmax);
          // Winding faces up in raylib space (x, z_up, -y): culling eats
          // downward triangles, so p00 -> p10 -> p01, never the reverse.
 DrawTriangle3D(to_raylib(x, y, h00 + 0.001),
 to_raylib(x2, y, h10 + 0.001),
 to_raylib(x, y2, h01 + 0.001), col);
 DrawTriangle3D(to_raylib(x2, y, h10 + 0.001),
 to_raylib(x2, y2, h11 + 0.001),
 to_raylib(x, y2, h01 + 0.001), col);
        }
      }
    }
  }
  // Trail at ground level.
 if (show_estimate) for (std::size_t i = 1; i < f.trail.size(); ++i) {
    DrawLine3D(to_raylib(f.trail[i - 1].first, f.trail[i - 1].second, 0.02),
               to_raylib(f.trail[i].first, f.trail[i].second, 0.02), BLUE);
  }
 // Boxes as two-tone blocks with footprint-matched contact shadows.
  for (const auto& b : f.boxes) {
    const float h = static_cast<float>(b.height_m > 0.02 ? b.height_m : 0.02);
    const Vector3 center =
        to_raylib((b.x0_m + b.x1_m) * 0.5, (b.y0_m + b.y1_m) * 0.5, h * 0.5);
    const Vector3 size{static_cast<float>(std::abs(b.x1_m - b.x0_m)), h,
                       static_cast<float>(std::abs(b.y1_m - b.y0_m))};
    const Color wall =
        b.selected ? YELLOW
                   : (f.static_collision ? RED
                                         : Color{110, 118, 128, 255});
 const SceneInstance* scene_visual =
 scene != nullptr
 ? scene->visual_for_box(b.box_id)
 : nullptr;
 if (scene_visual != nullptr && assets != nullptr && !b.selected) {
 const double ground_z = world_ns::sample_terrain(
 world.terrain_patches(), world.support_slabs(),
 scene_visual->position_m[0], scene_visual->position_m[1]).height_m;
 draw_scene_instance(*scene_visual, *assets, ground_z);
    if (draw_contact_shadow_disc)
 box_shadow_3d(b.x0_m, b.y0_m, b.x1_m, b.y1_m, ground_z);
 continue;
 }
    DrawCubeV(center, size, wall);
 if (presentation_effects) DrawCubeWiresV(center, size, BLACK);
    // Sun-lit cap: thin slab on top, warm white.
 if (presentation_effects) DrawCubeV(
 to_raylib((b.x0_m + b.x1_m) * 0.5, (b.y0_m + b.y1_m) * 0.5,
                        h + 0.008),
              {size.x, 0.016f, size.z},
              b.selected ? Color{255, 240, 160, 255}
                         : Color{168, 175, 184, 255});
    if (draw_contact_shadow_disc)
 box_shadow_3d(b.x0_m, b.y0_m, b.x1_m, b.y1_m);
    if (b.selected) {
      const double cx[4] = {b.x0_m, b.x1_m, b.x1_m, b.x0_m};
      const double cy[4] = {b.y0_m, b.y0_m, b.y1_m, b.y1_m};
      for (int c = 0; c < 4; ++c) {
        DrawSphere(to_raylib(cx[c], cy[c], h),
                   c == b.selected_corner ? 0.12f : 0.08f,
                   c == b.selected_corner ? RED : YELLOW);
      }
    }
  }
 // Objective markers are GAME presentation: the arrow teaches the authored
 // terminal yaw, while color and pulse show progress. They never enter any
 // observation, estimate, command, collision, or score calculation.
 if (show_places) for (const auto& p : f.places) {
 const double place_ground = world_ns::sample_terrain(
 world.terrain_patches(), world.support_slabs(), p.x_m, p.y_m).height_m;
 draw_goal_beacon_3d(p, place_ground, f.step);
 if (p.selected) DrawSphereWires(to_raylib(p.x_m, p.y_m, place_ground + 0.68), 0.25f, 8,
                                    8, RED);
  }
  // Actors: forklift-orange vehicles, pedestrian-blue walkers, true height,
 // heading tick, blob shadow. Patrol routes are an opt-in SIM diagnostic.
  for (const auto& a : f.actors) {
    const Color body =
        a.pedestrian ? Color{70, 130, 220, 255} : Color{235, 130, 40, 255};
    const float h = static_cast<float>(a.height_m > 0.1 ? a.height_m : 0.5);
 const bool physical_pose = a.world_pose.valid;
 const double visual_x = physical_pose ? a.world_pose.x_m : a.x_m;
 const double visual_y = physical_pose ? a.world_pose.y_m : a.y_m;
 const double ground_z = physical_pose ? a.world_pose.z_m
 : world_ns::sample_terrain(world.terrain_patches(),
 world.support_slabs(), a.x_m, a.y_m).height_m;
 if (draw_contact_shadow_disc)
 blob_shadow_3d(visual_x, visual_y, a.radius_m * 1.3, ground_z);
 const Model* actor_model = nullptr;
 const AssetDefinition* actor_definition = nullptr;
 if (assets != nullptr) actor_model = a.pedestrian ? &assets->worker
 : &assets->forklift;
 if (assets != nullptr) {
 actor_definition = a.pedestrian ? assets->worker_definition
 : assets->forklift_definition;
 }
 bool modular_forklift_ready = false;
 if (!a.pedestrian && assets != nullptr && a.world_pose.valid) {
 const ForkliftRenderParts parts = forklift_render_parts(*assets);
 if (forklift_parts_ready(parts) && !a.links.empty()) {
 draw_forklift_parts(a, parts, ground_z);
 modular_forklift_ready = true;
 }
 }
 const ActorRenderPath render_path = choose_actor_render_path(
 a.pedestrian, modular_forklift_ready, actor_model != nullptr &&
 model_ready(*actor_model), actor_definition != nullptr);
 if (render_path == ActorRenderPath::kFullModel) {
 if (a.pedestrian && assets != nullptr &&
 assets->worker_animations != nullptr &&
 assets->worker_animation_count > 0 &&
 assets->worker_definition != nullptr &&
 assets->worker_definition->animation_stride_m > 0.0) {
 const double speed = std::hypot(a.vx_mps, a.vy_mps);
 const int frame_count = std::max(1,
 assets->worker_animations[0].frameCount);
 int animation_frame = 0;
 if (speed > 1e-6) {
 // Distance based phase is deterministic across pause, replay and Turbo;
 // the actor root pose remains the MuJoCo body pose below.
 const double cycles = f.sim_time_s * speed /
 assets->worker_definition->animation_stride_m;
 const double wrapped = cycles - std::floor(cycles);
 animation_frame = std::min(
 frame_count - 1, static_cast<int>(wrapped * frame_count));
 }
 // The shared raylib Model is updated immediately before each actor draw,
 // so multiple workers can have independent phases without wall-clock state.
 UpdateModelAnimationBones(*actor_model,
 assets->worker_animations[0], animation_frame);
 }
 if (a.world_pose.valid) {
 draw_actor_model_physical(*actor_model, *actor_definition,
 a.world_pose, a.radius_m);
 } else if (render_path == ActorRenderPath::kProceduralFallback) {
 draw_actor_model_grounded(*actor_model, a.x_m, a.y_m, *actor_definition,
 ground_z, a.radius_m, a.heading_rad);
 }
 } else if (render_path == ActorRenderPath::kProceduralFallback) {
 // raylib's DrawCylinder position is the base center (not the geometric
 // center). Passing ground+h/2 made the procedural pedestrian float by half
 // its height; the SIM terrain sample is the sole contact anchor.
 DrawCylinder(to_raylib(visual_x, visual_y, ground_z),
 static_cast<float>(a.radius_m), static_cast<float>(a.radius_m),
 static_cast<float>(h), 16, body);
 }
 DrawCylinder(to_raylib(visual_x, visual_y, ground_z + h + 0.03), 0.06f, 0.06f, 0.12f, 8,
                 DARKGRAY);  // roof beacon
 DrawLine3D(to_raylib(visual_x, visual_y, ground_z + h * 0.5),
 to_raylib(visual_x + a.vx_mps, visual_y + a.vy_mps,
 ground_z + h * 0.5), BLACK);
  }
 // Deployable ROBOT perception overlay. The current anonymous footprint is
 // cyan; predictor hypotheses and their growing uncertainty are purple.
 if (show_estimate) for (const auto& track : f.dynamic_tracks) {
 const Color current = track.prediction_ready
 ? Color{55, 218, 202, 230} : Color{55, 218, 202, 110};
 const Color predicted = track.prediction_ready
 ? Color{168, 85, 247, 190} : Color{168, 85, 247, 78};
 const double ground_z = world_ns::sample_terrain(
 world.terrain_patches(), world.support_slabs(),
 track.x_m, track.y_m).height_m + 0.035;
 const Vector3 center = to_raylib(track.x_m, track.y_m, ground_z);
 DrawCircle3D(center, static_cast<float>(std::max(0.02, track.radius_m)),
 {1.0f, 0.0f, 0.0f}, 90.0f, current);
 DrawCircle3D(center, static_cast<float>(std::max(
 0.02, track.radius_m + track.uncertainty_m)),
 {1.0f, 0.0f, 0.0f}, 90.0f,
 Color{current.r, current.g, current.b,
 static_cast<unsigned char>(current.a / 2)});
 const double velocity_ground_z = world_ns::sample_terrain(
 world.terrain_patches(), world.support_slabs(),
 track.x_m + track.vx_mps, track.y_m + track.vy_mps).height_m + 0.035;
 DrawLine3D(center, to_raylib(
 track.x_m + track.vx_mps, track.y_m + track.vy_mps,
 velocity_ground_z), current);
 for (const auto& hypothesis : track.hypotheses) {
 for (std::size_t index = 1; index < hypothesis.points.size(); ++index) {
 if (!track.prediction_ready && index % 2 == 0) continue;
 const auto& previous = hypothesis.points[index - 1];
 const auto& point = hypothesis.points[index];
 const double previous_z = world_ns::sample_terrain(
 world.terrain_patches(), world.support_slabs(),
 previous.x_m, previous.y_m).height_m + 0.04;
 const double point_z = world_ns::sample_terrain(
 world.terrain_patches(), world.support_slabs(),
 point.x_m, point.y_m).height_m + 0.04;
 const Vector3 a = to_raylib(previous.x_m, previous.y_m, previous_z);
 const Vector3 b = to_raylib(point.x_m, point.y_m, point_z);
 DrawCylinderEx(a, b, 0.012f, 0.012f, 6, predicted);
 if (index % 5 == 0 || index + 1 == hypothesis.points.size()) {
 DrawCircle3D(b, static_cast<float>(std::max(
 0.02, track.radius_m + point.uncertainty_m)),
 {1.0f, 0.0f, 0.0f}, 90.0f,
 Color{predicted.r, predicted.g, predicted.b,
 static_cast<unsigned char>(predicted.a / 2)});
 }
 }
 }
 }
 if (show_actor_routes) for (const auto& a : world.actors()) {
    if (a.behavior.kind !=
            lost_signal::sim::world::ActorBehaviorKind::kWaypoints ||
        a.behavior.waypoints.empty()) {
      continue;
    }
    Vector3 prev = to_raylib(a.x, a.y, 0.03);
    for (const auto& wp : a.behavior.waypoints) {
      const Vector3 cur = to_raylib(wp.x_m, wp.y_m, 0.03);
      DrawLine3D(prev, cur, {150, 100, 220, 180});
      prev = cur;
    }
    if (a.behavior.loop && a.behavior.waypoints.size() > 1) {
      DrawLine3D(prev, to_raylib(a.behavior.waypoints.front().x_m,
                                 a.behavior.waypoints.front().y_m, 0.03),
                 {150, 100, 220, 180});
    }
  }
 // Costmap evidence stays on the rendered ground for legibility. Terrain
 // sampling happens only in SIM presentation and never returns to ROBOT.
 if (show_estimate && f.decision.local_costmap_resolution_m > 0.0 &&
 f.decision.local_costmap_width > 0 &&
 f.decision.local_costmap_height > 0 &&
 f.decision.local_costmap_cells.size() ==
 static_cast<std::size_t>(f.decision.local_costmap_width) *
 f.decision.local_costmap_height) {
 const double resolution = f.decision.local_costmap_resolution_m;
 const auto is_unknown_frontier = [&](std::uint32_t x, std::uint32_t y) {
 const std::uint32_t width = f.decision.local_costmap_width;
 const std::uint32_t height = f.decision.local_costmap_height;
 const int dx[4] = {-1, 1, 0, 0};
 const int dy[4] = {0, 0, -1, 1};
 for (int n = 0; n < 4; ++n) {
 const int nx = static_cast<int>(x) + dx[n];
 const int ny = static_cast<int>(y) + dy[n];
 if (nx < 0 || ny < 0 || nx >= static_cast<int>(width) ||
 ny >= static_cast<int>(height)) continue;
 if (f.decision.local_costmap_cells[
 static_cast<std::size_t>(ny) * width + nx] != 255) return true;
 }
 return false;
 };
 for (std::uint32_t gy = 0; gy < f.decision.local_costmap_height; ++gy) {
 for (std::uint32_t gx = 0; gx < f.decision.local_costmap_width; ++gx) {
 const std::uint8_t cell = f.decision.local_costmap_cells[
 static_cast<std::size_t>(gy) * f.decision.local_costmap_width + gx];
 if (cell == 0 || (cell == 255 && !is_unknown_frontier(gx, gy))) continue;
 Color color{};
 if (cell == 255) {
 color = {100, 116, 139, 42};
 } else if (cell == 254) {
 color = {239, 68, 68, 78};
 } else {
 color = {245, 158, 11,
 static_cast<unsigned char>(20 + 65 * cell / 253)};
 }
 const double cx = f.decision.local_costmap_origin_x_m +
 (gx + 0.5) * resolution;
 const double cy = f.decision.local_costmap_origin_y_m +
 (gy + 0.5) * resolution;
 const double ground_z = world_ns::sample_terrain(
 world.terrain_patches(), world.support_slabs(), cx, cy).height_m + 0.012;
 DrawCubeV(to_raylib(cx, cy, ground_z),
 {static_cast<float>(resolution), 0.006f,
 static_cast<float>(resolution)}, color);
 }
 }
 }
 if (show_estimate &&
 f.decision.validator_swept_envelope_radius_m > 0.0 &&
 !f.decision.validator_swept_centerline.empty()) {
 const float radius = static_cast<float>(
 f.decision.validator_swept_envelope_radius_m);
 const Color fill{132, 204, 22, 34};
 const Color edge{163, 230, 53, 210};
 const std::size_t stride = std::max<std::size_t>(
 1, f.decision.validator_swept_centerline.size() / 20);
 Vector3 previous{};
 bool have_previous = false;
 Vector3 first{};
 for (std::size_t i = 0;
 i < f.decision.validator_swept_centerline.size(); i += stride) {
 const auto& center = f.decision.validator_swept_centerline[i];
 const Vector3 point = to_raylib(center.first, center.second, 0.021);
 if (!have_previous) first = point;
if (have_previous && (previous.x != point.x || previous.z != point.z)) {
 DrawCylinderEx(previous, point, radius, radius, 16, fill);
 DrawLine3D(previous, point, edge);
 }
 previous = point;
 have_previous = true;
 }
 const auto& terminal = f.decision.validator_swept_centerline.back();
 const Vector3 terminal_point = to_raylib(
 terminal.first, terminal.second, 0.021);
 if (have_previous && (previous.x != terminal_point.x ||
 previous.z != terminal_point.z)) {
 DrawCylinderEx(previous, terminal_point, radius, radius, 16, fill);
 DrawLine3D(previous, terminal_point, edge);
 previous = terminal_point;
 }
 if (have_previous) {
 DrawCircle3D(first, radius,
 {1.0f, 0.0f, 0.0f}, 90.0f, edge);
 DrawCircle3D(previous, radius,
 {1.0f, 0.0f, 0.0f}, 90.0f, edge);
 }
 }
 // ROBOT scan projected from the recorded ROBOT estimate. The truth-assisted
 // fan is held separately for an explicitly labeled evaluation view.
 if (show_estimate &&
 f.decision.safety_static_envelope_radius_m > 0.0 &&
 !f.decision.safety_static_sweep.empty()) {
 const bool veto = f.decision.safety_state != "NOMINAL";
 const Color fill = veto ? Color{236, 82, 72, 36}
 : Color{55, 218, 202, 28};
 const Color edge = veto ? Color{236, 82, 72, 210}
 : Color{55, 218, 202, 185};
 const float radius = static_cast<float>(
 f.decision.safety_static_envelope_radius_m);
 const std::size_t stride = std::max<std::size_t>(
 1, f.decision.safety_static_sweep.size() / 16);
 Vector3 previous{};
 bool have_previous = false;
 Vector3 first{};
 for (std::size_t i = 0;
 i < f.decision.safety_static_sweep.size(); i += stride) {
 const auto& center = f.decision.safety_static_sweep[i];
 const Vector3 point = to_raylib(center.first, center.second, 0.018);
 if (!have_previous) first = point;
if (have_previous && (previous.x != point.x || previous.z != point.z)) {
 DrawCylinderEx(previous, point, radius, radius, 16, fill);
 DrawLine3D(previous, point, edge);
 }
 previous = point;
 have_previous = true;
 }
 const auto& terminal = f.decision.safety_static_sweep.back();
 const Vector3 terminal_point = to_raylib(
 terminal.first, terminal.second, 0.018);
 if (have_previous && (previous.x != terminal_point.x ||
 previous.z != terminal_point.z)) {
 DrawCylinderEx(previous, terminal_point, radius, radius, 16, fill);
 DrawLine3D(previous, terminal_point, edge);
 previous = terminal_point;
 }
 if (have_previous) {
 DrawCircle3D(first, radius,
 {1.0f, 0.0f, 0.0f}, 90.0f, edge);
 DrawCircle3D(previous, radius,
 {1.0f, 0.0f, 0.0f}, 90.0f, edge);
 }
 }
  if (show_lidar && f.truth_lidar_valid) {
    const Vector3 o = to_raylib(
        f.body.sens_x_m, f.body.sens_y_m, f.body.sens_z_m);
    const std::size_t stride =
        std::max<std::size_t>(1, f.truth_lidar_hits.size() / 72);
    const float oldest = f.truth_lidar_time_offsets_s.empty() ? 0.0f
        : *std::min_element(f.truth_lidar_time_offsets_s.begin(),
                            f.truth_lidar_time_offsets_s.end());
    const float sweep_span = std::max(1e-6f, -oldest);
    for (std::size_t i = 0; i < f.truth_lidar_hits.size(); i += stride) {
      const auto& hit = f.truth_lidar_hits[i];
      const float phase = i < f.truth_lidar_time_offsets_s.size()
          ? std::clamp((f.truth_lidar_time_offsets_s[i] - oldest) /
                           sweep_span,
                       0.0f, 1.0f)
          : 1.0f;
      const Vector3 endpoint =
          to_raylib(hit.first, hit.second, f.body.sens_z_m);
      DrawLine3D(o, endpoint,
                 {64, 220, 225,
                  static_cast<unsigned char>(25.0f + 70.0f * phase)});
      DrawSphere(endpoint, 0.012f + 0.010f * phase,
                 {95, 245, 235,
                  static_cast<unsigned char>(95.0f + 145.0f * phase)});
    }
  }
  if (show_estimate && f.lidar_valid) {
 const Vector3 o = to_raylib(
 f.observation.projected_sensor_x_m,
 f.observation.projected_sensor_y_m,
 f.observation.projected_sensor_z_m);
    DrawSphereWires(o, 0.055f, 8, 8, DARKGREEN);
 const std::size_t stride =
 std::max<std::size_t>(1, f.lidar_hits.size() / 72);
 const float oldest = f.lidar_time_offsets_s.empty() ? 0.0f
 : *std::min_element(f.lidar_time_offsets_s.begin(),
 f.lidar_time_offsets_s.end());
 const float sweep_span = std::max(1e-6f, -oldest);
 for (std::size_t i = 0; i < f.lidar_hits.size(); i += stride) {
 const auto& hit = f.lidar_hits[i];
 const float phase = i < f.lidar_time_offsets_s.size()
 ? std::clamp((f.lidar_time_offsets_s[i] - oldest) / sweep_span,
 0.0f, 1.0f)
 : 1.0f;
 const Vector3 endpoint =
 to_raylib(hit.first, hit.second,
 f.observation.projected_sensor_z_m);
 DrawLine3D(o, endpoint,
 {64, 220, 225, static_cast<unsigned char>(25.0f + 70.0f * phase)});
 DrawSphere(endpoint, 0.012f + 0.010f * phase,
 {95, 245, 235, static_cast<unsigned char>(95.0f + 145.0f * phase)});
    }
  }
 if (show_actor_routes && !show_lidar && f.truth_lidar_valid) {
 const Vector3 o = to_raylib(
 f.body.sens_x_m, f.body.sens_y_m, f.body.sens_z_m);
 const std::size_t stride =
 std::max<std::size_t>(1, f.truth_lidar_hits.size() / 72);
 const float oldest = f.truth_lidar_time_offsets_s.empty() ? 0.0f
 : *std::min_element(f.truth_lidar_time_offsets_s.begin(),
 f.truth_lidar_time_offsets_s.end());
 const float sweep_span = std::max(1e-6f, -oldest);
 for (std::size_t i = 0; i < f.truth_lidar_hits.size(); i += stride) {
 const auto& hit = f.truth_lidar_hits[i];
 const float phase = i < f.truth_lidar_time_offsets_s.size()
 ? std::clamp((f.truth_lidar_time_offsets_s[i] - oldest) / sweep_span,
 0.0f, 1.0f)
 : 1.0f;
 const Vector3 endpoint =
 to_raylib(hit.first, hit.second, f.body.sens_z_m);
 DrawLine3D(o, endpoint,
 {246, 169, 61, static_cast<unsigned char>(18.0f + 45.0f * phase)});
 DrawSphere(endpoint, 0.010f + 0.006f * phase,
 {246, 169, 61, static_cast<unsigned char>(85.0f + 105.0f * phase)});
 }
 }
  // Estimate ghost: wireframe box at the ROBOT pose.
  if (show_estimate && f.body.estimate_valid) {
    DrawCubeWiresV(
        to_raylib(f.body.est_x_m, f.body.est_y_m, 0.25),
        {static_cast<float>(f.body.half_length_m * 2.0), 0.5f,
         static_cast<float>(f.body.half_width_m * 2.0)},
        DARKGREEN);
  }
 if (show_estimate && f.estimate.correction_transition_valid) {
 const Vector3 before = to_raylib(
 f.estimate.before_correction_x_m,
 f.estimate.before_correction_y_m, 0.26);
 const Vector3 after = to_raylib(
 f.estimate.after_correction_x_m,
 f.estimate.after_correction_y_m, 0.26);
 const Color prior{246, 169, 61, 230};
 const Color corrected{55, 218, 202, 240};
 DrawCubeWiresV(before,
 {static_cast<float>(f.body.half_length_m * 2.0), 0.5f,
 static_cast<float>(f.body.half_width_m * 2.0)}, prior);
 DrawLine3D(before, after, corrected);
 DrawSphereWires(after, 0.08f, 8, 8, corrected);
 }
  // Body: box at engine pose with live attitude. Exact world->raylib
  // rotation M = Q*R*Q' for R = Rz(yaw)*Ry(pitch)*Rx(roll) — no Euler
  // folklore, slopes tilt the viz exactly as the engine holds them. (This
  // also fixes a latent mirror: the old -yaw drew the nose south for a
  // north truth; actors/spheres carry no heading so nobody noticed.)
  {
    const double cr = std::cos(f.body.roll_rad),
                 sr = std::sin(f.body.roll_rad);
    const double cp = std::cos(f.body.pitch_rad),
                 sp = std::sin(f.body.pitch_rad);
    const double cy = std::cos(f.body.yaw_rad),
                 sy = std::sin(f.body.yaw_rad);
    // R = Rz*Ry*Rx, row-major.
    const double r00 = cy * cp, r01 = cy * sp * sr - sy * cr,
                 r02 = cy * sp * cr + sy * sr;
    const double r10 = sy * cp, r11 = sy * sp * sr + cy * cr,
                 r12 = sy * sp * cr - cy * sr;
    const double r20 = -sp, r21 = cp * sr, r22 = cp * cr;
    // M = Q*R*Q' with Q:(x,y,z)->(x,z,-y); column-major for rlMultMatrixf.
    float m[16] = {
        static_cast<float>(r00), static_cast<float>(r20),
        static_cast<float>(-r10), 0.0f,
        static_cast<float>(r02), static_cast<float>(r22),
        static_cast<float>(-r12), 0.0f,
        static_cast<float>(-r01), static_cast<float>(-r21),
        static_cast<float>(r11), 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f};
 const double robot_ground_z = world_ns::sample_terrain(
 world.terrain_patches(), world.support_slabs(), f.body.x_m, f.body.y_m).height_m;
 if (draw_contact_shadow_disc) blob_shadow_3d(f.body.x_m, f.body.y_m,
 std::max(f.body.half_length_m, f.body.half_width_m) * 1.4,
 robot_ground_z);
    rlPushMatrix();
    rlTranslatef(static_cast<float>(f.body.x_m),
 static_cast<float>(f.body.z_m),
                 static_cast<float>(-f.body.y_m));
    rlMultMatrixf(m);
    {
      // Layered shop model (local frame, y up): charcoal skirt, pearl
 // hull, orange deck stripe, lidar mast + puck and headlights. Collision
 // still flashes red; the status LED below follows ROBOT safety state.
      const float bl = static_cast<float>(f.body.half_length_m * 2.0);
      const float bw = static_cast<float>(f.body.half_width_m * 2.0);
      const Color hull = f.static_collision
                             ? RED
                             : Color{238, 241, 244, 255};
 if (assets != nullptr && model_ready(assets->chassis)) {
 const BoundingBox cb = raylib_asset_bounds(assets->chassis);
 const float csx = std::max(1e-3f, cb.max.x - cb.min.x);
 const float csy = std::max(1e-3f, cb.max.y - cb.min.y);
 const float csz = std::max(1e-3f, cb.max.z - cb.min.z);
 // GLB is a body component: preserve the configured footprint and deck
 // height independently instead of fitting it into one sphere.
 const Vector3 chassis_scale{
 std::max(0.01f, bl / csx),
 std::max(0.01f, 0.18f / csy),
 std::max(0.01f, bw / csz)};
 const float cmin_y = cb.min.y;
 const float deck_bottom =
 -static_cast<float>(world.motor_params().body_half_height_m);
 DrawModelEx(assets->chassis,
 {0.0f, deck_bottom - cmin_y * chassis_scale.y, 0.0f},
 {0.0f, 1.0f, 0.0f}, 0.0f, chassis_scale, WHITE);
 } else {
      DrawCube({0, -0.20f, 0}, bl, 0.12f, bw, Color{45, 48, 55, 255});
      DrawCube({0, -0.03f, 0}, bl, 0.24f, bw, hull);
      DrawCubeWires({0, -0.03f, 0}, bl, 0.24f, bw, DARKBLUE);
 }
 const float deck_top = static_cast<float>(world.motor_params().body_half_height_m);
 DrawCube({0, deck_top + 0.01f, 0}, bl * 0.7f, 0.03f, bw * 0.55f,
               Color{235, 120, 35, 255});  // deck stripe
 DrawSphere({bl * 0.5f, -0.02f, bw * 0.30f}, 0.035f,
                 Color{255, 242, 200, 255});  // headlights
      DrawSphere({bl * 0.5f, -0.02f, -bw * 0.30f}, 0.035f,
                 Color{255, 242, 200, 255});
    }
 // The lidar is a mounted component, not a decorative mast. Its origin is
 // the configured sensor mount, so its height stays identical to the sensor
 // rays and the ROBOT TF when a profile changes.
 const double mount_x = lidar_mount != nullptr ? lidar_mount->xyz_m[0] : 0.25;
 const double mount_y = lidar_mount != nullptr ? lidar_mount->xyz_m[1] : 0.0;
 const double mount_z = lidar_mount != nullptr ? lidar_mount->xyz_m[2] : 0.35;
 const double mount_yaw = lidar_mount != nullptr ? lidar_mount->rpy_rad[2] : 0.0;
 const float mast_base = static_cast<float>(world.motor_params().body_half_height_m) + 0.01f;
 const float mast_top = static_cast<float>(mount_z);
 if (mast_top > mast_base) {
 DrawCylinderEx({static_cast<float>(mount_x), mast_base,
 static_cast<float>(-mount_y)},
 {static_cast<float>(mount_x), mast_top,
 static_cast<float>(-mount_y)}, 0.025f, 0.025f, 10,
 Color{45, 48, 55, 255});
 }
 if (assets != nullptr && model_ready(assets->lidar)) {
 DrawModelEx(assets->lidar,
 {static_cast<float>(mount_x), static_cast<float>(mount_z),
 static_cast<float>(-mount_y)},
 {0.0f, 1.0f, 0.0f}, static_cast<float>(-mount_yaw * RAD2DEG),
 {1.0f, 1.0f, 1.0f}, WHITE);
 } else {
 DrawCylinder({static_cast<float>(mount_x), mast_top,
 static_cast<float>(-mount_y)}, 0.075f, 0.075f, 0.07f, 14,
 Color{200, 208, 215, 255});
 }
 const Color status_led = !f.decision.valid
 ? Color{120, 128, 132, 255}
 : f.decision.safety_state == "STOPPED" ? RED
 : f.decision.safety_state == "DEGRADED"
 ? Color{245, 172, 55, 255} : Color{50, 200, 90, 255};
 DrawSphere({static_cast<float>(mount_x),
 static_cast<float>(mount_z + 0.11), static_cast<float>(-mount_y)},
 0.025f, status_led);
 // Passive caster assemblies: swivel and roll come from the MuJoCo joints;
 // the fork pivot and wheel contact remain separate when trail is non-zero.
 for (const auto& caster : f.body.casters) {
 if (caster.fork_pose.valid && caster.wheel_pose.valid) {
 const double caster_half_width = std::min(
 static_cast<double>(world.motor_params().tire_half_width_m),
 caster.radius_m * 0.75);
 const float caster_scale = assets != nullptr &&
 assets->caster_wheel_definition != nullptr &&
 assets->caster_wheel_definition->reference_radius_m > 0.0
 ? static_cast<float>(caster.radius_m /
 assets->caster_wheel_definition->reference_radius_m)
 : 1.0f;
 if (assets != nullptr && model_ready(assets->caster_fork)) {
 draw_model_quaternion(assets->caster_fork,
 link_position(caster.fork_pose),
 link_rotation(caster.fork_pose, true),
 {caster_scale, caster_scale, caster_scale}, WHITE);
 }
 if (assets != nullptr && model_ready(assets->caster_wheel)) {
 draw_model_quaternion(assets->caster_wheel,
 link_position(caster.wheel_pose),
 link_rotation(caster.wheel_pose, true),
 wheel_asset_scale(assets->caster_wheel_definition, caster.radius_m,
 caster_half_width), WHITE);
 } else {
 const Quaternion rotation = QuaternionMultiply(
 link_rotation(caster.wheel_pose, true),
 QuaternionFromAxisAngle({1.0f, 0.0f, 0.0f}, PI * 0.5f));
 Vector3 axis{0.0f, 1.0f, 0.0f};
 float angle = 0.0f;
 QuaternionToAxisAngle(rotation, &axis, &angle);
 rlPushMatrix();
 const Vector3 p = link_position(caster.wheel_pose);
 rlTranslatef(p.x, p.y, p.z);
 rlRotatef(angle * RAD2DEG, axis.x, axis.y, axis.z);
 draw_crowned_tire(caster.radius_m, caster_half_width,
 Color{36, 38, 43, 255});
 rlPopMatrix();
 }
 continue;
 }
 // MuJoCo places the caster wheel below the drive axle by (r-R). Keep the
 // offset in the body frame so slopes and engine body height are preserved;
 // anchoring at an absolute radius made supports detach on terrain.
 const float axle = static_cast<float>(
 -(world.motor_params().wheel_radius_m - caster.radius_m));
 const float caster_lift = 0.0f;
 const double caster_half_width = std::min(
 static_cast<double>(world.motor_params().tire_half_width_m),
 caster.radius_m * 0.75);
 const Vector3 caster_wheel_scale = wheel_asset_scale(
 assets != nullptr ? assets->caster_wheel_definition : nullptr,
 caster.radius_m, caster_half_width);
 const float caster_scale = assets != nullptr &&
 assets->caster_wheel_definition != nullptr &&
 assets->caster_wheel_definition->reference_radius_m > 0.0
 ? static_cast<float>(caster.radius_m /
 assets->caster_wheel_definition->reference_radius_m)
 : 1.0f;
 if (assets != nullptr && model_ready(assets->caster_wheel)) {
 const Quaternion swivel = QuaternionFromAxisAngle(
 {0.0f, 1.0f, 0.0f}, static_cast<float>(caster.yaw_rad));
 // The normalized wheel link uses raylib -Z as its axle. Swivel and roll
 // remain separate live joints; no anonymous fixed mount is required.
 const Quaternion spin = QuaternionFromAxisAngle(
 {0.0f, 0.0f, -1.0f}, static_cast<float>(caster.spin_rad));
 const Quaternion swivel_asset = asset_frame_rotation(swivel);
 if (assets != nullptr && model_ready(assets->caster_fork)) {
 draw_model_quaternion(assets->caster_fork,
 {static_cast<float>(caster.pivot_x_m), axle - 0.30f * axle + caster_lift,
 static_cast<float>(-caster.pivot_y_m)}, swivel_asset,
 {caster_scale, caster_scale, caster_scale}, WHITE);
 }
 draw_model_quaternion(assets->caster_wheel,
 {static_cast<float>(caster.x_m), axle + caster_lift,
 static_cast<float>(-caster.y_m)},
 QuaternionMultiply(swivel, spin),
 caster_wheel_scale, WHITE);
 // The GLB already contains the crowned tread and rotation witness. Do not
 // overlay an analytic cylinder: that duplicate used to create a second
 // black contact patch which appeared embedded when the physical caster was
 // compressed on a slope.
 continue;
 }
 rlPushMatrix();
 rlTranslatef(static_cast<float>(caster.pivot_x_m), axle + caster_lift,
 static_cast<float>(-caster.pivot_y_m));
 rlRotatef(static_cast<float>(caster.yaw_rad * RAD2DEG), 0.0f, 1.0f, 0.0f);
 if (assets != nullptr && model_ready(assets->caster_fork)) {
 DrawModelEx(assets->caster_fork,
 {0.0f, -0.30f * axle, 0.0f}, {0.0f, 1.0f, 0.0f}, 0.0f,
 {caster_scale, caster_scale, caster_scale}, WHITE);
 } else {
 DrawCube({0.0f, 0.075f, 0.0f}, 0.025f, 0.15f, 0.025f,
 Color{65, 68, 74, 255});
 }
 if (assets != nullptr && model_ready(assets->caster_wheel)) {
 rlPushMatrix();
      rlTranslatef(static_cast<float>(-caster.trail_m),
 0.0f, 0.0f);
 rlRotatef(-90.0f, 1.0f, 0.0f, 0.0f);
 rlRotatef(static_cast<float>(caster.spin_rad * RAD2DEG), 0.0f, 0.0f,
 1.0f);
 DrawModelEx(assets->caster_wheel, {0.0f, 0.0f, 0.0f},
 {0.0f, 1.0f, 0.0f}, 0.0f,
 {caster_scale, caster_scale, caster_scale}, WHITE);
 rlPopMatrix();
 } else {
      rlTranslatef(static_cast<float>(-caster.trail_m),
 0.0f, 0.0f);
 rlRotatef(static_cast<float>(caster.spin_rad * RAD2DEG), 0.0f, 0.0f,
 1.0f);
 draw_crowned_tire(caster.radius_m, caster_half_width,
 Color{72, 72, 78, 255});
 }
 rlPopMatrix();
 }
 // Wheels from live joints (drive spin + steer yaw per module, same
    // sources as physics/encoders). A spoke makes spin visible; slipping
    // wheels draw red from the engine traction flags. Mecanum modules get
    // static 45-degree tread stripes (rollers are analytic-wrench state,
    // never meshed — honest decoration, documented in the frame).
    // Skid-steer bases draw track slabs instead (wheels hide under tracks,
    // like the real machine); slab color per side from the same flags.
    if (f.body.tracked) {
      // Track slabs sized from the chassis params; per-side color from
      // the traction flags (wheels[0] = left drive). Local Z = -bodyY, so
      // the left slab draws at negative Z (mirroring would swap the red
      // sides — pinned by the drive convention, not by symmetry).
      const float slab_len =
          static_cast<float>(f.body.half_length_m * 2.0f * 0.9f);
 // Track mesh is attached to the base frame. The engine base origin is the
 // drive axle height, so this fixed local offset keeps the 0.12 m slab
 // tangent to the same wheel contact envelope on ramps.
 const float slab_z = -0.04f;
      for (int s = -1; s <= 1; s += 2) {
 const bool slip = show_actor_routes && (s > 0
 ? (f.body.wheels.size() > 1 &&
                                   f.body.wheels[1].slipping)
                                : (f.body.wheels.size() > 0 &&
 f.body.wheels[0].slipping));
 const Vector3 track_pos{0.0f, slab_z,
 static_cast<float>(s * (f.body.half_width_m - 0.075))};
 if (assets != nullptr && model_ready(assets->track)) {
 draw_fitted_model(assets->track, track_pos,
 static_cast<float>(f.body.half_width_m * 0.35), 0.18f,
 slip ? Color{220, 70, 55, 255} : WHITE);
 } else {
 DrawCube(track_pos, slab_len, 0.12f, 0.15f, slip ? RED : DARKGRAY);
 }
 const double phase = f.body.wheels.empty() ? 0.0
 : f.body.wheels[static_cast<std::size_t>(s > 0 ? 1 : 0)].spin_rad;
 for (int pad = 0; pad < 8; ++pad) {
 const double u = (static_cast<double>(pad) +
 std::fmod(phase / (2.0 * 3.141592653589793), 1.0)) / 8.0;
 const float x = static_cast<float>((u - 0.5) * slab_len);
 DrawCube({x, slab_z + 0.067f,
 static_cast<float>(s * (f.body.half_width_m - 0.075))},
 slab_len / 8.8f, 0.018f, 0.17f,
 slip ? Color{180, 50, 48, 255} : Color{92, 96, 99, 255});
 }
      }
    } else {
      // Draw-local axes are raylib-world (Y up, Z = -bodyY): steer yaws
      // about Y (truth + = left, verified by the ackermann left-arc pin),
      // the axle lies lateral (Z) so the cylinder rolls 90 deg about X
 // first, spin follows about the laid-down axle. Spoke motion comes from
 // live joints; slip-red is restricted to SIM truth debug. Mecanum stripes
      // frame (rollers are passive analytic state — static decoration).
      for (const auto& w : f.body.wheels) {
 // Ackermann steering knuckles are independent MuJoCo links. Bind the GLB
 // to the recorded knuckle pose so the kingpin axis and front wheel remain
 // coincident on ramps and during steering; never rebuild this from a
 // guessed steer angle in the renderer.
 if (w.steering_pose.valid && assets != nullptr &&
 (model_ready(assets->knuckle_left) || model_ready(assets->knuckle_right))) {
 const bool right = w.y_m < 0.0;
 const Model& knuckle = right ? assets->knuckle_right : assets->knuckle_left;
 const AssetDefinition* definition = right
 ? assets->knuckle_right_definition : assets->knuckle_left_definition;
 if (model_ready(knuckle)) {
 const BoundingBox bounds = raylib_asset_bounds(knuckle);
 const float sx = std::max(1e-3f, bounds.max.x - bounds.min.x);
 const float sy = std::max(1e-3f, bounds.max.y - bounds.min.y);
 const float sz = std::max(1e-3f, bounds.max.z - bounds.min.z);
 const float authored = definition != nullptr &&
 definition->reference_radius_m > 0.0
 ? static_cast<float>(definition->reference_radius_m) : 0.10f;
 const float scale = std::max(0.01f, std::min({authored / sx,
 authored / sy, authored / sz}));
 draw_model_quaternion(knuckle, link_position(w.steering_pose),
 link_rotation(w.steering_pose, true), {scale, scale, scale}, WHITE);
 }
 }
 if (w.pose.valid) {
 const Vector3 position = link_position(w.pose);
 Quaternion rotation = link_rotation(w.pose, true);
  const Model* wheel_model = assets != nullptr ? &assets->wheel : nullptr;
  const AssetDefinition* wheel_definition = assets != nullptr
  ? assets->wheel_definition : nullptr;
  // Mecanum roller geometry is handed: the right-side asset is selected by
  // the physical module lateral coordinate, never by draw-list index.
  if (w.mecanum_rollers && w.y_m < 0.0 && assets != nullptr &&
  model_ready(assets->wheel_right)) {
  wheel_model = &assets->wheel_right;
  wheel_definition = assets->wheel_right_definition;
  }
  if (wheel_model != nullptr && model_ready(*wheel_model)) {
  draw_model_quaternion(*wheel_model, position, rotation,
  wheel_asset_scale(wheel_definition, w.radius_m,
 world.motor_params().tire_half_width_m),
 show_actor_routes && w.slipping ? Color{210, 60, 48, 255} : WHITE);
 } else {
 rotation = QuaternionMultiply(rotation,
 QuaternionFromAxisAngle({1.0f, 0.0f, 0.0f}, PI * 0.5f));
 Vector3 axis{0.0f, 1.0f, 0.0f};
 float angle = 0.0f;
 QuaternionToAxisAngle(rotation, &axis, &angle);
 rlPushMatrix();
 rlTranslatef(position.x, position.y, position.z);
 rlRotatef(angle * RAD2DEG, axis.x, axis.y, axis.z);
 const Color tire = show_actor_routes && w.slipping ? RED
 : (w.driven ? Color{40, 40, 40, 255}
 : Color{86, 86, 90, 255});
 DrawCylinder({0.0f, 0.0f, 0.0f},
 static_cast<float>(w.radius_m), static_cast<float>(w.radius_m),
 static_cast<float>(2.0 * world.motor_params().tire_half_width_m),
 20, tire);
 DrawCube({0.0f, 0.0f, 0.0f},
 2.0f * static_cast<float>(w.radius_m), 0.02f, 0.03f,
 YELLOW);
 rlPopMatrix();
 }
 continue;
 }
 // MuJoCo wheel bodies are children of the base at local z=0; the base
 // qpos already is the live axle height. Re-applying radius-body_z here
 // detached wheels whenever the engine body rose on a terrain patch.
 const float axle = 0.0f;
 const float wheel_lift = 0.0f;
 if (!w.mecanum_rollers && assets != nullptr && model_ready(assets->wheel)) {
 // The normalized wheel link already has a horizontal raylib -Z axle. A
 // differential module has steer_rad=0; only a steering module adds yaw.
 const Quaternion steer = QuaternionFromAxisAngle(
 {0.0f, 1.0f, 0.0f}, static_cast<float>(w.steer_rad));
 const Quaternion spin = QuaternionFromAxisAngle(
 {0.0f, 0.0f, -1.0f}, static_cast<float>(w.spin_rad));
 const Quaternion rotation = QuaternionMultiply(steer, spin);
 const Vector3 wheel_scale = wheel_asset_scale(
 assets->wheel_definition, w.radius_m,
 world.motor_params().tire_half_width_m);
 draw_model_quaternion(assets->wheel,
 {static_cast<float>(w.x_m), axle + wheel_lift, static_cast<float>(-w.y_m)},
 rotation, wheel_scale,
 show_actor_routes && w.slipping ? Color{210, 60, 48, 255} : WHITE);
 if (std::abs(w.steer_rad) > 1e-3 || std::abs(w.steer_rate_radps) > 1e-3) {
 DrawSphere({static_cast<float>(w.x_m), axle,
 static_cast<float>(-w.y_m)}, 0.035f, Color{210, 170, 70, 255});
 }
 continue;
 }
        rlPushMatrix();
 rlTranslatef(static_cast<float>(w.x_m), axle + wheel_lift,
                     static_cast<float>(-w.y_m));
        rlRotatef(static_cast<float>(w.steer_rad * RAD2DEG), 0.0f, 1.0f,
                  0.0f);
        rlRotatef(90.0f, 1.0f, 0.0f, 0.0f);
        if (w.mecanum_rollers) {
 for (int k = -2; k <= 2; ++k) {
            rlPushMatrix();
 rlTranslatef(static_cast<float>(k) * 0.026f, 0.0f, 0.0f);
 rlRotatef(static_cast<float>((k & 1) == 0 ? 45.0f : -45.0f),
 0.0f, 0.0f, 1.0f);
 rlRotatef(static_cast<float>(w.spin_rad * RAD2DEG), 0.0f, 0.0f, 1.0f);
 DrawCube({0.0f, 0.0f, 0.0f}, 0.018f, 0.024f,
 2.0f * static_cast<float>(w.radius_m),
 show_actor_routes && w.slipping
 ? Color{168, 55, 52, 255} : DARKGRAY);
            rlPopMatrix();
          }
        }
 rlRotatef(static_cast<float>(w.spin_rad * RAD2DEG), 0.0f, 0.0f,
 1.0f);
 const Color tire = show_actor_routes && w.slipping ? RED
 : (w.driven ? Color{40, 40, 40, 255} : Color{86, 86, 90, 255});
 if (std::abs(w.steer_rad) > 1e-3 || std::abs(w.steer_rate_radps) > 1e-3) {
 DrawSphere({0.0f, 0.0f, 0.0f}, 0.035f, Color{210, 170, 70, 255});
 const float arm = 0.16f * std::clamp(
 static_cast<float>(std::abs(w.steer_rate_radps) / 2.0), 0.0f, 1.0f);
 DrawLine3D({0.0f, 0.0f, 0.0f}, {arm, 0.0f, 0.0f},
 Color{240, 180, 70, 220});
 }
        DrawCylinder({0.0f, 0.0f, 0.0f}, static_cast<float>(w.radius_m),
                     static_cast<float>(w.radius_m), 0.08f, 12, tire);
        DrawCube({0.0f, 0.0f, 0.0f}, 2.0f * static_cast<float>(w.radius_m),
                 0.02f, 0.03f, YELLOW);  // spoke: spin reads visually
        rlPopMatrix();
      }
    }
    rlPopMatrix();
  }
 // Optional articulated manipulator. Each asset is a link component whose
 // pivot is declared in the manifest; its transform is the corresponding
 // MuJoCo body pose, so gravity and joint motion remain visible in replay.
 if (!f.body.arm_links.empty() && assets != nullptr) {
 const Model* arm_models[4] = {&assets->arm_turret, &assets->arm_upper,
 &assets->arm_forearm, &assets->arm_tool};
 const AssetDefinition* arm_defs[4] = {assets->arm_turret_definition,
 assets->arm_upper_definition, assets->arm_forearm_definition,
 assets->arm_tool_definition};
 for (std::size_t i = 0; i < f.body.arm_links.size() && i < 4; ++i) {
 const auto& link = f.body.arm_links[i];
 if (!link.valid || !link.pose.valid || !model_ready(*arm_models[i])) {
 continue;
 }
 const float scale = arm_defs[i] != nullptr &&
 arm_defs[i]->meters_per_unit > 0.0
 ? static_cast<float>(1.0 / arm_defs[i]->meters_per_unit) : 1.0f;
 draw_model_quaternion(*arm_models[i], link_position(link.pose),
 link_rotation(link.pose, true), {scale, scale, scale}, WHITE);
 }
 }
 // Engineering view: COM, live contacts, collision proxies, and static
 // collision prisms are all read from engine/authoring state. This entire
 // pass is behind the explicit SIM-truth diagnostics gate.
 if (show_physics_overlay) {
 FrameLinkPose com_pose;
 com_pose.valid = true;
 com_pose.x_m = f.body.com_x_m;
 com_pose.y_m = f.body.com_y_m;
 com_pose.z_m = f.body.com_z_m;
 const Vector3 com_world = body_local_to_world(f.body, com_pose);
 DrawSphere(com_world, 0.035f, MAGENTA);
 for (const auto& joint : f.body.joint_axes) {
 if (!joint.valid) continue;
 const Vector3 origin = body_local_to_world(f.body, joint.link_pose);
 FrameLinkPose endpoint = joint.link_pose;
 endpoint.x_m += joint.axis_x * 0.16;
 endpoint.y_m += joint.axis_y * 0.16;
 endpoint.z_m += joint.axis_z * 0.16;
 const Vector3 end = body_local_to_world(f.body, endpoint);
 DrawLine3D(origin, end, GOLD);
 DrawSphereWires(origin, 0.018f, 6, 6, GOLD);
 }
 for (const auto& contact : f.body.contacts) {
 if (!contact.valid) continue;
 const Vector3 point = to_raylib(contact.x_m, contact.y_m, contact.z_m);
 const Vector3 normal = {
 static_cast<float>(contact.normal_x),
 static_cast<float>(contact.normal_z),
 static_cast<float>(-contact.normal_y)};
 DrawSphere(point, 0.018f, LIME);
 DrawLine3D(point, Vector3Add(point, Vector3Scale(normal, 0.08f)), LIME);
 }
 for (const auto& geom : f.body.collision_geoms) {
 if (geom.valid) draw_collision_geom_wireframe(geom);
 }
 for (const auto& box : f.boxes) {
 const auto wires = lost_signal::sim::viewer::build_static_box_wireframe(
 box.x0_m, box.y0_m, box.x1_m, box.y1_m, box.height_m);
 for (const auto& line : wires.segments) {
 DrawLine3D({line.start.x, line.start.y, line.start.z},
 {line.end.x, line.end.y, line.end.z},
 Color{226, 74, 210, 150});
 }
 }
 }
  // Particle positions are already in world coordinates. Draw only after
  // leaving the robot transform so moving/turning cannot transform them
  // twice.
 if (presentation_effects) {
 draw_surface_traces_3d(surface_traces);
 draw_motion_particles_3d(particles, camera, assets);
 }
}

bool render_shadow_map(const Frame& frame,
 const lost_signal::sim::world::World& world, VisualAssets& assets,
 const SceneManifest* scene,
 const lost_signal::sim::viewer::LidarMount* lidar_mount,
 const Camera3D& view_camera) {
 const lost_signal::sim::viewer::SceneLighting defaults;
 const auto& lighting = scene != nullptr ? scene->lighting : defaults;
 if (!ensure_shadow_map(assets, lighting.shadow_map_resolution)) {
 return false;
 }
 Vector3 light_direction{
 static_cast<float>(lighting.direction_xyz[0]),
 static_cast<float>(lighting.direction_xyz[2]),
 static_cast<float>(-lighting.direction_xyz[1])};
 if (Vector3Length(light_direction) <= 1e-6f) return false;
 light_direction = Vector3Normalize(light_direction);
 // Anchor the directional shadow camera to the rendered view rather than to
 // the robot body. Following the body makes the orthographic projection move
 // by sub-texel amounts every tick, which is the classic shadow shimmer seen
 // while driving. Snap the anchor in light-camera texel units so a world
 // point keeps the same shadow texel until a full texel is crossed.
 const Vector3 view_center = view_camera.target;
 const Vector3 light_forward = Vector3Normalize(light_direction);
 Vector3 light_up = {0.0f, 1.0f, 0.0f};
 if (std::abs(Vector3DotProduct(light_forward, light_up)) > 0.96f) {
 light_up = {0.0f, 0.0f, -1.0f};
 }
 const Vector3 light_right = Vector3Normalize(
 Vector3CrossProduct(light_forward, light_up));
 light_up = Vector3Normalize(
 Vector3CrossProduct(light_right, light_forward));
 const float shadow_texel = static_cast<float>(lighting.shadow_extent_m) /
 static_cast<float>(std::max(1, lighting.shadow_map_resolution));
 const auto snap_axis = [shadow_texel](float value) {
 return shadow_texel > 0.0f
 ? std::round(value / shadow_texel) * shadow_texel : value;
 };
 const float center_right = Vector3DotProduct(view_center, light_right);
 const float center_up = Vector3DotProduct(view_center, light_up);
 const Vector3 center = Vector3Add(
 Vector3Add(Vector3Scale(light_right, snap_axis(center_right)),
 Vector3Scale(light_up, snap_axis(center_up))),
 Vector3Scale(light_forward,
 Vector3DotProduct(view_center, light_forward)));
 Camera3D light_camera{};
 light_camera.target = center;
 light_camera.position = Vector3Subtract(
 center, Vector3Scale(light_direction,
 static_cast<float>(lighting.shadow_extent_m)));
 // Use the same orthonormal basis that produced the snapped shadow center.
 // Replacing it with world-up changes the light-camera roll and makes the
 // depth lookup slide when the view follows a moving robot.
 light_camera.up = light_up;
 light_camera.fovy = static_cast<float>(lighting.shadow_extent_m);
 light_camera.projection = CAMERA_ORTHOGRAPHIC;

 rlActiveTextureSlot(10);
 rlDisableTexture();
 rlActiveTextureSlot(0);
 BeginTextureMode(assets.shadow_map);
 ClearBackground(WHITE);
 BeginMode3D(light_camera);
 const Matrix light_view = rlGetMatrixModelview();
 const Matrix light_projection = rlGetMatrixProjection();
 BeginShaderMode(assets.lighting);
 ParticleSystem no_particles;
 SurfaceTraceSystem no_surface_traces;
 draw_frame_3d(frame, world, light_camera, no_particles,
 no_surface_traces, false, false, false, false, &assets, scene,
  lidar_mount, false, false, false);
 EndShaderMode();
 EndMode3D();
 EndTextureMode();
 const Matrix light_view_projection = MatrixMultiply(
 light_view, light_projection);
 SetShaderValueMatrix(assets.lighting, assets.light_view_projection_loc,
 light_view_projection);
 return true;
}

bool draw_frame_3d_postprocessed(const Frame& frame,
 const lost_signal::sim::world::World& world, const Camera3D& camera,
 const ParticleSystem& particles, const SurfaceTraceSystem& surface_traces,
 bool show_lidar, bool show_estimate,
 bool show_actor_routes, bool show_places, bool show_physics_overlay,
 VisualAssets& assets,
 const SceneManifest* scene,
 const lost_signal::sim::viewer::LidarMount* lidar_mount) {
 if (!IsShaderValid(assets.lighting) ||
 !IsShaderValid(assets.bloom_extract) ||
 !IsShaderValid(assets.bloom_blur) ||
 !IsShaderValid(assets.composite) ||
 !ensure_post_targets(assets, GetScreenWidth(), GetScreenHeight())) {
 return false;
 }
 const lost_signal::sim::viewer::SceneLighting defaults;
 const auto& lighting = scene != nullptr ? scene->lighting : defaults;
 const bool shadows_ready = render_shadow_map(
 frame, world, assets, scene, lidar_mount, camera);
 configure_scene_lighting(assets, scene, frame, camera);
 if (shadows_ready) {
 const int shadow_texture_slot = 10;
 rlActiveTextureSlot(shadow_texture_slot);
 rlEnableTexture(assets.shadow_map.depth.id);
 rlSetUniform(assets.shadow_map_loc, &shadow_texture_slot,
 SHADER_UNIFORM_INT, 1);
 rlActiveTextureSlot(0);
 }
 BeginTextureMode(assets.scene_target);
 ClearBackground(scene_background(scene));
 BeginMode3D(camera);
 BeginShaderMode(assets.lighting);
 draw_frame_3d(frame, world, camera, particles, surface_traces, show_lidar,
 show_estimate, show_actor_routes, show_places, &assets, scene,
 lidar_mount, show_physics_overlay, false);
 EndShaderMode();
 EndMode3D();
 EndTextureMode();
 if (shadows_ready) {
 rlActiveTextureSlot(10);
 rlDisableTexture();
 rlActiveTextureSlot(0);
 }

 const float bloom_strength = static_cast<float>(lighting.bloom_strength);
 if (bloom_strength > 0.001f) {
 const float threshold = static_cast<float>(lighting.bloom_threshold);
 SetShaderValue(assets.bloom_extract, assets.bloom_threshold_loc,
 &threshold, SHADER_UNIFORM_FLOAT);
 BeginTextureMode(assets.bloom_target_a);
 ClearBackground(BLACK);
 BeginShaderMode(assets.bloom_extract);
 draw_render_texture(assets.scene_target,
 static_cast<float>(assets.bloom_target_a.texture.width),
 static_cast<float>(assets.bloom_target_a.texture.height));
 EndShaderMode();
 EndTextureMode();

 const Vector2 texel_size{
 1.0f / static_cast<float>(assets.bloom_target_a.texture.width),
 1.0f / static_cast<float>(assets.bloom_target_a.texture.height)};
 SetShaderValue(assets.bloom_blur, assets.blur_texel_size_loc,
 &texel_size, SHADER_UNIFORM_VEC2);
 for (int pass = 0; pass < 2; ++pass) {
 const Vector2 horizontal{1.0f, 0.0f};
 SetShaderValue(assets.bloom_blur, assets.blur_direction_loc,
 &horizontal, SHADER_UNIFORM_VEC2);
 BeginTextureMode(assets.bloom_target_b);
 ClearBackground(BLACK);
 BeginShaderMode(assets.bloom_blur);
 draw_render_texture(assets.bloom_target_a,
 static_cast<float>(assets.bloom_target_b.texture.width),
 static_cast<float>(assets.bloom_target_b.texture.height));
 EndShaderMode();
 EndTextureMode();

 const Vector2 vertical{0.0f, 1.0f};
 SetShaderValue(assets.bloom_blur, assets.blur_direction_loc,
 &vertical, SHADER_UNIFORM_VEC2);
 BeginTextureMode(assets.bloom_target_a);
 ClearBackground(BLACK);
 BeginShaderMode(assets.bloom_blur);
 draw_render_texture(assets.bloom_target_b,
 static_cast<float>(assets.bloom_target_a.texture.width),
 static_cast<float>(assets.bloom_target_a.texture.height));
 EndShaderMode();
 EndTextureMode();
 }
 } else {
 BeginTextureMode(assets.bloom_target_a);
 ClearBackground(BLACK);
 EndTextureMode();
 }

 const float exposure = static_cast<float>(lighting.exposure);
 const float vignette = static_cast<float>(lighting.vignette_strength);
 SetShaderValue(assets.composite, assets.composite_exposure_loc,
 &exposure, SHADER_UNIFORM_FLOAT);
 SetShaderValue(assets.composite, assets.composite_bloom_strength_loc,
 &bloom_strength, SHADER_UNIFORM_FLOAT);
 SetShaderValue(assets.composite, assets.composite_vignette_strength_loc,
 &vignette, SHADER_UNIFORM_FLOAT);
 SetShaderValueTexture(assets.composite,
 assets.composite_bloom_texture_loc, assets.bloom_target_a.texture);
 BeginShaderMode(assets.composite);
 draw_render_texture(assets.scene_target,
 static_cast<float>(assets.target_width),
 static_cast<float>(assets.target_height));
 EndShaderMode();
 return true;
}

void draw_frame_3d_direct(const Frame& frame,
 const lost_signal::sim::world::World& world, const Camera3D& camera,
 const ParticleSystem& particles, const SurfaceTraceSystem& surface_traces,
 bool show_lidar, bool show_estimate,
 bool show_actor_routes, bool show_places, bool show_physics_overlay,
 VisualAssets& assets,
 const SceneManifest* scene,
 const lost_signal::sim::viewer::LidarMount* lidar_mount) {
 release_shadow_map(assets);
 configure_scene_lighting(assets, scene, frame, camera);
 BeginMode3D(camera);
 if (IsShaderValid(assets.lighting)) BeginShaderMode(assets.lighting);
 // The direct/Turbo path has no physical shadow map.  Do not draw the old
 // flat contact discs here: they are a second, body-independent projection
 // and visibly float or slide when E toggles the evidence layer or the camera
 // follows a moving actor.  A fast frame may be shadowless; it must never
 // fabricate a shadow that disagrees with MuJoCo contact geometry.
 draw_frame_3d(frame, world, camera, particles, surface_traces, show_lidar,
 show_estimate, show_actor_routes, show_places, &assets, scene,
 lidar_mount, show_physics_overlay, false);
 if (IsShaderValid(assets.lighting)) EndShaderMode();
 EndMode3D();
}

}  // namespace lost_signal::sim::viewer::render
