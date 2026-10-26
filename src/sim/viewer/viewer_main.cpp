// SIM-side live viewer (2D top-down). Reads truth + recorded episode state
// through ScenarioSession (SIM composition root) and draws it with raylib.
// Never publishes to ROBOT: the session steps headless exactly as without
// a window; the viewer only observes. Headless CI never runs this binary
// (needs a display); frame logic is unit-tested in test_viewer_frame.

#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <filesystem>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <vector>
#include "robot/core/json.hpp"
#include "robot/platform/robot_config.hpp"
#include "sim/experiment/experience_runtime.hpp"
#include "sim/experiment/episode_pair.hpp"
#include "sim/eval/robot_config.hpp"
#include "sim/eval/sim_config.hpp"
#include "sim/eval/world_io.hpp"
#include "sim/scenario/actor_sidecar.hpp"
#include "sim/scenario/scenario_session.hpp"
#include "sim/viewer/asset_manifest.hpp"
#include "sim/viewer/experience_flow.hpp"
#include "sim/viewer/experience_manifest.hpp"
#include "sim/viewer/frame_builder.hpp"
#include "sim/viewer/keyframe_review.hpp"
#include "sim/viewer/map_editor.hpp"
#include "sim/viewer/particle_system.hpp"
#include "sim/viewer/playback_controller.hpp"
#include "sim/viewer/scene_manifest.hpp"
#include "sim/viewer/view_transform.hpp"
#include "sim/viewer/viewer_audio.hpp"
#include "sim/world/actor_behavior.hpp"
#include "sim/world/terrain.hpp"

namespace {
using lost_signal::sim::viewer::Frame;
using lost_signal::sim::viewer::FrameBody;
using lost_signal::sim::viewer::FrameLinkPose;
using lost_signal::sim::viewer::FramePlace;
using lost_signal::sim::viewer::FrameWheel;
using lost_signal::sim::viewer::ViewTransform;
using lost_signal::sim::viewer::AssetDefinition;
using lost_signal::sim::viewer::AssetManifest;
using lost_signal::sim::viewer::ExperienceFlow;
using lost_signal::sim::viewer::ExperienceManifest;
using lost_signal::sim::viewer::experience_variable_spec;
using lost_signal::sim::viewer::ExperiencePhase;
using lost_signal::sim::viewer::ExperienceRunKind;
using lost_signal::sim::viewer::PredictionAssessment;
using lost_signal::sim::viewer::ParticleKind;
using lost_signal::sim::viewer::ParticleSystem;
using lost_signal::sim::viewer::SurfaceTraceKind;
using lost_signal::sim::viewer::SurfaceTraceSystem;
using lost_signal::sim::viewer::ReviewKeyframe;
using lost_signal::sim::viewer::SceneInstance;
using lost_signal::sim::viewer::SceneManifest;

constexpr int kScreenW = 1100;
constexpr int kScreenH = 750;
// Open-ended sandbox runs retain a local replay window. Authored experiences
// keep their complete, independently scored episode for debrief and A/B.
constexpr double kDefaultSandboxHistorySeconds = 120.0;

Font g_ui_font{};
bool g_owns_ui_font = false;
bool g_zh_cn = false;

bool use_zh_cn_locale(const std::string& requested) {
 if (requested == "zh-CN") return true;
 if (requested == "en") return false;
 const char* locale = std::getenv("LC_ALL");
 if (locale == nullptr || *locale == '\0') locale = std::getenv("LC_MESSAGES");
 if (locale == nullptr || *locale == '\0') locale = std::getenv("LANG");
 return locale != nullptr && std::string(locale).rfind("zh", 0) == 0;
}

const char* tr(const char* english, const char* simplified_chinese) {
 return g_zh_cn ? simplified_chinese : english;
}

Font ui_font() {
 return g_owns_ui_font ? g_ui_font : GetFontDefault();
}

void draw_ui_text(const char* text, int x, int y, int font_size,
 Color color) {
 DrawTextEx(ui_font(), text, {static_cast<float>(x), static_cast<float>(y)},
 static_cast<float>(font_size), 0.5f, color);
}

float measure_ui_text(const char* text, float font_size) {
 return MeasureTextEx(ui_font(), text, font_size, 0.5f).x;
}

void unload_ui_font();

void load_ui_font(const ExperienceManifest* experience) {
 unload_ui_font();
 const std::filesystem::path path =
 "assets/fonts/NotoSansCJKsc-Regular.otf";
 if (!std::filesystem::is_regular_file(path)) return;
 std::string glyph_text =
 "机器人现场实验室任务简报部件检查开始实验检查全部先做预测点击选项"
 "或按数字键开始实时证据数据权限观察估计规划移动制动稳定物理真值"
 "运行中隐藏复盘时显示离线评估不会向机器人发布证据姿态来源"
 "本次运行证明了什么关键帧受控对比重试下一章通过安全门后解锁"
 "暂停单步激光估计真值调试跟随画质适配速度二维三维编辑轨道缩放"
 "物理部件已检查时间用尽操作失败碰撞最小净空平均误差重新运行基线"
 "变量种子预测得到本局支持需要修正开始对照实验重试通过安全验收后"
 "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789"
 "：，。；！？、（）《》【】 /.-+_%#=<>";
 if (experience != nullptr) {
 auto append = [&](const std::string& value) {
 glyph_text.push_back('\n');
 glyph_text += value;
 };
 append(experience->title);
 append(experience->subtitle);
 append(experience->prediction_question);
 append(experience->running_prompt);
 append(experience->success_title);
 append(experience->failure_title);
 for (const auto& value : experience->briefing) append(value);
 for (const auto& value : experience->lessons) append(value);
 for (const auto& stop : experience->walkaround) {
 append(stop.component_id);
 append(stop.title);
 append(stop.lesson);
 }
 for (const auto& choice : experience->choices) {
 append(choice.label);
 append(choice.explanation);
 }
 for (const auto& experiment : experience->experiments) {
 append(experiment.title);
 append(experience_variable_spec(experiment.controlled_variable).id);
 }
 }
 int decoded_count = 0;
 int* decoded = LoadCodepoints(glyph_text.c_str(), &decoded_count);
 if (decoded == nullptr || decoded_count <= 0) {
 UnloadCodepoints(decoded);
 return;
 }
 std::vector<int> codepoints(decoded, decoded + decoded_count);
 UnloadCodepoints(decoded);
 std::sort(codepoints.begin(), codepoints.end());
 codepoints.erase(std::unique(codepoints.begin(), codepoints.end()),
 codepoints.end());
 g_ui_font = LoadFontEx(path.string().c_str(), 48, codepoints.data(),
 static_cast<int>(codepoints.size()));
 g_owns_ui_font = g_ui_font.texture.id != 0;
 if (g_owns_ui_font) {
 SetTextureFilter(g_ui_font.texture, TEXTURE_FILTER_BILINEAR);
 }
}

void unload_ui_font() {
 if (g_owns_ui_font) UnloadFont(g_ui_font);
 g_ui_font = {};
 g_owns_ui_font = false;
}

enum class RenderQuality { kFast, kHigh };

const char* render_quality_name(RenderQuality quality) {
 return quality == RenderQuality::kHigh ? "HIGH" : "FAST";
}

struct VisualAssets {
 struct EnvironmentModel {
 std::string asset_id;
 Model model{};
 const AssetDefinition* definition = nullptr;
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
 AssetManifest manifest;
 Model chassis{};
 const AssetDefinition* chassis_definition = nullptr;
 Model wheel{};
 const AssetDefinition* wheel_definition = nullptr;
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
 Model worker{};
 const AssetDefinition* worker_definition = nullptr;
 std::vector<EnvironmentModel> environment;
 bool loaded = false;
};

bool model_ready(const Model& model) { return model.meshCount > 0; }

RenderTexture2D load_shadow_map(int width, int height) {
 RenderTexture2D target{};
 target.id = rlLoadFramebuffer();
 target.texture.width = width;
 target.texture.height = height;
 if (target.id == 0) return target;
 rlEnableFramebuffer(target.id);
 target.depth.id = rlLoadTextureDepth(width, height, false);
 target.depth.width = width;
 target.depth.height = height;
 target.depth.format = 19;
 target.depth.mipmaps = 1;
 rlFramebufferAttach(target.id, target.depth.id, RL_ATTACHMENT_DEPTH,
 RL_ATTACHMENT_TEXTURE2D, 0);
 if (!rlFramebufferComplete(target.id)) {
 rlUnloadFramebuffer(target.id);
 rlUnloadTexture(target.depth.id);
 return {};
 }
 return target;
}

void release_shadow_map(VisualAssets& assets) {
 if (assets.shadow_map.id > 0) rlUnloadFramebuffer(assets.shadow_map.id);
 if (assets.shadow_map.depth.id > 0)
 rlUnloadTexture(assets.shadow_map.depth.id);
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
 const std::string& robot_id) {
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
 std::string family = robot_id;
 // Legacy robot configs use descriptive ids (ro1-tracked, ro1-mecanum,
 // ...), while the content tree keys models by the master asset family.
 // Resolve that alias at the SIM asset boundary so every chassis gets its
 // authored GLB without changing the hardware contract or leaking content
 // ids into ROBOT.
 if (assets.manifest.find("robots." + family + ".chassis") == nullptr &&
 assets.manifest.find("robots." + family + ".hull") == nullptr) {
 const auto has = [&](const char* token) {
 return robot_id.find(token) != std::string::npos;
 };
 family = has("tracked") ? "ro3"
 : has("mecanum") ? "ro5"
 : has("swerve") ? "ro4"
 : has("ackermann") ? "ro2"
 : "ro1";
 }
 const std::string chassis_id = family == "ro3"
 ? "robots.ro3.hull"
 : "robots." + family + ".chassis";
 assets.chassis = load_manifest_model(models_root, assets.manifest,
 chassis_id, assets.chassis_definition);
 std::string wheel_id = "robots.ro1.wheel_standard";
 if (family == "ro4") wheel_id = "robots.ro4.swerve_module";
 if (family == "ro5") wheel_id = "robots.ro5.roller_left";
 assets.wheel = load_manifest_model(models_root, assets.manifest,
 wheel_id, assets.wheel_definition);
 assets.lidar = load_manifest_model(models_root, assets.manifest,
 "robots.ro1.lidar_standard", assets.lidar_definition);
 assets.caster_fork = load_manifest_model(models_root, assets.manifest,
 "robots.ro1.caster_fork", assets.caster_fork_definition);
 assets.caster_wheel = load_manifest_model(models_root, assets.manifest,
 "robots.ro1.caster_wheel", assets.caster_wheel_definition);
 assets.track = load_manifest_model(models_root, assets.manifest,
 "robots.ro3.track", assets.track_definition);
 assets.forklift = load_manifest_model(models_root, assets.manifest,
 "actors.forklift", assets.forklift_definition);
 assets.worker = load_manifest_model(models_root, assets.manifest,
 "actors.worker", assets.worker_definition);
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
 }
 log_model_bounds("chassis", assets.chassis);
 log_model_bounds("wheel", assets.wheel);
 log_model_bounds("lidar", assets.lidar);
 log_model_bounds("caster_fork", assets.caster_fork);
 log_model_bounds("caster_wheel", assets.caster_wheel);
 log_model_bounds("track", assets.track);
 log_model_bounds("forklift", assets.forklift);
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
 bind_lighting(assets.lidar, assets.lighting);
 bind_lighting(assets.caster_fork, assets.lighting);
 bind_lighting(assets.caster_wheel, assets.lighting);
 bind_lighting(assets.track, assets.lighting);
 bind_lighting(assets.forklift, assets.lighting);
 bind_lighting(assets.worker, assets.lighting);
 for (auto& environment : assets.environment) {
 bind_lighting(environment.model, assets.lighting);
 }
 }
 assets.loaded = model_ready(assets.chassis) || model_ready(assets.wheel) ||
 model_ready(assets.lidar) || model_ready(assets.caster_fork) ||
 model_ready(assets.caster_wheel) ||
 model_ready(assets.track) || model_ready(assets.forklift) ||
 model_ready(assets.worker) || !assets.environment.empty();
}

void unload_visual_assets(VisualAssets& assets) {
 release_shadow_map(assets);
 release_post_targets(assets);
 if (assets.dust.id > 0) UnloadTexture(assets.dust);
 if (model_ready(assets.chassis)) UnloadModel(assets.chassis);
 if (model_ready(assets.wheel)) UnloadModel(assets.wheel);
 if (model_ready(assets.lidar)) UnloadModel(assets.lidar);
 if (model_ready(assets.caster_fork)) UnloadModel(assets.caster_fork);
 if (model_ready(assets.caster_wheel)) UnloadModel(assets.caster_wheel);
 if (model_ready(assets.track)) UnloadModel(assets.track);
 if (model_ready(assets.forklift)) UnloadModel(assets.forklift);
 if (model_ready(assets.worker)) UnloadModel(assets.worker);
 for (auto& environment : assets.environment) {
 if (model_ready(environment.model)) UnloadModel(environment.model);
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
 } else {
 DrawSphere(position, size * (water ? 0.45f : 0.65f), tint);
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

struct VisibleRunSummary {
 bool recorded = false;
 bool timed_out = false;
 bool objectives_done = false;
 bool objectives_passed = false;
 double scan_period_s = 0.0;
 lost_signal::sim::eval::EpisodeScore score;

 bool safe() const {
 return recorded && !timed_out && objectives_done && objectives_passed &&
 score.collision_steps == 0;
 }
};

struct ExperimentUiState {
 VisibleRunSummary baseline;
 VisibleRunSummary variant;
 lost_signal::sim::experiment::EpisodePair episodes;
 std::optional<lost_signal::sim::experiment::EpisodeArtifact> standalone;
};

struct ExperienceHitRegions {
 Rectangle world_hotspot{};
 Rectangle previous{};
 Rectangle next{};
 Rectangle primary{};
 Rectangle retry{};
 Rectangle next_chapter{};
 std::vector<Rectangle> walkaround;
 std::vector<Rectangle> choices;

 void clear() {
 world_hotspot = {};
 previous = {};
 next = {};
 primary = {};
 retry = {};
 next_chapter = {};
 walkaround.clear();
 choices.clear();
 }
};

struct PlaybackHitRegions {
 std::array<Rectangle, 4> rates{};
 Rectangle turbo{};
};

PlaybackHitRegions playback_hit_regions() {
 PlaybackHitRegions regions;
 constexpr float kYFromBottom = 62.0f;
 constexpr float kX = 12.0f;
 constexpr float kWidth = 46.0f;
 constexpr float kGap = 6.0f;
 const float y = static_cast<float>(GetScreenHeight()) - kYFromBottom;
 for (std::size_t index = 0; index < regions.rates.size(); ++index) {
 regions.rates[index] = {
 kX + static_cast<float>(index) * (kWidth + kGap), y, kWidth, 30.0f};
 }
 regions.turbo = {
 kX + static_cast<float>(regions.rates.size()) * (kWidth + kGap),
 y, 76.0f, 30.0f};
 return regions;
}

void draw_panel_button(Rectangle bounds, const char* label, bool primary,
 Color accent, Color text, bool enabled = true) {
 const bool hovered = enabled &&
 CheckCollisionPointRec(GetMousePosition(), bounds);
 if (!enabled) {
 accent = Color{70, 88, 84, 255};
 text = Color{112, 127, 121, 255};
 }
 const Color fill = primary
 ? (hovered ? Color{67, 235, 216, 255} : accent)
 : (hovered ? Color{49, 72, 71, 255} : Color{29, 48, 48, 255});
 DrawRectangleRounded(bounds, 0.16f, 8, fill);
 if (!primary) DrawRectangleRoundedLinesEx(bounds, 0.16f, 8, 1.0f, accent);
 const float width = measure_ui_text(label, 14.0f);
 draw_ui_text(label,
 static_cast<int>(bounds.x + (bounds.width - width) * 0.5f),
 static_cast<int>(bounds.y + (bounds.height - 14.0f) * 0.5f), 14,
 primary ? Color{7, 18, 18, 255} : text);
}

void draw_playback_controls(const PlaybackHitRegions& regions,
 const lost_signal::sim::viewer::PlaybackController& playback) {
 constexpr std::array<double, 4> kRates{1.0, 2.0, 4.0, 8.0};
 constexpr std::array<const char*, 4> kLabels{"1x", "2x", "4x", "8x"};
 constexpr Color kCyan{55, 218, 202, 255};
 constexpr Color kText{226, 235, 230, 255};
 for (std::size_t index = 0; index < kRates.size(); ++index) {
 const bool selected = !playback.turbo() &&
 std::abs(playback.requested_rate() - kRates[index]) < 1e-9;
 draw_panel_button(regions.rates[index], kLabels[index], selected,
 kCyan, kText);
 }
 draw_panel_button(regions.turbo, "TURBO", playback.turbo(),
 Color{246, 169, 61, 255}, kText);
}

lost_signal::sim::viewer::ExperienceEvidence experience_evidence(
 const lost_signal::sim::scenario::ScenarioSession& session,
 bool timed_out, const ExperienceManifest& experience,
 const ExperimentUiState& experiment_ui) {
 lost_signal::sim::viewer::ExperienceEvidence evidence;
 evidence.terminal = session.objectives_done() || timed_out;
 const auto& results = session.objectives().results();
 evidence.all_robot_done = !results.empty() && std::all_of(
 results.begin(), results.end(), [](const auto& result) {
 return result.robot_done;
 });
 evidence.all_sim_accepted = !results.empty() && std::all_of(
 results.begin(), results.end(), [](const auto& result) {
 return result.successful();
 });
 evidence.collision_free = session.score().collision_steps == 0;
 if (!experience.experiments.empty()) {
 const auto& definition = experience.experiments.front();
 evidence.comparison_complete =
 experiment_ui.baseline.recorded && experiment_ui.variant.recorded;
 evidence.baseline_safe = experiment_ui.baseline.safe();
 evidence.variant_safe = experiment_ui.variant.safe();
 if (evidence.comparison_complete) {
 evidence.elapsed_delta_s = experiment_ui.variant.score.elapsed_s -
 experiment_ui.baseline.score.elapsed_s;
 evidence.clearance_delta_m =
 experiment_ui.variant.score.min_clearance_m -
 experiment_ui.baseline.score.min_clearance_m;
 }
 evidence.minimum_elapsed_increase_s =
 definition.minimum_elapsed_increase_s;
 evidence.minimum_clearance_loss_m =
 definition.minimum_clearance_loss_m;
 evidence.equivalent_elapsed_tolerance_s =
 definition.equivalent_elapsed_tolerance_s;
 evidence.equivalent_clearance_tolerance_m =
 definition.equivalent_clearance_tolerance_m;
 }
 return evidence;
}

float draw_wrapped_text(const std::string& text, float x, float y,
 float width, float font_size, float line_gap, Color color) {
 const Font font = ui_font();
 if (std::any_of(text.begin(), text.end(), [](unsigned char value) {
 return value >= 0x80;
 })) {
 const float spacing = 1.0f;
 std::string line;
 for (std::size_t offset = 0; offset < text.size();) {
 int bytes = 0;
 (void)GetCodepoint(text.c_str() + offset, &bytes);
 if (bytes <= 0) bytes = 1;
 const std::size_t glyph_bytes = std::min<std::size_t>(
 static_cast<std::size_t>(bytes), text.size() - offset);
 const std::string glyph = text.substr(offset, glyph_bytes);
 offset += glyph_bytes;
 const std::string candidate = line + glyph;
 if (!line.empty() &&
 MeasureTextEx(font, candidate.c_str(), font_size, spacing).x > width) {
 DrawTextEx(font, line.c_str(), {x, y}, font_size, spacing, color);
 y += font_size + line_gap;
 line = glyph;
 } else {
 line = candidate;
 }
 }
 if (!line.empty()) {
 DrawTextEx(font, line.c_str(), {x, y}, font_size, spacing, color);
 y += font_size + line_gap;
 }
 return y;
 }
 std::istringstream input(text);
 std::string word;
 std::string line;
 const float spacing = 1.0f;
 while (input >> word) {
 const std::string candidate = line.empty() ? word : line + " " + word;
 if (!line.empty() &&
 MeasureTextEx(font, candidate.c_str(), font_size, spacing).x > width) {
 DrawTextEx(font, line.c_str(), {x, y}, font_size, spacing, color);
 y += font_size + line_gap;
 line = word;
 } else {
 line = candidate;
 }
 }
 if (!line.empty()) {
 DrawTextEx(font, line.c_str(), {x, y}, font_size, spacing, color);
 y += font_size + line_gap;
 }
 return y;
}

void draw_experience_overlay(const ExperienceManifest& experience,
 ExperiencePhase phase, ExperienceRunKind run_kind,
 bool comparison_complete, double active_scan_period_s,
 int selected_choice, int walkaround_stop,
 std::size_t walkaround_visited_count,
 std::size_t required_walkaround_visited_count,
 std::size_t required_walkaround_count, bool walkaround_complete,
 const std::vector<ReviewKeyframe>& review_keyframes,
 int selected_review_keyframe,
 bool objectives_done,
 bool objectives_passed, const lost_signal::sim::eval::EpisodeScore& score,
 const Frame* frame,
 const ExperimentUiState* experiment_ui, bool timed_out,
 PredictionAssessment prediction_assessment,
 const lost_signal::sim::experiment::ExperienceGateVerdict* gate_verdict,
 const std::string& action_error, bool actions_enabled,
 ExperienceHitRegions* hit_regions) {
 constexpr Color kPanel{13, 21, 22, 242};
 constexpr Color kPanelSoft{24, 37, 38, 238};
 constexpr Color kText{226, 235, 230, 255};
 constexpr Color kMuted{157, 177, 169, 255};
 constexpr Color kCyan{55, 218, 202, 255};
 constexpr Color kAmber{246, 169, 61, 255};
 constexpr Color kRed{236, 82, 72, 255};
 if (hit_regions != nullptr) hit_regions->clear();
 if (phase == ExperiencePhase::kRunning) {
 const float width = 370.0f;
 const float x = static_cast<float>(GetScreenWidth()) - width - 18.0f;
 DrawRectangleRounded({x, 58.0f, width, 450.0f}, 0.04f, 8, kPanelSoft);
 draw_ui_text(tr("LIVE EVIDENCE / AUTHORITY", "实时证据 / 数据权限"),
 static_cast<int>(x + 18), 74, 14, kCyan);
 float y = 101.0f;
 char condition[160];
 std::snprintf(condition, sizeof(condition),
 g_zh_cn ? "%s条件　LiDAR %.0f ms"
 : "%s CONDITION  /  LiDAR %.0f ms",
 run_kind == ExperienceRunKind::kVariant
 ? tr("VARIANT", "慢扫描") : tr("BASELINE", "基线"),
 active_scan_period_s * 1000.0);
 draw_ui_text(condition, static_cast<int>(x + 18), static_cast<int>(y),
 14, kAmber);
 y += 27.0f;
 if (frame != nullptr) {
 char line[320];
 const auto& observation = frame->observation;
   const auto& estimate = frame->estimate;
   const auto& decision = frame->decision;
 const double task_time_s = std::max({
 0.0, observation.stamp_s, estimate.stamp_s, decision.stamp_s});
 const FramePlace* active_place = nullptr;
 for (const auto& place : frame->places) {
 if (!place.reached) {
 active_place = &place;
 break;
 }
 }
 const double goal_distance_m =
 estimate.valid && active_place != nullptr
 ? std::hypot(active_place->x_m - estimate.map_x_m,
 active_place->y_m - estimate.map_y_m)
 : -1.0;
 const char* stage = tr("OBSERVE", "观察");
   if (observation.accepted && observation.scan_valid) {
 stage = !estimate.valid ? tr("ESTIMATE", "估计")
 : !decision.valid || decision.action_code.empty()
 ? tr("PLAN", "规划")
    : std::abs(decision.command_forward_mps) <= 0.03 &&
      std::abs(decision.command_yaw_rate_radps) <= 0.03
 ? tr("BRAKE / SETTLE", "制动 / 稳定")
 : tr("PLAN / MOVE", "规划 / 移动");
   }
 draw_ui_text(stage, static_cast<int>(x + 18), static_cast<int>(y), 20,
    kAmber);
   y += 31.0f;
 draw_ui_text(tr("GAME / TASK", "游戏 / 当前任务"),
 static_cast<int>(x + 18), static_cast<int>(y), 13, kAmber);
 if (goal_distance_m >= 0.0) {
 std::snprintf(line, sizeof(line), "%s %.2f m t %.1f s",
 active_place->place_id.c_str(), goal_distance_m, task_time_s);
 } else {
 std::snprintf(line, sizeof(line), "target -- t %.1f s", task_time_s);
 }
 draw_ui_text(line, static_cast<int>(x + 18), static_cast<int>(y + 19),
 14, kText);
 y += 48.0f;
 draw_ui_text(tr("ROBOT / OBSERVATION", "机器人 / 传感器观测"),
 static_cast<int>(x + 18),
 static_cast<int>(y), 13, kCyan);
 std::snprintf(line, sizeof(line),
 "scan %s  beams %zu/%zu  odom %s  IMU %s",
 observation.accepted && observation.scan_valid ? "ACCEPTED" : "WAIT",
 observation.lidar_returns, observation.lidar_beams,
 observation.odometry_valid ? "OK" : "--",
 observation.imu_valid ? "OK" : "--");
 draw_ui_text(line, static_cast<int>(x + 18), static_cast<int>(y + 19), 13,
 kText);
 std::snprintf(line, sizeof(line), "%s seq %llu t=%.3fs",
 observation.scan_source.empty() ? "unknown source"
 : observation.scan_source.c_str(),
 static_cast<unsigned long long>(observation.scan_sequence),
 observation.scan_stamp_s);
 draw_ui_text(line, static_cast<int>(x + 18), static_cast<int>(y + 37), 12,
 kMuted);
 y += 62.0f;

 draw_ui_text(tr("ROBOT / ESTIMATE", "机器人 / 状态估计"),
 static_cast<int>(x + 18),
 static_cast<int>(y), 13, kCyan);
 if (estimate.valid) {
 std::snprintf(line, sizeof(line),
 "map (%.2f, %.2f)  sigma %.2f m  %s",
 estimate.map_x_m, estimate.map_y_m, estimate.position_sigma_m,
 estimate.localized ? "LOCALIZED" : "TRACKING");
 } else {
 std::snprintf(line, sizeof(line), "waiting for a valid state estimate");
 }
 draw_ui_text(line, static_cast<int>(x + 18), static_cast<int>(y + 19), 13,
 kText);
 y += 47.0f;

 draw_ui_text(tr("ROBOT / DECISION", "机器人 / 规划决策"),
 static_cast<int>(x + 18),
 static_cast<int>(y), 13, Color{180, 132, 255, 255});
 std::snprintf(line, sizeof(line), "%s / %s",
 decision.action_code.empty() ? "idle" : decision.action_code.c_str(),
 decision.safety_state.empty() ? "UNKNOWN" : decision.safety_state.c_str());
 draw_ui_text(line, static_cast<int>(x + 18), static_cast<int>(y + 19), 13,
 kText);
 std::snprintf(line, sizeof(line), "cmd %.2f m/s  %.2f rad/s  candidates %zu",
 decision.command_forward_mps, decision.command_yaw_rate_radps,
 decision.moving_candidate_count);
 draw_ui_text(line, static_cast<int>(x + 18), static_cast<int>(y + 37), 13,
 kMuted);
 y += 66.0f;
 }
 draw_ui_text(tr("SIM / PHYSICS TRUTH", "仿真 / 物理真值"),
 static_cast<int>(x + 18),
 static_cast<int>(y), 13, kAmber);
 draw_ui_text(tr("HIDDEN DURING RUN / REVEALED IN DEBRIEF",
 "运行时隐藏 / 复盘时显示"),
 static_cast<int>(x + 18), static_cast<int>(y + 19), 13, kMuted);
 y += 47.0f;
 draw_ui_text(tr("GAME / OFFLINE EVAL", "游戏 / 离线验收"),
 static_cast<int>(x + 18),
 static_cast<int>(y), 13, kAmber);
 draw_ui_text(tr("does not publish evidence back to ROBOT",
 "这些证据不会发布回机器人"),
 static_cast<int>(x + 18), static_cast<int>(y + 19), 13, kMuted);
  if (!experience.running_prompt.empty()) {
   draw_wrapped_text(experience.running_prompt, x + 18, y + 48,
    width - 36, 12, 3, kText);
  }
 return;
 }

 const bool briefing = phase == ExperiencePhase::kBriefing;
 const float screen_width = static_cast<float>(GetScreenWidth());
 const float screen_height = static_cast<float>(GetScreenHeight());
 const float panel_width = briefing
 ? std::min(400.0f, screen_width - 48.0f)
 : std::min(720.0f, screen_width - 100.0f);
 const float panel_height = briefing
 ? std::min(700.0f, screen_height - 48.0f)
 : phase == ExperiencePhase::kPrediction
 ? 610.0f
 : std::min(700.0f, screen_height - 48.0f);
 const float x = briefing ? 24.0f : (screen_width - panel_width) * 0.5f;
 const float y0 = (static_cast<float>(GetScreenHeight()) - panel_height) * 0.5f;
 DrawRectangle(0, 0, GetScreenWidth(), GetScreenHeight(),
 briefing ? Color{5, 10, 12, 38} : Color{5, 10, 12, 150});
 DrawRectangleRounded({x, y0, panel_width, panel_height}, 0.035f, 10,
 kPanel);
 DrawRectangleRoundedLinesEx({x, y0, panel_width, panel_height}, 0.035f, 10,
 2.0f, phase == ExperiencePhase::kDebrief ? kAmber : kCyan);
 draw_ui_text(tr("LOST SIGNAL / FIELD LAB", "失落信号 / 机器人现场实验室"),
 static_cast<int>(x + 34),
 static_cast<int>(y0 + 28), 15, kCyan);
 float y = draw_wrapped_text(experience.title, x + 34, y0 + 58,
 panel_width - 68, briefing ? 22.0f : 25.0f, 4.0f, kText);
 y = draw_wrapped_text(experience.subtitle, x + 34, y + 7,
 panel_width - 68, 16, 5, kMuted);
 y += 14.0f;

 if (phase == ExperiencePhase::kBriefing) {
 draw_ui_text(tr("MISSION BRIEF / COMPONENT WALKAROUND",
 "任务简报 / 物理部件检查"),
 static_cast<int>(x + 34), static_cast<int>(y), 15, kAmber);
  y += 27.0f;
  for (const auto& line : experience.briefing) {
   DrawCircle(static_cast<int>(x + 42), static_cast<int>(y + 6), 3.0f,
    kCyan);
   y = draw_wrapped_text(line, x + 54, y, panel_width - 92, 13, 4,
    kText) + 4.0f;
  }
  y += 5.0f;
 if (!experience.walkaround.empty()) {
 const int count = static_cast<int>(experience.walkaround.size());
 const int index = std::clamp(walkaround_stop, 0, count - 1);
 const auto& stop = experience.walkaround[static_cast<std::size_t>(index)];
   char progress[96];
std::snprintf(progress, sizeof(progress),
 g_zh_cn ? "物理部件 %02d / %02d　已检查 %02zu / %02d"
 : "PHYSICAL COMPONENT %02d / %02d    INSPECTED %02zu / %02d",
    index + 1, count, walkaround_visited_count, count);
 std::snprintf(progress, sizeof(progress),
 g_zh_cn ? "必看部件 %02zu / %02zu"
 : "REQUIRED COMPONENTS %02zu / %02zu",
 required_walkaround_visited_count, required_walkaround_count);
 draw_ui_text(progress, static_cast<int>(x + 34), static_cast<int>(y), 13,
 kMuted);
   y += 23.0f;
   y = draw_wrapped_text(stop.title, x + 34, y, panel_width - 68, 18, 4,
    kCyan) + 9.0f;
   y = draw_wrapped_text(stop.lesson, x + 34, y, panel_width - 68, 14, 5,
    kText) + 10.0f;
 draw_ui_text(tr("POSE SOURCE", "姿态来源"), static_cast<int>(x + 34),
 static_cast<int>(y),
 13, kAmber);
 y += 22.0f;
 const std::string source = stop.component_id + tr(" / MuJoCo body + joint", " / MuJoCo 刚体与关节");
 draw_wrapped_text(source, x + 34, y, panel_width - 68, 14, 5, kMuted);
 }
 const int tab_count = static_cast<int>(experience.walkaround.size());
 const int selected_tab = std::clamp(walkaround_stop, 0,
 std::max(0, tab_count - 1));
 const float tab_y = y0 + panel_height - 108.0f;
 const float tab_gap = 6.0f;
 const float tab_width = (panel_width - 68.0f -
 tab_gap * static_cast<float>(std::max(0, tab_count - 1))) /
 static_cast<float>(std::max(1, tab_count));
 for (int tab = 0; tab < tab_count; ++tab) {
 const auto& tab_stop =
 experience.walkaround[static_cast<std::size_t>(tab)];
 const char* label = tab_stop.component_id == "drive_wheel"
 ? tr("WHEEL", "DRIVE")
 : tab_stop.component_id == "caster"
 ? tr("CASTER", "CASTER")
 : tab_stop.component_id == "lidar"
 ? "LiDAR"
 : tr("BODY", "BODY");
 const Rectangle tab_bounds{
 x + 34.0f + static_cast<float>(tab) * (tab_width + tab_gap),
 tab_y, tab_width, 32.0f};
 draw_panel_button(tab_bounds, label, tab == selected_tab,
 tab_stop.required ? kCyan : Color{93, 126, 119, 255}, kText);
 if (hit_regions != nullptr) {
 hit_regions->walkaround.push_back(tab_bounds);
 }
 }
 const float button_y = y0 + panel_height - 64.0f;
 const Rectangle primary{x + 34.0f, button_y,
 panel_width - 68.0f, 36.0f};
  char primary_label[64];
  if (walkaround_complete) {
 std::snprintf(primary_label, sizeof(primary_label), "%s",
 tr("START EXPERIMENT", "开始实验"));
  } else {
 std::snprintf(primary_label, sizeof(primary_label),
 g_zh_cn ? "完成部件检查 %zu/%zu" : "INSPECT ALL %zu/%zu",
    walkaround_visited_count, experience.walkaround.size());
  }
 if (!walkaround_complete) {
 std::snprintf(primary_label, sizeof(primary_label), "%s",
 tr("VIEW WHEEL + LiDAR", "查看驱动轮与 LiDAR"));
 }
  draw_panel_button(primary, primary_label, true, kCyan, kText,
   walkaround_complete);
 if (hit_regions != nullptr) {
   if (walkaround_complete) hit_regions->primary = primary;
 }
 return;
 }

 if (phase == ExperiencePhase::kPrediction) {
 draw_ui_text(tr("PREDICT FIRST", "先做预测"), static_cast<int>(x + 34),
 static_cast<int>(y), 15, kAmber);
 y = draw_wrapped_text(experience.prediction_question, x + 34, y + 28,
 panel_width - 68, 19, 6, kText) + 10.0f;
 for (std::size_t i = 0; i < experience.choices.size(); ++i) {
 const float card_y = y;
 const Rectangle card{x + 32, card_y, panel_width - 64, 78};
 const bool hovered = CheckCollisionPointRec(GetMousePosition(), card);
 DrawRectangleRounded(card, 0.08f, 8,
 hovered ? Color{42, 67, 66, 255} : Color{29, 48, 48, 255});
 if (hovered) DrawRectangleRoundedLinesEx(card, 0.08f, 8, 1.0f, kCyan);
 if (hit_regions != nullptr) hit_regions->choices.push_back(card);
 const char key[2]{static_cast<char>('1' + i), '\0'};
 DrawCircle(static_cast<int>(x + 58), static_cast<int>(card_y + 39),
 15.0f, kCyan);
 draw_ui_text(key, static_cast<int>(x + 53), static_cast<int>(card_y + 29),
 19, Color{7, 18, 18, 255});
 draw_wrapped_text(experience.choices[i].label, x + 84, card_y + 17,
 panel_width - 132, 16, 4, kText);
 y += 91.0f;
 }
 draw_ui_text(tr("CLICK A CHOICE OR PRESS ITS NUMBER TO START",
 "点击一个预测，或按数字键开始实验"),
 static_cast<int>(x + 34), static_cast<int>(y0 + panel_height - 40),
 15, kAmber);
 return;
 }

 const bool safe = !timed_out && objectives_done && objectives_passed &&
 score.collision_steps == 0;
draw_ui_text((safe ? experience.success_title : experience.failure_title).c_str(),
 static_cast<int>(x + 34), static_cast<int>(y), 20,
 safe ? kCyan : kRed);
 y += 38.0f;
 if (timed_out) {
 y = draw_wrapped_text(
tr("TIME LIMIT REACHED: the robot did not produce a terminal mission result within the authored experiment budget.",
"时间用尽：机器人未能在本次实验的时间预算内给出终态任务结果。"),
 x + 34, y, panel_width - 68, 15, 5, kRed) + 10.0f;
 }
 if (!action_error.empty()) {
y = draw_wrapped_text(std::string(tr("ACTION FAILED: ", "操作失败：")) +
action_error,
 x + 34, y, panel_width - 68, 14, 4, kRed) + 8.0f;
 }
 char metrics[256];
 std::snprintf(metrics, sizeof(metrics),
 "SIM OFFLINE EVAL   collisions=%zu   min clearance=%.2f m   mean estimate error=%.2f m",
 score.collision_steps, score.min_clearance_m, score.mean_pos_err_m);
 if (g_zh_cn) {
 std::snprintf(metrics, sizeof(metrics),
 "仿真离线验收　碰撞=%zu　最小净空=%.2f m　平均估计误差=%.2f m",
 score.collision_steps, score.min_clearance_m, score.mean_pos_err_m);
 }
 y = draw_wrapped_text(metrics, x + 34, y, panel_width - 68, 15, 5,
 safe ? kText : kRed) + 12.0f;
 if (gate_verdict != nullptr) {
 char gates[320];
 const auto mark = [](bool value) { return value ? "PASS" : "FAIL"; };
 std::snprintf(gates, sizeof(gates),
 g_zh_cn
 ? "四项裁决　仿真 %s　安全 %s　任务 %s　实验 %s"
 : "VERDICTS | SIM %s | SAFETY %s | MISSION %s | EXPERIMENT %s",
 mark(gate_verdict->simulation_valid),
 mark(gate_verdict->safety_passed),
 mark(gate_verdict->mission_passed),
 mark(gate_verdict->experiment_passed));
 y = draw_wrapped_text(gates, x + 34, y, panel_width - 68, 14, 4,
 gate_verdict->can_advance ? kCyan : kAmber) + 10.0f;
 }
 const bool has_comparison = !experience.experiments.empty() &&
 experiment_ui != nullptr && experiment_ui->baseline.recorded;
 if (has_comparison) {
 const auto& baseline = experiment_ui->baseline;
 const auto& experiment = experience.experiments.front();
 draw_ui_text(comparison_complete
 ? tr("VISIBLE CONTROLLED COMPARISON", "可视化受控对照完成")
 : tr("BASELINE RECORDED / VARIANT NEXT", "基线已记录 / 下一步运行变体"),
 static_cast<int>(x + 34), static_cast<int>(y), 14, kAmber);
 y += 22.0f;
 if (!comparison_complete || !experiment_ui->variant.recorded) {
 char baseline_line[320];
 std::snprintf(baseline_line, sizeof(baseline_line),
 g_zh_cn
 ? "基线 LiDAR %.0f ms　时间 %.2f s　碰撞 %zu　最小净空 %.3f m"
 : "Baseline LiDAR %.0f ms | time %.2f s | collisions %zu | clearance %.3f m",
 baseline.scan_period_s * 1000.0, baseline.score.elapsed_s,
 baseline.score.collision_steps, baseline.score.min_clearance_m);
 y = draw_wrapped_text(baseline_line, x + 34, y,
 panel_width - 68, 14, 4, kText) + 7.0f;
 char invitation[320];
 std::snprintf(invitation, sizeof(invitation),
 g_zh_cn
 ? "下一次运行只把 %s 改为 %.0f ms；场景、机器人、初始状态和随机种子全部重建为相同值。"
 : "The next visible run changes only %s to %.0f ms. World, robot, initial state, and seeds are rebuilt identically.",
 lost_signal::sim::viewer::experience_variable_spec(
 experiment.controlled_variable).id,
 experiment.variant_value * 1000.0);
 y = draw_wrapped_text(invitation, x + 34, y,
 panel_width - 68, 14, 4, kMuted) + 8.0f;
 } else {
 const auto& variant = experiment_ui->variant;
 const double elapsed_delta_s =
 variant.score.elapsed_s - baseline.score.elapsed_s;
 const double clearance_delta_m =
 variant.score.min_clearance_m - baseline.score.min_clearance_m;
 char comparison[384];
 std::snprintf(comparison, sizeof(comparison),
 g_zh_cn
 ? "基线 %.0f ms：%.2f s / 净空 %.3f m / 碰撞 %zu\n变体 %.0f ms：%.2f s / 净空 %.3f m / 碰撞 %zu"
 : "Baseline %.0f ms: %.2f s / %.3f m clearance / %zu collisions\nVariant %.0f ms: %.2f s / %.3f m clearance / %zu collisions",
 baseline.scan_period_s * 1000.0, baseline.score.elapsed_s,
 baseline.score.min_clearance_m, baseline.score.collision_steps,
 variant.scan_period_s * 1000.0, variant.score.elapsed_s,
 variant.score.min_clearance_m, variant.score.collision_steps);
 y = draw_wrapped_text(comparison, x + 34, y,
 panel_width - 68, 14, 4, kText) + 7.0f;
 char delta[256];
 std::snprintf(delta, sizeof(delta),
 g_zh_cn ? "实际差值　时间 %+.2f s　最小净空 %+.3f m"
 : "Measured delta | time %+.2f s | minimum clearance %+.3f m",
 elapsed_delta_s, clearance_delta_m);
 y = draw_wrapped_text(delta, x + 34, y,
 panel_width - 68, 15, 4,
 baseline.safe() && variant.safe() ? kCyan : kRed) + 8.0f;
 }
 }
 if (!review_keyframes.empty()) {
 const int count = static_cast<int>(review_keyframes.size());
 const int index = std::clamp(selected_review_keyframe, 0, count - 1);
 const auto& keyframe =
 review_keyframes[static_cast<std::size_t>(index)];
 char event_header[160];
 const double time_s = keyframe.stamp_ns ==
 lost_signal::robot::core::kInvalidTime
 ? 0.0 : static_cast<double>(keyframe.stamp_ns) * 1e-9;
 std::snprintf(event_header, sizeof(event_header),
 "KEYFRAME %02d/%02d  t=%.2fs  record=%llu",
 index + 1, count, time_s,
 static_cast<unsigned long long>(keyframe.step));
 DrawText(event_header, static_cast<int>(x + 34), static_cast<int>(y),
 13, kAmber);
 y += 24.0f;
 DrawText(keyframe.title.c_str(), static_cast<int>(x + 34),
 static_cast<int>(y), 17, kCyan);
 y = draw_wrapped_text(keyframe.evidence, x + 34, y + 24,
 panel_width - 68, 14, 4, kText) + 10.0f;
  const float controls_y = y0 + panel_height - 72.0f;
  const Rectangle previous{x + 34.0f, controls_y, 46.0f, 34.0f};
  const Rectangle next{x + 88.0f, controls_y, 46.0f, 34.0f};
  draw_panel_button(previous, "<", false, kCyan, kText);
  draw_panel_button(next, ">", false, kCyan, kText);
  if (hit_regions != nullptr) {
   hit_regions->previous = previous;
   hit_regions->next = next;
  }
 }
 if (selected_choice >= 0 &&
 selected_choice < static_cast<int>(experience.choices.size())) {
 const auto& choice = experience.choices[static_cast<std::size_t>(selected_choice)];
 const bool prediction_pending =
 prediction_assessment == PredictionAssessment::kPending;
 const bool prediction_supported =
 prediction_assessment == PredictionAssessment::kSupported;
 const bool prediction_refuted =
 prediction_assessment == PredictionAssessment::kRefuted;
 draw_ui_text(prediction_pending
 ? tr("YOUR PREDICTION IS LOCKED UNTIL BOTH RUNS FINISH",
 "你的预测已锁定，完成两次运行后验证")
 : prediction_supported
 ? tr("YOUR PREDICTION: SUPPORTED BY BOTH RUNS",
 "你的预测：得到两次运行的证据支持")
 : prediction_refuted
 ? tr("YOUR PREDICTION: REFUTED BY THE RUN",
 "你的预测：被本次运行的证据反驳")
 : tr("YOUR PREDICTION: INSUFFICIENT EVIDENCE",
 "你的预测：当前证据不足"),
 static_cast<int>(x + 34), static_cast<int>(y), 15,
 !prediction_pending && prediction_supported ? kCyan : kAmber);
 y = draw_wrapped_text(choice.explanation, x + 34, y + 26,
 panel_width - 68, 16, 5, kText) + 13.0f;
 }
 draw_ui_text(tr("WHAT THE RUN ESTABLISHED", "本次运行证明了什么"),
 static_cast<int>(x + 34),
 static_cast<int>(y), 15, kAmber);
 y += 28.0f;
 for (const auto& lesson : experience.lessons) {
 DrawCircle(static_cast<int>(x + 42), static_cast<int>(y + 7), 3.5f,
 kCyan);
 y = draw_wrapped_text(lesson, x + 56, y, panel_width - 100, 15, 5,
 kText) + 6.0f;
 }
 if (!experience.experiments.empty() && experiment_ui != nullptr &&
 experiment_ui->baseline.recorded && !comparison_complete) {
 const Rectangle experiment_button{x + 136.0f, y0 + panel_height - 72.0f,
   158.0f, 34.0f};
 const bool can_run = actions_enabled &&
 run_kind == ExperienceRunKind::kBaseline;
  draw_panel_button(experiment_button,
 tr("RUN VISIBLE VARIANT", "可视运行慢扫描"),
   true, kAmber, kText, can_run);
  if (hit_regions != nullptr && can_run) {
   hit_regions->primary = experiment_button;
  }
 }
 const Rectangle retry_button{x + 306.0f, y0 + panel_height - 72.0f,
 116.0f, 34.0f};
 draw_panel_button(retry_button, tr("RETRY RUN", "重试本局"),
 false, kCyan, kText,
 actions_enabled);
 if (hit_regions != nullptr && actions_enabled) {
 hit_regions->retry = retry_button;
 }
 if (!experience.next_experience_path.empty()) {
 const Rectangle next_chapter_button{
 x + 434.0f, y0 + panel_height - 72.0f, 252.0f, 34.0f};
 const bool can_advance = actions_enabled && gate_verdict != nullptr &&
 gate_verdict->can_advance;
 draw_panel_button(next_chapter_button,
 can_advance ? tr("NEXT CHAPTER", "下一章")
 : tr("NEXT: PASS SAFETY GATE", "下一章：通过安全验收后解锁"),
 true, kCyan, kText, can_advance);
 if (hit_regions != nullptr && can_advance) {
 hit_regions->next_chapter = next_chapter_button;
 }
 }
 draw_ui_text(tr("KEYFRAMES / CONTROLLED A/B / ROBOT never received SIM truth.",
 "关键帧 / 受控 A/B / 机器人从未收到仿真真值。"),
  static_cast<int>(x + 34), static_cast<int>(y0 + panel_height - 27),
  13, kAmber);
}

// Map extents over every authored layer (auto-fit on --map load).
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

void setup_demo(lost_signal::sim::scenario::ScenarioSession& session) {
  session.setup_world().set_static_boxes({
      {5.2, 1.0, 6.0, 1.8, 0.0},
      {2.5, 5.0, 3.3, 5.8, 0.0},
      {-2.4, 2.0, -1.6, 2.8, 0.0},
      {1.0, -2.4, 1.8, -1.6, 0.0},
  });
  session.setup_world().add_place({"loopA", 4.0, 0.0, 0.0});
  session.setup_world().add_place({"loopB", 4.0, 4.0, 0.0});
  session.setup_world().add_place({"loopC", 0.0, 4.0, 0.0});
  session.setup_world().add_place({"loopD", 0.0, 0.0, 0.0});
  session.set_objectives({"loopA", "loopB", "loopC", "loopD"});
  lost_signal::robot::iface::TaskRequest req;
  req.request_id = "view";
  req.place_id = "loopA";
  session.request_goal(req);
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
 for (const auto& center : f.decision.safety_static_sweep) {
 const auto projected = view.world_to_screen(center.first, center.second);
 const Vector2 point{projected.x_px, projected.y_px};
 if (have_previous) {
 DrawLineEx(previous, point, std::max(1.0f, 2.0f * radius_px), fill);
 DrawLineEx(previous, point, 1.5f, edge);
 }
 DrawCircleV(point, radius_px, fill);
 DrawCircleLinesV(point, radius_px, edge);
 previous = point;
 have_previous = true;
 }
 }
  if (show_lidar && f.lidar_valid) {
 const auto o = view.world_to_screen(
 f.observation.projected_sensor_x_m,
 f.observation.projected_sensor_y_m);
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
 if (show_truth_debug && f.truth_lidar_valid) {
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
struct CameraState {
  double target_x = 2.0, target_y = 1.0;
 double target_z = 0.0;
  float yaw_deg = 35.0f;    // orbit around the target
  float pitch_deg = 50.0f;  // elevation above the ground plane
  float dist_m = 12.0f;
};

inline Vector3 to_raylib(double x_m, double y_m, double z_m) {
  return {static_cast<float>(x_m), static_cast<float>(z_m),
          static_cast<float>(-y_m)};
}

inline Camera3D camera_from_state(const CameraState& st) {
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

struct WalkaroundView {
 Camera3D camera;
 Vector3 focus;
};

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

// Ground-plane pick for 3D edit mode: intersect the mouse ray with z_up=0.
inline bool screen_to_ground_3d(const Camera3D& cam, Vector2 mouse,
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

void draw_frame_3d(const Frame& f,
                   const lost_signal::sim::world::World& world,
 const Camera3D& camera, const ParticleSystem& particles,
 const SurfaceTraceSystem& surface_traces,
 bool show_lidar, bool show_estimate,
 bool show_actor_routes, bool show_places,
 const VisualAssets* assets = nullptr,
 const SceneManifest* scene = nullptr,
 const lost_signal::sim::viewer::LidarMount* lidar_mount = nullptr,
 bool presentation_effects = true) {
  namespace world_ns = lost_signal::sim::world;
  // Ground base sized to the authored map (concrete apron under everything).
  double gx0, gy0, gx1, gy1;
  map_bounds(world, gx0, gy0, gx1, gy1);
  {
    const float w = static_cast<float>(gx1 - gx0 + 8.0);
    const float h = static_cast<float>(gy1 - gy0 + 8.0);
 const auto& rgba = scene != nullptr ? scene->floor.base_rgba
 : std::array<int, 4>{198, 203, 196, 255};
 DrawPlane(to_raylib((gx0 + gx1) * 0.5, (gy0 + gy1) * 0.5, -0.001),
 {w, h}, {static_cast<unsigned char>(rgba[0]),
 static_cast<unsigned char>(rgba[1]), static_cast<unsigned char>(rgba[2]),
 static_cast<unsigned char>(rgba[3])});
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
    double hmax = 0.2;
    for (const auto& p : f.patches) {
      for (double x = p.x0_m; x < p.x1_m; x += 0.5) {
        for (double y = p.y0_m; y < p.y1_m; y += 0.5) {
          const auto s = world_ns::sample_terrain(
              world.terrain_patches(), world.support_slabs(), x, y);
          hmax = std::max(hmax, s.height_m);
        }
      }
    }
    for (const auto& p : f.patches) {
      for (double x = p.x0_m; x < p.x1_m; x += 0.5) {
        for (double y = p.y0_m; y < p.y1_m; y += 0.5) {
          const double x2 = std::min(x + 0.5, p.x1_m);
          const double y2 = std::min(y + 0.5, p.y1_m);
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
 if (presentation_effects)
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
 if (presentation_effects)
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
 const double ground_z = world_ns::sample_terrain(
 world.terrain_patches(), world.support_slabs(), a.x_m, a.y_m).height_m;
 if (presentation_effects)
 blob_shadow_3d(a.x_m, a.y_m, a.radius_m * 1.3, ground_z);
 const Model* actor_model = nullptr;
 const AssetDefinition* actor_definition = nullptr;
 if (assets != nullptr) actor_model = a.pedestrian ? &assets->worker
 : &assets->forklift;
 if (assets != nullptr) {
 actor_definition = a.pedestrian ? assets->worker_definition
 : assets->forklift_definition;
 }
 if (actor_model != nullptr && model_ready(*actor_model) &&
 actor_definition != nullptr) {
draw_actor_model_grounded(*actor_model, a.x_m, a.y_m, *actor_definition,
 ground_z, a.radius_m, a.heading_rad);
 } else {
 // raylib's DrawCylinder position is the base center (not the geometric
 // center). Passing ground+h/2 made the procedural pedestrian float by half
 // its height; the SIM terrain sample is the sole contact anchor.
 DrawCylinder(to_raylib(a.x_m, a.y_m, ground_z),
 static_cast<float>(a.radius_m), static_cast<float>(a.radius_m),
 static_cast<float>(h), 16, body);
 }
 DrawCylinder(to_raylib(a.x_m, a.y_m, ground_z + h + 0.03), 0.06f, 0.06f, 0.12f, 8,
                 DARKGRAY);  // roof beacon
 DrawLine3D(to_raylib(a.x_m, a.y_m, ground_z + h * 0.5),
 to_raylib(a.x_m + a.vx_mps, a.y_m + a.vy_mps,
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
 for (std::size_t i = 0;
 i < f.decision.safety_static_sweep.size(); i += stride) {
 const auto& center = f.decision.safety_static_sweep[i];
 const Vector3 point = to_raylib(center.first, center.second, 0.018);
 if (have_previous) DrawLine3D(previous, point, edge);
 DrawCylinderEx(point, {point.x, point.y + 0.008f, point.z},
 radius, radius, 32, fill);
 DrawCircle3D(point, radius, {1.0f, 0.0f, 0.0f}, 90.0f, edge);
 previous = point;
 have_previous = true;
 }
 const auto& terminal = f.decision.safety_static_sweep.back();
 const Vector3 terminal_point = to_raylib(
 terminal.first, terminal.second, 0.018);
 if (have_previous && (previous.x != terminal_point.x ||
 previous.z != terminal_point.z)) {
 DrawLine3D(previous, terminal_point, edge);
 DrawCircle3D(terminal_point, radius, {1.0f, 0.0f, 0.0f}, 90.0f, edge);
 }
 }
  if (show_lidar && f.lidar_valid) {
 const Vector3 o = to_raylib(
 f.observation.projected_sensor_x_m,
 f.observation.projected_sensor_y_m,
 f.observation.projected_sensor_z_m);
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
 if (show_actor_routes && f.truth_lidar_valid) {
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
 if (presentation_effects) blob_shadow_3d(f.body.x_m, f.body.y_m,
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
 Vector3 caster_wheel_scale = wheel_asset_scale(
 assets != nullptr ? assets->caster_wheel_definition : nullptr,
 caster.radius_m, caster_half_width);
 if (assets != nullptr && model_ready(assets->caster_wheel)) {
 caster_wheel_scale = wheel_asset_scale(assets->caster_wheel_definition,
 caster.radius_m, caster_half_width);
 }
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
 // Keep the authored GLB detail and add the physical crowned centre rib so
 // the visible tire profile matches the compound contact geometry.
 rlPushMatrix();
 rlTranslatef(static_cast<float>(caster.x_m), axle + caster_lift,
 static_cast<float>(-caster.y_m));
 rlRotatef(static_cast<float>(caster.yaw_rad * RAD2DEG), 0.0f, 1.0f,
 0.0f);
 rlRotatef(static_cast<float>(caster.spin_rad * RAD2DEG), 0.0f, 0.0f,
 1.0f);
 draw_crowned_tire(caster.radius_m, caster_half_width,
 Color{36, 38, 43, 255});
 rlPopMatrix();
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
 if (w.pose.valid) {
 const Vector3 position = link_position(w.pose);
 Quaternion rotation = link_rotation(w.pose, true);
 if (assets != nullptr && model_ready(assets->wheel)) {
 draw_model_quaternion(assets->wheel, position, rotation,
 wheel_asset_scale(assets->wheel_definition, w.radius_m,
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
 if (!w.mecanum_rollers && assets != nullptr && model_ready(assets->wheel)) {
 // The raw authored wheel link uses +Z as its axle. In raylib's mapped
 // vehicle frame that local Z is already the horizontal depth axis; the
 // fixed link mount above lays the wheel plane vertical. A second -90
 // degree conversion would make the axle vertical and is intentionally
 // absent.
 rlRotatef(static_cast<float>(w.spin_rad * RAD2DEG), 0.0f, 0.0f, 1.0f);
 DrawModelEx(assets->wheel, {0.0f, 0.0f, 0.0f},
 {0.0f, 1.0f, 0.0f}, 0.0f,
 {1.0f, 1.0f, 1.0f}, show_actor_routes && w.slipping
 ? Color{210, 60, 48, 255} : WHITE);
 if (std::abs(w.steer_rad) > 1e-3 || std::abs(w.steer_rate_radps) > 1e-3) {
 DrawSphere({0.0f, 0.0f, 0.0f}, 0.035f, Color{210, 170, 70, 255});
 }
 rlPopMatrix();
 continue;
 }
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
 const lost_signal::sim::viewer::LidarMount* lidar_mount) {
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
 const Vector3 center{static_cast<float>(frame.body.x_m), 0.4f,
 static_cast<float>(-frame.body.y_m)};
 Camera3D light_camera{};
 light_camera.target = center;
 light_camera.position = Vector3Subtract(
 center, Vector3Scale(light_direction,
 static_cast<float>(lighting.shadow_extent_m)));
 light_camera.up = std::abs(Vector3DotProduct(
 light_direction, {0.0f, 1.0f, 0.0f})) > 0.96f
 ? Vector3{0.0f, 0.0f, -1.0f} : Vector3{0.0f, 1.0f, 0.0f};
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
 lidar_mount, false);
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
 bool show_actor_routes, bool show_places, VisualAssets& assets,
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
 frame, world, assets, scene, lidar_mount);
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
 lidar_mount);
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
 bool show_actor_routes, bool show_places, VisualAssets& assets,
 const SceneManifest* scene,
 const lost_signal::sim::viewer::LidarMount* lidar_mount) {
 release_shadow_map(assets);
 configure_scene_lighting(assets, scene, frame, camera);
 BeginMode3D(camera);
 if (IsShaderValid(assets.lighting)) BeginShaderMode(assets.lighting);
 draw_frame_3d(frame, world, camera, particles, surface_traces, show_lidar,
 show_estimate, show_actor_routes, show_places, &assets, scene,
 lidar_mount);
 if (IsShaderValid(assets.lighting)) EndShaderMode();
 EndMode3D();
}
}  // namespace

static lost_signal::sim::viewer::ExperienceFlow::Config experience_flow_config(
 const lost_signal::sim::viewer::ExperienceManifest& experience) {
 lost_signal::sim::viewer::ExperienceFlow::Config config;
 config.walkaround_count = experience.walkaround.size();
 config.choice_count = experience.choices.size();
 config.time_limit_steps = experience.time_limit_steps;
 config.required_walkaround.reserve(experience.walkaround.size());
 for (const auto& stop : experience.walkaround) {
 config.required_walkaround.push_back(stop.required);
 }
 if (!experience.experiments.empty()) {
 config.variant_time_limit_steps = static_cast<std::uint64_t>(
 experience.experiments.front().step_budget);
 }
 return config;
}

int main(int argc, char** argv) {
  using namespace lost_signal;
 // Optional single-source configuration: --robot <config> builds the
 // session from one RobotConfig file (ROBOT description + SIM motor
 // numbers), --map <world file> replaces the demo geometry. `--assets`
 // enables optional SIM-only GLB presentation models from the old content
 // tree; procedural parts remain the deterministic fallback.
 std::string robot_path, sim_path, map_path, assets_path, scene_path,
 experience_path;
 std::string requested_locale = "auto";
 double initial_playback_rate = 1.0;
 double sandbox_history_seconds = kDefaultSandboxHistorySeconds;
 bool initial_turbo = false;
 RenderQuality render_quality = RenderQuality::kHigh;
 bool sandbox_requested = false;
 bool play_requested = false;
 bool source_requested = false;
 bool mute_audio = false;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
 if (a == "--help" || a == "-h") {
 std::puts(
 "usage: sim_viewer [--play | --sandbox | --experience FILE]\n"
 " [--robot FILE] [--sim FILE] [--map FILE] [--scene FILE]\n"
 " [--rate 0.05..64 | --turbo] [--quality high|fast] [--3d]\n"
 " [--locale auto|en|zh-CN] [--mute] [--history-seconds N]\n"
 "\n"
 "No arguments starts the First Contact field lab. --sandbox keeps the\n"
 "open engineering demo. During a run: Space pause, N step, F follow,\n"
 "Q quality, +/- rate, T turbo, V 2D/3D, right-drag orbit, wheel zoom.");
 return 0;
 }
 if (a == "--play") play_requested = true;
 if (a == "--sandbox") sandbox_requested = true;
 if (a == "--mute") mute_audio = true;
 if (a == "--robot" && i + 1 < argc) {
 robot_path = argv[++i];
 source_requested = true;
 }
 if (a == "--sim" && i + 1 < argc) {
 sim_path = argv[++i];
 source_requested = true;
 }
 if (a == "--map" && i + 1 < argc) {
 map_path = argv[++i];
 source_requested = true;
 }
 if (a == "--assets" && i + 1 < argc) assets_path = argv[++i];
 if (a == "--scene" && i + 1 < argc) {
 scene_path = argv[++i];
 source_requested = true;
 }
 if (a == "--experience" && i + 1 < argc) {
 experience_path = argv[++i];
 source_requested = true;
 }
 if (a == "--locale" && i + 1 < argc) requested_locale = argv[++i];
 if (a == "--history-seconds" && i + 1 < argc) {
 char* end = nullptr;
 const double parsed = std::strtod(argv[++i], &end);
 if (end == argv[i] || *end != '\0' || !std::isfinite(parsed) ||
 parsed < 0.0) {
 std::fprintf(stderr,
 "bad --history-seconds: expected a finite non-negative value\n");
 return 1;
 }
 sandbox_history_seconds = parsed;
 }
 if (a == "--turbo") initial_turbo = true;
 if (a == "--quality" && i + 1 < argc) {
 const std::string quality = argv[++i];
 if (quality == "fast") render_quality = RenderQuality::kFast;
 else if (quality == "high") render_quality = RenderQuality::kHigh;
 else {
 std::fprintf(stderr, "bad --quality: expected fast or high\n");
 return 1;
 }
 }
 if (a == "--rate" && i + 1 < argc) {
 char* end = nullptr;
 const double parsed = std::strtod(argv[++i], &end);
 if (end == argv[i] || *end != '\0' || !std::isfinite(parsed) ||
 parsed < 0.05 || parsed > 64.0) {
 std::fprintf(stderr, "bad --rate: expected a value in [0.05, 64]\n");
 return 1;
 }
 initial_playback_rate = parsed;
 }
  }
 if (requested_locale != "auto" && requested_locale != "en" &&
 requested_locale != "zh-CN") {
 std::fprintf(stderr, "bad --locale: expected auto, en, or zh-CN\n");
 return 1;
 }
 g_zh_cn = use_zh_cn_locale(requested_locale);
 if (play_requested || (!source_requested && !sandbox_requested)) {
 experience_path = g_zh_cn
 ? "configs/experiences/zh-CN/first_contact.json"
 : "configs/experiences/first_contact.json";
 }
 ExperienceManifest experience;
 bool experience_loaded = false;
 if (!experience_path.empty()) {
 std::string experience_error;
 if (!lost_signal::sim::viewer::load_experience_manifest(
 experience_path, experience, &experience_error)) {
 std::fprintf(stderr, "bad experience file: %s\n",
 experience_error.c_str());
 return 1;
 }
 experience_loaded = true;
 if (robot_path.empty()) robot_path = experience.robot_path;
 if (sim_path.empty()) sim_path = experience.sim_path;
 if (map_path.empty()) map_path = experience.map_path;
 if (scene_path.empty()) scene_path = experience.scene_path;
 std::fprintf(stderr, "experience: %s\n", experience.experience_id.c_str());
 }
 std::optional<robot::platform::RobotConfig> robot_config;
 std::optional<sim::eval::SimConfig> sim_config;
  std::unique_ptr<sim::scenario::ScenarioSession> session;
 std::uint64_t runtime_invariant_digest = 0;
 sim::experiment::ExperienceRuntimeConfig runtime_config;
 runtime_config.robot_path = robot_path;
 runtime_config.sim_path = sim_path;
 runtime_config.map_path = map_path;
 if (experience_loaded) runtime_config.objectives = experience.objectives;
  if (!map_path.empty()) {
 sim::experiment::ExperienceRuntime runtime;
 std::string runtime_error;
 if (!sim::experiment::build_experience_runtime(
 runtime_config, runtime, &runtime_error)) {
 std::fprintf(stderr, "cannot build simulation: %s\n",
 runtime_error.c_str());
      return 1;
    }
 robot_config = std::move(runtime.robot_config);
 sim_config = std::move(runtime.sim_config);
 runtime_invariant_digest = runtime.invariant_digest;
 if (!runtime.used_robot_path.empty()) {
 std::fprintf(stderr, "robot config: %s\n",
 runtime.used_robot_path.c_str());
 }
 if (!runtime.used_sim_path.empty()) {
 std::fprintf(stderr, "sim config: %s\n", runtime.used_sim_path.c_str());
 }
 session = std::move(runtime.session);
  } else {
 std::string used_config, config_error;
 robot_config = robot::platform::load_default_robot_config(
 robot_path, &used_config, &config_error);
 if (!robot_path.empty() && !robot_config.has_value()) {
 std::fprintf(stderr, "bad --robot file: %s\n", config_error.c_str());
 return 1;
 }
 if (robot_config.has_value()) {
 session = std::make_unique<sim::scenario::ScenarioSession>(
 robot::platform::description_from_config(*robot_config),
 sim::eval::motor_from_config(*robot_config));
 } else {
 session = std::make_unique<sim::scenario::ScenarioSession>();
 }
 sim_config = sim::eval::load_default_sim_config(
 sim_path, nullptr, &config_error);
 if (!sim_path.empty() && !sim_config.has_value()) {
 std::fprintf(stderr, "bad --sim file: %s\n", config_error.c_str());
 return 1;
 }
 if (sim_config.has_value()) session->apply_sim_config(*sim_config);
    setup_demo(*session);
 session->prepare_world();
  }
 // Applied lidar mount for the viz ray fan: read back from the session
 // (the exact extrinsics generation runs — never a separately authored
 // copy, so a misconfigured mount shows up instead of hiding).
 sim::viewer::LidarMount session_lidar_mount;
 session->lidar_mount(session_lidar_mount.xyz_m, session_lidar_mount.rpy_rad);

 // A deque-backed recent window keeps Turbo and 64x sandbox sessions bounded
 // without changing full-run scoring. Zero explicitly requests full history.
 if (!experience_loaded && sandbox_history_seconds > 0.0) {
 constexpr double transition_s =
 static_cast<double>(sim::kScenarioTransitionNs) /
 static_cast<double>(sim::kNanosecondsPerSecond);
 const double record_period_s = sim_config.has_value()
 ? std::max(transition_s, sim_config->record_period_s) : transition_s;
 const double requested_steps = std::ceil(
 sandbox_history_seconds / record_period_s);
 const double size_limit = static_cast<double>(
 std::numeric_limits<std::size_t>::max());
 const std::size_t history_steps = requested_steps >= size_limit
 ? std::numeric_limits<std::size_t>::max()
 : static_cast<std::size_t>(std::max(1.0, requested_steps));
 session->set_episode_history_limit(history_steps);
 std::fprintf(stderr,
 "viewer history: recent %.1f sim seconds (%zu records); full score retained\n",
 sandbox_history_seconds, history_steps);
 }

 InitWindow(kScreenW, kScreenH, experience_loaded
 ? tr("Robot Simu - First Contact", "Robot Simu - 初次接触")
 : tr("Robot Simu - Simulation Lab", "Robot Simu - 仿真实验室"));
 load_ui_font(experience_loaded ? &experience : nullptr);
 if (assets_path.empty() && std::filesystem::is_directory("assets"))
 assets_path = "assets";
 sim::viewer::ViewerAudio viewer_audio(assets_path, !mute_audio);
 VisualAssets visual_assets;
 load_visual_assets(visual_assets, assets_path,
 robot_config.has_value() ? robot_config->robot_id : "ro1");
 SceneManifest visual_scene;
 bool scene_loaded = false;
 if (scene_path.empty() && !map_path.empty()) {
 std::filesystem::path candidate(map_path);
 candidate.replace_extension(".scene.json");
 if (std::filesystem::is_regular_file(candidate)) {
 scene_path = candidate.string();
 }
 }
 if (!scene_path.empty()) {
 std::string scene_error;
 if (!lost_signal::sim::viewer::load_scene_manifest(
 scene_path, visual_assets.manifest,
 session->setup_world().static_boxes(), visual_scene, &scene_error)) {
 std::fprintf(stderr, "bad scene file: %s\n", scene_error.c_str());
 unload_visual_assets(visual_assets);
 unload_ui_font();
 CloseWindow();
 return 1;
 }
 scene_loaded = true;
 std::fprintf(stderr, "scene: %s (%zu instances)\n", scene_path.c_str(),
 visual_scene.instances.size());
 }
  SetTargetFPS(60);

  sim::viewer::ViewTransform view(50.0f, 2.0, 1.0, kScreenW, kScreenH);
  view.fit(-4.0, -4.0, 8.0, 7.0);
 bool paused = experience_loaded;
 std::unique_ptr<ExperienceFlow> experience_flow;
 if (experience_loaded) {
 experience_flow = std::make_unique<ExperienceFlow>(
 experience_flow_config(experience));
 }
 ExperienceHitRegions experience_hit_regions;
 std::vector<ReviewKeyframe> review_keyframes;
 int selected_review_keyframe = 0;
 ExperimentUiState experiment_ui;
  bool show_lidar = false;
  bool show_estimate = false;
  bool show_actor_routes = false;
  bool follow = true;
 // Drive deterministic 20 ms simulation time from elapsed wall time. The
 // old fixed four-steps-per-frame loop demanded 4.8x real time at 60 FPS and
 // stacked periodic localization/planning work into the window thread.
 sim::viewer::PlaybackController::Config playback_config;
 playback_config.requested_rate = initial_playback_rate;
 sim::viewer::PlaybackController playback(playback_config);
 playback.set_turbo(initial_turbo);
 double previous_wall_s = GetTime();
  Vector2 last_mouse{0, 0};
 bool dragging = false;
 bool middle_dragging = false;
 Vector2 last_middle_mouse{0, 0};
  // Map editor: entering edit mode snapshots the world and pauses the sim;
  // leaving keeps the applied geometry.
  sim::viewer::MapEditor editor;
  bool edit_mode = false;
  std::string edit_msg;
  // 3D orbit camera over the same frame (V toggles 2D/3D, --3d starts in 3D).
 bool view3d = experience_loaded;
  // --shot N path: render N frames, screenshot, exit (visual smoke check).
  int shot_frames = -1;
  std::string shot_path;
 float camera_dist = 0.0f;
 float camera_yaw = 0.0f, camera_pitch = 0.0f;
 bool camera_angle_set = false;
 bool camera_target_set = false;
 double camera_target_x = 0.0, camera_target_y = 0.0;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--3d") view3d = true;
 if (a == "--camera-dist" && i + 1 < argc) {
 camera_dist = std::max(0.6f, static_cast<float>(std::atof(argv[++i])));
 }
 if (a == "--camera-target" && i + 2 < argc) {
 camera_target_set = true;
 camera_target_x = std::atof(argv[++i]);
 camera_target_y = std::atof(argv[++i]);
 }
 if (a == "--camera-yaw" && i + 1 < argc) {
  camera_yaw = static_cast<float>(std::atof(argv[++i]));
  camera_angle_set = true;
 }
 if (a == "--camera-pitch" && i + 1 < argc) {
  camera_pitch = static_cast<float>(std::atof(argv[++i]));
  camera_angle_set = true;
 }
    if (a == "--shot" && i + 2 < argc) {
      shot_frames = std::atoi(argv[++i]);
      shot_path = argv[++i];
    }
  }
  CameraState cam_st;
  if (!map_path.empty()) {
    // Frame the authored map instead of the demo window.
    double bx0, by0, bx1, by1;
    map_bounds(session->setup_world(), bx0, by0, bx1, by1);
    view.fit(bx0 - 2.0, by0 - 2.0, bx1 + 2.0, by1 + 2.0);
    cam_st.target_x = (bx0 + bx1) * 0.5;
    cam_st.target_y = (by0 + by1) * 0.5;
    cam_st.dist_m = view3d ? 4.2f : static_cast<float>(
        std::min(60.0, std::max(10.0, std::max(bx1 - bx0, by1 - by0) * 0.9)));
    cam_st.pitch_deg = view3d ? 34.0f : cam_st.pitch_deg;
    cam_st.target_z = view3d ? 0.18 : 0.0;
    follow = view3d;
 } else {
 cam_st.dist_m = 5.0f;
 cam_st.pitch_deg = 35.0f;
 cam_st.target_z = 0.20;
 }
 if (scene_loaded) {
 cam_st.dist_m = static_cast<float>(visual_scene.camera.distance_m);
 cam_st.yaw_deg = static_cast<float>(visual_scene.camera.yaw_deg);
 cam_st.pitch_deg = static_cast<float>(visual_scene.camera.pitch_deg);
 cam_st.target_z = visual_scene.camera.target_height_m;
 }
 if (camera_dist > 0.0f) cam_st.dist_m = camera_dist;
 if (camera_angle_set) {
  if (camera_yaw != 0.0f) cam_st.yaw_deg = camera_yaw;
  if (camera_pitch != 0.0f) cam_st.pitch_deg = camera_pitch;
 }
 if (camera_target_set) {
 cam_st.target_x = camera_target_x;
 cam_st.target_y = camera_target_y;
 follow = false;
 } else if (follow) {
 // A 3D follow view must start on the robot immediately. Starting at the
 // authored map center and easing toward the robot left it off-screen for
 // several seconds on large maps, even though the HUD already said FOLLOW.
 cam_st.target_x = session->setup_world().truth().body_x_m;
 cam_st.target_y = session->setup_world().truth().body_y_m;
 }
 float follow_base_distance_m = cam_st.dist_m;
 ParticleSystem motion_particles;
 SurfaceTraceSystem surface_traces;
 std::optional<std::size_t> effects_episode_cursor;
 std::string experience_action_error;
 const auto reset_experience_presentation = [&](bool reset_progress = true) {
 if (reset_progress) {
 experience_flow = std::make_unique<ExperienceFlow>(
 experience_flow_config(experience));
 experiment_ui = ExperimentUiState{};
 }
 review_keyframes.clear();
 selected_review_keyframe = 0;
 experience_hit_regions.clear();
 motion_particles.clear();
 surface_traces.clear();
 effects_episode_cursor.reset();
 viewer_audio.reset();
 const double requested_rate = playback.requested_rate();
 const bool turbo = playback.turbo();
 auto reset_playback_config = playback_config;
 reset_playback_config.requested_rate = requested_rate;
 playback = sim::viewer::PlaybackController(reset_playback_config);
 playback.set_turbo(turbo);
 previous_wall_s = GetTime();
 paused = true;
 show_lidar = false;
 show_estimate = false;
 show_actor_routes = false;
 dragging = false;
 middle_dragging = false;
 follow = view3d;

 cam_st = CameraState{};
 double bx0, by0, bx1, by1;
 map_bounds(session->setup_world(), bx0, by0, bx1, by1);
 view.fit(bx0 - 2.0, by0 - 2.0, bx1 + 2.0, by1 + 2.0);
 cam_st.target_x = (bx0 + bx1) * 0.5;
 cam_st.target_y = (by0 + by1) * 0.5;
 cam_st.dist_m = view3d ? 4.2f : static_cast<float>(
 std::min(60.0, std::max(10.0,
 std::max(bx1 - bx0, by1 - by0) * 0.9)));
 cam_st.pitch_deg = view3d ? 34.0f : cam_st.pitch_deg;
 cam_st.target_z = view3d ? 0.18 : 0.0;
 if (scene_loaded) {
 cam_st.dist_m = static_cast<float>(visual_scene.camera.distance_m);
 cam_st.yaw_deg = static_cast<float>(visual_scene.camera.yaw_deg);
 cam_st.pitch_deg = static_cast<float>(visual_scene.camera.pitch_deg);
 cam_st.target_z = visual_scene.camera.target_height_m;
 }
 if (camera_dist > 0.0f) cam_st.dist_m = camera_dist;
 if (camera_angle_set) {
 if (camera_yaw != 0.0f) cam_st.yaw_deg = camera_yaw;
 if (camera_pitch != 0.0f) cam_st.pitch_deg = camera_pitch;
 }
 if (camera_target_set) {
 cam_st.target_x = camera_target_x;
 cam_st.target_y = camera_target_y;
 follow = false;
 } else {
 cam_st.target_x = session->setup_world().truth().body_x_m;
 cam_st.target_y = session->setup_world().truth().body_y_m;
 }
 follow_base_distance_m = cam_st.dist_m;
 };
 const auto retry_experience = [&]() {
 if (!experience_loaded || runtime_config.map_path.empty() ||
 experience_flow->phase() != ExperiencePhase::kDebrief) {
 return false;
 }
 sim::experiment::ExperienceRuntime rebuilt;
 std::string runtime_error;
 if (!sim::experiment::build_experience_runtime(
 runtime_config, rebuilt, &runtime_error)) {
 experience_action_error = runtime_error;
 std::fprintf(stderr, "cannot retry experience: %s\n",
 runtime_error.c_str());
 return false;
 }
 session = std::move(rebuilt.session);
 robot_config = std::move(rebuilt.robot_config);
 sim_config = std::move(rebuilt.sim_config);
 runtime_invariant_digest = rebuilt.invariant_digest;
 session->lidar_mount(
 session_lidar_mount.xyz_m, session_lidar_mount.rpy_rad);
 if (!experience_flow->retry_current(0)) return false;
 if (experience_flow->run_kind() == ExperienceRunKind::kBaseline) {
 experiment_ui.baseline = {};
 experiment_ui.episodes.baseline.reset();
 experiment_ui.episodes.variant.reset();
 } else {
 experiment_ui.variant = {};
 experiment_ui.episodes.variant.reset();
 }
 experiment_ui.standalone.reset();
 experience_action_error.clear();
 reset_experience_presentation(false);
 paused = false;
 return true;
 };

 const auto start_visible_variant = [&]() {
 if (!experience_loaded || experience.experiments.empty() ||
 experience_flow->phase() != ExperiencePhase::kDebrief ||
 experience_flow->run_kind() != ExperienceRunKind::kBaseline ||
 !experiment_ui.baseline.recorded) {
 return false;
 }
 const auto& definition = experience.experiments.front();
 auto variant_config = runtime_config;
 variant_config.variable_override =
 sim::experiment::ExperienceVariableOverride{
 definition.controlled_variable, definition.variant_value};
 sim::experiment::ExperienceRuntime rebuilt;
 std::string runtime_error;
 if (!sim::experiment::build_experience_runtime(
 variant_config, rebuilt, &runtime_error)) {
 experience_action_error = runtime_error;
 return false;
 }
 session = std::move(rebuilt.session);
 robot_config = std::move(rebuilt.robot_config);
 sim_config = std::move(rebuilt.sim_config);
 runtime_invariant_digest = rebuilt.invariant_digest;
 runtime_config = std::move(variant_config);
 session->lidar_mount(
 session_lidar_mount.xyz_m, session_lidar_mount.rpy_rad);
 if (!experience_flow->start_variant(0)) return false;
 experiment_ui.variant = {};
 experiment_ui.episodes.variant.reset();
 experience_action_error.clear();
 reset_experience_presentation(false);
 paused = false;
 return true;
 };

 const auto advance_experience = [&]() {
 if (!experience_loaded || experience.next_experience_path.empty()) {
 return false;
 }
 const sim::experiment::EpisodeArtifact* active_artifact = nullptr;
 if (experience.progression.require_experiment) {
 if (experiment_ui.episodes.variant.has_value()) {
 active_artifact = &*experiment_ui.episodes.variant;
 } else if (experiment_ui.episodes.baseline.has_value()) {
 active_artifact = &*experiment_ui.episodes.baseline;
 }
 } else if (experiment_ui.standalone.has_value()) {
 active_artifact = &*experiment_ui.standalone;
 }
 if (active_artifact == nullptr ||
 !sim::experiment::evaluate_experience_gate(
 experience.progression, *active_artifact,
 experiment_ui.episodes).can_advance) {
 return false;
 }
 ExperienceManifest candidate_experience;
 std::string candidate_error;
 const std::string candidate_path = experience.next_experience_path;
 if (!lost_signal::sim::viewer::load_experience_manifest(
 candidate_path, candidate_experience, &candidate_error)) {
 experience_action_error = candidate_error;
 return false;
 }
 sim::experiment::ExperienceRuntimeConfig candidate_config;
 candidate_config.robot_path = candidate_experience.robot_path;
 candidate_config.sim_path = candidate_experience.sim_path;
 candidate_config.map_path = candidate_experience.map_path;
 candidate_config.objectives = candidate_experience.objectives;
 sim::experiment::ExperienceRuntime candidate_runtime;
 if (!sim::experiment::build_experience_runtime(
 candidate_config, candidate_runtime, &candidate_error)) {
 experience_action_error = candidate_error;
 return false;
 }
 SceneManifest candidate_scene;
 if (!lost_signal::sim::viewer::load_scene_manifest(
 candidate_experience.scene_path, visual_assets.manifest,
 candidate_runtime.session->setup_world().static_boxes(),
 candidate_scene, &candidate_error)) {
 experience_action_error = candidate_error;
 return false;
 }

 experience = std::move(candidate_experience);
 experience_path = candidate_path;
 runtime_config = std::move(candidate_config);
 robot_path = runtime_config.robot_path;
 sim_path = runtime_config.sim_path;
 map_path = runtime_config.map_path;
 scene_path = experience.scene_path;
 session = std::move(candidate_runtime.session);
 robot_config = std::move(candidate_runtime.robot_config);
 sim_config = std::move(candidate_runtime.sim_config);
 runtime_invariant_digest = candidate_runtime.invariant_digest;
 visual_scene = std::move(candidate_scene);
 scene_loaded = true;
 session->lidar_mount(
 session_lidar_mount.xyz_m, session_lidar_mount.rpy_rad);
 unload_visual_assets(visual_assets);
 load_visual_assets(visual_assets, assets_path,
 robot_config.has_value() ? robot_config->robot_id : "ro1");
 load_ui_font(&experience);
 experience_action_error.clear();
 reset_experience_presentation();
 SetWindowTitle(("Robot Simu - " + experience.title).c_str());
 std::fprintf(stderr, "experience: %s\n", experience.experience_id.c_str());
 return true;
 };

 while (!WindowShouldClose()) {
 const double wall_now_s = GetTime();
 // Playback must see the complete wall interval. If the window stalls for
 // longer than one input frame, PlaybackController deliberately drops the
 // excess simulation demand and reports LIMIT/actual RTF against that real
 // interval. Clamping here used to hide long stalls and overstate throughput.
 const double playback_wall_dt_s = std::max(
 0.0, wall_now_s - previous_wall_s);
 // Presentation smoothing is independent of simulation scheduling. Bound it
 // so a debugger pause or window drag cannot snap the follow camera.
 const double presentation_wall_dt_s = std::min(
 playback_wall_dt_s, 0.25);
 previous_wall_s = wall_now_s;
    view.set_screen(GetScreenWidth(), GetScreenHeight());
    if (IsKeyPressed(KEY_V)) view3d = !view3d;
    // Edit-mode picking: 2D uses the view transform, 3D the ground ray.
    const Vector2 mpos = GetMousePosition();
    double pick_x = 0.0, pick_y = 0.0;
    bool on_map = false;
    if (view3d) {
      const Camera3D pick_cam =
          camera_from_state(cam_st);
      on_map = screen_to_ground_3d(pick_cam, mpos, pick_x,
                                                pick_y);
    } else {
      on_map = view.screen_to_world(mpos.x, mpos.y, pick_x, pick_y);
    }
 if (experience_loaded) {
 const bool clicked = IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
 const Vector2 pointer = GetMousePosition();
 if (experience_flow->phase() == ExperiencePhase::kBriefing) {
 const bool hotspot_clicked = clicked && CheckCollisionPointRec(
 pointer, experience_hit_regions.world_hotspot);
 if (hotspot_clicked) {
 experience_flow->inspect(experience_flow->walkaround_index());
 } else if (IsKeyPressed(KEY_ENTER) &&
 !experience_flow->walkaround_visited(
 experience_flow->walkaround_index())) {
 // Keyboard/gamepad accessibility path for the same explicit inspection.
 experience_flow->inspect(experience_flow->walkaround_index());
 } else if (IsKeyPressed(KEY_ENTER) ||
 (clicked && CheckCollisionPointRec(
 pointer, experience_hit_regions.primary))) {
 experience_flow->continue_to_prediction();
 } else {
 if (clicked) {
 for (std::size_t index = 0;
 index < experience_hit_regions.walkaround.size(); ++index) {
 if (CheckCollisionPointRec(
 pointer, experience_hit_regions.walkaround[index])) {
 experience_flow->focus(index);
 break;
 }
 }
 }
 if (IsKeyPressed(KEY_RIGHT) ||
 (clicked && CheckCollisionPointRec(
 pointer, experience_hit_regions.next))) {
 experience_flow->inspect_next();
 }
 if (IsKeyPressed(KEY_LEFT) ||
 (clicked && CheckCollisionPointRec(
 pointer, experience_hit_regions.previous))) {
 experience_flow->inspect_previous();
 }
 }
 } else if (experience_flow->phase() == ExperiencePhase::kPrediction) {
 for (std::size_t i = 0; i < experience.choices.size(); ++i) {
 const int key = KEY_ONE + static_cast<int>(i);
 const bool clicked_choice = clicked &&
 i < experience_hit_regions.choices.size() &&
 CheckCollisionPointRec(pointer, experience_hit_regions.choices[i]);
 if (IsKeyPressed(key) || clicked_choice) {
 experience_flow->select_choice(i, session->episode().steps().size());
 paused = false;
 break;
 }
 }
 } else if (experience_flow->phase() == ExperiencePhase::kDebrief) {
 const bool requested_retry = clicked && CheckCollisionPointRec(
 pointer, experience_hit_regions.retry);
 if (requested_retry && retry_experience()) {
 continue;
 }
 const bool requested_next = clicked && CheckCollisionPointRec(
 pointer, experience_hit_regions.next_chapter);
 if (requested_next && advance_experience()) {
 continue;
 }
 if (!review_keyframes.empty()) {
 const int count = static_cast<int>(review_keyframes.size());
 if (IsKeyPressed(KEY_RIGHT) ||
 (clicked && CheckCollisionPointRec(
 pointer, experience_hit_regions.next))) {
 selected_review_keyframe = (selected_review_keyframe + 1) % count;
 }
 if (IsKeyPressed(KEY_LEFT) ||
 (clicked && CheckCollisionPointRec(
 pointer, experience_hit_regions.previous))) {
 selected_review_keyframe =
 (selected_review_keyframe + count - 1) % count;
 }
 }
 const bool requested_experiment = IsKeyPressed(KEY_A) ||
 (clicked && CheckCollisionPointRec(
 pointer, experience_hit_regions.primary));
 if (requested_experiment && start_visible_variant()) {
 continue;
 }
 }
 }
 // Edit-mode toggle + keys (window thread only).
 if (!experience_loaded && IsKeyPressed(KEY_M)) {
      edit_mode = !edit_mode;
      if (edit_mode) {
        editor.load_from(session->setup_world());
        paused = true;
      } else {
        editor.apply(session->setup_world());
      }
    }
    if (edit_mode) {
      if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && on_map) {
        editor.press(pick_x, pick_y);
      }
      if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) && on_map) {
        editor.drag(pick_x, pick_y);
      }
      if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) editor.release();
      if (IsKeyPressed(KEY_B) && on_map) editor.add_box(pick_x, pick_y);
      if (IsKeyPressed(KEY_P) && on_map) editor.add_place(pick_x, pick_y);
      if (IsKeyPressed(KEY_DELETE) || IsKeyPressed(KEY_BACKSPACE)) {
        editor.remove_selected();
      }
      if (IsKeyPressed(KEY_S)) {
        editor.save("viewer_map.json");
        edit_msg = editor.message();
      }
      // Live preview: paused sim, so pushing every dirty frame costs only
      // an engine model rebuild per edit, no stepping hazard.
      if (editor.dirty()) editor.apply(session->setup_world());
    }
    // Input (window thread only; sim stays deterministic).
 if (IsKeyPressed(KEY_SPACE) &&
 (!experience_loaded ||
 experience_flow->phase() == ExperiencePhase::kRunning)) {
 paused = !paused;
 }
 if (IsKeyPressed(KEY_N) && paused &&
 (!experience_loaded ||
 experience_flow->phase() == ExperiencePhase::kRunning)) {
 session->step_once();
 }
    if (IsKeyPressed(KEY_L)) show_lidar = !show_lidar;
    if (IsKeyPressed(KEY_E)) show_estimate = !show_estimate;
 const bool sim_truth_debug_allowed = !experience_loaded ||
 experience_flow->phase() == ExperiencePhase::kDebrief;
 if (!sim_truth_debug_allowed) show_actor_routes = false;
 if (IsKeyPressed(KEY_K) && sim_truth_debug_allowed) {
 show_actor_routes = !show_actor_routes;
 }
 if (IsKeyPressed(KEY_Q)) {
 render_quality = render_quality == RenderQuality::kHigh
 ? RenderQuality::kFast : RenderQuality::kHigh;
 }
    if (IsKeyPressed(KEY_F)) follow = !follow;
 if (IsKeyPressed(KEY_R)) {
 cam_st.yaw_deg = 35.0f;
 cam_st.pitch_deg = 50.0f;
 cam_st.dist_m = 12.0f;
 follow = false;
 }
 if (IsKeyPressed(KEY_T)) playback.set_turbo(!playback.turbo());
 const bool playback_controls_visible = !experience_loaded ||
 experience_flow->phase() == ExperiencePhase::kRunning;
 if (playback_controls_visible &&
 IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
 const PlaybackHitRegions controls = playback_hit_regions();
 const Vector2 pointer = GetMousePosition();
 constexpr std::array<double, 4> kRates{1.0, 2.0, 4.0, 8.0};
 for (std::size_t index = 0; index < controls.rates.size(); ++index) {
 if (!CheckCollisionPointRec(pointer, controls.rates[index])) continue;
 playback.set_turbo(false);
 playback.set_requested_rate(kRates[index]);
 break;
 }
 if (CheckCollisionPointRec(pointer, controls.turbo)) {
 playback.set_turbo(true);
 }
 }
 if (IsKeyPressed(KEY_EQUAL) && playback.requested_rate() < 64.0) {
 playback.set_turbo(false);
 playback.set_requested_rate(
 std::min(64.0, playback.requested_rate() * 2.0));
 }
 if (IsKeyPressed(KEY_MINUS) && playback.requested_rate() > 0.125) {
 playback.set_turbo(false);
 playback.set_requested_rate(
 std::max(0.125, playback.requested_rate() * 0.5));
 }
    if (IsKeyPressed(KEY_ZERO)) view.fit(-4.0, -4.0, 8.0, 7.0);
    const float wheel = GetMouseWheelMove();
    if (view3d) {
      if (wheel != 0.0f) {
        cam_st.dist_m *= wheel > 0.0f ? 1.0f / 1.15f : 1.15f;
 if (cam_st.dist_m < 0.6f) cam_st.dist_m = 0.6f;
        if (cam_st.dist_m > 60.0f) cam_st.dist_m = 60.0f;
 follow_base_distance_m = cam_st.dist_m;
      }
    } else if (wheel != 0.0f) {
      view.set_screen(GetScreenWidth(), GetScreenHeight());
      view.zoom_at(GetMouseX(), GetMouseY(),
                   wheel > 0.0f ? 1.15f : 1.0f / 1.15f);
    }
    if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
      const Vector2 m = GetMousePosition();
      if (dragging) {
        if (view3d) {
          cam_st.yaw_deg += (m.x - last_mouse.x) * 0.3f;
          cam_st.pitch_deg += (last_mouse.y - m.y) * 0.3f;
          if (cam_st.pitch_deg < 10.0f) cam_st.pitch_deg = 10.0f;
          if (cam_st.pitch_deg > 85.0f) cam_st.pitch_deg = 85.0f;
        } else {
          view.set_screen(GetScreenWidth(), GetScreenHeight());
          view.pan_px(last_mouse.x - m.x, last_mouse.y - m.y);
        }
        follow = false;
      }
      last_mouse = m;
      dragging = true;
    } else {
      dragging = false;
    }
 if (view3d && IsMouseButtonDown(MOUSE_BUTTON_MIDDLE)) {
 const Vector2 m = GetMousePosition();
 if (middle_dragging) {
 const float scale = cam_st.dist_m * 0.0025f;
 cam_st.target_x -= (m.x - last_middle_mouse.x) * scale;
 cam_st.target_y += (m.y - last_middle_mouse.y) * scale;
 follow = false;
 }
 last_middle_mouse = m;
 middle_dragging = true;
 } else {
 middle_dragging = false;
 }

 playback.begin_frame(playback_wall_dt_s);
 if (!paused && !session->objectives_done()) {
 const auto frame_work_begin = std::chrono::steady_clock::now();
 while (true) {
 const double work_ms = std::chrono::duration<double, std::milli>(
 std::chrono::steady_clock::now() - frame_work_begin).count();
 if (!playback.step_due(work_ms)) break;
 const auto step_begin = std::chrono::steady_clock::now();
        session->step_once();
 // Advance SIM-owned presentation effects before closing this transition's
 // work sample. Turbo may execute many transitions between draws; consuming
 // each completed record here preserves impacts and single-wheel slip while
 // keeping particle, trace, and audio work inside the same bounded UI budget.
 const auto& live_steps = session->episode().steps();
 if (!live_steps.empty()) {
 const std::size_t live_index = live_steps.size() - 1;
 const sim::viewer::Frame effect_frame = sim::viewer::build_frame(
 session->setup_world(), live_steps.back(), {}, {}, {},
 session_lidar_mount);
 motion_particles.advance(effect_frame);
 surface_traces.advance(effect_frame);
 const bool live_audio = !experience_loaded ||
 experience_flow->phase() == ExperiencePhase::kRunning;
 viewer_audio.update(effect_frame, false, false,
 playback.turbo() || !live_audio);
 effects_episode_cursor = live_index;
 }
 const double step_ms = std::chrono::duration<double, std::milli>(
 std::chrono::steady_clock::now() - step_begin).count();
 playback.on_step(step_ms);
        if (session->objectives_done()) break;
      }
 } else {
 // Paused wall time is not simulation debt.
 playback.discard_debt();
    }
 playback.end_frame();
 if (experience_loaded &&
 experience_flow->update(session->episode().steps().size(),
 session->objectives_done())) {
 paused = true;
 sim::viewer::KeyframeReviewConfig review_config;
 if (robot_config.has_value()) {
 review_config.maximum_forward_speed_mps =
 robot_config->arrival.maximum_forward_speed_mps;
 review_config.maximum_yaw_rate_radps =
 robot_config->arrival.maximum_yaw_rate_radps;
 review_config.settle_samples = robot_config->arrival.settle_cycles;
 }
 sim::viewer::KeyframeReviewOutcome review_outcome;
 review_outcome.terminal = true;
 review_outcome.timed_out = experience_flow->timed_out();
 const auto terminal_score = session->score();
 review_outcome.sim_accepted = !review_outcome.timed_out &&
 session->objectives_done() && session->objectives_passed() &&
 terminal_score.collision_steps == 0;
 if (review_outcome.timed_out) {
 review_outcome.rejection_reasons.push_back("experiment_time_limit");
 }
 if (!session->objectives_done()) {
 review_outcome.rejection_reasons.push_back("mission_incomplete");
 }
 if (terminal_score.collision_steps > 0) {
 review_outcome.rejection_reasons.push_back("collision");
 }
 for (const auto& result : session->objectives().results()) {
 review_outcome.rejection_reasons.insert(
 review_outcome.rejection_reasons.end(), result.failure_reasons.begin(),
 result.failure_reasons.end());
 }
 review_keyframes = sim::viewer::extract_review_keyframes(
 session->episode(), review_config, std::move(review_outcome));
 selected_review_keyframe = 0;
 for (std::size_t index = 0; index < review_keyframes.size(); ++index) {
 if (review_keyframes[index].kind ==
 sim::viewer::ReviewKeyframeKind::kRobotDone) {
 selected_review_keyframe = static_cast<int>(index);
 break;
 }
 }
 VisibleRunSummary run_summary;
 run_summary.recorded = true;
 run_summary.timed_out = experience_flow->timed_out();
 run_summary.objectives_done = session->objectives_done();
 run_summary.objectives_passed = session->objectives_passed();
 run_summary.scan_period_s = sim_config.has_value()
 ? sim_config->scan_period_s : 0.0;
 run_summary.score = terminal_score;
 const lost_signal::sim::viewer::ExperienceVariableSpec* variable = nullptr;
 double controlled_value = 0.0;
 if (!experience.experiments.empty() && sim_config.has_value()) {
 variable = &lost_signal::sim::viewer::experience_variable_spec(
 experience.experiments.front().controlled_variable);
 controlled_value = sim::experiment::experience_variable_value(
 experience.experiments.front().controlled_variable, *sim_config);
 }
 auto artifact = sim::experiment::capture_episode_artifact(
 experience_flow->run_kind() == ExperienceRunKind::kBaseline
 ? "baseline" : "variant",
 variable, controlled_value, runtime_invariant_digest,
 experience_flow->timed_out(), *session);
 if (experience_flow->run_kind() == ExperienceRunKind::kBaseline) {
 experiment_ui.baseline = run_summary;
 if (experience.experiments.empty()) {
 experiment_ui.standalone = std::move(artifact);
 } else {
 experiment_ui.episodes.baseline = std::move(artifact);
 }
 } else {
 experiment_ui.variant = run_summary;
 experiment_ui.episodes.variant = std::move(artifact);
 }
 if (!session->episode().steps().empty()) {
 const auto terminal_frame = sim::viewer::build_frame(
 session->setup_world(), session->episode().steps().back(), {}, {}, {},
 session_lidar_mount);
 viewer_audio.update(terminal_frame, true,
 session->objectives_done() && session->objectives_passed() &&
 terminal_score.collision_steps == 0,
 playback.turbo());
 }
 }
    const auto& steps = session->episode().steps();
 if (steps.empty()) {
 if (!experience_loaded) continue;
 sim::eval::EvalStep preview_step;
 preview_step.truth_after = session->setup_world().truth();
 const sim::viewer::Frame preview = sim::viewer::build_frame(
 session->setup_world(), preview_step, {}, {}, {}, session_lidar_mount);
 BeginDrawing();
 ClearBackground(view3d ? scene_background(
 scene_loaded ? &visual_scene : nullptr)
 : Color{18, 31, 34, 255});
 Rectangle projected_world_hotspot{};
 if (view3d) {
 std::string component_id = "chassis";
 if (!experience.walkaround.empty()) {
 const int index = std::clamp(
 static_cast<int>(experience_flow->walkaround_index()), 0,
 static_cast<int>(experience.walkaround.size()) - 1);
 component_id = experience.walkaround[static_cast<std::size_t>(index)]
 .component_id;
 }
 const WalkaroundView walkaround = walkaround_view(
 preview, component_id, cam_st);
 const RenderQuality active_quality = playback.turbo()
 ? RenderQuality::kFast : render_quality;
 if (active_quality == RenderQuality::kHigh &&
 !draw_frame_3d_postprocessed(
 preview, session->setup_world(), walkaround.camera,
 motion_particles, surface_traces, false, false, false, false, visual_assets,
 scene_loaded ? &visual_scene : nullptr, &session_lidar_mount)) {
 draw_frame_3d_direct(
 preview, session->setup_world(), walkaround.camera,
 motion_particles, surface_traces, false, false, false, false, visual_assets,
 scene_loaded ? &visual_scene : nullptr, &session_lidar_mount);
 } else if (active_quality == RenderQuality::kFast) {
 draw_frame_3d_direct(
 preview, session->setup_world(), walkaround.camera,
 motion_particles, surface_traces, false, false, false, false, visual_assets,
 scene_loaded ? &visual_scene : nullptr, &session_lidar_mount);
 }
 const Vector2 marker = GetWorldToScreen(
 walkaround.focus, walkaround.camera);
 if (marker.x >= 0.0f && marker.x < GetScreenWidth() &&
 marker.y >= 0.0f && marker.y < GetScreenHeight()) {
 const Rectangle hotspot{marker.x - 28.0f, marker.y - 28.0f,
 56.0f, 56.0f};
 projected_world_hotspot = hotspot;
 DrawCircle(static_cast<int>(marker.x), static_cast<int>(marker.y),
 12.0f, Color{55, 218, 202, 45});
 DrawCircleLines(static_cast<int>(marker.x), static_cast<int>(marker.y),
 19.0f, Color{55, 218, 202, 230});
 DrawCircleLines(static_cast<int>(marker.x), static_cast<int>(marker.y),
 25.0f, Color{55, 218, 202, 110});
 draw_ui_text(tr("CLICK PART", "点击部件"),
 static_cast<int>(marker.x + 31.0f),
 static_cast<int>(marker.y - 8.0f), 12, Color{55, 218, 202, 255});
 }
 } else {
 draw_frame_2d(preview, view, false, false, false);
 }
 draw_experience_overlay(experience, experience_flow->phase(),
 experience_flow->run_kind(), experience_flow->comparison_complete(),
 sim_config.has_value() ? sim_config->scan_period_s : 0.0,
 experience_flow->selected_choice(),
 static_cast<int>(experience_flow->walkaround_index()),
 experience_flow->walkaround_visited_count(),
 experience_flow->required_walkaround_visited_count(),
 experience_flow->required_walkaround_count(),
 experience_flow->walkaround_complete(), review_keyframes,
 selected_review_keyframe, false, false, {}, &preview, &experiment_ui,
 false, PredictionAssessment::kPending, nullptr, experience_action_error,
 true,
 &experience_hit_regions);

 // draw_experience_overlay clears and rebuilds the panel hit regions. Restore
 // the projected 3D hotspot after it so the next input frame can click the
 // actual part rather than a text tab.
 if (projected_world_hotspot.width > 0.0f) {
 experience_hit_regions.world_hotspot = projected_world_hotspot;
 }

 EndDrawing();
 if (shot_frames > 0 && --shot_frames == 0) {
 TakeScreenshot(shot_path.c_str());
 break;
 }
 continue;
 }
 std::size_t presented_index = steps.size() - 1;
 if (experience_loaded &&
 experience_flow->phase() == ExperiencePhase::kDebrief &&
 !review_keyframes.empty()) {
 const int selected = std::clamp(selected_review_keyframe, 0,
 static_cast<int>(review_keyframes.size()) - 1);
 const std::size_t requested =
 review_keyframes[static_cast<std::size_t>(selected)].episode_index;
 if (requested < steps.size()) presented_index = requested;
 }
 const sim::eval::EvalStep& presented = steps[presented_index];
    std::vector<std::string> reached;
    for (const auto& r : session->objectives().results()) {
 if (r.successful() && r.completed_step <= presented.truth_after.step) {
 reached.push_back(r.place_id);
 }
    }
 const auto historical_frame = sim::viewer::build_episode_frame(
 session->setup_world(), session->episode(), presented_index, reached,
        edit_mode ? sim::viewer::EditorSelection{editor.selected_box(),
                                                 editor.selected_corner(),
                                                 editor.selected_place()}
                  : sim::viewer::EditorSelection{},
        session_lidar_mount);
 if (!historical_frame.has_value()) continue;
 const sim::viewer::Frame& frame = *historical_frame;
 // Presentation consumes every completed physics frame even when Turbo
 // executes several scenario transitions between two draws. Rendering may
 // sample sparsely; physical events (contact traces, spray, dust and impact
 // sparks) must remain deterministic and replayable. A backward review seek
 // rebuilds the bounded pools from episode history instead of carrying
 // effects from the future.
 if (effects_episode_cursor.has_value() &&
 presented_index < *effects_episode_cursor) {
 motion_particles.clear();
 surface_traces.clear();
 effects_episode_cursor.reset();
 }
 const std::size_t effects_begin = effects_episode_cursor.has_value()
 ? *effects_episode_cursor + 1 : 0;
 for (std::size_t index = effects_begin; index < presented_index; ++index) {
 const sim::viewer::Frame event_frame = sim::viewer::build_frame(
 session->setup_world(), steps[index], {}, {}, {}, session_lidar_mount);
 motion_particles.advance(event_frame);
 surface_traces.advance(event_frame);
 const bool live_audio = !experience_loaded ||
 experience_flow->phase() == ExperiencePhase::kRunning;
 viewer_audio.update(event_frame, false, false,
 playback.turbo() || !live_audio);
 }
 if (!effects_episode_cursor.has_value() ||
 presented_index > *effects_episode_cursor) {
 motion_particles.advance(frame);
 surface_traces.advance(frame);
 effects_episode_cursor = presented_index;
 }
 const bool live_audio = !experience_loaded ||
 experience_flow->phase() == ExperiencePhase::kRunning;
 const auto audio_score = session->score();
 viewer_audio.update(frame, session->objectives_done(),
 session->objectives_done() && session->objectives_passed() &&
 audio_score.collision_steps == 0,
 playback.turbo() || !live_audio);
    if (follow) {
      view.set_screen(GetScreenWidth(), GetScreenHeight());
      // Re-center without touching zoom: pan by the screen-space error.
      const auto cur =
 view.world_to_screen(presented.truth_after.body_x_m,
 presented.truth_after.body_y_m);
      view.pan_px(GetScreenWidth() * 0.5f - cur.x_px,
                  GetScreenHeight() * 0.5f - cur.y_px);
 const auto& camera_config = scene_loaded
 ? visual_scene.camera : lost_signal::sim::viewer::SceneCamera{};
 const double speed_mps = std::hypot(
 presented.truth_after.forward_mps, presented.truth_after.lateral_mps);
 const double lookahead_m =
 std::abs(presented.truth_after.forward_mps) * camera_config.lookahead_s;
 const double direction =
 presented.truth_after.forward_mps < 0.0 ? -1.0 : 1.0;
 const double desired_x = presented.truth_after.body_x_m +
 direction * lookahead_m * std::cos(presented.truth_after.body_yaw_rad);
 const double desired_y = presented.truth_after.body_y_m +
 direction * lookahead_m * std::sin(presented.truth_after.body_yaw_rad);
 const double blend = camera_config.follow_smoothing_s > 0.0
 ? 1.0 - std::exp(
 presentation_wall_dt_s / -camera_config.follow_smoothing_s)
 : 1.0;
 cam_st.target_x += (desired_x - cam_st.target_x) * blend;
 cam_st.target_y += (desired_y - cam_st.target_y) * blend;
 const double desired_distance = std::clamp(
 static_cast<double>(follow_base_distance_m) +
 speed_mps * camera_config.speed_distance_gain_s,
 camera_config.minimum_distance_m, camera_config.maximum_distance_m);
 cam_st.dist_m += static_cast<float>(
 (desired_distance - cam_st.dist_m) * blend);
    }

    BeginDrawing();
 ClearBackground(view3d ? scene_background(
 scene_loaded ? &visual_scene : nullptr) : RAYWHITE);
    if (view3d) {
      const Camera3D cam = camera_from_state(cam_st);
 const RenderQuality active_quality = playback.turbo()
 ? RenderQuality::kFast : render_quality;
 if (active_quality == RenderQuality::kHigh &&
 !draw_frame_3d_postprocessed(
 frame, session->setup_world(), cam, motion_particles, surface_traces,
 show_lidar, show_estimate, show_actor_routes, true, visual_assets,
 scene_loaded ? &visual_scene : nullptr, &session_lidar_mount)) {
 draw_frame_3d_direct(
 frame, session->setup_world(), cam, motion_particles, surface_traces,
 show_lidar, show_estimate, show_actor_routes, true, visual_assets,
 scene_loaded ? &visual_scene : nullptr, &session_lidar_mount);
 } else if (active_quality == RenderQuality::kFast) {
 draw_frame_3d_direct(
 frame, session->setup_world(), cam, motion_particles, surface_traces,
 show_lidar, show_estimate, show_actor_routes, true, visual_assets,
 scene_loaded ? &visual_scene : nullptr, &session_lidar_mount);
 }
    } else {
      view.set_screen(GetScreenWidth(), GetScreenHeight());
 draw_frame_2d(frame, view, show_lidar, show_estimate,
 show_actor_routes);
    }
 const auto playback_stats = playback.statistics();
 const RenderQuality active_quality = playback_stats.turbo
 ? RenderQuality::kFast : render_quality;
 char speed[160];
 if (playback_stats.turbo) {
 std::snprintf(speed, sizeof(speed),
 "req=TURBO actual=%.2fx %.0fstep/s p99=%.1fms%s",
 playback_stats.actual_rtf, playback_stats.steps_per_s,
 playback_stats.tick_p99_ms,
 playback_stats.throttled ? " LIMIT" : "");
 } else {
 std::snprintf(speed, sizeof(speed),
 "req=%.2fx actual=%.2fx %.0fstep/s p99=%.1fms%s",
 playback_stats.requested_rate, playback_stats.actual_rtf,
 playback_stats.steps_per_s, playback_stats.tick_p99_ms,
 playback_stats.throttled ? " LIMIT" : "");
 }
 char hud[512];
 std::snprintf(hud, sizeof(hud),
 "step %llu x=%.2f y=%.2f yaw=%.2f clr=%.2f obj %d/%zu %s %dfps cam=%s quality=%s %s",
 static_cast<unsigned long long>(frame.step), frame.body.x_m,
 frame.body.y_m, frame.body.yaw_rad, frame.static_clearance_m,
 static_cast<int>(reached.size()), session->objectives().total(), speed,
 GetFPS(), follow ? "FOLLOW" : "FREE",
 render_quality_name(active_quality),
 paused ? "PAUSED"
 : session->objectives_done()
 ? (session->objectives_passed() ? "PASSED" : "FAILED")
 : "RUN");
 DrawText(hud, 10, 10, 14, BLACK);
 if (show_estimate) {
 DrawText("ROBOT AI cyan=perception/safety sweep purple=prediction rings=uncertainty",
 10, 32, 13, Color{78, 54, 132, 255});
 }
    if (edit_mode) {
      char ebuf[256];
      std::snprintf(ebuf, sizeof(ebuf),
                    "EDIT  click/drag move  B box  P place  Del remove  S save viewer_map.json  M exit  %s",
                    edit_msg.c_str());
      DrawText(ebuf, 10, 32, 14, RED);
    }
 const char* control_help = sim_truth_debug_allowed
 ? "space pause  N step  L lidar  E ROBOT AI  K SIM truth debug  F follow  Q quality  0 fit  +/- speed  V 2D/3D  M edit  right-drag pan/orbit  wheel zoom"
 : "space pause  N step  L lidar  E ROBOT AI  F follow  Q quality  0 fit  +/- speed  V 2D/3D  right-drag pan/orbit  wheel zoom";
 DrawText(control_help, 10, GetScreenHeight() - 24, 12, DARKGRAY);
 DrawText("T turbo", GetScreenWidth() - 64, GetScreenHeight() - 24,
 12, DARKGRAY);
 if (experience_loaded) {
 PredictionAssessment prediction_assessment =
 PredictionAssessment::kPending;
 const int selected_choice = experience_flow->selected_choice();
 if (selected_choice >= 0 &&
 selected_choice < static_cast<int>(experience.choices.size())) {
 prediction_assessment = lost_signal::sim::viewer::assess_prediction(
 static_cast<std::size_t>(selected_choice), experience.choices,
 experience_evidence(*session, experience_flow->timed_out(), experience,
 experiment_ui));
 }
 std::optional<sim::experiment::ExperienceGateVerdict> gate_verdict;
 const sim::experiment::EpisodeArtifact* active_artifact = nullptr;
 if (experience.progression.require_experiment) {
 if (experiment_ui.episodes.variant.has_value()) {
 active_artifact = &*experiment_ui.episodes.variant;
 } else if (experiment_ui.episodes.baseline.has_value()) {
 active_artifact = &*experiment_ui.episodes.baseline;
 }
 } else if (experiment_ui.standalone.has_value()) {
 active_artifact = &*experiment_ui.standalone;
 }
 if (active_artifact != nullptr) {
 gate_verdict = sim::experiment::evaluate_experience_gate(
 experience.progression, *active_artifact, experiment_ui.episodes);
 }
 draw_experience_overlay(experience, experience_flow->phase(),
 experience_flow->run_kind(), experience_flow->comparison_complete(),
 sim_config.has_value() ? sim_config->scan_period_s : 0.0,
 selected_choice,
 static_cast<int>(experience_flow->walkaround_index()),
 experience_flow->walkaround_visited_count(),
 experience_flow->required_walkaround_visited_count(),
 experience_flow->required_walkaround_count(),
 experience_flow->walkaround_complete(), review_keyframes,
 selected_review_keyframe, session->objectives_done(),
 session->objectives_passed(), session->score(), &frame, &experiment_ui,
 experience_flow->timed_out(), prediction_assessment,
 gate_verdict.has_value() ? &*gate_verdict : nullptr,
 experience_action_error,
 true,
 &experience_hit_regions);
 }
 if (playback_controls_visible) {
 draw_playback_controls(playback_hit_regions(), playback);
 }
    EndDrawing();
    if (shot_frames > 0 && --shot_frames == 0) {
      TakeScreenshot(shot_path.c_str());
      break;
    }
  }
 const auto final_playback_stats = playback.statistics();
 std::fprintf(stderr,
 "viewer performance: requested=%s actual=%.2fx steps=%.0f/s tick_p99=%.2fms quality=%s\n",
 final_playback_stats.turbo ? "turbo" : "rate",
 final_playback_stats.actual_rtf, final_playback_stats.steps_per_s,
 final_playback_stats.tick_p99_ms,
 render_quality_name(final_playback_stats.turbo
 ? RenderQuality::kFast : render_quality));
 unload_visual_assets(visual_assets);
 unload_ui_font();
 CloseWindow();
  return 0;
}
