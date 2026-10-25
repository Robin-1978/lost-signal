// SIM-side live viewer (2D top-down). Reads truth + recorded episode state
// through ScenarioSession (SIM composition root) and draws it with raylib.
// Never publishes to ROBOT: the session steps headless exactly as without
// a window; the viewer only observes. Headless CI never runs this binary
// (needs a display); frame logic is unit-tested in test_viewer_frame.

#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>

#include <cmath>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <filesystem>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include "robot/core/json.hpp"
#include "robot/platform/robot_config.hpp"
#include "sim/eval/robot_config.hpp"
#include "sim/eval/sim_config.hpp"
#include "sim/eval/world_io.hpp"
#include "sim/scenario/actor_sidecar.hpp"
#include "sim/scenario/scenario_session.hpp"
#include "sim/viewer/frame_builder.hpp"
#include "sim/viewer/map_editor.hpp"
#include "sim/viewer/view_transform.hpp"
#include "sim/world/actor_behavior.hpp"
#include "sim/world/terrain.hpp"

namespace {
using lost_signal::sim::viewer::Frame;
using lost_signal::sim::viewer::FrameBody;
using lost_signal::sim::viewer::FrameLinkPose;
using lost_signal::sim::viewer::ViewTransform;

constexpr int kScreenW = 1100;
constexpr int kScreenH = 750;

struct VisualAssets {
 Shader lighting{};
 Model chassis{};
 Model wheel{};
 Model lidar{};
 Model caster_fork{};
 Model caster_wheel{};
 Model track{};
 Model forklift{};
 Model worker{};
 bool loaded = false;
};

bool model_ready(const Model& model) { return model.meshCount > 0; }

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
 for (int i = 0; i < model.materialCount; ++i)
 model.materials[i].shader = shader;
}

BoundingBox raylib_asset_bounds(const Model& model, bool zup_converted) {
 (void)zup_converted;
 const BoundingBox b = GetModelBoundingBox(model);
 const float x0 = std::min(b.min.x, b.max.x);
 const float x1 = std::max(b.min.x, b.max.x);
 const float y0 = std::min(b.min.y, b.max.y);
 const float y1 = std::max(b.min.y, b.max.y);
 const float z0 = std::min(b.min.z, b.max.z);
 const float z1 = std::max(b.min.z, b.max.z);
 // raylib's GetModelBoundingBox already applies Model::transform. Keep the
 // normalized bounds in raylib coordinates; manually applying the GLB axis
 // permutation here would convert the same asset twice.
 return {{x0, y0, z0}, {x1, y1, z1}};
}

Model load_optional_model(const std::filesystem::path& path,
 bool convert_zup = true) {
 if (!std::filesystem::is_regular_file(path)) return {};
 Model model = LoadModel(path.string().c_str());
 if (!model_ready(model)) return {};
 // Content GLBs use Z-up; the SIM viewer uses raylib's Y-up convention.
 // Keep this conversion at the asset boundary so every component shares
 // the same upright frame.
 if (convert_zup) model.transform = MatrixRotateX(-PI / 2.0f);
 return model;
}

void load_visual_assets(VisualAssets& assets, const std::string& root,
 const std::string& robot_id) {
 if (root.empty()) return;
 const std::filesystem::path base(root);
 std::filesystem::path robot_dir = base / "models" / "robots" / robot_id;
 // Legacy robot configs use descriptive ids (ro1-tracked, ro1-mecanum,
 // ...), while the content tree keys models by the master asset family.
 // Resolve that alias at the SIM asset boundary so every chassis gets its
 // authored GLB without changing the hardware contract or leaking content
 // ids into ROBOT.
 if (!std::filesystem::is_directory(robot_dir)) {
 const auto has = [&](const char* token) {
 return robot_id.find(token) != std::string::npos;
 };
 const char* family = has("tracked") ? "ro3"
 : has("mecanum") ? "ro5"
 : has("swerve") ? "ro4"
 : has("ackermann") ? "ro2"
 : "ro1";
 robot_dir = base / "models" / "robots" / family;
 }
 const auto actor_dir = base / "models" / "actors";
 assets.chassis = load_optional_model(robot_dir / "chassis.glb");
 if (!model_ready(assets.chassis))
 assets.chassis = load_optional_model(robot_dir / "hull.glb");
 // Wheel links are authored in their physical joint frame (+Z axle). Keep
 // their transform explicit in the joint renderer; do not hide it in the
 // global asset conversion used by chassis/actors.
 assets.wheel = load_optional_model(robot_dir / "wheel_standard.glb", false);
 if (!model_ready(assets.wheel))
 assets.wheel = load_optional_model(robot_dir / "swerve_module.glb", false);
 if (!model_ready(assets.wheel))
 assets.wheel = load_optional_model(robot_dir / "roller_left.glb", false);
 assets.lidar = load_optional_model(robot_dir / "lidar_standard.glb");
 if (!model_ready(assets.lidar))
 assets.lidar = load_optional_model(robot_dir / "lidar_long.glb");
 assets.caster_fork = load_optional_model(robot_dir / "caster_fork.glb");
 assets.caster_wheel = load_optional_model(robot_dir / "caster_wheel.glb", false);
 assets.track = load_optional_model(robot_dir / "track.glb");
 assets.forklift = load_optional_model(actor_dir / "forklift.glb");
 assets.worker = load_optional_model(actor_dir / "worker.glb");
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
 if (IsShaderValid(assets.lighting)) {
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
 }
 assets.loaded = model_ready(assets.chassis) || model_ready(assets.wheel) ||
 model_ready(assets.lidar) || model_ready(assets.caster_fork) ||
 model_ready(assets.caster_wheel) ||
 model_ready(assets.track) || model_ready(assets.forklift) ||
 model_ready(assets.worker);
}

void unload_visual_assets(VisualAssets& assets) {
 if (model_ready(assets.chassis)) UnloadModel(assets.chassis);
 if (model_ready(assets.wheel)) UnloadModel(assets.wheel);
 if (model_ready(assets.lidar)) UnloadModel(assets.lidar);
 if (model_ready(assets.caster_fork)) UnloadModel(assets.caster_fork);
 if (model_ready(assets.caster_wheel)) UnloadModel(assets.caster_wheel);
 if (model_ready(assets.track)) UnloadModel(assets.track);
 if (model_ready(assets.forklift)) UnloadModel(assets.forklift);
 if (model_ready(assets.worker)) UnloadModel(assets.worker);
 if (IsShaderValid(assets.lighting)) UnloadShader(assets.lighting);
 assets = {};
}

void draw_fitted_model(const Model& model, Vector3 position, float radius,
 float height, Color tint, float rotation_deg = 0.0f) {
 if (!model_ready(model)) return;
 const BoundingBox bounds = raylib_asset_bounds(model, true);
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
 double ground_z_m, double radius_m, bool pedestrian, double heading_rad) {
 if (!model_ready(model)) return;
 const float reference_radius = pedestrian ? 0.25f : 0.45f;
 const float scale = static_cast<float>(radius_m) /
 std::max(1e-3f, reference_radius);
 const BoundingBox bounds = raylib_asset_bounds(model, true);
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

// Presentation-only particles. The phase is derived from the recorded SIM
// step and live wheel state, so screenshots and replay views are stable while
// dust/slip still follows the actual plant motion. Nothing here is fed back
// to the sensor suite or ROBOT.
inline float visual_hash(std::uint64_t step, int a, int b) {
 std::uint64_t v = step * 0x9e3779b97f4a7c15ULL;
 v ^= static_cast<std::uint64_t>(a + 17) * 0xbf58476d1ce4e5b9ULL;
 v ^= static_cast<std::uint64_t>(b + 31) * 0x94d049bb133111ebULL;
 v ^= v >> 30;
 v *= 0xbf58476d1ce4e5b9ULL;
 v ^= v >> 27;
 return static_cast<float>((v & 0xffffU) / 65535.0);
}

inline Vector3 body_to_world(const FrameBody& body, double bx, double by,
 double z) {
 const double c = std::cos(body.yaw_rad), s = std::sin(body.yaw_rad);
 return {static_cast<float>(body.x_m + c * bx - s * by),
 static_cast<float>(z),
 static_cast<float>(-(body.y_m + s * bx + c * by))};
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

// MuJoCo links use x-forward/y-left/z-up. Inside the already transformed
// base frame, raylib consumes x-forward/y-up/z-right. Raw joint assets are
// authored in the MuJoCo link frame, so their draw rotation is Q*R_link.
// Globally converted Z-up assets already carry Q in Model::transform and
// therefore need the conjugated Q*R_link*Q^-1 rotation instead.
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
Vector3 wheel_asset_scale(const Model& model, double radius_m,
 double half_width_m) {
 const BoundingBox b = raylib_asset_bounds(model, false);
 const float ex = std::max(1e-4f, b.max.x - b.min.x);
 const float ey = std::max(1e-4f, b.max.y - b.min.y);
 const float ez = std::max(1e-4f, b.max.z - b.min.z);
 // Blender authors the joint in robot coordinates (X forward, Y axle,
 // Z up); glTF's Y-up export maps that axle to runtime +Z. Keep this one
 // asset-boundary convention for every wheel GLB: radial X/Y, axle Z.
 const float radial = std::max(ex, ey) * 0.5f;
 return {static_cast<float>(radius_m / radial),
 static_cast<float>(radius_m / radial),
 static_cast<float>((2.0 * half_width_m) / ez)};
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

void draw_motion_particles_3d(const Frame& f) {
 const double speed = std::hypot(f.body.forward_mps, f.body.lateral_mps);
 BeginBlendMode(BLEND_ALPHA);
 for (std::size_t wi = 0; wi < f.body.wheels.size(); ++wi) {
 const auto& w = f.body.wheels[wi];
 const int count = w.slipping ? 12 : (speed > 0.25 && w.driven ? 4 : 0);
 if (count == 0) continue;
 for (int k = 0; k < count; ++k) {
 const float phase = std::fmod(
 static_cast<float>(f.step) * 0.17f + visual_hash(f.step,
 static_cast<int>(wi), k) * 0.83f, 1.0f);
 const float spread = (visual_hash(f.step, static_cast<int>(wi), k + 41) - 0.5f) *
 static_cast<float>(w.radius_m * 1.4);
 const float lift = 0.025f + phase * (w.slipping ? 0.42f : 0.20f);
 const double bx = w.x_m - std::copysign(
 static_cast<double>(phase) * w.radius_m * (w.slipping ? 1.4 : 0.8),
 f.body.forward_mps == 0.0 ? 1.0 : f.body.forward_mps);
 const double by = w.y_m + spread;
 const Vector3 p = body_to_world(f.body, bx, by,
 std::max(0.01, f.body.z_m - w.radius_m + lift));
 const float size = static_cast<float>(w.slipping ? 0.025 + 0.04 * phase
 : 0.012 + 0.025 * phase);
 const unsigned char alpha = static_cast<unsigned char>(
 std::max(0.0f, 180.0f * (1.0f - phase)));
 DrawSphere(p, size, w.slipping ? Color{244, 96, 52, alpha}
 : Color{190, 166, 126, alpha});
 }
 }
 // A low, expanding exhaust/dust veil behind a moving body gives the
 // viewer a speed cue even when the wheel modules are occluded by terrain.
 if (speed > 0.35) {
 const float phase = std::fmod(static_cast<float>(f.step) * 0.11f, 1.0f);
 const double bx = -f.body.half_length_m * (0.7 + phase);
 const Vector3 p = body_to_world(f.body, bx, 0.0,
 std::max(0.015, f.body.z_m - 0.18 + 0.10 * phase));
 DrawSphere(p, static_cast<float>(0.08 + 0.18 * phase),
 {180, 165, 145, static_cast<unsigned char>(90 * (1.0f - phase))});
 }
 EndBlendMode();
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
                   bool show_estimate) {
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
  for (std::size_t i = 1; i < f.trail.size(); ++i) {
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
  // Lidar hits (from the true sensor origin, same as the 3D fan).
  if (show_lidar && f.lidar_valid) {
    const auto o = view.world_to_screen(f.body.sens_x_m, f.body.sens_y_m);
    for (const auto& hit : f.lidar_hits) {
      const auto s = view.world_to_screen(hit.first, hit.second);
      DrawLineV({o.x_px, o.y_px}, {s.x_px, s.y_px}, {255, 0, 0, 60});
      DrawPixelV({s.x_px, s.y_px}, RED);
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
  // red while the engine flags traction saturation (per side on tracks).
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
        const bool slip = k < static_cast<int>(f.body.wheels.size()) &&
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
        DrawLineEx(a, b, 3.0f, w.slipping ? RED : DARKGRAY);
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

void draw_frame_3d(const Frame& f,
                   const lost_signal::sim::world::World& world,
 bool show_lidar, bool show_estimate,
 const VisualAssets* assets = nullptr,
 const lost_signal::sim::viewer::LidarMount* lidar_mount = nullptr) {
  namespace world_ns = lost_signal::sim::world;
  // Ground base sized to the authored map (concrete apron under everything).
  double gx0, gy0, gx1, gy1;
  map_bounds(world, gx0, gy0, gx1, gy1);
  {
    const float w = static_cast<float>(gx1 - gx0 + 8.0);
    const float h = static_cast<float>(gy1 - gy0 + 8.0);
 DrawPlane(to_raylib((gx0 + gx1) * 0.5, (gy0 + gy1) * 0.5, -0.001),
              {w, h}, {198, 203, 196, 255});
  }
  DrawGrid(static_cast<int>(std::max(gx1 - gx0, gy1 - gy0) + 8.0), 1.0f);
  // Material zones as translucent paint quads (slick water reads blue,
  // sand/gravel warm — same table as the 2D overlay).
  for (const auto& z : f.zones) {
    const Vector3 center = to_raylib((z.x0_m + z.x1_m) * 0.5,
                                     (z.y0_m + z.y1_m) * 0.5, 0.005);
    const Vector3 size{static_cast<float>(std::abs(z.x1_m - z.x0_m)), 0.006f,
                       static_cast<float>(std::abs(z.y1_m - z.y0_m))};
    DrawCubeV(center, size, material_color(z.material_id, 150));
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
  for (std::size_t i = 1; i < f.trail.size(); ++i) {
    DrawLine3D(to_raylib(f.trail[i - 1].first, f.trail[i - 1].second, 0.02),
               to_raylib(f.trail[i].first, f.trail[i].second, 0.02), BLUE);
  }
  // Boxes as two-tone blocks (light cap, dark walls) with blob shadows.
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
    DrawCubeV(center, size, wall);
    DrawCubeWiresV(center, size, BLACK);
    // Sun-lit cap: thin slab on top, warm white.
    DrawCubeV(to_raylib((b.x0_m + b.x1_m) * 0.5, (b.y0_m + b.y1_m) * 0.5,
                        h + 0.008),
              {size.x, 0.016f, size.z},
              b.selected ? Color{255, 240, 160, 255}
                         : Color{168, 175, 184, 255});
    blob_shadow_3d((b.x0_m + b.x1_m) * 0.5, (b.y0_m + b.y1_m) * 0.5,
                   std::max(size.x, size.z) * 0.75);
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
  // Places as beacon poles with a ground ring (green = objective done).
  for (const auto& p : f.places) {
    const Color col = p.reached ? GREEN : ORANGE;
 const double place_ground = world_ns::sample_terrain(
 world.terrain_patches(), world.support_slabs(), p.x_m, p.y_m).height_m;
 // DrawCylinder is base-anchored; keep the objective marker on the same
 // authored ground as the robot and actors.
 DrawCylinder(to_raylib(p.x_m, p.y_m, place_ground), 0.05f, 0.05f, 0.6f, 10,
                 DARKGRAY);
 DrawSphere(to_raylib(p.x_m, p.y_m, place_ground + 0.68), 0.13f, col);
 DrawCircle3D(to_raylib(p.x_m, p.y_m, place_ground + 0.02), 0.45f, {1, 0, 0}, 90.0f,
                 p.reached ? Color{40, 180, 80, 200} : Color{230, 140, 40, 200});
 if (p.selected) DrawSphereWires(to_raylib(p.x_m, p.y_m, place_ground + 0.68), 0.25f, 8,
                                    8, RED);
  }
  // Actors: forklift-orange vehicles, pedestrian-blue walkers, true height,
  // heading tick, blob shadow. Patrol routes draw as ground polylines.
  for (const auto& a : f.actors) {
    const Color body =
        a.pedestrian ? Color{70, 130, 220, 255} : Color{235, 130, 40, 255};
    const float h = static_cast<float>(a.height_m > 0.1 ? a.height_m : 0.5);
 const double ground_z = world_ns::sample_terrain(
 world.terrain_patches(), world.support_slabs(), a.x_m, a.y_m).height_m;
 blob_shadow_3d(a.x_m, a.y_m, a.radius_m * 1.3, ground_z);
 const Model* actor_model = nullptr;
 if (assets != nullptr) actor_model = a.pedestrian ? &assets->worker
 : &assets->forklift;
 if (actor_model != nullptr && model_ready(*actor_model)) {
 draw_actor_model_grounded(*actor_model, a.x_m, a.y_m, ground_z,
 a.radius_m, a.pedestrian, a.heading_rad);
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
  for (const auto& a : world.actors()) {
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
  // Lidar rays from the TRUE sensor origin (mount-composed, same source
  // generation runs). Endpoints stay planar-projected (see builder).
  if (show_lidar && f.lidar_valid) {
    const Vector3 o = to_raylib(f.body.sens_x_m, f.body.sens_y_m,
                                f.body.sens_z_m);
    for (const auto& hit : f.lidar_hits) {
      DrawLine3D(o, to_raylib(hit.first, hit.second, f.body.sens_z_m),
                 {255, 0, 0, 90});
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
 blob_shadow_3d(f.body.x_m, f.body.y_m,
 std::max(f.body.half_length_m, f.body.half_width_m) * 1.4,
 robot_ground_z);
    rlPushMatrix();
    rlTranslatef(static_cast<float>(f.body.x_m),
 static_cast<float>(f.body.z_m),
                 static_cast<float>(-f.body.y_m));
    rlMultMatrixf(m);
 bool slipping = false;
 for (const auto& sw : f.body.wheels) slipping = slipping || sw.slipping;
    {
      // Layered shop model (local frame, y up): charcoal skirt, pearl
      // hull, orange deck stripe, lidar mast + puck, headlights, status
      // LED (red while any wheel slips). Collision still flashes red.
      const float bl = static_cast<float>(f.body.half_length_m * 2.0);
      const float bw = static_cast<float>(f.body.half_width_m * 2.0);
      const Color hull = f.static_collision
                             ? RED
                             : Color{238, 241, 244, 255};
 if (assets != nullptr && model_ready(assets->chassis)) {
 const BoundingBox cb = raylib_asset_bounds(assets->chassis, true);
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
 DrawSphere({static_cast<float>(mount_x),
 static_cast<float>(mount_z + 0.11), static_cast<float>(-mount_y)},
 0.025f, slipping ? RED : Color{50, 200, 90, 255}); // status LED
 draw_motion_particles_3d(f);
 // Passive caster assemblies: swivel and roll come from the MuJoCo joints;
 // the fork pivot and wheel contact remain separate when trail is non-zero.
 for (const auto& caster : f.body.casters) {
 if (caster.fork_pose.valid && caster.wheel_pose.valid) {
 const double caster_half_width = std::min(
 static_cast<double>(world.motor_params().tire_half_width_m),
 caster.radius_m * 0.75);
 const float caster_scale = static_cast<float>(caster.radius_m / 0.055);
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
 wheel_asset_scale(assets->caster_wheel, caster.radius_m,
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
 Vector3 caster_wheel_scale{static_cast<float>(caster.radius_m / 0.055),
 static_cast<float>(caster.radius_m / 0.055),
 static_cast<float>(caster.radius_m / 0.055)};
 if (assets != nullptr && model_ready(assets->caster_wheel)) {
 caster_wheel_scale = wheel_asset_scale(assets->caster_wheel,
 caster.radius_m, caster_half_width);
 }
 const float caster_scale = static_cast<float>(caster.radius_m / 0.055);
 if (assets != nullptr && model_ready(assets->caster_wheel)) {
 const Quaternion swivel = QuaternionFromAxisAngle(
 {0.0f, 1.0f, 0.0f}, static_cast<float>(caster.yaw_rad));
 // Wheel assets stay in their authored joint frame (+Z axle). Rx(180) is
 // the complete fixed installation into raylib and lays that axle on
 // horizontal raylib Z; swivel and roll remain separate live joints.
 const Quaternion mount = QuaternionFromAxisAngle(
 {1.0f, 0.0f, 0.0f}, PI);
 const Quaternion spin = QuaternionFromAxisAngle(
 {0.0f, 0.0f, 1.0f}, static_cast<float>(caster.spin_rad));
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
 QuaternionMultiply(swivel, QuaternionMultiply(mount, spin)),
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
 rlTranslatef(static_cast<float>(-world.motor_params().idler_trail_m),
 0.0f, 0.0f);
 rlRotatef(-90.0f, 1.0f, 0.0f, 0.0f);
 rlRotatef(static_cast<float>(caster.spin_rad * RAD2DEG), 0.0f, 0.0f,
 1.0f);
 DrawModelEx(assets->caster_wheel, {0.0f, 0.0f, 0.0f},
 {0.0f, 1.0f, 0.0f}, 0.0f,
 {caster_scale, caster_scale, caster_scale}, WHITE);
 rlPopMatrix();
 } else {
 rlTranslatef(static_cast<float>(-world.motor_params().idler_trail_m),
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
        const bool slip = s > 0 ? (f.body.wheels.size() > 1 &&
                                   f.body.wheels[1].slipping)
                                : (f.body.wheels.size() > 0 &&
                                   f.body.wheels[0].slipping);
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
      // first, spin follows about the laid-down axle. Spoke + slip-red
      // from live joints/flags; mecanum tread stripes sit in the steer
      // frame (rollers are passive analytic state — static decoration).
      for (const auto& w : f.body.wheels) {
 if (w.pose.valid) {
 const Vector3 position = link_position(w.pose);
 Quaternion rotation = link_rotation(w.pose, true);
 if (assets != nullptr && model_ready(assets->wheel)) {
 draw_model_quaternion(assets->wheel, position, rotation,
 wheel_asset_scale(assets->wheel, w.radius_m,
 world.motor_params().tire_half_width_m),
 w.slipping ? Color{210, 60, 48, 255} : WHITE);
 } else {
 rotation = QuaternionMultiply(rotation,
 QuaternionFromAxisAngle({1.0f, 0.0f, 0.0f}, PI * 0.5f));
 Vector3 axis{0.0f, 1.0f, 0.0f};
 float angle = 0.0f;
 QuaternionToAxisAngle(rotation, &axis, &angle);
 rlPushMatrix();
 rlTranslatef(position.x, position.y, position.z);
 rlRotatef(angle * RAD2DEG, axis.x, axis.y, axis.z);
 const Color tire = w.slipping ? RED
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
 // The wheel asset stays in its authored joint frame (+Z axle). Rx(180)
 // lays it on horizontal raylib Z. A differential module has steer_rad=0;
 // only a true steering module adds the live Y-axis yaw.
 const Quaternion steer = QuaternionFromAxisAngle(
 {0.0f, 1.0f, 0.0f}, static_cast<float>(w.steer_rad));
 // Ackermann/swerve modules add their measured Y steering before the same
 // horizontal axle roll; differential modules keep this yaw at zero.
 const Quaternion mount = QuaternionFromAxisAngle(
 {1.0f, 0.0f, 0.0f}, PI);
 const Quaternion spin = QuaternionFromAxisAngle(
 {0.0f, 0.0f, 1.0f}, static_cast<float>(w.spin_rad));
 const Quaternion rotation = QuaternionMultiply(
 steer, QuaternionMultiply(mount, spin));
 const Vector3 wheel_scale = wheel_asset_scale(
 assets->wheel, w.radius_m, world.motor_params().tire_half_width_m);
 draw_model_quaternion(assets->wheel,
 {static_cast<float>(w.x_m), axle + wheel_lift, static_cast<float>(-w.y_m)},
 rotation, wheel_scale,
 w.slipping ? Color{210, 60, 48, 255} : WHITE);
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
 {1.0f, 1.0f, 1.0f}, w.slipping ? Color{210, 60, 48, 255} : WHITE);
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
 w.slipping ? Color{168, 55, 52, 255} : DARKGRAY);
            rlPopMatrix();
          }
        }
 rlRotatef(static_cast<float>(w.spin_rad * RAD2DEG), 0.0f, 0.0f,
 1.0f);
 const Color tire = w.slipping ? RED
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
}
}  // namespace

int main(int argc, char** argv) {
  using namespace lost_signal;
 // Optional single-source configuration: --robot <config> builds the
 // session from one RobotConfig file (ROBOT description + SIM motor
 // numbers), --map <world file> replaces the demo geometry. `--assets`
 // enables optional SIM-only GLB presentation models from the old content
 // tree; procedural parts remain the deterministic fallback.
 std::string robot_path, sim_path, map_path, assets_path;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--robot" && i + 1 < argc) robot_path = argv[++i];
    if (a == "--sim" && i + 1 < argc) sim_path = argv[++i];
    if (a == "--map" && i + 1 < argc) map_path = argv[++i];
 if (a == "--assets" && i + 1 < argc) assets_path = argv[++i];
  }
  std::string used_config, config_err;
  auto robot_config = robot::platform::load_default_robot_config(
      robot_path, &used_config, &config_err);
  if (!robot_path.empty() && !robot_config.has_value()) {
    std::fprintf(stderr, "bad --robot file: %s\n", config_err.c_str());
    return 1;
  }
  std::unique_ptr<sim::scenario::ScenarioSession> session;
  if (robot_config.has_value()) {
    std::fprintf(stderr, "robot config: %s\n", used_config.c_str());
    session = std::make_unique<sim::scenario::ScenarioSession>(
        robot::platform::description_from_config(*robot_config),
        sim::eval::motor_from_config(*robot_config));
  } else {
    session = std::make_unique<sim::scenario::ScenarioSession>();
  }
  // Applied lidar mount for the viz ray fan: read back from the session
  // (the exact extrinsics generation runs — never a separately authored
  // copy, so a misconfigured mount shows up instead of hiding).
  sim::viewer::LidarMount session_lidar_mount;
  session->lidar_mount(session_lidar_mount.xyz_m, session_lidar_mount.rpy_rad);
  std::string used_sim, sim_err;
  auto sim_config = sim::eval::load_default_sim_config(sim_path, &used_sim,
                                                       &sim_err);
  if (!sim_path.empty() && !sim_config.has_value()) {
    std::fprintf(stderr, "bad --sim file: %s\n", sim_err.c_str());
    return 1;
  }
  if (sim_config.has_value()) {
    std::fprintf(stderr, "sim config: %s\n", used_sim.c_str());
    session->apply_sim_config(*sim_config);
  }
  if (!map_path.empty()) {
    sim::eval::WorldFileContent content;
    std::string err;
    if (!sim::eval::load_world_file(map_path, content, &err)) {
      std::fprintf(stderr, "bad --map file: %s\n", err.c_str());
      return 1;
    }
    session->setup_world().set_static_boxes(std::move(content.boxes));
    std::vector<std::string> ids;
    for (const auto& p : content.places) ids.push_back(p.place_id);
    session->setup_world().set_places(std::move(content.places));
    session->setup_world().set_materials(std::move(content.materials));
    session->setup_world().set_material_zones(std::move(content.zones));
    session->setup_world().set_terrain_patches(std::move(content.patches));
    session->setup_world().set_support_slabs(std::move(content.slabs));
    session->setup_world().set_environment(std::move(content.environment));
    session->set_objectives(std::move(ids));
    // Survey covers the authored map (far-field goals need a route).
    session->fit_map_window_to_world();
    // Converted master maps ship patrols beside the world file.
    lost_signal::sim::scenario::load_actors_sidecar(map_path, *session);
  } else {
    setup_demo(*session);
  }

 InitWindow(kScreenW, kScreenH, "robot_simu viewer (SIM truth, 2D)");
 if (assets_path.empty() && std::filesystem::is_directory("assets"))
 assets_path = "assets";
 VisualAssets visual_assets;
 load_visual_assets(visual_assets, assets_path,
 robot_config.has_value() ? robot_config->robot_id : "ro1");
  SetTargetFPS(60);

  sim::viewer::ViewTransform view(50.0f, 2.0, 1.0, kScreenW, kScreenH);
  view.fit(-4.0, -4.0, 8.0, 7.0);
  std::deque<std::pair<double, double>> trail;
  bool paused = false;
  bool show_lidar = true;
  bool show_estimate = true;
  bool follow = true;
 // Drive deterministic 20 ms simulation time from elapsed wall time. The
 // old fixed four-steps-per-frame loop demanded 4.8x real time at 60 FPS and
 // stacked periodic localization/planning work into the window thread.
 double playback_rate = 1.0;
 double sim_debt_s = 0.0;
 double previous_wall_s = GetTime();
 double last_step_ms = 0.0;
 double peak_step_ms = 0.0;
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
  bool view3d = false;
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
    cam_st.dist_m = static_cast<float>(
        std::min(60.0, std::max(10.0, std::max(bx1 - bx0, by1 - by0) * 0.9)));
    follow = false;  // start on the map overview, not glued to the robot
 } else {
 cam_st.dist_m = 5.0f;
 cam_st.pitch_deg = 35.0f;
 cam_st.target_z = 0.20;
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
 }

 while (!WindowShouldClose()) {
 const double wall_now_s = GetTime();
 const double wall_dt_s = std::clamp(
 wall_now_s - previous_wall_s, 0.0, 0.25);
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
    // Edit-mode toggle + keys (window thread only).
    if (IsKeyPressed(KEY_M)) {
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
    if (IsKeyPressed(KEY_SPACE)) paused = !paused;
 if (IsKeyPressed(KEY_N) && paused) {
 const auto begin = std::chrono::steady_clock::now();
 session->step_once();
 last_step_ms = std::chrono::duration<double, std::milli>(
 std::chrono::steady_clock::now() - begin).count();
 peak_step_ms = std::max(peak_step_ms, last_step_ms);
 }
    if (IsKeyPressed(KEY_L)) show_lidar = !show_lidar;
    if (IsKeyPressed(KEY_E)) show_estimate = !show_estimate;
    if (IsKeyPressed(KEY_F)) follow = !follow;
 if (IsKeyPressed(KEY_R)) {
 cam_st.yaw_deg = 35.0f;
 cam_st.pitch_deg = 50.0f;
 cam_st.dist_m = 12.0f;
 follow = false;
 }
 if (IsKeyPressed(KEY_EQUAL) && playback_rate < 8.0) playback_rate *= 2.0;
 if (IsKeyPressed(KEY_MINUS) && playback_rate > 0.25) playback_rate *= 0.5;
    if (IsKeyPressed(KEY_ZERO)) view.fit(-4.0, -4.0, 8.0, 7.0);
    const float wheel = GetMouseWheelMove();
    if (view3d) {
      if (wheel != 0.0f) {
        cam_st.dist_m *= wheel > 0.0f ? 1.0f / 1.15f : 1.15f;
 if (cam_st.dist_m < 0.6f) cam_st.dist_m = 0.6f;
        if (cam_st.dist_m > 60.0f) cam_st.dist_m = 60.0f;
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

    if (!paused && !session->objectives_done()) {
 constexpr double kSimStepS = 0.02;
 constexpr int kMaxStepsPerFrame = 4;
 constexpr double kFrameWorkBudgetMs = 10.0;
 // Keep bounded debt so a debugger stop or dragged window cannot trigger a
 // catch-up spiral. Remaining debt carries across frames; the deterministic
 // simulation clock never depends on rendering time.
 sim_debt_s = std::min(0.5, sim_debt_s + wall_dt_s * playback_rate);
 const auto frame_work_begin = std::chrono::steady_clock::now();
 int steps_this_frame = 0;
 while (sim_debt_s + 1e-12 >= kSimStepS &&
 steps_this_frame < kMaxStepsPerFrame) {
 const auto step_begin = std::chrono::steady_clock::now();
        session->step_once();
 last_step_ms = std::chrono::duration<double, std::milli>(
 std::chrono::steady_clock::now() - step_begin).count();
 peak_step_ms = std::max(peak_step_ms, last_step_ms);
 sim_debt_s -= kSimStepS;
 ++steps_this_frame;
        if (session->objectives_done()) break;
 const double work_ms = std::chrono::duration<double, std::milli>(
 std::chrono::steady_clock::now() - frame_work_begin).count();
 if (work_ms >= kFrameWorkBudgetMs) break;
      }
 } else {
 // Paused wall time is not simulation debt.
 sim_debt_s = 0.0;
    }
    const auto& steps = session->episode().steps();
    if (steps.empty()) continue;
    const sim::eval::EvalStep& last = steps.back();
    trail.emplace_back(last.truth_after.body_x_m, last.truth_after.body_y_m);
    while (trail.size() > 600) trail.pop_front();
    std::vector<std::string> reached;
    for (const auto& r : session->objectives().results()) {
      if (r.successful()) reached.push_back(r.place_id);
    }
    const sim::viewer::Frame frame = sim::viewer::build_frame(
        session->setup_world(), last, reached,
        {trail.begin(), trail.end()},
        edit_mode ? sim::viewer::EditorSelection{editor.selected_box(),
                                                 editor.selected_corner(),
                                                 editor.selected_place()}
                  : sim::viewer::EditorSelection{},
        session_lidar_mount);
    if (follow) {
      view.set_screen(GetScreenWidth(), GetScreenHeight());
      // Re-center without touching zoom: pan by the screen-space error.
      const auto cur =
          view.world_to_screen(last.truth_after.body_x_m,
                               last.truth_after.body_y_m);
      view.pan_px(GetScreenWidth() * 0.5f - cur.x_px,
                  GetScreenHeight() * 0.5f - cur.y_px);
      cam_st.target_x = last.truth_after.body_x_m;
      cam_st.target_y = last.truth_after.body_y_m;
    }

    BeginDrawing();
    ClearBackground(view3d ? Color{168, 204, 228, 255} : RAYWHITE);
    if (view3d) {
      const Camera3D cam = camera_from_state(cam_st);
      BeginMode3D(cam);
 draw_frame_3d(frame, session->setup_world(), show_lidar, show_estimate,
 &visual_assets, &session_lidar_mount);
      EndMode3D();
    } else {
      view.set_screen(GetScreenWidth(), GetScreenHeight());
      draw_frame_2d(frame, view, show_lidar, show_estimate);
    }
    char hud[384];
    std::snprintf(hud, sizeof(hud),
 "step %llu x=%.2f y=%.2f yaw=%.2f clr=%.2f obj %d/%zu %.2fx %dfps tick=%.1f/%.1fms cam=%s %s",
                  static_cast<unsigned long long>(frame.step), frame.body.x_m,
                  frame.body.y_m, frame.body.yaw_rad, frame.static_clearance_m,
                  static_cast<int>(reached.size()), session->objectives().total(),
 playback_rate, GetFPS(), last_step_ms, peak_step_ms,
 follow ? "FOLLOW" : "FREE",
 paused ? "PAUSED"
 : session->objectives_done()
 ? (session->objectives_passed() ? "PASSED" : "FAILED")
 : "RUN");
    DrawText(hud, 10, 10, 14, BLACK);
    if (edit_mode) {
      char ebuf[256];
      std::snprintf(ebuf, sizeof(ebuf),
                    "EDIT  click/drag move  B box  P place  Del remove  S save viewer_map.json  M exit  %s",
                    edit_msg.c_str());
      DrawText(ebuf, 10, 32, 14, RED);
    }
    DrawText("space pause  N step  L lidar  E estimate  F follow  0 fit  +/- speed  V 2D/3D  M edit  right-drag pan/orbit  wheel zoom",
             10, GetScreenHeight() - 24, 12, DARKGRAY);
    EndDrawing();
    if (shot_frames > 0 && --shot_frames == 0) {
      TakeScreenshot(shot_path.c_str());
      break;
    }
  }
 unload_visual_assets(visual_assets);
 CloseWindow();
  return 0;
}
