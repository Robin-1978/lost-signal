#pragma once

#include <array>
#include <string>
#include <vector>

#include "sim/viewer/asset_manifest.hpp"
#include "sim/world/world.hpp"

namespace lost_signal::sim::viewer {

struct SceneCamera {
 double distance_m = 4.8;
 double yaw_deg = 35.0;
 double pitch_deg = 32.0;
 double target_height_m = 0.20;
 double follow_smoothing_s = 0.35;
 double lookahead_s = 0.65;
 double speed_distance_gain_s = 0.70;
 double minimum_distance_m = 2.2;
 double maximum_distance_m = 9.0;
};

struct SceneLighting {
 // Direction in SIM world coordinates (x-forward, y-left, z-up).
 std::array<double, 3> direction_xyz{-0.48, 0.32, -1.0};
 std::array<int, 4> directional_rgba{255, 234, 205, 255};
 std::array<int, 4> ambient_rgba{70, 82, 78, 255};
 std::array<int, 4> fog_rgba{18, 31, 34, 255};
 double directional_intensity = 1.08;
 double ambient_intensity = 0.90;
 double fog_start_m = 12.0;
 double fog_end_m = 31.0;
 double exposure = 0.92;
 double bloom_threshold = 0.78;
 double bloom_strength = 0.20;
 double vignette_strength = 0.14;
 int shadow_map_resolution = 2048;
 double shadow_extent_m = 27.0;
 double shadow_strength = 0.62;
};

struct SceneFloor {
 std::array<int, 4> base_rgba{47, 53, 52, 255};
 std::array<int, 4> grid_rgba{77, 88, 85, 120};
 double grid_spacing_m = 1.0;
};

struct SceneInstance {
 std::string instance_id;
 std::string asset_id;
 // Exactly one authority is used. A collision-bound visual replaces the
 // matching physical proxy in the viewer. A surface-only instance must stay
 // near the ground and is decoration such as paint or a warning strip.
 std::string collision_box_id;
 bool surface_only = false;
 std::array<double, 3> position_m{};
 std::array<double, 3> scale{1.0, 1.0, 1.0};
 double yaw_deg = 0.0;
};

struct SceneManifest {
 std::string scene_id;
 // Explicit scene contract for paint/pads declared surface_only. A scene
 // that uses surface_only must author this value; it is not a loader magic
 // number and is applied to both the instance base and scaled asset height.
 double surface_only_max_height_m = 0.0;
 SceneCamera camera;
 SceneLighting lighting;
 SceneFloor floor;
 std::vector<SceneInstance> instances;

 const SceneInstance* visual_for_box(const std::string& box_id) const;
};

// Loads presentation-only scene composition. Every solid visual must bind to
// a stable world collision id whose centre matches the authored instance;
// unbound visuals must explicitly declare surface_only. Asset ids are checked
// against the versioned GLB manifest. No scene value enters ROBOT.
bool load_scene_manifest(const std::string& path,
 const AssetManifest& assets,
 const std::vector<world::StaticBox>& boxes,
 SceneManifest& scene,
 std::string* error = nullptr);

} // namespace lost_signal::sim::viewer
