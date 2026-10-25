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
 SceneCamera camera;
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
