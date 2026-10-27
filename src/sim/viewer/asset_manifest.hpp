#pragma once

#include <array>
#include <cstddef>
#include <string>
#include <vector>

namespace lost_signal::sim::viewer {

struct AssetBounds {
 std::array<double, 3> minimum_m{};
 std::array<double, 3> maximum_m{};
};

struct AssetLod {
 int level = 0;
 std::string relative_path;
 double maximum_distance_m = 0.0; // 0 for LOD0; increasing thereafter
};

struct AssetDefinition {
 std::string asset_id;
 int version = 0;
 std::string kind;
 std::string relative_path;
 std::string source;
 std::string source_artifact;
 std::string license;
 std::string content_hash;
 double meters_per_unit = 1.0;
 std::string up_axis;
 std::string forward_axis;
 std::string axle_axis;
 std::array<double, 3> pivot_m{};
 std::string pivot_semantic;
 // Position of the declared pivot in the link frame. Together with RPY it
 // is the complete asset-to-link rigid transform.
 std::array<double, 3> asset_to_link_translation_m{};
 std::array<double, 3> asset_to_link_rpy_rad{};
 std::string link_binding;
 std::vector<std::array<double, 3>> grounding_points_m;
 bool has_sensor_optical_center = false;
 std::array<double, 3> sensor_optical_center_m{};
 std::vector<std::string> material_roles;
 std::vector<AssetLod> lods;
 std::size_t triangle_budget = 0;
 std::size_t material_budget = 0;
 std::string skeleton_root;
 std::vector<std::string> animation_clips;
 // Distance travelled by the authored root for one animation cycle. This is
 // a SIM presentation contract, never a robot or physics parameter.
 double animation_stride_m = 0.0;
 AssetBounds expected_bounds_m;
 double reference_radius_m = 0.0;
 double reference_half_width_m = 0.0;
};

// SIM-only asset selection. The hardware chassis kind is the source of
// truth for component family; robot_id remains an identity label and must not
// make an Ackermann/Mecanum/Swerve robot render as the differential model.
struct AssetSelection {
 std::string chassis;
 std::string wheel;
 std::string wheel_right;
 std::string steering_knuckle_left;
 std::string steering_knuckle_right;
 std::string lidar;
 std::string caster_fork;
 std::string caster_wheel;
 std::string track;
 std::string arm_turret;
 std::string arm_upper;
 std::string arm_forearm;
 std::string arm_tool;
};

// Rigid transform from the asset's authored coordinates to the component
// link frame. Rotation is row-major and translation is expressed in metres.
// Axis declarations establish the coarse frame; asset_to_link RPY is the
// explicit residual correction and translation places pivot_m in the link.
struct AssetFrameTransform {
 std::array<double, 9> rotation{};
 std::array<double, 3> translation_m{};
};

AssetFrameTransform asset_frame_transform(const AssetDefinition& asset);
AssetSelection asset_selection_for_robot(const std::string& robot_id,
 const std::string& chassis_kind);
std::array<double, 3> transform_asset_point(
 const AssetFrameTransform& transform,
 const std::array<double, 3>& point_m);
// Converts one point from the coordinates stored in the GLB/GLTF into the
// raylib render basis (X forward, Y up, Z right). Units, the declared pivot,
// and the complete asset-to-link rotation are applied exactly once.
std::array<double, 3> asset_point_to_render(
 const AssetDefinition& asset,
 const std::array<double, 3>& point_asset_units);

class AssetManifest {
 public:
 const AssetDefinition* find(const std::string& asset_id) const;
  const AssetDefinition* find_by_path(const std::string& relative_path) const;
 const std::vector<AssetDefinition>& assets() const { return assets_; }

 private:
 friend bool load_asset_manifest(const std::string&, AssetManifest&,
 std::string*);
 std::vector<AssetDefinition> assets_;
};

// Strict SIM presentation manifest loader. Paths are relative to the models
// directory containing the manifest. It verifies content hashes, provenance,
// rigid frames, semantic pivots, LOD order, budgets, axes, bounds, duplicate
// ids/paths, and containment. The manifest is never visible to ROBOT.
bool load_asset_manifest(const std::string& path, AssetManifest& manifest,
 std::string* error = nullptr);

} // namespace lost_signal::sim::viewer
