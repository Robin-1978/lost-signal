#pragma once

#include <array>
#include <string>
#include <vector>

namespace lost_signal::sim::viewer {

struct AssetBounds {
 std::array<double, 3> minimum_m{};
 std::array<double, 3> maximum_m{};
};

struct AssetDefinition {
 std::string asset_id;
 int version = 0;
 std::string kind;
 std::string relative_path;
 std::string source;
 double meters_per_unit = 1.0;
 std::string up_axis;
 std::string forward_axis;
 std::string axle_axis;
 std::array<double, 3> pivot_m{};
 std::array<double, 3> asset_to_link_rpy_rad{};
 AssetBounds expected_bounds_m;
 double reference_radius_m = 0.0;
 double reference_half_width_m = 0.0;
};

// Rigid transform from the asset's authored coordinates to the component
// link frame. Rotation is row-major and translation is expressed in metres.
// Axis declarations establish the coarse frame; asset_to_link_rpy_rad is the
// explicit residual correction and pivot_m becomes the link origin.
struct AssetFrameTransform {
 std::array<double, 9> rotation{};
 std::array<double, 3> translation_m{};
};

AssetFrameTransform asset_frame_transform(const AssetDefinition& asset);
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
// directory containing the manifest. Invalid axes, units, bounds, duplicate
// ids/paths, missing source metadata, and files outside that directory fail
// closed. The manifest is never visible to ROBOT.
bool load_asset_manifest(const std::string& path, AssetManifest& manifest,
 std::string* error = nullptr);

} // namespace lost_signal::sim::viewer
