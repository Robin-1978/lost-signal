#include <cassert>
#include <filesystem>
#include <fstream>
#include <set>
#include <string>

#include "sim/viewer/asset_manifest.hpp"

namespace {

namespace fs = std::filesystem;
using lost_signal::sim::viewer::AssetManifest;
using lost_signal::sim::viewer::asset_point_to_render;
using lost_signal::sim::viewer::asset_frame_transform;
using lost_signal::sim::viewer::load_asset_manifest;
using lost_signal::sim::viewer::transform_asset_point;

void write_text(const fs::path& path, const std::string& text) {
  std::ofstream output(path, std::ios::binary);
  assert(output.is_open());
  output << text;
  assert(output.good());
}

std::string one_asset(const std::string& overrides = {}) {
  const std::string fields = overrides.empty()
      ? R"("asset_id":"test.asset","version":1,"kind":"wheel","path":"part.glb","source":"unit test","meters_per_unit":1,"up_axis":"+Z","forward_axis":"+X","axle_axis":"+Y","pivot_m":[0,0,0],"asset_to_link_rpy_rad":[0,0,0],"expected_bounds_m":{"min":[-1,-1,-1],"max":[1,1,1]})"
      : overrides;
  return "{\"format\":\"lost-signal-assets/1\",\"assets\":[{" + fields + "}]}";
}

bool rejected(const fs::path& manifest_path, const std::string& json) {
  write_text(manifest_path, json);
  AssetManifest manifest;
  std::string error;
  return !load_asset_manifest(manifest_path.string(), manifest, &error) &&
         !error.empty();
}

}  // namespace

int main() {
  const fs::path source_root = TEST_SOURCE_DIR;
  const fs::path models_root = source_root / "assets" / "models";
  const fs::path real_manifest = models_root / "manifest.json";

  AssetManifest manifest;
  std::string error;
  assert(load_asset_manifest(real_manifest.string(), manifest, &error));
  assert(!manifest.assets().empty());

  std::set<std::string> inventory;
  for (const auto& entry : fs::recursive_directory_iterator(models_root)) {
    if (!entry.is_regular_file()) continue;
    const auto extension = entry.path().extension().string();
    if (extension != ".glb" && extension != ".gltf") continue;
    inventory.insert(fs::relative(entry.path(), models_root).generic_string());
  }
  std::set<std::string> declared;
  for (const auto& asset : manifest.assets()) {
    assert(declared.insert(asset.relative_path).second);
    assert(manifest.find(asset.asset_id) == &asset);
    assert(manifest.find_by_path(asset.relative_path) == &asset);
  }
  assert(declared == inventory);

  const fs::path temp = fs::temp_directory_path() / "lost_signal_asset_test";
  fs::remove_all(temp);
  fs::create_directories(temp);
  write_text(temp / "part.glb", "test fixture");
  const fs::path test_manifest = temp / "manifest.json";

  write_text(test_manifest, one_asset());
  AssetManifest valid;
  assert(load_asset_manifest(test_manifest.string(), valid, &error));
  assert(valid.assets().size() == 1);
 const auto canonical = asset_frame_transform(valid.assets().front());
 assert(transform_asset_point(canonical, {1.0, 2.0, 3.0}) ==
 (std::array<double, 3>{1.0, 2.0, 3.0}));
 assert(asset_point_to_render(valid.assets().front(), {1.0, 2.0, 3.0}) ==
 (std::array<double, 3>{1.0, 3.0, -2.0}));

 write_text(test_manifest, one_asset(
 R"("asset_id":"rotated","version":1,"kind":"wheel","path":"part.glb","source":"unit test","meters_per_unit":1,"up_axis":"+Y","forward_axis":"+Z","axle_axis":"+X","pivot_m":[1,2,3],"asset_to_link_rpy_rad":[0,0,0],"expected_bounds_m":{"min":[-1,-1,-1],"max":[1,1,1]})"));
 AssetManifest rotated_manifest;
 assert(load_asset_manifest(test_manifest.string(), rotated_manifest, &error));
 const auto rotated = asset_frame_transform(rotated_manifest.assets().front());
 // Authored +Z/+X/+Y become link +X/+Y/+Z and the declared pivot is zero.
 assert(transform_asset_point(rotated, {1.0, 2.0, 3.0}) ==
 (std::array<double, 3>{0.0, 0.0, 0.0}));
 assert(transform_asset_point(rotated, {1.0, 2.0, 4.0}) ==
 (std::array<double, 3>{1.0, 0.0, 0.0}));
 assert(asset_point_to_render(
 rotated_manifest.assets().front(), {1.0, 2.0, 3.0}) ==
 (std::array<double, 3>{0.0, 0.0, 0.0}));

 write_text(test_manifest, one_asset(
 R"("asset_id":"legacy.wheel","version":1,"kind":"wheel","path":"part.glb","source":"unit test","meters_per_unit":1,"up_axis":"+Y","forward_axis":"+X","axle_axis":"-Z","pivot_m":[0,0,0],"asset_to_link_rpy_rad":[0,0,0],"expected_bounds_m":{"min":[-1,-1,-1],"max":[1,1,1]})"));
 AssetManifest wheel_manifest;
 assert(load_asset_manifest(test_manifest.string(), wheel_manifest, &error));
 const auto& wheel = wheel_manifest.assets().front();
 // The legacy file stores its radial up direction in +Y and its +left
 // axle in -Z. Both become the expected raylib axes without a draw hack.
 assert(asset_point_to_render(wheel, {0.0, 1.0, 0.0}) ==
 (std::array<double, 3>{0.0, 1.0, 0.0}));
 assert(asset_point_to_render(wheel, {0.0, 0.0, -1.0}) ==
 (std::array<double, 3>{0.0, 0.0, -1.0}));

  assert(rejected(test_manifest,
                  R"({"format":"lost-signal-assets/1","assets":[{"asset_id":"same","version":1,"kind":"wheel","path":"part.glb","source":"test","meters_per_unit":1,"up_axis":"+Z","forward_axis":"+X","axle_axis":"+Y","pivot_m":[0,0,0],"asset_to_link_rpy_rad":[0,0,0],"expected_bounds_m":{"min":[-1,-1,-1],"max":[1,1,1]}},{"asset_id":"same","version":1,"kind":"wheel","path":"other.glb","source":"test","meters_per_unit":1,"up_axis":"+Z","forward_axis":"+X","axle_axis":"+Y","pivot_m":[0,0,0],"asset_to_link_rpy_rad":[0,0,0],"expected_bounds_m":{"min":[-1,-1,-1],"max":[1,1,1]}}]})"));
 assert(rejected(test_manifest,
 R"({"format":"lost-signal-assets/1","assets":[{"asset_id":"first","version":1,"kind":"wheel","path":"part.glb","source":"test","meters_per_unit":1,"up_axis":"+Z","forward_axis":"+X","axle_axis":"+Y","pivot_m":[0,0,0],"asset_to_link_rpy_rad":[0,0,0],"expected_bounds_m":{"min":[-1,-1,-1],"max":[1,1,1]}},{"asset_id":"second","version":1,"kind":"wheel","path":"part.glb","source":"test","meters_per_unit":1,"up_axis":"+Z","forward_axis":"+X","axle_axis":"+Y","pivot_m":[0,0,0],"asset_to_link_rpy_rad":[0,0,0],"expected_bounds_m":{"min":[-1,-1,-1],"max":[1,1,1]}}]})"));
  assert(rejected(test_manifest,
                  one_asset(R"("asset_id":"bad.axis","version":1,"kind":"wheel","path":"part.glb","source":"test","meters_per_unit":1,"up_axis":"+Z","forward_axis":"-Z","axle_axis":"+Y","pivot_m":[0,0,0],"asset_to_link_rpy_rad":[0,0,0],"expected_bounds_m":{"min":[-1,-1,-1],"max":[1,1,1]})")));
 assert(rejected(test_manifest,
 one_asset(R"("asset_id":"left.handed","version":1,"kind":"wheel","path":"part.glb","source":"test","meters_per_unit":1,"up_axis":"+Z","forward_axis":"+X","axle_axis":"-Y","pivot_m":[0,0,0],"asset_to_link_rpy_rad":[0,0,0],"expected_bounds_m":{"min":[-1,-1,-1],"max":[1,1,1]})")));
  assert(rejected(test_manifest,
                  one_asset(R"("asset_id":"bad.bounds","version":1,"kind":"wheel","path":"part.glb","source":"test","meters_per_unit":1,"up_axis":"+Z","forward_axis":"+X","axle_axis":"+Y","pivot_m":[0,0,0],"asset_to_link_rpy_rad":[0,0,0],"expected_bounds_m":{"min":[0,-1,-1],"max":[0,1,1]})")));
  assert(rejected(test_manifest,
                  one_asset(R"("asset_id":"escape","version":1,"kind":"wheel","path":"../part.glb","source":"test","meters_per_unit":1,"up_axis":"+Z","forward_axis":"+X","axle_axis":"+Y","pivot_m":[0,0,0],"asset_to_link_rpy_rad":[0,0,0],"expected_bounds_m":{"min":[-1,-1,-1],"max":[1,1,1]})")));
  assert(rejected(test_manifest,
                  one_asset(R"("asset_id":"missing","version":1,"kind":"wheel","path":"missing.glb","source":"test","meters_per_unit":1,"up_axis":"+Z","forward_axis":"+X","axle_axis":"+Y","pivot_m":[0,0,0],"asset_to_link_rpy_rad":[0,0,0],"expected_bounds_m":{"min":[-1,-1,-1],"max":[1,1,1]})")));

  fs::remove_all(temp);
  return 0;
}
