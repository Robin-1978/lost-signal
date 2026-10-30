#include <cassert>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <string>
#include <unistd.h>

#include "sim/viewer/asset_manifest.hpp"

namespace {

namespace fs = std::filesystem;
using lost_signal::sim::viewer::AssetManifest;
using lost_signal::sim::viewer::asset_point_to_render;
using lost_signal::sim::viewer::asset_frame_transform;
using lost_signal::sim::viewer::asset_selection_for_robot;
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
 const std::string v2 =
 R"(,"source_artifact":"unit-test-generator","license":"NOASSERTION","content_hash":"fnv1a64:0e46fd16c3ae8b48","pivot_semantic":"wheel_roll_joint","asset_to_link_translation_m":[0,0,0],"link_binding":"wheel_link","grounding_points_m":[[0,-1,0]],"material_roles":["rubber","metal"],"lods":[{"level":0,"path":"part.glb","maximum_distance_m":0}],"triangle_budget":100,"material_budget":2,"animation_clips":[])";
 return "{\"format\":\"lost-signal-assets/2\",\"assets\":[{" + fields +
 v2 + "}]}";
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
 const auto ackermann = asset_selection_for_robot("ro1", "ackermann");
 assert(ackermann.chassis == "robots.ro2.chassis");
 assert(ackermann.steering_knuckle_left ==
 "robots.ro2.front_left_knuckle");
 const auto mecanum = asset_selection_for_robot("ro1", "mecanum");
 assert(mecanum.chassis == "robots.ro5.chassis");
 assert(mecanum.wheel == "robots.ro5.roller_left");
 assert(mecanum.wheel_right == "robots.ro5.roller_right");
 const auto tracked = asset_selection_for_robot("ro1", "tracked");
 assert(tracked.chassis == "robots.ro3.hull");
 assert(tracked.track == "robots.ro3.track");

  const fs::path source_root = TEST_SOURCE_DIR;
  const fs::path models_root = source_root / "assets" / "models";
  const fs::path real_manifest = models_root / "manifest.json";

  AssetManifest manifest;
  std::string error;
 const bool real_loaded =
 load_asset_manifest(real_manifest.string(), manifest, &error);
 if (!real_loaded) std::cerr << error << '\n';
 assert(real_loaded);
  assert(!manifest.assets().empty());
 for (const auto& asset : manifest.assets()) {
   // Release assets may not carry the old unverified provenance sentinel.
   assert(asset.license != "NOASSERTION");
 }
 const auto* worker = manifest.find("actors.worker");
 assert(worker != nullptr);
 assert(worker->animation_clips.size() == 1);
 assert(worker->animation_clips.front() == "WorkerWalk");
 assert(worker->animation_stride_m > 0.0);

  // Articulated assets are link-frame meshes, not upright decorations.  The
  // first arm link may include its bearing around the pivot, while the three
  // downstream links must start at x=0 and extend along +X.  These checks
  // pin the Blender/MuJoCo assembly contract that prevents a vertical legacy
  // mesh from silently passing the generic bounds/hash audit.
  const auto* arm_turret = manifest.find("robots.ro6.turret");
  const auto* arm_upper = manifest.find("robots.ro6.upper_arm");
  const auto* arm_forearm = manifest.find("robots.ro6.forearm");
  const auto* arm_tool = manifest.find("robots.ro6.tool");
  assert(arm_turret != nullptr && arm_upper != nullptr &&
         arm_forearm != nullptr && arm_tool != nullptr);
  assert(arm_turret->expected_bounds_m.maximum_m[0] >= 0.30 - 1e-9);
  assert(arm_upper->expected_bounds_m.minimum_m[0] >= -1e-9 &&
         arm_upper->expected_bounds_m.maximum_m[0] >= 0.25 - 1e-9);
  assert(arm_forearm->expected_bounds_m.minimum_m[0] >= -1e-9 &&
         arm_forearm->expected_bounds_m.maximum_m[0] >= 0.18 - 1e-9);
  assert(arm_tool->expected_bounds_m.minimum_m[0] >= -1e-9 &&
         arm_tool->expected_bounds_m.maximum_m[0] >= 0.12 - 1e-9);

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

  const fs::path temp = fs::temp_directory_path() /
                        ("lost_signal_asset_test_" +
                         std::to_string(static_cast<long long>(::getpid())));
  fs::remove_all(temp);
  fs::create_directories(temp);
  write_text(temp / "part.glb", "test fixture");
  const fs::path test_manifest = temp / "manifest.json";

  write_text(test_manifest, one_asset());
  AssetManifest valid;
  assert(load_asset_manifest(test_manifest.string(), valid, &error));
  assert(valid.assets().size() == 1);
 assert(valid.assets().front().content_hash ==
 "fnv1a64:0e46fd16c3ae8b48");
 assert(valid.assets().front().pivot_semantic == "wheel_roll_joint");
 assert(valid.assets().front().link_binding == "wheel_link");
 assert(valid.assets().front().animation_stride_m == 0.0);
 assert(valid.assets().front().lods.size() == 1);
 assert(valid.assets().front().triangle_budget == 100);
 const auto canonical = asset_frame_transform(valid.assets().front());
 assert(transform_asset_point(canonical, {1.0, 2.0, 3.0}) ==
 (std::array<double, 3>{1.0, 2.0, 3.0}));
 assert(asset_point_to_render(valid.assets().front(), {1.0, 2.0, 3.0}) ==
 (std::array<double, 3>{1.0, 3.0, -2.0}));

 std::string translated_json = one_asset();
 const std::string zero_translation =
 "\"asset_to_link_translation_m\":[0,0,0]";
 translated_json.replace(translated_json.find(zero_translation),
 zero_translation.size(),
 "\"asset_to_link_translation_m\":[4,5,6]");
 write_text(test_manifest, translated_json);
 AssetManifest translated_manifest;
 assert(load_asset_manifest(
 test_manifest.string(), translated_manifest, &error));
 assert(transform_asset_point(
 asset_frame_transform(translated_manifest.assets().front()),
 {0.0, 0.0, 0.0}) == (std::array<double, 3>{4.0, 5.0, 6.0}));

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

 const std::string valid_json = one_asset();
 const std::size_t object_begin = valid_json.find("[{") + 1;
 const std::size_t object_end = valid_json.rfind(']');
 const std::string object = valid_json.substr(
 object_begin, object_end - object_begin);
 assert(rejected(test_manifest,
 "{\"format\":\"lost-signal-assets/2\",\"assets\":[" + object +
 "," + object + "]}"));
 std::string second_object = object;
 second_object.replace(second_object.find("test.asset"),
 std::string("test.asset").size(), "second.asset");
 assert(rejected(test_manifest,
 "{\"format\":\"lost-signal-assets/2\",\"assets\":[" + object +
 "," + second_object + "]}"));
  assert(rejected(test_manifest,
                  one_asset(R"("asset_id":"bad.axis","version":1,"kind":"wheel","path":"part.glb","source":"test","meters_per_unit":1,"up_axis":"+Z","forward_axis":"-Z","axle_axis":"+Y","pivot_m":[0,0,0],"asset_to_link_rpy_rad":[0,0,0],"expected_bounds_m":{"min":[-1,-1,-1],"max":[1,1,1]})")));
 assert(rejected(test_manifest,
 one_asset(R"("asset_id":"left.handed","version":1,"kind":"wheel","path":"part.glb","source":"test","meters_per_unit":1,"up_axis":"+Z","forward_axis":"+X","axle_axis":"-Y","pivot_m":[0,0,0],"asset_to_link_rpy_rad":[0,0,0],"expected_bounds_m":{"min":[-1,-1,-1],"max":[1,1,1]})")));
  assert(rejected(test_manifest,
                  one_asset(R"("asset_id":"bad.bounds","version":1,"kind":"wheel","path":"part.glb","source":"test","meters_per_unit":1,"up_axis":"+Z","forward_axis":"+X","axle_axis":"+Y","pivot_m":[0,0,0],"asset_to_link_rpy_rad":[0,0,0],"expected_bounds_m":{"min":[0,-1,-1],"max":[0,1,1]})")));
 assert(rejected(test_manifest,
 one_asset(R"("asset_id":"bad.grounding","version":1,"kind":"wheel","path":"part.glb","source":"test","meters_per_unit":1,"up_axis":"+Z","forward_axis":"+X","axle_axis":"+Y","pivot_m":[0,0,0],"asset_to_link_rpy_rad":[0,0,0],"grounding_points_m":[[0,2,0]],"expected_bounds_m":{"min":[-1,-1,-1],"max":[1,1,1]})")));
  assert(rejected(test_manifest,
                  one_asset(R"("asset_id":"escape","version":1,"kind":"wheel","path":"../part.glb","source":"test","meters_per_unit":1,"up_axis":"+Z","forward_axis":"+X","axle_axis":"+Y","pivot_m":[0,0,0],"asset_to_link_rpy_rad":[0,0,0],"expected_bounds_m":{"min":[-1,-1,-1],"max":[1,1,1]})")));
  assert(rejected(test_manifest,
                  one_asset(R"("asset_id":"missing","version":1,"kind":"wheel","path":"missing.glb","source":"test","meters_per_unit":1,"up_axis":"+Z","forward_axis":"+X","axle_axis":"+Y","pivot_m":[0,0,0],"asset_to_link_rpy_rad":[0,0,0],"expected_bounds_m":{"min":[-1,-1,-1],"max":[1,1,1]})")));
 std::string changed_hash = one_asset();
 changed_hash.replace(changed_hash.find("0e46fd16c3ae8b48"), 16,
 "0000000000000000");
 assert(rejected(test_manifest, changed_hash));
 assert(rejected(test_manifest,
 R"({"format":"lost-signal-assets/1","assets":[]})"));

  fs::remove_all(temp);
  return 0;
}
