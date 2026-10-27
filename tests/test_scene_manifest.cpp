#include <cassert>
#include <algorithm>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#include "sim/eval/world_io.hpp"
#include "sim/viewer/asset_manifest.hpp"
#include "sim/viewer/scene_manifest.hpp"

namespace {

void write_text(const std::filesystem::path& path, const std::string& text) {
  std::ofstream output(path, std::ios::binary);
  assert(output.is_open());
  output << text;
  assert(output.good());
}

// A minimal valid scene around one instance. Loader negative tests vary only
// the instance, so a rejection can only come from that instance.
std::string scene_with_instance(const std::string& instance_json,
 bool declare_surface_limit = true) {
 const std::string surface_limit = declare_surface_limit
 ? "\"surface_only_max_height_m\":0.10," : "";
 return std::string(
 "{\"format\":\"lost-signal-scene/1\",\"scene_id\":\"probe\"," +
 surface_limit +
 "\"camera\":{\"distance_m\":4,\"yaw_deg\":0,\"pitch_deg\":30,\"target_height_m\":0.2}," +
 "\"lighting\":{\"direction_xyz\":[0,0,-1],\"directional_rgba\":[255,255,255,255]," +
 "\"ambient_rgba\":[64,64,64,255],\"fog_rgba\":[10,10,10,255]," +
 "\"directional_intensity\":1,\"ambient_intensity\":1,\"fog_start_m\":10,\"fog_end_m\":20}," +
 "\"floor\":{\"base_rgba\":[1,2,3,255],\"grid_rgba\":[1,2,3,255],\"grid_spacing_m\":1}," +
 "\"instances\":[") + instance_json + "]}";
}

} // namespace

int main() {
 const std::filesystem::path root = TEST_SOURCE_DIR;
 lost_signal::sim::viewer::AssetManifest assets;
 std::string error;
 assert(lost_signal::sim::viewer::load_asset_manifest(
 (root / "assets/models/manifest.json").string(), assets, &error));

 lost_signal::sim::eval::WorldFileContent world;
 assert(lost_signal::sim::eval::load_world_file(
 (root / "configs/worlds/research_base.json").string(), world, &error));
 lost_signal::sim::viewer::SceneManifest scene;
 assert(lost_signal::sim::viewer::load_scene_manifest(
 (root / "configs/worlds/research_base.scene.json").string(), assets,
 world.boxes, scene, &error));
 assert(scene.scene_id == "research_base");
 assert(scene.surface_only_max_height_m == 0.10);
 assert(scene.camera.follow_smoothing_s == 0.38);
 assert(scene.camera.maximum_distance_m == 8.0);
 assert(scene.lighting.fog_start_m == 12.0);
 assert(scene.lighting.direction_xyz[2] == -1.0);
 assert(scene.lighting.exposure == 0.92);
 assert(scene.lighting.bloom_strength == 0.20);
 assert(scene.lighting.shadow_map_resolution == 2048);
 assert(scene.lighting.shadow_strength == 0.62);
 assert(scene.instances.size() == 38);
 assert(scene.visual_for_box("north_wall_01") != nullptr);
 assert(scene.visual_for_box("west_shelf") != nullptr);
 assert(scene.visual_for_box("east_shelf") != nullptr);
 assert(scene.visual_for_box("glass_door") != nullptr);
 const auto has_instance = [&](const std::string& id) {
 return std::any_of(scene.instances.begin(), scene.instances.end(),
 [&](const auto& instance) { return instance.instance_id == id; });
 };
 assert(has_instance("signal_cross_tower_visual"));
 assert(has_instance("safety_lane_bollards_visual"));
 assert(has_instance("supply_pallet_visual"));
 // Tall props own a collision box. They were surface_only before A2, which
 // let a 2.27 m tower sit on a goal point with no physical volume.
 for (const char* box_id :
      {"signal_cross_tower", "safety_lane_bollards", "supply_pallet"}) {
 const auto* visual = scene.visual_for_box(box_id);
 assert(visual != nullptr);
 assert(!visual->surface_only);
 }

 lost_signal::sim::eval::WorldFileContent slam_world;
 assert(lost_signal::sim::eval::load_world_file(
 (root / "configs/worlds/slam_loop_lab.json").string(), slam_world,
 &error));
 assert(lost_signal::sim::viewer::load_scene_manifest(
 (root / "configs/worlds/slam_loop_lab.scene.json").string(), assets,
 slam_world.boxes, scene, &error));
 assert(scene.scene_id == "slam_loop_lab");
 assert(scene.instances.size() == 12);
 assert(scene.visual_for_box("east_landmark") != nullptr);
 assert(scene.visual_for_box("lab_south_wall") != nullptr);

 const auto legacy = std::filesystem::temp_directory_path() /
 "lost_signal_legacy_scene.json";
 write_text(legacy,
 R"({"format":"lost-signal-scene/1","scene_id":"legacy","camera":{"distance_m":4,"yaw_deg":0,"pitch_deg":30,"target_height_m":0.2},"floor":{"base_rgba":[1,2,3,255],"grid_rgba":[1,2,3,255],"grid_spacing_m":1},"instances":[]})");
 assert(lost_signal::sim::viewer::load_scene_manifest(
 legacy.string(), assets, world.boxes, scene, &error));
 assert(scene.lighting.fog_end_m == 31.0);
 std::filesystem::remove(legacy);

 const auto temporary = std::filesystem::temp_directory_path() /
 "lost_signal_bad_scene.json";
 write_text(temporary,
 R"({"format":"lost-signal-scene/1","scene_id":"bad","camera":{"distance_m":4,"yaw_deg":0,"pitch_deg":30,"target_height_m":0.2},"lighting":{"direction_xyz":[0,0,-1],"directional_rgba":[255,255,255,255],"ambient_rgba":[64,64,64,255],"fog_rgba":[10,10,10,255],"directional_intensity":1,"ambient_intensity":1,"fog_start_m":10,"fog_end_m":20},"floor":{"base_rgba":[1,2,3,255],"grid_rgba":[1,2,3,255],"grid_spacing_m":1},"instances":[{"instance_id":"orphan","asset_id":"environment.research_base.warning_strip","position_m":[0,0,0],"scale":[1,1,1],"yaw_deg":0}]})");
 assert(!lost_signal::sim::viewer::load_scene_manifest(
 temporary.string(), assets, world.boxes, scene, &error));
 assert(!error.empty());
 std::filesystem::remove(temporary);

 // surface_only means flat paint (R1): the instance top, position z plus the
 // scaled asset height, stays within 0.10 m. A solid prop cannot skip its
 // collision box by claiming surface_only, whatever its z offset or scale.
 const auto probe = std::filesystem::temp_directory_path() /
 "lost_signal_surface_probe.json";
 const auto load_probe = [&](const std::string& instance_json) {
 write_text(probe, scene_with_instance(instance_json));
 return lost_signal::sim::viewer::load_scene_manifest(
 probe.string(), assets, world.boxes, scene, &error);
 };
 // Control: a flat warning strip on the ground loads.
 assert(load_probe(
 R"({"instance_id":"flat_strip","asset_id":"environment.research_base.warning_strip","surface_only":true,"position_m":[0,0,0.004],"scale":[1,1,1],"yaw_deg":0})"));
 // Solid pallet (0.63 m) declared surface_only: rejected, and the message
 // names the instance so the author can find it.
 assert(!load_probe(
 R"({"instance_id":"tall_pallet","asset_id":"environment.research_base.pallet_stack","surface_only":true,"position_m":[0,0,0],"scale":[1,1,1],"yaw_deg":0})"));
 assert(error.find("tall_pallet") != std::string::npos);
 assert(error.find("collision_box_id") != std::string::npos);
 // Scale counts: a 0.027 m strip scaled 10x vertically is 0.27 m tall.
 assert(!load_probe(
 R"({"instance_id":"scaled_strip","asset_id":"environment.research_base.warning_strip","surface_only":true,"position_m":[0,0,0],"scale":[1,1,10],"yaw_deg":0})"));
 // z offset counts: both pass the position-only check (|z| <= 0.10), but the
 // top is 0.117 m for z = 0.09 and 0.097 m for z = 0.07.
 assert(!load_probe(
 R"({"instance_id":"raised_strip","asset_id":"environment.research_base.warning_strip","surface_only":true,"position_m":[0,0,0.09],"scale":[1,1,1],"yaw_deg":0})"));
 assert(load_probe(
 R"({"instance_id":"low_strip","asset_id":"environment.research_base.warning_strip","surface_only":true,"position_m":[0,0,0.07],"scale":[1,1,1],"yaw_deg":0})"));

 // A surface contract is mandatory for a scene that uses surface_only; the
 // loader has no hidden fallback threshold.
 write_text(probe, scene_with_instance(
 R"({"instance_id":"undeclared_strip","asset_id":"environment.research_base.warning_strip","surface_only":true,"position_m":[0,0,0.004],"scale":[1,1,1],"yaw_deg":0})",
 false));
 assert(!lost_signal::sim::viewer::load_scene_manifest(
 probe.string(), assets, world.boxes, scene, &error));
 assert(error.find("surface_only_max_height_m") != std::string::npos);
 std::filesystem::remove(probe);

 // Every checked-in scene must load against its own world, so R1 and the
 // binding rules hold for committed content and not only for the fixtures
 // above. A new scene file is picked up without editing this test.
 {
 const std::filesystem::path worlds = root / "configs/worlds";
 constexpr const char* kSuffix = ".scene.json";
 std::size_t checked = 0;
 for (const auto& entry : std::filesystem::directory_iterator(worlds)) {
 const std::string filename = entry.path().filename().string();
 if (!filename.ends_with(kSuffix)) continue;
 const std::string map_name =
 filename.substr(0, filename.size() -
 std::char_traits<char>::length(kSuffix)) + ".json";
 // lab_default.scene.json is an intentional presentation fallback for
 // authored worlds that do not have a one-to-one scene file.  It is loaded
 // against the active world's collision boxes by the Viewer, so there is no
 // lab_default.json companion to validate here.
 if (!std::filesystem::is_regular_file(worlds / map_name)) continue;
 lost_signal::sim::eval::WorldFileContent content;
 const bool world_loaded = lost_signal::sim::eval::load_world_file(
 (worlds / map_name).string(), content, &error);
 if (!world_loaded) std::fprintf(stderr, "%s: %s\n", map_name.c_str(), error.c_str());
 assert(world_loaded);
 const bool scene_loaded = lost_signal::sim::viewer::load_scene_manifest(
 entry.path().string(), assets, content.boxes, scene, &error);
 if (!scene_loaded) std::fprintf(stderr, "%s: %s\n", filename.c_str(), error.c_str());
 assert(scene_loaded);
 ++checked;
 }
 assert(checked >= 3);
 }
 return 0;
}
