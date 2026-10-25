#include <cassert>
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
 assert(scene.camera.follow_smoothing_s == 0.38);
 assert(scene.camera.maximum_distance_m == 8.0);
 assert(scene.lighting.fog_start_m == 12.0);
 assert(scene.lighting.direction_xyz[2] == -1.0);
 assert(scene.lighting.exposure == 0.92);
 assert(scene.lighting.bloom_strength == 0.20);
 assert(scene.lighting.shadow_map_resolution == 2048);
 assert(scene.lighting.shadow_strength == 0.62);
 assert(scene.instances.size() == 35);
 assert(scene.visual_for_box("north_wall_01") != nullptr);
 assert(scene.visual_for_box("west_shelf") != nullptr);
 assert(scene.visual_for_box("east_shelf") != nullptr);
 assert(scene.visual_for_box("glass_door") != nullptr);

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
 return 0;
}
