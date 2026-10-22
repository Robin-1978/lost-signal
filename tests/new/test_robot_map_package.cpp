#include "robot/map_io/robot_map_package.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

namespace {

using Json = nlohmann::json;

Json read_json(const std::filesystem::path& path) {
 std::ifstream input(path);
 return Json::parse(input);
}

void write_json(const std::filesystem::path& path, const Json& value) {
 std::ofstream output(path, std::ios::trunc);
 output << value.dump(2) << '\n';
}

bool load_rejected(const std::filesystem::path& root) {
 try {
 (void)sim::map_io::load_robot_map_package(root);
 return false;
 } catch (const std::exception&) {
 return true;
 }
}

sim::runtime::MapSnapshot source_map() {
 sim::runtime::MapSnapshot map;
 map.map_id = "slam/factory-a";
 map.revision = 42;
 map.frame_id = "map";
 map.provenance = {"survey", "test.map-compiler", "survey-2026-10"};
 map.grid.emplace();
 map.grid->width = 2;
 map.grid->height = 2;
 map.grid->resolution_m = 0.05;
 map.grid->origin_m = {-3.25, 8.5};
 map.grid->origin_yaw_rad = 0.3;
 map.grid->occupancy = {-1, 0, 37, 100};
 map.grid->elevation_m = {std::numeric_limits<float>::quiet_NaN(), 0.1F, -0.2F, 1.5F};
 map.grid->elevation_sigma_m = {std::numeric_limits<float>::quiet_NaN(), 0.02F, 0.03F, 0.04F};
 map.grid->roughness_m = {0.01F, 0.02F, std::numeric_limits<float>::quiet_NaN(), 0.03F};
 map.grid->roughness_confidence = {0.9F, 0.6F, std::numeric_limits<float>::quiet_NaN(), 0.7F};
 map.grid->surface_classes = {"unknown", "concrete", "gravel"};
 map.grid->surface_class = {0, 1, 2, 1};
 map.places.emplace("dock", sim::runtime::SemanticPlace{{-3.0, 8.6}, 1.0});
 map.features.push_back({
 "virtual-wall-1", "virtual_wall", sim::runtime::SemanticGeometry::Polyline,
 {{-3.0, 8.5}, {-2.0, 8.5}}, {{"source", "operator"}},
 });
 map.relations.push_back({
 "dock-wall", "approaches", "dock", "virtual-wall-1", true, {{"role", "route_hint"}},
 });
 return map;
}

} // namespace

int main() {
 const auto root = std::filesystem::temp_directory_path() / "lost-signal-robot-map-test.lsmap";
 std::error_code ignored;
 std::filesystem::remove_all(root, ignored);
 const auto source = source_map();
 sim::map_io::save_robot_map_package(source, root);
 CHECK(std::filesystem::is_regular_file(root / "manifest.json"));
 CHECK(std::filesystem::is_regular_file(root / "layers/occupancy.i8"));

 const auto loaded = sim::map_io::load_robot_map_package(root, sim::runtime::TimePoint{1234});
 CHECK(loaded.map_id == source.map_id);
 CHECK(loaded.revision == source.revision);
 CHECK(loaded.sample_time.nanoseconds == 1234);
 CHECK(loaded.grid->occupancy == source.grid->occupancy);
 CHECK(loaded.grid->surface_class == source.grid->surface_class);
 CHECK(loaded.grid->surface_classes == source.grid->surface_classes);
 CHECK(loaded.grid->elevation_m.size() == source.grid->elevation_m.size());
 CHECK(std::isnan(loaded.grid->elevation_m.front()));
 CHECK(loaded.grid->elevation_m[3] == source.grid->elevation_m[3]);
 CHECK(loaded.grid->roughness_m.size() == source.grid->roughness_m.size());
 CHECK(loaded.grid->roughness_confidence.size() == source.grid->roughness_confidence.size());
 CHECK(loaded.grid->roughness_m[0] == source.grid->roughness_m[0]);
 CHECK(std::isnan(loaded.grid->roughness_m[2]));
 CHECK(loaded.grid->roughness_confidence[1] == source.grid->roughness_confidence[1]);
 CHECK(loaded.places.at("dock").heading_rad == 1.0);
 CHECK(loaded.features.front().attributes.at("source") == "operator");
 CHECK(loaded.relations.front().from_id == "dock");

 // Content corruption must fail before a partial or shifted map reaches a
 // planner. The manifest checksum is over the exact encoded bytes.
 {
 std::fstream occupancy(root / "layers/occupancy.i8", std::ios::binary | std::ios::in | std::ios::out);
 CHECK(static_cast<bool>(occupancy));
 occupancy.put('\0');
 }
 CHECK(load_rejected(root));
 std::filesystem::remove_all(root, ignored);

 // Layer names are unique and a raster grid always carries explicit
 // occupancy. Files present on disk but absent from the manifest are ignored.
 sim::map_io::save_robot_map_package(source, root);
 {
 auto manifest = read_json(root / "manifest.json");
 manifest["layers"].push_back(manifest["layers"].front());
 write_json(root / "manifest.json", manifest);
 }
 CHECK(load_rejected(root));
 std::filesystem::remove_all(root, ignored);

 sim::map_io::save_robot_map_package(source, root);
 {
 auto manifest = read_json(root / "manifest.json");
 auto& layers = manifest["layers"];
 layers.erase(std::remove_if(layers.begin(), layers.end(), [](const Json& layer) {
 return layer.at("name") == "occupancy";
 }), layers.end());
 write_json(root / "manifest.json", manifest);
 }
 CHECK(load_rejected(root));
 std::filesystem::remove_all(root, ignored);

 // Package references cannot leave the package directory. This is checked
 // structurally before any external file is read.
 sim::map_io::save_robot_map_package(source, root);
 {
 auto manifest = read_json(root / "manifest.json");
 manifest["layers"][0]["file"] = "../outside.i8";
 write_json(root / "manifest.json", manifest);
 }
 CHECK(load_rejected(root));
 std::filesystem::remove_all(root, ignored);

 auto vector_only = source;
 vector_only.map_id = "hd-map/campus";
 vector_only.revision = 1;
 vector_only.grid.reset();
 sim::map_io::save_robot_map_package(vector_only, root);
 CHECK(!std::filesystem::exists(root / "layers/occupancy.i8"));
 const auto loaded_vector = sim::map_io::load_robot_map_package(root);
 CHECK(!loaded_vector.grid);
 CHECK(loaded_vector.features.size() == 1);
 CHECK(loaded_vector.relations.size() == 1);

 // Replacing an existing package publishes the new revision as one complete
 // directory and removes the backup after success.
 vector_only.revision = 2;
 sim::map_io::save_robot_map_package(vector_only, root);
 CHECK(sim::map_io::load_robot_map_package(root).revision == 2);
 CHECK(!std::filesystem::exists(root.string() + ".old"));
 std::filesystem::remove_all(root, ignored);
 return 0;
}
