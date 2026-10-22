#include "robot/map_io/robot_map_package.hpp"

#include "robot/core/sha256.hpp"

#include <nlohmann/json.hpp>

#include <cstdint>
#include <cstring>
#include <fstream>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>

namespace sim::map_io {
namespace {

using Json = nlohmann::ordered_json;

std::string read_bytes(const std::filesystem::path& path) {
 std::ifstream input(path, std::ios::binary);
 if (!input)
 throw std::runtime_error("cannot read robot-map file: " + path.string());
 return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

void write_bytes(const std::filesystem::path& path, std::string_view bytes) {
 std::ofstream output(path, std::ios::binary | std::ios::trunc);
 if (!output)
 throw std::runtime_error("cannot write robot-map file: " + path.string());
 output.write(bytes.data(), static_cast<std::streamsize>(bytes.size()));
 if (!output)
 throw std::runtime_error("incomplete robot-map write: " + path.string());
}

std::string encode_float32(const std::vector<float>& values) {
 std::string bytes;
 bytes.reserve(values.size() * 4);
 for (const float value : values) {
 std::uint32_t word = 0;
 static_assert(sizeof(word) == sizeof(value));
 std::memcpy(&word, &value, sizeof(word));
 for (int shift = 0; shift < 32; shift += 8)
 bytes.push_back(static_cast<char>((word >> shift) & 0xffU));
 }
 return bytes;
}

std::vector<float> decode_float32(std::string_view bytes, std::size_t count) {
 if (bytes.size() != count * 4)
 throw std::runtime_error("robot-map float32 layer has the wrong byte length");
 std::vector<float> values;
 values.reserve(count);
 for (std::size_t offset = 0; offset < bytes.size(); offset += 4) {
 std::uint32_t word = 0;
 for (int index = 0; index < 4; ++index)
 word |= static_cast<std::uint32_t>(static_cast<unsigned char>(bytes[offset + index])) << (index * 8);
 float value = 0.0F;
 std::memcpy(&value, &word, sizeof(value));
 values.push_back(value);
 }
 return values;
}

std::string encode_uint16(const std::vector<std::uint16_t>& values) {
 std::string bytes;
 bytes.reserve(values.size() * 2);
 for (const auto value : values) {
 bytes.push_back(static_cast<char>(value & 0xffU));
 bytes.push_back(static_cast<char>((value >> 8) & 0xffU));
 }
 return bytes;
}

std::vector<std::uint16_t> decode_uint16(std::string_view bytes, std::size_t count) {
 if (bytes.size() != count * 2)
 throw std::runtime_error("robot-map uint16 layer has the wrong byte length");
 std::vector<std::uint16_t> values;
 values.reserve(count);
 for (std::size_t offset = 0; offset < bytes.size(); offset += 2) {
 values.push_back(static_cast<std::uint16_t>(
 static_cast<unsigned char>(bytes[offset]) |
 (static_cast<std::uint16_t>(static_cast<unsigned char>(bytes[offset + 1])) << 8)));
 }
 return values;
}

const char* geometry_name(runtime::SemanticGeometry geometry) {
 switch (geometry) {
 case runtime::SemanticGeometry::Point: return "point";
 case runtime::SemanticGeometry::Polyline: return "polyline";
 case runtime::SemanticGeometry::Polygon: return "polygon";
 }
 throw std::runtime_error("unknown semantic geometry");
}

runtime::SemanticGeometry parse_geometry(const std::string& value) {
 if (value == "point") return runtime::SemanticGeometry::Point;
 if (value == "polyline") return runtime::SemanticGeometry::Polyline;
 if (value == "polygon") return runtime::SemanticGeometry::Polygon;
 throw std::runtime_error("unknown robot-map semantic geometry: " + value);
}

Json point(Vec2 value) {
 return {value.x, value.y};
}

Vec2 parse_point(const Json& value) {
 if (!value.is_array() || value.size() != 2)
 throw std::runtime_error("robot-map point must contain two coordinates");
 return {value.at(0).get<double>(), value.at(1).get<double>()};
}

Json features_json(const runtime::MapSnapshot& map) {
 Json places = Json::object();
 for (const auto& [id, place] : map.places)
 places[id] = {place.position_m.x, place.position_m.y, place.heading_rad};
 Json features = Json::array();
 for (const auto& feature : map.features) {
 Json vertices = Json::array();
 for (const auto vertex : feature.vertices_m)
 vertices.push_back(point(vertex));
 features.push_back({
 {"id", feature.id},
 {"kind", feature.kind},
 {"geometry", geometry_name(feature.geometry)},
 {"vertices_m", std::move(vertices)},
 {"attributes", feature.attributes},
 });
 }
 Json relations = Json::array();
 for (const auto& relation : map.relations) {
 relations.push_back({
 {"id", relation.id},
 {"kind", relation.kind},
 {"from", relation.from_id},
 {"to", relation.to_id},
 {"directed", relation.directed},
 {"attributes", relation.attributes},
 });
 }
 return {
 {"schema_version", 1},
 {"places", std::move(places)},
 {"features", std::move(features)},
 {"relations", std::move(relations)},
 };
}

void parse_features(const Json& root, runtime::MapSnapshot& map) {
 if (root.at("schema_version").get<int>() != 1)
 throw std::runtime_error("unsupported robot-map feature schema version");
 for (const auto& [id, value] : root.at("places").items()) {
 if (!value.is_array() || value.size() != 3)
 throw std::runtime_error("robot-map semantic place must contain x, y, and heading");
 map.places.emplace(id, runtime::SemanticPlace{
 {value.at(0).get<double>(), value.at(1).get<double>()}, value.at(2).get<double>()});
 }
 for (const auto& value : root.at("features")) {
 runtime::SemanticFeature feature;
 feature.id = value.at("id").get<std::string>();
 feature.kind = value.at("kind").get<std::string>();
 feature.geometry = parse_geometry(value.at("geometry").get<std::string>());
 for (const auto& vertex : value.at("vertices_m"))
 feature.vertices_m.push_back(parse_point(vertex));
 feature.attributes = value.at("attributes").get<std::map<std::string, std::string>>();
 map.features.push_back(std::move(feature));
 }
 for (const auto& value : root.value("relations", Json::array())) {
 runtime::SemanticRelation relation;
 relation.id = value.at("id").get<std::string>();
 relation.kind = value.at("kind").get<std::string>();
 relation.from_id = value.at("from").get<std::string>();
 relation.to_id = value.at("to").get<std::string>();
 relation.directed = value.value("directed", true);
 relation.attributes = value.at("attributes").get<std::map<std::string, std::string>>();
 map.relations.push_back(std::move(relation));
 }
}

Json layer_entry(std::string name, std::string file, std::string dtype,
 std::string_view bytes, std::string unknown) {
 return {
 {"name", std::move(name)},
 {"file", std::move(file)},
 {"dtype", std::move(dtype)},
 {"byte_order", "little"},
 {"unknown", std::move(unknown)},
 {"sha256", sim::sha256(bytes)},
 };
}

std::string checked_layer(const std::filesystem::path& root, const Json& layer,
 std::string_view expected_name, std::string_view expected_dtype) {
 if (layer.at("name").get<std::string>() != expected_name ||
 layer.at("dtype").get<std::string>() != expected_dtype ||
 layer.at("byte_order").get<std::string>() != "little") {
 throw std::runtime_error("robot-map layer contract mismatch");
 }
 const auto relative = std::filesystem::path(layer.at("file").get<std::string>());
 if (relative.empty() || relative.is_absolute() || relative.has_root_name() ||
 relative.has_root_directory()) {
 throw std::runtime_error("robot-map layer path escapes package");
 }
 for (const auto& component : relative) {
 if (component == ".." || component == ".")
 throw std::runtime_error("robot-map layer path escapes package");
 }
 const auto package_root = std::filesystem::weakly_canonical(root);
 const auto resolved = std::filesystem::weakly_canonical(root / relative);
 auto root_component = package_root.begin();
 auto resolved_component = resolved.begin();
 for (; root_component != package_root.end() && resolved_component != resolved.end();
 ++root_component, ++resolved_component) {
 if (*root_component != *resolved_component)
 throw std::runtime_error("robot-map layer path escapes package");
 }
 if (root_component != package_root.end())
 throw std::runtime_error("robot-map layer path escapes package");
 const auto bytes = read_bytes(resolved);
 if (sim::sha256(bytes) != layer.at("sha256").get<std::string>())
 throw std::runtime_error("robot-map layer checksum mismatch: " + relative.string());
 return bytes;
}

std::filesystem::path checked_package_path(const std::filesystem::path& root,
 const std::string& value) {
 const auto relative = std::filesystem::path(value);
 if (relative.empty() || relative.is_absolute() || relative.has_root_name() ||
 relative.has_root_directory()) {
 throw std::runtime_error("robot-map path escapes package");
 }
 for (const auto& component : relative) {
 if (component == ".." || component == ".")
 throw std::runtime_error("robot-map path escapes package");
 }
 const auto package_root = std::filesystem::weakly_canonical(root);
 const auto resolved = std::filesystem::weakly_canonical(root / relative);
 auto root_component = package_root.begin();
 auto resolved_component = resolved.begin();
 for (; root_component != package_root.end() && resolved_component != resolved.end();
 ++root_component, ++resolved_component) {
 if (*root_component != *resolved_component)
 throw std::runtime_error("robot-map path escapes package");
 }
 if (root_component != package_root.end())
 throw std::runtime_error("robot-map path escapes package");
 return resolved;
}

} // namespace

void save_robot_map_package(const runtime::MapSnapshot& map,
 const std::filesystem::path& package_path) {
 std::string reason;
 if (!runtime::valid_map_snapshot(map, &reason))
 throw std::invalid_argument("cannot save invalid robot map: " + reason);

 const auto temporary = package_path.string() + ".tmp";
 const auto backup = package_path.string() + ".old";
 std::error_code ignored;
 std::filesystem::remove_all(temporary, ignored);
 std::filesystem::create_directories(std::filesystem::path(temporary) / "layers");

 Json layers = Json::array();
 Json grid_manifest;
 if (map.grid) {
 const auto& grid = *map.grid;
 std::string occupancy;
 occupancy.reserve(grid.occupancy.size());
 for (const auto value : grid.occupancy)
 occupancy.push_back(static_cast<char>(value));
 write_bytes(std::filesystem::path(temporary) / "layers/occupancy.i8", occupancy);
 layers.push_back(layer_entry("occupancy", "layers/occupancy.i8", "int8", occupancy, "-1"));

 const auto write_float_layer = [&](std::string name, const std::vector<float>& values) {
 if (values.empty())
 return;
 const auto bytes = encode_float32(values);
 const std::string file = "layers/" + name + ".f32";
 write_bytes(std::filesystem::path(temporary) / file, bytes);
 layers.push_back(layer_entry(name, file, "float32", bytes, "NaN"));
 };
 write_float_layer("elevation_m", grid.elevation_m);
 write_float_layer("elevation_sigma_m", grid.elevation_sigma_m);
 write_float_layer("roughness_m", grid.roughness_m);
 write_float_layer("roughness_confidence", grid.roughness_confidence);
 if (!grid.surface_class.empty()) {
 const auto bytes = encode_uint16(grid.surface_class);
 write_bytes(std::filesystem::path(temporary) / "layers/surface_class.u16", bytes);
 layers.push_back(layer_entry(
 "surface_class", "layers/surface_class.u16", "uint16", bytes, "0"));
 }
 Json surface_properties = Json::array();
 for (const auto& properties : grid.surface_class_properties) {
  surface_properties.push_back({
  {"nominal_friction_mu", properties.nominal_friction_mu},
  {"rolling_resistance", properties.rolling_resistance},
  });
 }
 grid_manifest = {
 {"width", grid.width},
 {"height", grid.height},
 {"resolution_m", grid.resolution_m},
 {"origin", {grid.origin_m.x, grid.origin_m.y, grid.origin_yaw_rad}},
 {"layout", "row-major-x-fastest-y-up"},
 {"surface_classes", grid.surface_classes},
  {"surface_class_properties", std::move(surface_properties)},
 };
 }

 const std::string feature_text = features_json(map).dump(2) + "\n";
 write_bytes(std::filesystem::path(temporary) / "features.json", feature_text);
 Json manifest = {
 {"format", "lost-signal-robot-map"},
 {"schema_version", 2},
 {"map_id", map.map_id},
 {"content_revision", map.revision},
 {"frame_id", map.frame_id},
 {"provenance", {
 {"source_kind", map.provenance.source_kind},
 {"producer_id", map.provenance.producer_id},
 {"source_revision", map.provenance.source_revision},
 }},
 {"grid", map.grid ? grid_manifest : Json(nullptr)},
 {"layers", std::move(layers)},
 {"features", {{"file", "features.json"}, {"sha256", sim::sha256(feature_text)}}},
 };
 // The manifest is written last. A package without it is never loadable.
 write_bytes(std::filesystem::path(temporary) / "manifest.json", manifest.dump(2) + "\n");

 std::filesystem::remove_all(backup, ignored);
 if (std::filesystem::exists(package_path))
 std::filesystem::rename(package_path, backup);
 try {
 std::filesystem::rename(temporary, package_path);
 std::filesystem::remove_all(backup, ignored);
 } catch (...) {
 if (std::filesystem::exists(backup) && !std::filesystem::exists(package_path))
 std::filesystem::rename(backup, package_path);
 throw;
 }
}

runtime::MapSnapshot load_robot_map_package(const std::filesystem::path& package_path,
 runtime::TimePoint publication_time) {
 const auto manifest = Json::parse(read_bytes(package_path / "manifest.json"));
 if (manifest.at("format").get<std::string>() != "lost-signal-robot-map" ||
 manifest.at("schema_version").get<int>() != 2) {
 throw std::runtime_error("unsupported robot-map package format");
 }
 runtime::MapSnapshot map;
 map.map_id = manifest.at("map_id").get<std::string>();
 map.revision = manifest.at("content_revision").get<std::uint64_t>();
 map.frame_id = manifest.at("frame_id").get<std::string>();
 if (manifest.contains("provenance")) {
 const auto& provenance = manifest.at("provenance");
 map.provenance.source_kind = provenance.value("source_kind", "");
 map.provenance.producer_id = provenance.value("producer_id", "");
 map.provenance.source_revision = provenance.value("source_revision", "");
 }
 map.sample_time = publication_time;
 if (!manifest.at("grid").is_null()) {
 const auto& description = manifest.at("grid");
 auto& grid = map.grid.emplace();
 grid.width = description.at("width").get<std::uint32_t>();
 grid.height = description.at("height").get<std::uint32_t>();
 grid.resolution_m = description.at("resolution_m").get<double>();
 const auto origin = description.at("origin");
 if (!origin.is_array() || origin.size() != 3 ||
 description.at("layout").get<std::string>() != "row-major-x-fastest-y-up") {
 throw std::runtime_error("unsupported robot-map grid layout");
 }
 grid.origin_m = {origin.at(0).get<double>(), origin.at(1).get<double>()};
 grid.origin_yaw_rad = origin.at(2).get<double>();
 grid.surface_classes = description.at("surface_classes").get<std::vector<std::string>>();
 if (description.contains("surface_class_properties")) {
  for (const auto& item : description.at("surface_class_properties")) {
   grid.surface_class_properties.push_back({
   item.at("nominal_friction_mu").get<double>(),
   item.at("rolling_resistance").get<double>(),
   });
  }
 }
 const auto count = static_cast<std::size_t>(grid.width) * grid.height;
 std::set<std::string> layer_names;
 for (const auto& layer : manifest.at("layers")) {
 const auto name = layer.at("name").get<std::string>();
 if (!layer_names.insert(name).second)
 throw std::runtime_error("duplicate robot-map layer: " + name);
 if (name == "occupancy") {
 const auto bytes = checked_layer(package_path, layer, name, "int8");
 if (bytes.size() != count)
 throw std::runtime_error("robot-map occupancy layer has the wrong byte length");
 grid.occupancy.reserve(count);
 for (const unsigned char value : bytes) {
 std::int8_t signed_value = 0;
 std::memcpy(&signed_value, &value, sizeof(signed_value));
 grid.occupancy.push_back(signed_value);
 }
 } else if (name == "elevation_m") {
 grid.elevation_m = decode_float32(checked_layer(package_path, layer, name, "float32"), count);
 } else if (name == "elevation_sigma_m") {
 grid.elevation_sigma_m = decode_float32(
 checked_layer(package_path, layer, name, "float32"), count);
 } else if (name == "roughness_m") {
  grid.roughness_m = decode_float32(
  checked_layer(package_path, layer, name, "float32"), count);
 } else if (name == "roughness_confidence") {
 grid.roughness_confidence = decode_float32(
 checked_layer(package_path, layer, name, "float32"), count);
 } else if (name == "surface_class") {
 grid.surface_class = decode_uint16(
 checked_layer(package_path, layer, name, "uint16"), count);
 } else {
 throw std::runtime_error("unsupported robot-map layer: " + name);
 }
 }
 if (grid.occupancy.empty())
 throw std::runtime_error("robot-map grid requires an occupancy layer");
 } else if (!manifest.at("layers").empty()) {
 throw std::runtime_error("vector-only robot map cannot contain raster layers");
 }
 const auto feature_info = manifest.at("features");
 const auto feature_path = checked_package_path(
 package_path, feature_info.at("file").get<std::string>());
 const auto feature_text = read_bytes(feature_path);
 if (sim::sha256(feature_text) != feature_info.at("sha256").get<std::string>())
 throw std::runtime_error("robot-map feature checksum mismatch");
 parse_features(Json::parse(feature_text), map);
 std::string reason;
 if (!runtime::valid_map_snapshot(map, &reason))
 throw std::runtime_error("invalid robot-map package: " + reason);
 return map;
}

} // namespace sim::map_io
