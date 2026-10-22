#include "construction_plan.hpp"

#include "robot/core/math.hpp"
#include "robot/core/sha256.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>

namespace mission {
namespace {

using ordered_json = nlohmann::ordered_json;

bool allows_socket(const ConstructionPieceDefinition& piece, const std::string& type) {
 return std::find(piece.allowed_socket_types.begin(), piece.allowed_socket_types.end(), type) !=
 piece.allowed_socket_types.end();
}

double distance(Vec3 a, Vec3 b) {
 return std::hypot(std::hypot(b.x - a.x, b.y - a.y), b.z - a.z);
}

} // namespace

std::string serialize_construction_plan_canonical(const ConstructionPlan& plan) {
 auto placements = plan.placements;
 std::sort(placements.begin(), placements.end(), [](const auto& left, const auto& right) {
 return left.instance_id < right.instance_id;
 });

 ordered_json encoded_placements = ordered_json::array();
 for (const auto& placement : placements) {
 ordered_json item;
 // RFC 8785-style lexicographic key order. Values are identifiers, avoiding
 // floating-point spelling differences in the hashed plan.
 item["end_socket"] = placement.end_socket;
 item["instance_id"] = placement.instance_id;
 item["piece_id"] = placement.piece_id;
 item["start_socket"] = placement.start_socket;
 encoded_placements.push_back(std::move(item));
 }
 ordered_json root;
 root["placements"] = std::move(encoded_placements);
 root["scenario"] = plan.scenario;
 root["schema_version"] = plan.schema_version;
 return root.dump();
}

std::string construction_plan_sha256(const ConstructionPlan& plan) {
 return sim::sha256(serialize_construction_plan_canonical(plan));
}

ConstructionPlanResolution resolve_construction_plan(
 const ConstructionScenarioDefinition& scenario,
 const std::unordered_map<std::string, ConstructionPieceDefinition>& catalog,
 const ConstructionPlan& plan) {
 ConstructionPlanResolution result;
 result.canonical_plan = serialize_construction_plan_canonical(plan);
 result.plan_sha256 = sim::sha256(result.canonical_plan);
 const auto reject = [&](std::string reason) {
 result.reason = std::move(reason);
 result.pieces.clear();
 return result;
 };

 if (plan.schema_version != 1)
 return reject("unsupported construction plan schema");
 if (plan.scenario != scenario.id)
 return reject("construction plan targets another scenario");

 std::unordered_map<std::string, const ConstructionSocketDefinition*> sockets;
 for (const auto& socket : scenario.sockets) {
 if (!sockets.emplace(socket.id, &socket).second)
 return reject("construction scenario contains duplicate socket '" + socket.id + "'");
 }
 auto remaining = scenario.inventory;
 std::set<std::string> instances;
 std::set<std::string> occupied_sockets;
 auto placements = plan.placements;
 std::sort(placements.begin(), placements.end(), [](const auto& left, const auto& right) {
 return left.instance_id < right.instance_id;
 });

 for (const auto& placement : placements) {
 if (placement.instance_id.empty() || !instances.insert(placement.instance_id).second)
 return reject("construction plan contains an empty or duplicate instance id");
 const auto piece_it = catalog.find(placement.piece_id);
 if (piece_it == catalog.end())
 return reject("construction plan references unknown piece '" + placement.piece_id + "'");
 auto inventory_it = remaining.find(placement.piece_id);
 if (inventory_it == remaining.end() || inventory_it->second <= 0)
 return reject("construction plan exceeds inventory for '" + placement.piece_id + "'");
 --inventory_it->second;

 const auto start_it = sockets.find(placement.start_socket);
 const auto end_it = sockets.find(placement.end_socket);
 if (start_it == sockets.end() || end_it == sockets.end())
 return reject("construction plan references an unknown socket");
 if (placement.start_socket == placement.end_socket)
 return reject("construction piece endpoints must use different sockets");
 if (!occupied_sockets.insert(placement.start_socket).second ||
 !occupied_sockets.insert(placement.end_socket).second) {
 return reject("construction socket is occupied more than once");
 }

 const auto& piece = piece_it->second;
 const auto& start = *start_it->second;
 const auto& end = *end_it->second;
 if (!allows_socket(piece, start.type) || !allows_socket(piece, end.type))
 return reject("construction piece is incompatible with a socket type");
 if (piece.required_endpoint_contacts != 2)
 return reject("bounded construction currently requires two endpoint contacts");

 const double span = distance(start.position_m, end.position_m);
 if (!(span > 0.0) || span > piece.max_span_m + scenario.snap_tolerance_m ||
 std::abs(span - piece.size_m.x) > scenario.snap_tolerance_m) {
 return reject("construction piece length does not match its socket span");
 }
 const double dx = end.position_m.x - start.position_m.x;
 const double dy = end.position_m.y - start.position_m.y;
 const double dz = end.position_m.z - start.position_m.z;
 const double horizontal = std::hypot(dx, dy);
 result.pieces.push_back({
 placement,
 {(start.position_m.x + end.position_m.x) * 0.5,
 (start.position_m.y + end.position_m.y) * 0.5,
 (start.position_m.z + end.position_m.z) * 0.5},
 horizontal > 1e-9 ? std::atan2(dy, dx) : 0.0,
 std::atan2(dz, horizontal),
 });
 }

 result.ok = true;
 return result;
}

ConstructionPlan construction_plan_from_option(
 const ConstructionScenarioDefinition& scenario, std::string_view option_id) {
 const auto option = std::find_if(scenario.plans.begin(), scenario.plans.end(),
 [&](const auto& candidate) { return candidate.id == option_id; });
 if (option == scenario.plans.end())
 throw std::invalid_argument("unknown construction plan option '" + std::string(option_id) + "'");
 ConstructionPlan result;
 result.scenario = scenario.id;
 result.placements.reserve(option->placements.size());
 for (const auto& placement : option->placements) {
 result.placements.push_back({placement.instance_id, placement.piece_id,
 placement.start_socket, placement.end_socket});
 }
 return result;
}

ConstructionWorldCommit prepare_construction_world_commit(
 const ConstructionScenarioDefinition& scenario,
 const std::unordered_map<std::string, ConstructionPieceDefinition>& catalog,
 const ConstructionPlan& plan, const sim::runtime::MapSnapshot& base_map,
 sim::runtime::TimePoint sample_time) {
 ConstructionWorldCommit commit;
 commit.resolution = resolve_construction_plan(scenario, catalog, plan);
 if (!commit.resolution.ok || !base_map.grid)
 return commit;

 auto map = base_map;
 map.revision = base_map.revision + 1;
 map.sample_time = sample_time;
 map.provenance.source_kind = "construction-plan";
 map.provenance.producer_id = "lost-signal.construction";
 map.provenance.source_revision = commit.resolution.plan_sha256;
 auto& grid = *map.grid;

 for (const auto& resolved : commit.resolution.pieces) {
 const auto& piece = catalog.at(resolved.placement.piece_id);
 auto material = std::find(grid.surface_classes.begin(), grid.surface_classes.end(),
 piece.surface_material);
 if (material == grid.surface_classes.end()) {
 grid.surface_classes.push_back(piece.surface_material);
 material = std::prev(grid.surface_classes.end());
 if (!grid.surface_class_properties.empty())
 grid.surface_class_properties.push_back({});
 }
 const auto material_index = static_cast<std::uint16_t>(
 std::distance(grid.surface_classes.begin(), material));
 const double cosine = std::cos(resolved.yaw_rad);
 const double sine = std::sin(resolved.yaw_rad);
 const double horizontal_half_length = piece.size_m.x * 0.5 * std::cos(resolved.pitch_rad);
 const double half_width = piece.size_m.y * 0.5;
 for (std::uint32_t y = 0; y < grid.height; ++y) {
 for (std::uint32_t x = 0; x < grid.width; ++x) {
 const auto center = sim::runtime::grid_cell_center(grid, {x, y});
 const double dx = center.x - resolved.center_m.x;
 const double dy = center.y - resolved.center_m.y;
 const double local_x = cosine * dx + sine * dy;
 const double local_y = -sine * dx + cosine * dy;
 if (std::abs(local_x) > horizontal_half_length || std::abs(local_y) > half_width)
 continue;
 const auto index = static_cast<std::size_t>(y) * grid.width + x;
 grid.elevation_m[index] = static_cast<float>(
 resolved.center_m.z + std::tan(resolved.pitch_rad) * local_x +
 piece.size_m.z * 0.5 * std::cos(resolved.pitch_rad));
 grid.surface_class[index] = material_index;
 if (grid.roughness_m.size() ==
 static_cast<std::size_t>(grid.width) * grid.height) {
 grid.roughness_m[index] = std::numeric_limits<float>::quiet_NaN();
 if (grid.roughness_confidence.size() == grid.roughness_m.size())
 grid.roughness_confidence[index] = 0.0F;
 }
 // A bridge deck is support terrain. Occupancy describes blocking volume
 // in the navigation projection, so committing one must not create a wall.
 grid.occupancy[index] = 0;
 }
 }

 sim::runtime::SemanticFeature feature;
 feature.id = resolved.placement.instance_id;
 feature.kind = "constructed_surface";
 feature.geometry = sim::runtime::SemanticGeometry::Polygon;
 const sim::Vec2 local_corners[] = {
 {-horizontal_half_length, -half_width},
 {horizontal_half_length, -half_width},
 {horizontal_half_length, half_width},
 {-horizontal_half_length, half_width},
 };
 for (const auto corner : local_corners) {
 feature.vertices_m.push_back({
 resolved.center_m.x + cosine * corner.x - sine * corner.y,
 resolved.center_m.y + sine * corner.x + cosine * corner.y,
 });
 }
 feature.attributes.emplace("piece_id", resolved.placement.piece_id);
 feature.attributes.emplace("plan_sha256", commit.resolution.plan_sha256);
 feature.attributes.emplace("surface_material", piece.surface_material);
 map.features.push_back(std::move(feature));
 }

 std::string reason;
 if (!sim::runtime::valid_map_snapshot(map, &reason)) {
 commit.resolution.ok = false;
 commit.resolution.reason = "constructed map is invalid: " + reason;
 commit.resolution.pieces.clear();
 return commit;
 }
 commit.map = std::move(map);
 return commit;
}

} // namespace mission
