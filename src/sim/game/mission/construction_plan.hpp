#pragma once

#include "content.hpp"
#include "robot/runtime/map_provider.hpp"

#include <optional>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace mission {

struct ConstructionPlacement {
 std::string instance_id;
 std::string piece_id;
 std::string start_socket;
 std::string end_socket;
};

struct ConstructionPlan {
 int schema_version{1};
 std::string scenario;
 std::vector<ConstructionPlacement> placements;
};

struct ResolvedConstructionPiece {
 ConstructionPlacement placement;
 Vec3 center_m;
 // The box's local +X axis runs from start_socket to end_socket.
 double yaw_rad{0.0};
 double pitch_rad{0.0};
};

struct ConstructionPlanResolution {
 bool ok{false};
 std::string reason;
 std::string canonical_plan;
 std::string plan_sha256;
 std::vector<ResolvedConstructionPiece> pieces;
};

struct ConstructionWorldCommit {
 ConstructionPlanResolution resolution;
 std::optional<sim::runtime::MapSnapshot> map;
};

// The canonical representation is semantic: placement array order does not
// affect it, while directional socket order does. Only validated plans may be
// published to physics or the map manager.
std::string serialize_construction_plan_canonical(const ConstructionPlan& plan);
std::string construction_plan_sha256(const ConstructionPlan& plan);

ConstructionPlanResolution resolve_construction_plan(
 const ConstructionScenarioDefinition& scenario,
 const std::unordered_map<std::string, ConstructionPieceDefinition>& catalog,
 const ConstructionPlan& plan);

ConstructionPlan construction_plan_from_option(
 const ConstructionScenarioDefinition& scenario, std::string_view option_id);

// Prepare the complete immutable map side of a construction transaction.
// Driveable pieces update elevation and surface class while remaining free in
// occupancy. Callers publish this snapshot and matching physics bodies at one
// simulation boundary; rejected plans return no partial map.
ConstructionWorldCommit prepare_construction_world_commit(
 const ConstructionScenarioDefinition& scenario,
 const std::unordered_map<std::string, ConstructionPieceDefinition>& catalog,
 const ConstructionPlan& plan, const sim::runtime::MapSnapshot& base_map,
 sim::runtime::TimePoint sample_time);

} // namespace mission
