#include "sim/game/mission/construction_plan.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <unordered_map>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

namespace {

mission::ConstructionPieceDefinition deck() {
 mission::ConstructionPieceDefinition piece;
 piece.id = "construction.deck_short";
 piece.size_m = {3.0, 1.2, 0.16};
 piece.required_endpoint_contacts = 2;
 piece.max_span_m = 3.0;
 piece.allowed_socket_types = {"bank_anchor", "support_top"};
 return piece;
}

mission::ConstructionScenarioDefinition scenario() {
 mission::ConstructionScenarioDefinition result;
 result.id = "washout";
 result.snap_tolerance_m = 0.01;
 result.inventory.emplace("construction.deck_short", 2);
 result.sockets = {
 {"a", "bank_anchor", {0.0, 0.0, 0.0}},
 {"b", "support_top", {3.0, 0.0, 0.0}},
 {"c", "bank_anchor", {0.0, 2.0, 0.0}},
 {"d", "support_top", {3.0, 2.0, 0.0}},
 };
 return result;
}

} // namespace

int main() {
 std::unordered_map<std::string, mission::ConstructionPieceDefinition> catalog;
 catalog.emplace("construction.deck_short", deck());
 const auto fixture = scenario();
 mission::ConstructionPlan one{
 1, "washout", {{"bridge-1", "construction.deck_short", "a", "b"}},
 };
 const std::string expected =
 R"({"placements":[{"end_socket":"b","instance_id":"bridge-1","piece_id":"construction.deck_short","start_socket":"a"}],"scenario":"washout","schema_version":1})";
 CHECK(mission::serialize_construction_plan_canonical(one) == expected);
 CHECK(mission::construction_plan_sha256(one) ==
 "eec6ce2ea3e3d15df5a2cbacea19d45e49547aa19a867fe9dc256ef08ee697b8");

 auto result = mission::resolve_construction_plan(fixture, catalog, one);
 CHECK(result.ok);
 CHECK(result.pieces.size() == 1);
 CHECK(std::abs(result.pieces.front().center_m.x - 1.5) < 1e-12);
 CHECK(std::abs(result.pieces.front().yaw_rad) < 1e-12);
 CHECK(std::abs(result.pieces.front().pitch_rad) < 1e-12);

 mission::ConstructionPlan ordered{
 1, "washout",
 {{"bridge-1", "construction.deck_short", "a", "b"},
 {"bridge-2", "construction.deck_short", "c", "d"}},
 };
 auto reversed = ordered;
 std::reverse(reversed.placements.begin(), reversed.placements.end());
 CHECK(mission::construction_plan_sha256(ordered) ==
 mission::construction_plan_sha256(reversed));
 const auto ordered_result = mission::resolve_construction_plan(fixture, catalog, ordered);
 const auto reversed_result = mission::resolve_construction_plan(fixture, catalog, reversed);
 CHECK(ordered_result.ok && reversed_result.ok);
 CHECK(ordered_result.pieces.size() == reversed_result.pieces.size());
 for (std::size_t index = 0; index < ordered_result.pieces.size(); ++index) {
 CHECK(ordered_result.pieces[index].placement.instance_id ==
 reversed_result.pieces[index].placement.instance_id);
 CHECK(ordered_result.pieces[index].center_m.x == reversed_result.pieces[index].center_m.x);
 CHECK(ordered_result.pieces[index].center_m.y == reversed_result.pieces[index].center_m.y);
 }

 auto invalid = ordered;
 invalid.placements.push_back(
 {"bridge-3", "construction.deck_short", "a", "d"});
 CHECK(!mission::resolve_construction_plan(fixture, catalog, invalid).ok);
 CHECK(mission::resolve_construction_plan(fixture, catalog, invalid).pieces.empty());

 invalid = one;
 invalid.placements.push_back(
 {"bridge-2", "construction.deck_short", "a", "d"});
 CHECK(!mission::resolve_construction_plan(fixture, catalog, invalid).ok);

 auto incompatible_fixture = fixture;
 incompatible_fixture.sockets[1].type = "ground_anchor";
 CHECK(!mission::resolve_construction_plan(incompatible_fixture, catalog, one).ok);

 auto wrong_length_fixture = fixture;
 wrong_length_fixture.sockets[1].position_m.x = 2.5;
 CHECK(!mission::resolve_construction_plan(wrong_length_fixture, catalog, one).ok);

 sim::runtime::MapSnapshot base_map;
 base_map.map_id = "washout-map";
 base_map.revision = 7;
 base_map.sample_time = sim::runtime::TimePoint::from_seconds(1.0);
 base_map.provenance = {"authored", "fixture", "base"};
 auto& grid = base_map.grid.emplace();
 grid.width = 8;
 grid.height = 4;
 grid.resolution_m = 0.5;
 const std::size_t cell_count = grid.width * grid.height;
 grid.occupancy.assign(cell_count, 100);
 grid.elevation_m.assign(cell_count, 0.0F);
 grid.surface_class.assign(cell_count, 0);
 grid.surface_classes = {"default"};
 catalog.at("construction.deck_short").surface_material = "steel_deck";
 const auto commit = mission::prepare_construction_world_commit(
 fixture, catalog, one, base_map, sim::runtime::TimePoint::from_seconds(2.0));
 CHECK(commit.resolution.ok);
 CHECK(commit.map.has_value());
 CHECK(commit.map->revision == 8);
 CHECK(commit.map->sample_time == sim::runtime::TimePoint::from_seconds(2.0));
 CHECK(commit.map->provenance.source_revision == commit.resolution.plan_sha256);
 CHECK(commit.map->features.size() == 1);
 CHECK(commit.map->features.front().id == "bridge-1");
 const auto bridge_cell = sim::runtime::world_to_grid(*commit.map->grid, {1.5, 0.25});
 CHECK(bridge_cell.has_value());
 const auto bridge_index = static_cast<std::size_t>(bridge_cell->y) * grid.width + bridge_cell->x;
 CHECK(commit.map->grid->occupancy[bridge_index] == 0);
 CHECK(std::abs(commit.map->grid->elevation_m[bridge_index] - 0.08F) < 1e-6F);
 CHECK(commit.map->grid->surface_classes[commit.map->grid->surface_class[bridge_index]] ==
 "steel_deck");
 sim::runtime::MapManager manager;
 CHECK(manager.configure(base_map));
 CHECK(manager.activate());
 CHECK(manager.replace(*commit.map) == sim::runtime::MapUpdateResult::Accepted);
 CHECK(manager.current_snapshot()->revision == 8);

 const auto rejected_commit = mission::prepare_construction_world_commit(
 fixture, catalog, invalid, base_map, sim::runtime::TimePoint::from_seconds(2.0));
 CHECK(!rejected_commit.resolution.ok);
 CHECK(!rejected_commit.map.has_value());
 return 0;
}
