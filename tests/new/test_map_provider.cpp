#include "robot/runtime/map_provider.hpp"

#include <cmath>
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

sim::runtime::MapSnapshot map_snapshot() {
 sim::runtime::MapSnapshot map;
 map.map_id = "lab/floor-1";
 map.revision = 7;
 map.frame_id = "map";
 map.sample_time = sim::runtime::TimePoint{100};
 map.provenance = {"slam", "test.mapper", "session-17"};
 map.grid.emplace();
 map.grid->width = 3;
 map.grid->height = 2;
 map.grid->resolution_m = 0.5;
 map.grid->origin_m = {10.0, -2.0};
 map.grid->origin_yaw_rad = sim::kPi * 0.5;
 map.grid->occupancy = {0, 25, 100, -1, 0, 0};
 map.grid->elevation_m = {0.0F, 0.1F, 0.2F,
 std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F};
 map.grid->elevation_sigma_m = {0.01F, 0.02F, 0.03F,
 std::numeric_limits<float>::quiet_NaN(), 0.01F, 0.01F};
 map.grid->surface_classes = {"unknown", "paved"};
 map.grid->surface_class = {1, 1, 1, 0, 1, 1};
 map.grid->roughness_m = {0.01F, 0.02F, 0.03F,
 std::numeric_limits<float>::quiet_NaN(), 0.01F, 0.01F};
 map.grid->roughness_confidence = {0.8F, 0.7F, 0.6F,
 std::numeric_limits<float>::quiet_NaN(), 0.9F, 0.5F};
 map.places.emplace("dock", sim::runtime::SemanticPlace{{9.5, -1.5}, 1.2});
 map.features.push_back({
 "keepout-1", "keepout", sim::runtime::SemanticGeometry::Polygon,
 {{9.0, -2.0}, {9.0, -1.0}, {8.0, -1.0}}, {{"authority", "operator"}},
 });
 map.relations.push_back({
 "dock-to-keepout", "near", "dock", "keepout-1", false, {},
 });
 return map;
}

} // namespace

int main() {
 auto map = map_snapshot();
 std::string reason;
 CHECK(sim::runtime::valid_map_snapshot(map, &reason));
 CHECK(reason.empty());
 auto planar = map;
 planar.grid->elevation_m.clear();
 planar.grid->elevation_sigma_m.clear();
 planar.grid->surface_class.clear();
 planar.grid->surface_classes.clear();
 CHECK(sim::runtime::valid_map_snapshot(planar, &reason));
 auto vector_only = planar;
 vector_only.grid.reset();
 CHECK(sim::runtime::valid_map_snapshot(vector_only, &reason));

 // A 90-degree map origin still round-trips through the documented
 // cell-centre transform; image row conventions never enter this contract.
 const auto world = sim::runtime::grid_cell_center(*map.grid, {2, 1});
 const auto cell = sim::runtime::world_to_grid(*map.grid, world);
 CHECK(cell && cell->x == 2 && cell->y == 1);
 CHECK(!sim::runtime::world_to_grid(*map.grid, {100.0, 100.0}));

 sim::runtime::MapManager manager;
 CHECK(manager.configure(map, &reason));
 CHECK(manager.state() == sim::runtime::LifecycleState::Inactive);
 CHECK(!manager.resolve_place("dock"));
 CHECK(manager.activate());
 const auto dock = manager.resolve_place("dock");
 CHECK(dock && std::abs(dock->heading_rad - 1.2) < 1e-12);

 // A provider can supply a bounded planning view without requiring consumers
 // to allocate or understand the complete global raster. Cropping preserves
 // the source revision and rotated-grid coordinate contract.
 const auto focus = sim::runtime::grid_cell_center(*map.grid, {2, 1});
 const auto view = manager.query_region(
 {{focus.x - 0.1, focus.y - 0.1}, {focus.x + 0.1, focus.y + 0.1}});
 CHECK(view);
 CHECK(view->map_id == map.map_id);
 CHECK(view->revision == map.revision);
 CHECK(view->grid);
 CHECK(view->grid->width < map.grid->width || view->grid->height < map.grid->height);
 CHECK(view->grid->occupancy.size() ==
 static_cast<std::size_t>(view->grid->width) * view->grid->height);
 const auto cropped_world = sim::runtime::grid_cell_center(*view->grid, {0, 0});
 CHECK(sim::runtime::world_to_grid(*map.grid, cropped_world).has_value());
 const auto semantic_view = manager.query_region({{7.9, -2.1}, {10.1, -0.9}});
 CHECK(semantic_view && semantic_view->places.count("dock") == 1);
 CHECK(semantic_view->features.size() == 1);
 CHECK(semantic_view->relations.size() == 1);
 CHECK(!manager.query_region({{1.0, 1.0}, {1.0, 2.0}}));

 auto stale = map;
 stale.sample_time = sim::runtime::TimePoint{101};
 CHECK(manager.replace(stale) == sim::runtime::MapUpdateResult::StaleRevision);
 auto next = map;
 next.revision = 8;
 next.sample_time = sim::runtime::TimePoint{99};
 CHECK(manager.replace(next) == sim::runtime::MapUpdateResult::StaleTimestamp);
 next.sample_time = sim::runtime::TimePoint{101};
 next.grid->occupancy[0] = 100;
 CHECK(manager.replace(next) == sim::runtime::MapUpdateResult::Accepted);
 CHECK(manager.current_snapshot()->revision == 8);
 CHECK(manager.current_snapshot()->grid->occupancy[0] == 100);

 // Vector/topology-only providers use the same bounded query contract. The
 // absence of a raster is information, not a provider error.
 sim::runtime::MapManager vector_manager;
 CHECK(vector_manager.configure(vector_only));
 CHECK(vector_manager.activate());
 const auto vector_view = vector_manager.query_region({{8.0, -2.5}, {10.5, -0.5}});
 CHECK(vector_view && !vector_view->grid);
 CHECK(vector_view->places.size() == 1);
 CHECK(vector_view->features.size() == 1);

 auto invalid = map_snapshot();
 invalid.revision = 9;
 invalid.grid->occupancy[0] = 101;
 CHECK(!sim::runtime::valid_map_snapshot(invalid, &reason));
 CHECK(!reason.empty());
 auto unqualified_prior = map_snapshot();
 unqualified_prior.grid->roughness_confidence.clear();
 CHECK(!sim::runtime::valid_map_snapshot(unqualified_prior, &reason));
 auto invalid_confidence = map_snapshot();
 invalid_confidence.grid->roughness_confidence[0] = 1.1F;
 CHECK(!sim::runtime::valid_map_snapshot(invalid_confidence, &reason));
 return 0;
}
