#pragma once

#include "robot/core/math.hpp"
#include "robot/runtime/runtime_channel.hpp"

#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <utility>
#include <string>
#include <string_view>
#include <vector>

namespace sim::runtime {

// Transport-neutral robot map. The same value may come from an authored-map
// export, an offline mapping pipeline, SLAM, replay, or a remote map service.
// It deliberately contains no actors, physics truth, rendering, or story data.
struct SurfaceClassProperties {
 // Static priors published with the map revision. They describe the nominal
 // surface known to the robot, not the live contact coefficient in a physics
 // engine. Online traction beliefs remain a ROBOT estimate.
 double nominal_friction_mu{0.8};
 double rolling_resistance{0.0};

 bool operator==(const SurfaceClassProperties&) const = default;
};

struct MapGrid {
 std::uint32_t width{0};
 std::uint32_t height{0};
 double resolution_m{0.0};
 Vec2 origin_m;
 double origin_yaw_rad{0.0};
 // Optional compact/global raster for maps where one bounded array is useful.
 // Row-major values use the conventional -1 unknown, 0 free, 100 occupied.
 std::vector<std::int8_t> occupancy;
 // Optional ground elevation at each cell centre in map frame. NaN means
 // unknown. A planar mapper leaves this empty rather than inventing zeros.
 std::vector<float> elevation_m;
 // Optional one-sigma elevation uncertainty. NaN means unknown.
 std::vector<float> elevation_sigma_m;
 // Optional index into surface_classes. Index zero is provider-defined.
 std::vector<std::uint16_t> surface_class;
 std::vector<std::string> surface_classes;
 // Optional per-class navigation priors aligned with surface_classes.
 std::vector<SurfaceClassProperties> surface_class_properties;
 // Optional surveyed ground-relief amplitude prior. NaN means unknown. This
 // value is static map knowledge, never the physics provider's live contact
 // response. A populated layer requires roughness_confidence.
 std::vector<float> roughness_m;
 // Per-cell confidence in [0, 1] for roughness_m. ROBOT fuses this prior with
 // its own motion observations; zero means the cell has no usable prior.
 std::vector<float> roughness_confidence;
};

struct GridIndex {
 std::uint32_t x{0};
 std::uint32_t y{0};
};

struct MapBounds2D {
 Vec2 min_m;
 Vec2 max_m;
};

bool valid_map_bounds(MapBounds2D bounds);

std::optional<GridIndex> world_to_grid(const MapGrid& grid, Vec2 world_m);
Vec2 grid_cell_center(const MapGrid& grid, GridIndex cell);

struct SemanticPlace {
 Vec2 position_m;
 double heading_rad{0.0};
};

enum class SemanticGeometry { Point, Polyline, Polygon };

struct SemanticFeature {
 std::string id;
 std::string kind;
 SemanticGeometry geometry{SemanticGeometry::Point};
 std::vector<Vec2> vertices_m;
 std::map<std::string, std::string> attributes;
};

struct SemanticRelation {
 std::string id;
 std::string kind;
 std::string from_id;
 std::string to_id;
 bool directed{true};
 std::map<std::string, std::string> attributes;
};

struct MapProvenance {
 // authored, slam, survey, hd_map, imported, or another documented producer kind.
 std::string source_kind;
 std::string producer_id;
 std::string source_revision;
};

struct MapSnapshot {
 std::uint32_t schema_version{2};
 std::string map_id;
 std::uint64_t revision{0};
 std::string frame_id{"map"};
 TimePoint sample_time;
 MapProvenance provenance;
 // Optional for vector/topological maps. This compact-raster representation
 // is not a requirement that a large provider allocate a monolithic grid.
 // Raster consumers request or derive a bounded costmap around their region
 // of interest when the global authority has no complete grid.
 std::optional<MapGrid> grid;
 std::map<std::string, SemanticPlace> places;
 std::vector<SemanticFeature> features;
 std::vector<SemanticRelation> relations;
};

// Bounded, immutable view used by local mapping and raster planning. A view
// may be empty when the queried region contains no known static content. It
// always identifies the exact global map revision from which it was derived.
struct MapView {
 std::string map_id;
 std::uint64_t revision{0};
 std::string frame_id{"map"};
 TimePoint sample_time;
 MapBounds2D bounds;
 std::optional<MapGrid> grid;
 std::map<std::string, SemanticPlace> places;
 std::vector<SemanticFeature> features;
 std::vector<SemanticRelation> relations;
};

std::optional<MapView> make_map_view(const MapSnapshot& snapshot, MapBounds2D bounds);

bool valid_map_snapshot(const MapSnapshot& snapshot, std::string* reason = nullptr);

enum class MapUpdateResult {
 Accepted,
 Invalid,
 Inactive,
 IdentityChanged,
 StaleRevision,
 StaleTimestamp,
};

// Read-only consumer boundary used by task and autonomy code. Providers own
// loading, SLAM, storage, transport, and revision publication.
class IMapProvider {
 public:
 virtual ~IMapProvider() = default;
 virtual LifecycleState state() const = 0;
 virtual std::shared_ptr<const MapSnapshot> current_snapshot() const = 0;
 virtual std::optional<SemanticPlace> resolve_place(std::string_view id) const = 0;
 // The provider decides whether this is a crop of an in-memory compact map,
 // a tile fetch, or a remote spatial query. Consumers never require a global
 // raster to allocate a local planning window.
 virtual std::optional<MapView> query_region(MapBounds2D bounds) const = 0;
};

// Minimal in-process provider and lifecycle owner. Multi-process or SLAM
// providers implement IMapProvider and preserve these validation rules.
class MapManager final : public IMapProvider {
 public:
 bool configure(MapSnapshot snapshot, std::string* reason = nullptr);
 bool activate();
 bool deactivate();
 void cleanup();
 MapUpdateResult replace(MapSnapshot snapshot, std::string* reason = nullptr);

 LifecycleState state() const override { return state_; }
 std::shared_ptr<const MapSnapshot> current_snapshot() const override { return snapshot_; }
 std::optional<SemanticPlace> resolve_place(std::string_view id) const override;
 std::optional<MapView> query_region(MapBounds2D bounds) const override;

 private:
 LifecycleState state_{LifecycleState::Unconfigured};
 std::shared_ptr<const MapSnapshot> snapshot_;
};

// A deployment may keep several immutable map products available. The
// catalog owns no active robot state; it validates and indexes packages, then
// atomically selects one into a MapManager when the application has proved a
// safe switching point. Revision updates within the selected map still use
// MapManager::replace().
class MapCatalog {
 public:
 bool register_map(MapSnapshot snapshot, std::string* reason = nullptr);
 bool contains(std::string_view map_id) const;
 std::vector<std::string> map_ids() const;
 std::shared_ptr<const MapSnapshot> snapshot(std::string_view map_id) const;
 bool select(MapManager& active_map, std::string_view map_id,
 std::string* reason = nullptr) const;

 private:
 std::map<std::string, std::shared_ptr<const MapSnapshot>> maps_;
};

} // namespace sim::runtime
