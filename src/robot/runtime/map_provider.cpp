#include "robot/runtime/map_provider.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <utility>

namespace sim::runtime {
namespace {

bool reject(std::string* reason, std::string message) {
 if (reason)
 *reason = std::move(message);
 return false;
}

bool finite(Vec2 value) {
 return std::isfinite(value.x) && std::isfinite(value.y);
}

bool inside(MapBounds2D bounds, Vec2 point) {
 return point.x >= bounds.min_m.x && point.x <= bounds.max_m.x &&
 point.y >= bounds.min_m.y && point.y <= bounds.max_m.y;
}

bool feature_intersects(MapBounds2D bounds, const SemanticFeature& feature) {
 double min_x = feature.vertices_m.front().x;
 double max_x = min_x;
 double min_y = feature.vertices_m.front().y;
 double max_y = min_y;
 for (const auto vertex : feature.vertices_m) {
 min_x = std::min(min_x, vertex.x);
 max_x = std::max(max_x, vertex.x);
 min_y = std::min(min_y, vertex.y);
 max_y = std::max(max_y, vertex.y);
 }
 return max_x >= bounds.min_m.x && min_x <= bounds.max_m.x &&
 max_y >= bounds.min_m.y && min_y <= bounds.max_m.y;
}

template <typename T>
void crop_layer(const std::vector<T>& source, std::vector<T>& target,
 const MapGrid& grid, std::uint32_t x0, std::uint32_t y0,
 std::uint32_t width, std::uint32_t height) {
 if (source.empty())
 return;
 target.reserve(static_cast<std::size_t>(width) * height);
 for (std::uint32_t y = 0; y < height; ++y) {
 const auto begin = source.begin() + static_cast<std::ptrdiff_t>(y0 + y) * grid.width + x0;
 target.insert(target.end(), begin, begin + width);
 }
}

std::optional<MapGrid> crop_grid(const MapGrid& grid, MapBounds2D bounds) {
 const double cosine = std::cos(grid.origin_yaw_rad);
 const double sine = std::sin(grid.origin_yaw_rad);
 double min_local_x = std::numeric_limits<double>::infinity();
 double min_local_y = std::numeric_limits<double>::infinity();
 double max_local_x = -std::numeric_limits<double>::infinity();
 double max_local_y = -std::numeric_limits<double>::infinity();
 for (const Vec2 corner : {Vec2{bounds.min_m.x, bounds.min_m.y},
 Vec2{bounds.min_m.x, bounds.max_m.y}, Vec2{bounds.max_m.x, bounds.min_m.y},
 Vec2{bounds.max_m.x, bounds.max_m.y}}) {
 const double dx = corner.x - grid.origin_m.x;
 const double dy = corner.y - grid.origin_m.y;
 const double local_x = cosine * dx + sine * dy;
 const double local_y = -sine * dx + cosine * dy;
 min_local_x = std::min(min_local_x, local_x);
 min_local_y = std::min(min_local_y, local_y);
 max_local_x = std::max(max_local_x, local_x);
 max_local_y = std::max(max_local_y, local_y);
 }
 const auto clamp_index = [](double value, double resolution, std::uint32_t limit) {
 return static_cast<std::uint32_t>(std::clamp(
 std::floor(value / resolution), 0.0, static_cast<double>(limit)));
 };
 const auto clamp_end = [](double value, double resolution, std::uint32_t limit) {
 return static_cast<std::uint32_t>(std::clamp(
 std::ceil(value / resolution), 0.0, static_cast<double>(limit)));
 };
 const std::uint32_t x0 = clamp_index(min_local_x, grid.resolution_m, grid.width);
 const std::uint32_t y0 = clamp_index(min_local_y, grid.resolution_m, grid.height);
 const std::uint32_t x1 = clamp_end(max_local_x, grid.resolution_m, grid.width);
 const std::uint32_t y1 = clamp_end(max_local_y, grid.resolution_m, grid.height);
 if (x0 >= x1 || y0 >= y1)
 return std::nullopt;

 MapGrid cropped;
 cropped.width = x1 - x0;
 cropped.height = y1 - y0;
 cropped.resolution_m = grid.resolution_m;
 cropped.origin_yaw_rad = grid.origin_yaw_rad;
 const double local_x = static_cast<double>(x0) * grid.resolution_m;
 const double local_y = static_cast<double>(y0) * grid.resolution_m;
 cropped.origin_m = {
 grid.origin_m.x + cosine * local_x - sine * local_y,
 grid.origin_m.y + sine * local_x + cosine * local_y,
 };
 crop_layer(grid.occupancy, cropped.occupancy, grid, x0, y0, cropped.width, cropped.height);
 crop_layer(grid.elevation_m, cropped.elevation_m, grid, x0, y0, cropped.width, cropped.height);
 crop_layer(grid.elevation_sigma_m, cropped.elevation_sigma_m,
 grid, x0, y0, cropped.width, cropped.height);
 crop_layer(grid.surface_class, cropped.surface_class, grid,
 x0, y0, cropped.width, cropped.height);
 crop_layer(grid.roughness_m, cropped.roughness_m, grid,
 x0, y0, cropped.width, cropped.height);
 crop_layer(grid.roughness_confidence, cropped.roughness_confidence, grid,
 x0, y0, cropped.width, cropped.height);
 cropped.surface_classes = grid.surface_classes;
 cropped.surface_class_properties = grid.surface_class_properties;
 return cropped;
}

bool valid_optional_float_layer(const std::vector<float>& layer, std::size_t count,
 bool non_negative) {
 if (!layer.empty() && layer.size() != count)
 return false;
 return std::all_of(layer.begin(), layer.end(), [&](float value) {
 return std::isnan(value) || (std::isfinite(value) && (!non_negative || value >= 0.0F));
 });
}

} // namespace

bool valid_map_bounds(MapBounds2D bounds) {
 return finite(bounds.min_m) && finite(bounds.max_m) &&
 bounds.min_m.x < bounds.max_m.x && bounds.min_m.y < bounds.max_m.y;
}

std::optional<GridIndex> world_to_grid(const MapGrid& grid, Vec2 world_m) {
 if (grid.width == 0 || grid.height == 0 || !(grid.resolution_m > 0.0) ||
 !finite(world_m) || !finite(grid.origin_m) || !std::isfinite(grid.origin_yaw_rad)) {
 return std::nullopt;
 }
 const double dx = world_m.x - grid.origin_m.x;
 const double dy = world_m.y - grid.origin_m.y;
 const double cosine = std::cos(grid.origin_yaw_rad);
 const double sine = std::sin(grid.origin_yaw_rad);
 const double local_x = cosine * dx + sine * dy;
 const double local_y = -sine * dx + cosine * dy;
 const auto x = static_cast<std::int64_t>(std::floor(local_x / grid.resolution_m));
 const auto y = static_cast<std::int64_t>(std::floor(local_y / grid.resolution_m));
 if (x < 0 || y < 0 || x >= grid.width || y >= grid.height)
 return std::nullopt;
 return GridIndex{static_cast<std::uint32_t>(x), static_cast<std::uint32_t>(y)};
}

Vec2 grid_cell_center(const MapGrid& grid, GridIndex cell) {
 const double local_x = (static_cast<double>(cell.x) + 0.5) * grid.resolution_m;
 const double local_y = (static_cast<double>(cell.y) + 0.5) * grid.resolution_m;
 const double cosine = std::cos(grid.origin_yaw_rad);
 const double sine = std::sin(grid.origin_yaw_rad);
 return {
 grid.origin_m.x + cosine * local_x - sine * local_y,
 grid.origin_m.y + sine * local_x + cosine * local_y,
 };
}

bool valid_map_snapshot(const MapSnapshot& snapshot, std::string* reason) {
 if (snapshot.schema_version != 2)
 return reject(reason, "unsupported robot-map schema version");
 if (snapshot.map_id.empty() || snapshot.frame_id.empty() || snapshot.revision == 0)
 return reject(reason, "map identity, frame, and positive revision are required");
 if (!snapshot.grid && snapshot.places.empty() && snapshot.features.empty())
 return reject(reason, "map must contain a grid or semantic content");
 if (snapshot.grid) {
 const auto& grid = *snapshot.grid;
 if (grid.width == 0 || grid.height == 0 || !(grid.resolution_m > 0.0) ||
 !std::isfinite(grid.resolution_m) || !finite(grid.origin_m) ||
 !std::isfinite(grid.origin_yaw_rad)) {
 return reject(reason, "invalid map grid geometry");
 }
 const std::uint64_t count64 = static_cast<std::uint64_t>(grid.width) * grid.height;
 if (count64 > std::numeric_limits<std::size_t>::max())
 return reject(reason, "map grid is too large");
 const auto count = static_cast<std::size_t>(count64);
 if (grid.occupancy.size() != count ||
 !valid_optional_float_layer(grid.elevation_m, count, false) ||
 !valid_optional_float_layer(grid.elevation_sigma_m, count, true) ||
 !valid_optional_float_layer(grid.roughness_m, count, true) ||
 !valid_optional_float_layer(grid.roughness_confidence, count, true) ||
 (!grid.surface_class.empty() && grid.surface_class.size() != count)) {
 return reject(reason, "map layers do not match grid geometry or value contract");
 }
 for (const auto value : grid.occupancy) {
 if (value < -1 || value > 100)
 return reject(reason, "occupancy must be -1 or within [0,100]");
 }
 if (!grid.elevation_sigma_m.empty() && grid.elevation_m.empty())
 return reject(reason, "elevation uncertainty requires an elevation layer");
 if (!grid.roughness_confidence.empty() && grid.roughness_m.empty())
 return reject(reason, "roughness confidence requires a roughness prior");
 if (!grid.roughness_m.empty() && grid.roughness_confidence.empty())
 return reject(reason, "roughness prior requires a confidence layer");
 for (const auto confidence : grid.roughness_confidence) {
 if (!std::isnan(confidence) && confidence > 1.0F)
 return reject(reason, "roughness confidence must be within [0, 1]");
 }
 if (!grid.surface_class.empty() && grid.surface_classes.empty())
 return reject(reason, "surface indices require a class table");
 if (!grid.surface_class_properties.empty() &&
 grid.surface_class_properties.size() != grid.surface_classes.size())
 return reject(reason, "surface properties do not match the class table");
 for (const auto& properties : grid.surface_class_properties) {
 if (!std::isfinite(properties.nominal_friction_mu) ||
 properties.nominal_friction_mu < 0.0 ||
 !std::isfinite(properties.rolling_resistance) ||
 properties.rolling_resistance < 0.0)
 return reject(reason, "surface properties are invalid");
 }
 for (const auto index : grid.surface_class) {
 if (index >= grid.surface_classes.size())
 return reject(reason, "surface class index is out of range");
 }
 std::set<std::string> classes;
 for (const auto& id : grid.surface_classes) {
 if (id.empty() || !classes.insert(id).second)
 return reject(reason, "surface class ids must be non-empty and unique");
 }
 }
 std::set<std::string> feature_ids;
 for (const auto& [id, place] : snapshot.places) {
 if (id.empty() || !finite(place.position_m) || !std::isfinite(place.heading_rad))
 return reject(reason, "semantic place is invalid");
 feature_ids.insert(id);
 }
 for (const auto& feature : snapshot.features) {
 const std::size_t minimum_vertices = feature.geometry == SemanticGeometry::Point ? 1U :
 (feature.geometry == SemanticGeometry::Polyline ? 2U : 3U);
 if (feature.id.empty() || feature.kind.empty() || !feature_ids.insert(feature.id).second ||
 feature.vertices_m.size() < minimum_vertices ||
 !std::all_of(feature.vertices_m.begin(), feature.vertices_m.end(), finite)) {
 return reject(reason, "semantic feature is invalid");
 }
 }
 std::set<std::string> relation_ids;
 for (const auto& relation : snapshot.relations) {
 if (relation.id.empty() || relation.kind.empty() || relation.from_id.empty() ||
 relation.to_id.empty() || !relation_ids.insert(relation.id).second ||
 !feature_ids.count(relation.from_id) || !feature_ids.count(relation.to_id)) {
 return reject(reason, "semantic relation is invalid or references an unknown feature");
 }
 }
 if (reason)
 reason->clear();
 return true;
}

std::optional<MapView> make_map_view(const MapSnapshot& snapshot, MapBounds2D bounds) {
 if (!valid_map_bounds(bounds) || !valid_map_snapshot(snapshot))
 return std::nullopt;
 MapView view;
 view.map_id = snapshot.map_id;
 view.revision = snapshot.revision;
 view.frame_id = snapshot.frame_id;
 view.sample_time = snapshot.sample_time;
 view.bounds = bounds;
 if (snapshot.grid)
 view.grid = crop_grid(*snapshot.grid, bounds);

 std::set<std::string> included_ids;
 for (const auto& [id, place] : snapshot.places) {
 if (inside(bounds, place.position_m)) {
 view.places.emplace(id, place);
 included_ids.insert(id);
 }
 }
 for (const auto& feature : snapshot.features) {
 if (feature_intersects(bounds, feature)) {
 view.features.push_back(feature);
 included_ids.insert(feature.id);
 }
 }
 for (const auto& relation : snapshot.relations) {
 if (included_ids.count(relation.from_id) && included_ids.count(relation.to_id))
 view.relations.push_back(relation);
 }
 return view;
}

bool MapManager::configure(MapSnapshot snapshot, std::string* reason) {
 if (state_ != LifecycleState::Unconfigured)
 return reject(reason, "map manager is already configured");
 if (!valid_map_snapshot(snapshot, reason))
 return false;
 snapshot_ = std::make_shared<const MapSnapshot>(std::move(snapshot));
 state_ = LifecycleState::Inactive;
 return true;
}

bool MapManager::activate() {
 if (state_ != LifecycleState::Inactive || !snapshot_)
 return false;
 state_ = LifecycleState::Active;
 return true;
}

bool MapManager::deactivate() {
 if (state_ != LifecycleState::Active)
 return false;
 state_ = LifecycleState::Inactive;
 return true;
}

void MapManager::cleanup() {
 state_ = LifecycleState::Unconfigured;
 snapshot_.reset();
}

MapUpdateResult MapManager::replace(MapSnapshot snapshot, std::string* reason) {
 if (state_ != LifecycleState::Active)
 return MapUpdateResult::Inactive;
 if (!valid_map_snapshot(snapshot, reason))
 return MapUpdateResult::Invalid;
 if (snapshot.map_id != snapshot_->map_id || snapshot.frame_id != snapshot_->frame_id)
 return MapUpdateResult::IdentityChanged;
 if (snapshot.revision <= snapshot_->revision)
 return MapUpdateResult::StaleRevision;
 if (snapshot.sample_time < snapshot_->sample_time)
 return MapUpdateResult::StaleTimestamp;
 snapshot_ = std::make_shared<const MapSnapshot>(std::move(snapshot));
 return MapUpdateResult::Accepted;
}

std::optional<SemanticPlace> MapManager::resolve_place(std::string_view id) const {
 if (state_ != LifecycleState::Active || !snapshot_)
 return std::nullopt;
 const auto found = snapshot_->places.find(std::string(id));
 if (found == snapshot_->places.end())
 return std::nullopt;
 return found->second;
}

std::optional<MapView> MapManager::query_region(MapBounds2D bounds) const {
 if (state_ != LifecycleState::Active || !snapshot_)
 return std::nullopt;
 return make_map_view(*snapshot_, bounds);
}

bool MapCatalog::register_map(MapSnapshot snapshot, std::string* reason) {
 if (!valid_map_snapshot(snapshot, reason))
 return false;
 const auto id = snapshot.map_id;
 if (id.empty()) {
 if (reason)
 *reason = "map catalog requires a non-empty map id";
 return false;
 }
 maps_[id] = std::make_shared<const MapSnapshot>(std::move(snapshot));
 return true;
}

bool MapCatalog::contains(std::string_view map_id) const {
 return maps_.find(std::string(map_id)) != maps_.end();
}

std::vector<std::string> MapCatalog::map_ids() const {
 std::vector<std::string> ids;
 ids.reserve(maps_.size());
 for (const auto& [id, snapshot] : maps_)
 ids.push_back(id);
 return ids;
}

std::shared_ptr<const MapSnapshot> MapCatalog::snapshot(std::string_view map_id) const {
 const auto found = maps_.find(std::string(map_id));
 return found == maps_.end() ? nullptr : found->second;
}

bool MapCatalog::select(MapManager& active_map, std::string_view map_id,
 std::string* reason) const {
 const auto selected = snapshot(map_id);
 if (!selected) {
 if (reason)
 *reason = "map catalog does not contain '" + std::string(map_id) + "'";
 return false;
 }
 const auto previous = active_map.current_snapshot();
 if (previous && previous->map_id == selected->map_id &&
 previous->revision == selected->revision)
 return true;
 if (active_map.state() == LifecycleState::Active)
 active_map.deactivate();
 active_map.cleanup();
 if (active_map.configure(*selected, reason) && active_map.activate())
 return true;
 // A failed selection must not leave an empty map service if a previous
 // revision was available. Best-effort restoration preserves the last valid
 // provider state while returning false to the executive.
 if (previous) {
 std::string restore_reason;
 active_map.cleanup();
 if (active_map.configure(*previous, &restore_reason))
 active_map.activate();
 }
 if (reason && reason->empty())
 *reason = "map catalog selection failed to activate the requested map";
 return false;
}

} // namespace sim::runtime
