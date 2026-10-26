#include "sim/providers/static_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "sim/world/terrain.hpp"

namespace lost_signal::sim::providers {

StaticMapProvider::StaticMapProvider(world::World* world, double min_x_m,
                                     double min_y_m, double width_m,
                                     double height_m, double resolution_m)
    : world_(world),
      min_x_m_(min_x_m),
      min_y_m_(min_y_m),
      width_m_(width_m),
      height_m_(height_m),
      resolution_m_(resolution_m) {
  snapshot_.map_id = "sim-world";
  snapshot_.frame_id = "map";
}

StaticMapProvider::StaticMapProvider(const eval::MapPackage& package)
    : package_(package), from_package_(true) {
  snapshot_.map_id = package_.map_id;
  snapshot_.frame_id = "map";
}

void StaticMapProvider::rebuild() const {
  if (from_package_) {
    // Stored survey: raster + places verbatim, revision pinned (immutable).
    snapshot_.map_id = package_.map_id;
    snapshot_.frame_id = "map";
    snapshot_.width = package_.width;
    snapshot_.height = package_.height;
    snapshot_.resolution_m = package_.resolution_m;
    snapshot_.origin_x_m = package_.origin_x_m;
    snapshot_.origin_y_m = package_.origin_y_m;
    snapshot_.occupancy = package_.occupancy;
    snapshot_.places = package_.places;
    snapshot_.elevation_m.assign(package_.occupancy.size(),
                                 std::numeric_limits<float>::quiet_NaN());
    snapshot_.elevation_sigma_m.assign(package_.occupancy.size(), 0.0f);
    snapshot_.revision = 1;
    has_built_ = true;
    return;
  }
  const int w = static_cast<int>(std::ceil(width_m_ / resolution_m_));
  const int h = static_cast<int>(std::ceil(height_m_ / resolution_m_));
  snapshot_.width = w;
  snapshot_.height = h;
  snapshot_.resolution_m = resolution_m_;
  snapshot_.origin_x_m = min_x_m_;
  snapshot_.origin_y_m = min_y_m_;
  snapshot_.occupancy.assign(w * h, -1);
 // Flat ground is known zero-height survey data. Sparse obstacle
 // rasterization below touches only each box's clipped cell bounds.
 snapshot_.elevation_m.assign(w * h, 0.0f);
  snapshot_.elevation_sigma_m.assign(w * h, 0.0f);
  if (world_ != nullptr) {
    // Surveyed elevation from the SAME canonical height function the
    // engine builds contact geometry from: map and plant agree exactly.
    // Material zones are deliberately NOT published (unobservable physics
    // truth — the ROBOT estimates traction from its own slip evidence).
    // Occupancy keeps its v1 doctrine: terrain stays unknown (-1) until
    // the mapper observes it; the global planner penalizes unknown, the
    // safety layers never assume free.
 // Rasterize by obstacle bounds: sparse worlds are O(cells + painted cells),
 // rather than testing every authored box at every map cell.
 for (const auto& b : world_->static_boxes()) {
 if (!b.prior_map_visible) continue;
 if (!(b.height_m > 0.0)) continue; // flat paint: free space
 const int begin_x = std::max(0, static_cast<int>(std::floor(
 (b.x0 - min_x_m_) / resolution_m_)) - 1);
 const int end_x = std::min(w, static_cast<int>(std::ceil(
 (b.x1 - min_x_m_) / resolution_m_)) + 1);
 const int begin_y = std::max(0, static_cast<int>(std::floor(
 (b.y0 - min_y_m_) / resolution_m_)) - 1);
 const int end_y = std::min(h, static_cast<int>(std::ceil(
 (b.y1 - min_y_m_) / resolution_m_)) + 1);
 for (int gy = begin_y; gy < end_y; ++gy) {
 const double cy = min_y_m_ + (gy + 0.5) * resolution_m_;
 if (cy < b.y0 || cy >= b.y1) continue;
 for (int gx = begin_x; gx < end_x; ++gx) {
 const double cx = min_x_m_ + (gx + 0.5) * resolution_m_;
 if (cx < b.x0 || cx >= b.x1) continue;
 const std::size_t index = static_cast<std::size_t>(gy) * w + gx;
 snapshot_.occupancy[index] = 100;
 snapshot_.elevation_m[index] = std::max(
 snapshot_.elevation_m[index], static_cast<float>(b.height_m));
 }
 }
 }
    const std::vector<world::TerrainPatch>& patches =
        world_->terrain_patches();
    const std::vector<world::SupportSlab>& slabs = world_->support_slabs();
 if (!patches.empty() || !slabs.empty()) {
    for (int gy = 0; gy < h; ++gy) {
      for (int gx = 0; gx < w; ++gx) {
        const double cx = min_x_m_ + (gx + 0.5) * resolution_m_;
        const double cy = min_y_m_ + (gy + 0.5) * resolution_m_;
 const std::size_t index = static_cast<std::size_t>(gy) * w + gx;
 // No static information outside authored boxes in v1: free space stays
 // unknown until observed. Terrain is an independent height prior and only
 // needs cell-wise sampling when non-flat geometry exists.
 if (snapshot_.occupancy[index] < 50) {
 const world::TerrainSample sample =
 world::sample_terrain(patches, slabs, cx, cy);
 snapshot_.elevation_m[index] = static_cast<float>(sample.height_m);
 }
 }
 }
 }
 if (world_ != nullptr) {
      snapshot_.places.clear();
      for (const auto& p : world_->places()) {
        snapshot_.places.push_back({p.place_id, p.x_m, p.y_m, p.yaw_rad});
      }
    }
    snapshot_.revision = world_->geometry_version() + 1 + window_epoch_;
    built_version_ = world_->geometry_version();
  }
  has_built_ = true;
}

void StaticMapProvider::set_window(double min_x_m, double min_y_m,
                                   double width_m, double height_m) {
  if (from_package_) return;  // stored surveys are immutable
  if (!(std::isfinite(min_x_m) && std::isfinite(min_y_m) &&
        width_m > 0.0 && height_m > 0.0 && std::isfinite(width_m) &&
        std::isfinite(height_m))) {
    return;
  }
  min_x_m_ = min_x_m;
  min_y_m_ = min_y_m;
  width_m_ = width_m;
  height_m_ = height_m;
  ++window_epoch_;
  has_built_ = false;  // retire the cache; the next poll rebuilds
}

void StaticMapProvider::set_resolution(double resolution_m) {
 if (from_package_ || !std::isfinite(resolution_m) ||
 !(resolution_m > 0.0) || resolution_m_ == resolution_m) {
 return;
 }
 resolution_m_ = resolution_m;
 ++window_epoch_;
 has_built_ = false;
}

const robot::iface::MapSnapshot& StaticMapProvider::active_snapshot() const {
  const std::uint64_t v = world_ != nullptr ? world_->geometry_version() : 0;
  if (!has_built_ || v != built_version_) rebuild();
  return snapshot_;
}

bool StaticMapProvider::select_map(const std::string& map_id) {
  if (from_package_) return map_id == package_.map_id || map_id.empty();
  return map_id == "sim-world" || map_id.empty();
}

}  // namespace lost_signal::sim::providers
