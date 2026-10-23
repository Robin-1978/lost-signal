#include "sim/providers/static_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

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
  snapshot_.elevation_m.assign(w * h,
                               std::numeric_limits<float>::quiet_NaN());
  snapshot_.elevation_sigma_m.assign(w * h, 0.0f);
  if (world_ != nullptr) {
    for (int gy = 0; gy < h; ++gy) {
      for (int gx = 0; gx < w; ++gx) {
        const double cx = min_x_m_ + (gx + 0.5) * resolution_m_;
        const double cy = min_y_m_ + (gy + 0.5) * resolution_m_;
        bool occupied = false;
        float height = 0.0f;
        for (const auto& b : world_->static_boxes()) {
          if (!(b.height_m > 0.0)) continue;  // flat paint: free space
          if (cx >= b.x0 && cx < b.x1 && cy >= b.y0 && cy < b.y1) {
            occupied = true;
            height = std::max(height, static_cast<float>(b.height_m));
          }
        }
        // No static information outside authored boxes in v1: free space
        // stays unknown until the mapper observes it. The global planner
        // treats unknown as traversable-with-penalty; the local safety
        // layers never do.
        if (occupied) {
          snapshot_.occupancy[gy * w + gx] = 100;
          snapshot_.elevation_m[gy * w + gx] = height;
        }
      }
    }
    if (world_ != nullptr) {
      snapshot_.places.clear();
      for (const auto& p : world_->places()) {
        snapshot_.places.push_back({p.place_id, p.x_m, p.y_m, p.yaw_rad});
      }
    }
    snapshot_.revision = world_->geometry_version() + 1;
    built_version_ = world_->geometry_version();
  }
  has_built_ = true;
}

robot::iface::MapSnapshot StaticMapProvider::active_snapshot() const {
  const std::uint64_t v = world_ != nullptr ? world_->geometry_version() : 0;
  if (!has_built_ || v != built_version_) rebuild();
  return snapshot_;
}

bool StaticMapProvider::select_map(const std::string& map_id) {
  if (from_package_) return map_id == package_.map_id || map_id.empty();
  return map_id == "sim-world" || map_id.empty();
}

}  // namespace lost_signal::sim::providers
