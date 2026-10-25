#pragma once

#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace lost_signal::robot::iface {

// Immutable published map revision. Robot Map layers are navigation priors,
// never physics truth (no friction, contact, or live wetness here).
// Named semantic place. Task intents reference places by id; the ROBOT map
// provider resolves them to poses. Coordinates are navigation priors.
struct Place {
  std::string place_id;
  double x_m = 0.0;
  double y_m = 0.0;
  double yaw_rad = 0.0;
};

struct MapSnapshot {
  std::string map_id;
  std::uint64_t revision = 0;
  // Optional raster: row-major occupancy, -1 unknown, 0 free, 100 occupied.
  // Vector-only maps leave this empty. Cell (gx,gy) covers
  // [origin + g*res, origin + (g+1)*res) in frame_id.
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  double resolution_m = 0.0;
  double origin_x_m = 0.0;
  double origin_y_m = 0.0;
  std::vector<int8_t> occupancy;
  std::string frame_id = "map";
  // Named semantic places for task resolution (may be empty).
  std::vector<Place> places;
  // Optional 2.5D layers, same geometry as occupancy. NaN = unknown.
  // Elevation is surveyed prior (ramps, slopes, decks), never live physics.
  std::vector<float> elevation_m;
  std::vector<float> elevation_sigma_m;

  bool has_elevation() const {
    return has_raster() && elevation_m.size() == width * height;
  }
  // Nearest-cell elevation, NaN when unknown or absent.
  float elevation_at(double wx, double wy) const {
    if (!has_elevation()) return std::numeric_limits<float>::quiet_NaN();
    int gx = static_cast<int>(std::floor((wx - origin_x_m) / resolution_m));
    int gy = static_cast<int>(std::floor((wy - origin_y_m) / resolution_m));
    if (gx < 0 || gy < 0 || gx >= static_cast<int>(width) ||
        gy >= static_cast<int>(height)) {
      return std::numeric_limits<float>::quiet_NaN();
    }
    return elevation_m[gy * width + gx];
  }

  bool has_raster() const {
    return width > 0 && height > 0 && resolution_m > 0.0 &&
           occupancy.size() == width * height;
  }
 // A raster filled entirely with -1 is a transport-sized window, not a
 // surveyed map. Localization must enter mapless/SLAM startup until at
 // least one cell carries known occupancy evidence.
 bool has_known_occupancy() const {
 if (!has_raster()) return false;
 for (const auto cell : occupancy) {
  if (cell >= 0) return true;
 }
 return false;
 }
  // Unknown counts as blocked here; planners that assume unknown-free
  // implement their own policy on top of cell().
  bool blocked(double wx, double wy) const {
    if (!has_raster()) return true;
    int gx = static_cast<int>(std::floor((wx - origin_x_m) / resolution_m));
    int gy = static_cast<int>(std::floor((wy - origin_y_m) / resolution_m));
    if (gx < 0 || gy < 0 || gx >= static_cast<int>(width) ||
        gy >= static_cast<int>(height)) {
      return true;
    }
    return occupancy[gy * width + gx] >= 50;
  }
};

struct MapView {
  std::string map_id;
  std::uint64_t revision = 0;
};

// ROBOT-owned provider interface. SIM, SLAM, file, or remote service implement
// it; autonomy only consumes it.
class IMapProvider {
 public:
  virtual ~IMapProvider() = default;
 // Published revisions are immutable and owned by the provider. Returning a
 // view avoids copying every raster/elevation layer on each control tick;
 // callers that need to retain a revision can still copy it explicitly.
 virtual const MapSnapshot& active_snapshot() const = 0;
  virtual bool select_map(const std::string& map_id) = 0;
};

}  // namespace lost_signal::robot::iface
