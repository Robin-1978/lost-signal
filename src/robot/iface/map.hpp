#pragma once

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace lost_signal::robot::iface {

// Immutable published map revision. Robot Map layers are navigation priors,
// never physics truth (no friction, contact, or live wetness here).
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

  bool has_raster() const {
    return width > 0 && height > 0 && resolution_m > 0.0 &&
           occupancy.size() == width * height;
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
  virtual MapSnapshot active_snapshot() const = 0;
  virtual bool select_map(const std::string& map_id) = 0;
};

}  // namespace lost_signal::robot::iface
