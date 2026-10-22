#pragma once

#include <cstdint>
#include <string>

namespace lost_signal::robot::iface {

// Immutable published map revision. Robot Map layers are navigation priors,
// never physics truth (no friction, contact, or live wetness here).
struct MapSnapshot {
  std::string map_id;
  std::uint64_t revision = 0;
  // Optional tiny raster for v1. Vector-only maps leave this empty.
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  double resolution_m = 0.0;
  std::string frame_id = "map";
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
