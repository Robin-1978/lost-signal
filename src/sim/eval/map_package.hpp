#pragma once

// Versioned survey persistence (SIM eval, offline interchange only).
// A map package is one JSON document: manifest (format version + map id +
// geometry) + checksummed raster layer + semantics (places). Survey teams
// and offline tools exchange these; the static-map provider serves them
// from files. ROBOT never parses files.
//
// Byte-identical round-trip: save() emits keys in fixed order with %.17g
// doubles (exact), so save -> load -> save is byte-identical. The checksum
// (FNV-1a, integrity only — not crypto) covers everything but itself.

#include <cstdint>
#include <string>
#include <vector>

#include "robot/iface/map.hpp"

namespace lost_signal::sim::eval {

inline constexpr char kMapPackageFormat[] = "lost-signal-map-package/1";

struct MapPackage {
  std::string map_id;
  double resolution_m = 0.1;
  double origin_x_m = 0.0;
  double origin_y_m = 0.0;
  std::uint32_t width = 0;
  std::uint32_t height = 0;
  // Row-major occupancy, -1 unknown, 0 free, 100 occupied.
  std::vector<int8_t> occupancy;
  std::vector<robot::iface::Place> places;
};

// Snapshot (live or stored) into a package under the given id.
MapPackage package_from_snapshot(const std::string& map_id,
                                 const robot::iface::MapSnapshot& snapshot);

// File interchange. load verifies format + checksum + bounds and fails
// closed (untouched output) on any mismatch.
bool save_map_package(const std::string& path, const MapPackage& package,
                       std::string* error);
bool load_map_package(const std::string& path, MapPackage& package,
                       std::string* error);

}  // namespace lost_signal::sim::eval
