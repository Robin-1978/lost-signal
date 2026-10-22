#pragma once

// World geometry interchange (SIM-side offline tooling only): static boxes
// + named places as one JSON document. The map editor saves through here;
// loading fails closed (untouched output) on any mismatch. ROBOT never
// parses files.

#include <string>
#include <vector>

#include "sim/world/world.hpp"

namespace lost_signal::sim::eval {

inline constexpr char kWorldFileFormat[] = "lost-signal-world/1";

// Save boxes + places. Returns false on IO error.
bool save_world_file(const std::string& path,
                     const std::vector<world::StaticBox>& boxes,
                     const std::vector<world::NamedPlace>& places,
                     std::string* error);
inline bool save_world_file(const std::string& path,
                            const world::World& world, std::string* error) {
  return save_world_file(path, world.static_boxes(), world.places(), error);
}
// Load into the output vectors; world untouched on failure. After a
// successful load the caller applies via set_static_boxes()/set_places().
bool load_world_file(const std::string& path,
                     std::vector<world::StaticBox>& boxes,
                     std::vector<world::NamedPlace>& places,
                     std::string* error);

}  // namespace lost_signal::sim::eval
