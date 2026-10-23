#pragma once

// World geometry interchange (SIM-side offline tooling only): static boxes
// + named places + ground material library/zones + terrain patches/slabs +
// environment keyframes as one JSON document. Format v2 ( terrain/material
// import): v1 files fail closed on the format tag — no silent partial load.
// The map editor saves through here; loading fails closed (untouched
// output) on any mismatch. ROBOT never parses files.

#include <string>
#include <utility>
#include <vector>

#include "sim/world/world.hpp"

namespace lost_signal::sim::eval {

inline constexpr char kWorldFileFormat[] = "lost-signal-world/2";

// Whole-file content aggregate: one canonical representation from disk to
// World to engine/survey, so authored facts cannot diverge per consumer.
struct WorldFileContent {
  std::vector<world::StaticBox> boxes;
  std::vector<world::NamedPlace> places;
  std::vector<world::MaterialEntry> materials;
  std::vector<world::MaterialZone> zones;
  std::vector<world::TerrainPatch> patches;
  std::vector<world::SupportSlab> slabs;
  std::vector<world::EnvironmentKeyframe> environment;
};

// Save full content. Returns false on IO error.
bool save_world_file(const std::string& path, const WorldFileContent& content,
                      std::string* error);
inline bool save_world_file(const std::string& path, const world::World& world,
                             std::string* error) {
  WorldFileContent content;
  content.boxes = world.static_boxes();
  content.places = world.places();
  content.materials = world.materials();
  content.zones = world.material_zones();
  content.patches = world.terrain_patches();
  content.slabs = world.support_slabs();
  content.environment = world.environment();
  return save_world_file(path, content, error);
}
// Load into the output content; content untouched on failure. After a
// successful load the caller applies via the World setters.
bool load_world_file(const std::string& path, WorldFileContent& content,
                      std::string* error);
// Applies every section to the live world (each setter bumps the geometry
// version, rebuilding engine + survey once the batch lands).
inline void apply_world_content(world::World& world,
                                const WorldFileContent& content) {
  world.set_static_boxes(content.boxes);
  world.set_places(content.places);
  world.set_materials(content.materials);
  world.set_material_zones(content.zones);
  world.set_terrain_patches(content.patches);
  world.set_support_slabs(content.slabs);
  world.set_environment(content.environment);
}

}  // namespace lost_signal::sim::eval
