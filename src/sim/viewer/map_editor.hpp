#pragma once

// In-viewer map editor. Pure world-space logic, no raylib dependency, so
// unit tests drive it without a window. Screen-to-world picking stays in
// the viewer frontend; this model creates, moves, resizes and deletes map
// primitives on a working copy. apply() pushes to the live World,
// save() writes the world file and reloads it into a temp as the validity
// proof (same discipline as main's client map editor).

#include <string>
#include <vector>

#include "sim/eval/world_io.hpp"
#include "sim/world/world.hpp"

namespace lost_signal::sim::viewer {

class MapEditor {
 public:
  MapEditor() = default;

  bool active() const { return active_; }
  void toggle() { active_ = !active_; }
  bool dirty() const { return dirty_; }
  const std::string& message() const { return message_; }

  const std::vector<world::StaticBox>& boxes() const { return content_.boxes; }
  const std::vector<world::NamedPlace>& places() const {
    return content_.places;
  }
  // v2 sections without edit handles yet (materials/zones/terrain/slabs/
  // environment): carried through load/apply/save untouched so editing
  // boxes never drops authored terrain facts.

  // Selection: box index + grabbed corner 0..3
  // (min/min, max/min, max/max, min/max), -1 body-grab; place index, -1 none.
  int selected_box() const { return sel_box_; }
  int selected_corner() const { return sel_corner_; }
  int selected_place() const { return sel_place_; }

  // Snapshots the live world into the working copy (entering edit mode).
  void load_from(const world::World& world);
  // Pushes the working copy to the live world. No validation needed:
  // every mutation keeps boxes normalized and place ids non-empty.
  void apply(world::World& world);

  // Press: selected-box corner handles first, then nearest box body, then
  // nearest place within kGrabRadiusM. Drag moves the grabbed corner/body
  // or place by the world-space delta, clamped to the map bounds. Mark
  // dirty on any change.
  void press(double world_x, double world_y);
  void drag(double world_x, double world_y);
  void release();

  void add_box(double world_x, double world_y);
  void add_place(double world_x, double world_y);
  void remove_selected();

  // Serialize the working copy, write the file, then reload the whole file
  // into temp vectors as the validity proof. Returns true on success;
  // message() carries the result either way.
  bool save(const std::string& path);

 private:
  static constexpr double kGrabRadiusM = 0.35;
  static constexpr double kMapBoundM = 50.0;
  static constexpr double kDefaultBoxM = 0.8;
  // New boxes stand 1.0 m tall: above the ~0.4 m lidar so the scan sees
  // them (flat paint is authored by zeroing the height afterwards).
  static constexpr double kDefaultHeightM = 1.0;

  static double clamp_to_map(double v) {
    if (v < -kMapBoundM) return -kMapBoundM;
    if (v > kMapBoundM) return kMapBoundM;
    return v;
  }
  static void normalize(world::StaticBox& b);

  bool active_ = false;
  bool dirty_ = false;
  std::string message_;
  eval::WorldFileContent content_;
  int sel_box_ = -1;
  int sel_corner_ = -1;  // -1 body, 0..3 corner
  int sel_place_ = -1;
  bool pressing_ = false;
  double press_x_ = 0.0, press_y_ = 0.0;
  int place_seq_ = 0;
};

}  // namespace lost_signal::sim::viewer
