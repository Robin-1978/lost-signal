#pragma once

#include "sim/game/mission/content.hpp"
#include "sim/game/mission/content_catalog.hpp"

#include <string>

namespace client {

// In-client map editor. Pure world-space logic, with no raylib dependency,
// so unit tests drive it without a window. Screen-to-world picking stays in
// main.cpp; this model creates, moves, resizes and deletes map primitives.
// A full catalog reload validates references before a saved edit is accepted.
class MapEditor {
 public:
 enum class Kind { kNone, kMarker, kObstacle, kSurface, kTerrain };

 explicit MapEditor(const mission::ContentCatalog& catalog, std::string level_id);

 bool active() const { return active_; }
 void toggle() { active_ = !active_; }
 bool dirty() const { return dirty_; }
 const std::string& level_id() const { return level_id_; }
 const mission::MapDefinition& working() const { return working_; }
 Kind selected_kind() const { return selected_kind_; }
 // Grabbed box corner 0..3 (min/min, max/min, max/max, min/max), -1 when the
 // press took the body. Markers have no corners.
 int selected_corner() const { return selected_corner_; }
 const std::string& selected_id() const { return selected_id_; }
 const std::string& message() const { return message_; }

 // Press: corner handles of the current selection first, then the nearest
 // editable within kGrabRadiusM. Drag: move the selection (or the grabbed
 // corner) by the world-space delta, clamped to map bounds.
 void press(double world_x, double world_y);
 void drag(double world_x, double world_y);
 void release();
 // v2: id auto-generated (edit_box/marker/surface_NN, map-local unique);
 // new physics defaults to active_in=[this level] so other levels cannot
 // shift under a save. Markers carry no gate (unused markers are inert).
 // Deleting a level-referenced marker fails at save validation, loudly.
 void add_obstacle(double world_x, double world_y);
 void add_surface(double world_x, double world_y);
 void add_marker(double world_x, double world_y);
 void add_ramp(double world_x, double world_y);
 void add_platform(double world_x, double world_y);
 void adjust_selected_height(double delta_m);
 void adjust_selected_slope(double delta_grade);
 void adjust_selected_roughness(double delta_m);
 void rotate_selected_terrain();
 void remove_selected();

 // Serialize the working copy, write the map file, then reload the whole
 // catalog from disk into a temp object as the validity proof. Returns true
 // on success; message() carries the result either way. Caller (main loop)
 // reloads the live catalog + mission on success.
 bool save(const std::string& assets_root);

 private:
 static constexpr double kGrabRadiusM = 0.35;

 const mission::ContentCatalog& catalog_;
 std::string level_id_;
 mission::MapDefinition working_;
 bool active_{false};
 bool dirty_{false};
 Kind selected_kind_{Kind::kNone};
 std::string selected_id_;
 double last_x_{0.0};
 double last_y_{0.0};
 bool tracking_{false};
 int selected_corner_{-1};
 std::string free_id(const std::string& prefix) const;
 std::string message_;
};

} // namespace client
