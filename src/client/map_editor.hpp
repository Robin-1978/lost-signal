#pragma once

#include "game/content.hpp"
#include "game/content_catalog.hpp"

#include <string>

namespace client {

// In-client map editor, v1: select + drag-move + save. Pure world-space
// logic, no raylib dependency, so unit tests drive it without a window.
// Dragging the screen Murphy-to-world mapping stays in main.cpp; the
// renderer draws the overlay from this state (dashed working copy).
// Only moves exist in v1 (no create/delete/resize): every save keeps all
// ids, so unlock chains, presentations and replays cannot break by editing.
class MapEditor {
 public:
 enum class Kind { kNone, kMarker, kObstacle, kSurface };

 explicit MapEditor(const game::ContentCatalog& catalog, std::string level_id);

 bool active() const { return active_; }
 void toggle() { active_ = !active_; }
 bool dirty() const { return dirty_; }
 const std::string& level_id() const { return level_id_; }
 const game::MapDefinition& working() const { return working_; }
 Kind selected_kind() const { return selected_kind_; }
 const std::string& selected_id() const { return selected_id_; }
 const std::string& message() const { return message_; }

 // Press: pick the nearest editable within kGrabRadiusM. Drag: move the
 // selection by the world-space delta (clamped to map bounds).
 void press(double world_x, double world_y);
 void drag(double world_x, double world_y);
 void release();

 // Serialize the working copy, write the map file, then reload the whole
 // catalog from disk into a temp object as the validity proof. Returns true
 // on success; message() carries the result either way. Caller (main loop)
 // reloads the live catalog + mission on success.
 bool save(const std::string& assets_root);

 private:
 static constexpr double kGrabRadiusM = 0.35;

 const game::ContentCatalog& catalog_;
 std::string level_id_;
 game::MapDefinition working_;
 bool active_{false};
 bool dirty_{false};
 Kind selected_kind_{Kind::kNone};
 std::string selected_id_;
 double last_x_{0.0};
 double last_y_{0.0};
 bool tracking_{false};
 std::string message_{"E 编辑 · 点选拖动 · S 存盘"};
};

} // namespace client
