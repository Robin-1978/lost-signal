#include "client/map_editor.hpp"

#include "game/content_catalog.hpp"
#include "game/map_serialize.hpp"

#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>

#define CHECK(condition)                                                                                     \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n";                           \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

int main() {
 const auto catalog = game::ContentCatalog::load(LOST_SIGNAL_TEST_ASSETS);
 // Select nearest: marker wins over farther box centers.
 {
  client::MapEditor editor(catalog, "boot");
  CHECK(!editor.active());
  editor.toggle();
  CHECK(editor.active());
  editor.press(1.0, 1.0);
  CHECK(editor.selected_kind() == client::MapEditor::Kind::kMarker);
  CHECK(editor.selected_id() == "robot_bay");
 }
 // Drag moves the marker, clamps to map bounds, flags dirty.
 {
  client::MapEditor editor(catalog, "boot");
  editor.toggle();
  editor.press(1.0, 1.0);
  editor.drag(1.0, 1.0);
  editor.drag(2.5, 3.0);
  editor.release();
  const auto found = editor.working().markers.find("robot_bay");
  CHECK(found != editor.working().markers.end());
  CHECK(std::abs(found->second.x - 2.5) < 1e-9);
  CHECK(std::abs(found->second.y - 3.0) < 1e-9);
  CHECK(editor.dirty());
  editor.press(2.5, 3.0);
  editor.drag(2.5, 3.0);
  editor.drag(99.0, 99.0);
  editor.release();
  CHECK(found->second.x <= 20.0);
  CHECK(found->second.y <= 14.0);
 }
 // Empty press selects nothing; drag without selection is a no-op.
 {
  client::MapEditor editor(catalog, "boot");
  editor.toggle();
  editor.press(19.0, 1.0);
  CHECK(editor.selected_kind() == client::MapEditor::Kind::kNone);
  editor.drag(19.0, 1.0);
  editor.drag(10.0, 10.0);
  CHECK(!editor.dirty());
 }
 // v2: add obstacle/surface/marker with honest defaults (gated to this
 // level, paved numbers from the library), corner resize with clamps, delete.
 {
  client::MapEditor editor(catalog, "boot");
  editor.toggle();
  editor.add_obstacle(5.0, 5.0);
  CHECK(editor.selected_kind() == client::MapEditor::Kind::kObstacle);
  CHECK(editor.selected_id() == "edit_box_01");
  CHECK(editor.dirty());
  const auto& boxes = editor.working().obstacles;
  const auto& box = boxes.back();
  CHECK(std::abs(box.max.x - box.min.x - 1.0) < 1e-9);
  CHECK(box.active_in.size() == 1 && box.active_in[0] == "boot");
  editor.add_obstacle(6.0, 6.0);
  CHECK(editor.selected_id() == "edit_box_02");
  editor.add_surface(10.0, 10.0);
  CHECK(editor.selected_kind() == client::MapEditor::Kind::kSurface);
  const auto& ground = editor.working().surfaces.back();
  CHECK(ground.material == "paved");
  CHECK(std::abs(ground.friction_mu - 1.0) < 1e-12);
  CHECK(ground.active_in.size() == 1 && ground.active_in[0] == "boot");
  editor.add_marker(3.0, 3.0);
  CHECK(editor.selected_kind() == client::MapEditor::Kind::kMarker);
  CHECK(editor.working().markers.count(editor.selected_id()) == 1);
  // Resize the first box by its max/max corner, then delete the marker.
  editor.press(5.0, 5.0);
  CHECK(editor.selected_kind() == client::MapEditor::Kind::kObstacle);
  CHECK(editor.selected_id() == "edit_box_01");
  editor.press(5.5, 5.5);
  CHECK(editor.selected_corner() == 2);
  editor.drag(5.5, 5.5);
  editor.drag(9.0, 9.0);
  editor.release();
  const auto& grown = editor.working().obstacles[editor.working().obstacles.size() - 2];
  CHECK(std::abs(grown.max.x - 9.0) < 1e-9);
  CHECK(std::abs(grown.max.y - 9.0) < 1e-9);
  CHECK(std::abs(grown.min.x - 4.5) < 1e-9);
  editor.press(3.0, 3.0);
  CHECK(editor.selected_kind() == client::MapEditor::Kind::kMarker);
  editor.remove_selected();
  CHECK(editor.working().markers.count("edit_marker_01") == 0);
 }
 // Save round trip on a scratch copy: file bytes equal the serializer and
 // the reloaded catalog carries the edit.
 {
  const auto scratch = std::filesystem::temp_directory_path() / "lost_signal_editor_test";
  std::filesystem::remove_all(scratch);
  std::filesystem::copy(LOST_SIGNAL_TEST_ASSETS, scratch, std::filesystem::copy_options::recursive);
  auto scratch_catalog = game::ContentCatalog::load(scratch.string());
  client::MapEditor editor(scratch_catalog, "boot");
  editor.toggle();
  editor.press(1.0, 1.0);
  editor.drag(1.0, 1.0);
  editor.drag(2.0, 2.0);
  editor.release();
  CHECK(editor.save(scratch.string()));
  const auto map_path = scratch / "maps" / "research_base.json";
  std::ifstream file(map_path);
  std::ostringstream on_disk;
  on_disk << file.rdbuf();
  const auto reloaded = game::ContentCatalog::load(scratch.string());
  CHECK(on_disk.str() == game::serialize_map_canonical(reloaded.map("research_base")));
  const auto moved = reloaded.map("research_base").markers.find("robot_bay");
  CHECK(moved != reloaded.map("research_base").markers.end());
  CHECK(std::abs(moved->second.x - 2.0) < 1e-9);
  std::filesystem::remove_all(scratch);
 }
 std::cout << "map editor ok\n";
 return 0;
}
