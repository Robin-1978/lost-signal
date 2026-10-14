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
 // Save round trip on a scratch copy: file bytes equal the serializer and
 // the reloaded catalog carries the edit.
 {
  const auto scratch =
    std::filesystem::temp_directory_path() / "lost_signal_editor_test";
  std::filesystem::remove_all(scratch);
  std::filesystem::copy(LOST_SIGNAL_TEST_ASSETS, scratch,
                        std::filesystem::copy_options::recursive);
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
