#include <cassert>
#include <cstdio>
#include <filesystem>
#include <string>

#include "sim/eval/world_io.hpp"
#include "sim/viewer/map_editor.hpp"
#include "sim/world/world.hpp"

int main() {
  using namespace lost_signal;

  // Editor loads the live world, moves geometry, applies it back.
  sim::world::World w;
  w.set_static_boxes({{5.2, 1.0, 6.0, 1.8, 0.5}});
  w.add_place({"dock", 6.0, 0.0, 0.0});
  {
    sim::viewer::MapEditor ed;
    assert(!ed.active());
    ed.toggle();
    assert(ed.active());
    ed.load_from(w);
    assert(!ed.dirty());
    assert(ed.boxes().size() == 1 && ed.places().size() == 1);
    // Grab the box body and drag it +0.5 in x.
    ed.press(5.6, 1.4);
    assert(ed.selected_box() == 0 && ed.selected_corner() == -1);
    ed.drag(6.1, 1.4);
    ed.release();
    assert(ed.dirty());
    assert(ed.boxes()[0].x0 > 5.6 && ed.boxes()[0].x0 < 5.8);
    // Corner handle resize: grab max/max corner, pull +x.
    ed.press(ed.boxes()[0].x1, ed.boxes()[0].y1);
    assert(ed.selected_corner() == 2);
    const double x1_before = ed.boxes()[0].x1;
    ed.drag(ed.boxes()[0].x1 + 0.5, ed.boxes()[0].y1);
    ed.release();
    assert(ed.boxes()[0].x1 > x1_before);
    assert(ed.boxes()[0].x0 <= ed.boxes()[0].x1);  // stays normalized
    // Apply pushes to the live world and clears dirty.
    ed.apply(w);
    assert(!ed.dirty());
    assert(w.static_boxes().size() == 1);
    assert(w.static_boxes()[0].x0 > 5.6);
  }
  // Add + remove + place drag.
  {
    sim::viewer::MapEditor ed;
    ed.load_from(w);
    ed.add_box(0.0, 0.0);
    assert(ed.boxes().size() == 2 && ed.selected_box() == 1);
    ed.remove_selected();
    assert(ed.boxes().size() == 1);
    ed.add_place(1.0, 1.0);
    assert(ed.places().size() == 2 && ed.selected_place() == 1);
    assert(!ed.places()[1].place_id.empty());
    ed.drag(1.5, 1.5);  // no press: no-op, stays put
    ed.release();
    ed.press(1.0, 1.0);
    assert(ed.selected_place() == 1);
    ed.drag(2.0, 2.0);
    ed.release();
    assert(ed.places()[1].x_m > 1.9);
    ed.remove_selected();
    assert(ed.places().size() == 1);
    ed.apply(w);
    assert(w.places().size() == 1);
  }
  // Save writes the file and reload-proves it; load round-trips exactly.
  {
    const std::string path =
        (std::filesystem::temp_directory_path() / "test_world.json").string();
    sim::viewer::MapEditor ed;
    ed.load_from(w);
    ed.add_box(-1.0, -2.0);
    assert(ed.save(path));
    assert(!ed.dirty());
    std::vector<sim::world::StaticBox> rb;
    std::vector<sim::world::NamedPlace> rp;
    std::string err;
    assert(sim::eval::load_world_file(path, rb, rp, &err));
    assert(rb.size() == 2 && rp.size() == 1);
    assert(rb[1].x0 < rb[1].x1 && rb[1].y0 < rb[1].y1);
    assert(rp[0].place_id == "dock");
    // Corrupt file fails closed with untouched output.
    {
      std::FILE* fp = std::fopen(path.c_str(), "wb");
      std::fputs("{bad json", fp);
      std::fclose(fp);
    }
    rb.clear();
    rp.clear();
    assert(!sim::eval::load_world_file(path, rb, rp, &err));
    assert(rb.empty() && rp.empty());
    std::filesystem::remove(path);
  }

  std::printf("map_editor ok\n");
  return 0;
}
