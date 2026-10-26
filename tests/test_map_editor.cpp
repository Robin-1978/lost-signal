#include <cassert>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
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
    assert(ed.boxes()[1].height_m > 0.0);  // new wall stands, scan sees it
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
    sim::eval::WorldFileContent rc;
    std::string err;
    assert(sim::eval::load_world_file(path, rc, &err));
    assert(rc.boxes.size() == 2 && rc.places.size() == 1);
    assert(rc.boxes[1].x0 < rc.boxes[1].x1 &&
           rc.boxes[1].y0 < rc.boxes[1].y1);
    assert(rc.places[0].place_id == "dock");
    // Corrupt file fails closed with untouched output.
    {
      std::FILE* fp = std::fopen(path.c_str(), "wb");
      std::fputs("{bad json", fp);
      std::fclose(fp);
    }
    rc = sim::eval::WorldFileContent{};
    assert(!sim::eval::load_world_file(path, rc, &err));
    assert(rc.boxes.empty() && rc.places.empty());
    std::filesystem::remove(path);
  }
  // v2 sections round-trip field-exact, byte-identical on re-save.
  {
    const std::string path =
        (std::filesystem::temp_directory_path() / "test_world_v2.json")
            .string();
    sim::eval::WorldFileContent content;
 content.survey_resolution_m = 0.08;
 content.initial_pose = sim::world::InitialPosePrior{
 1.0, -2.0, 0.25, 0.12, 0.06};
 content.boxes = {{0.0, 0.0, 1.0, 1.0, 0.5, "fixture_box"}};
 content.boxes[0].impact_material = sim::world::ImpactMaterial::kMetal;
 content.boxes[0].prior_map_visible = false;
    content.places = {{"dock", 2.0, 0.0, 0.0}};
content.materials = {{"ice", 0.25, 0.005, 0.006, 1.0, 0.30},
 {"gravel", 0.7, 0.015, 0.01, 1.15, 0.28, "dust", 0.0}};
    content.zones = {{"slick", 3.0, -1.0, 6.0, 1.0, "ice"}};
    content.patches = {
        {"ramp", 6.0, -1.0, 9.0, 1.0, 0.0, 0.1, 0.0, 0.0, 0.0}};
    content.slabs = {
        {"deck", 10.0, 0.0, 1.5, 1.0, 0.0, 0.3, 0.0}};
    content.environment = {
        {0.0, {{"ice", 1.0}}}, {10.0, {{"ice", 0.8}}}};
    std::string err;
    assert(sim::eval::save_world_file(path, content, &err));
    sim::eval::WorldFileContent rc;
    assert(sim::eval::load_world_file(path, rc, &err));
 assert(rc.survey_resolution_m == 0.08);
    assert(rc.materials.size() == 2 && rc.zones.size() == 1 &&
           rc.patches.size() == 1 && rc.slabs.size() == 1 &&
           rc.environment.size() == 2);
 assert(rc.initial_pose.has_value());
 assert(rc.initial_pose->x_m == 1.0 && rc.initial_pose->y_m == -2.0);
 assert(rc.initial_pose->position_sigma_m == 0.12);
 assert(rc.boxes[0].box_id == "fixture_box");
 assert(rc.boxes[0].impact_material == sim::world::ImpactMaterial::kMetal);
 assert(!rc.boxes[0].prior_map_visible);
    assert(rc.materials[0].material_id == "ice");
    assert(std::abs(rc.materials[0].friction_mu - 0.25) < 1e-12);
 assert(rc.materials[1].surface_effect == "dust");
    assert(rc.zones[0].material_id == "ice");
    assert(std::abs(rc.patches[0].grade_x - 0.1) < 1e-12);
    assert(std::abs(rc.slabs[0].center_height_m - 0.3) < 1e-12);
    assert(rc.environment[1].material_grip.size() == 1 &&
           std::abs(rc.environment[1].material_grip[0].second - 0.8) <
               1e-12);
    // Re-save is byte-identical (canonical serialization).
    const std::string path2 =
        (std::filesystem::temp_directory_path() / "test_world_v2b.json")
            .string();
    assert(sim::eval::save_world_file(path2, rc, &err));
    {
      std::ifstream a(path, std::ios::binary), b(path2, std::ios::binary);
      std::string sa((std::istreambuf_iterator<char>(a)),
                     std::istreambuf_iterator<char>());
      std::string sb((std::istreambuf_iterator<char>(b)),
                     std::istreambuf_iterator<char>());
      assert(sa == sb);
    }
    // Editor carries v2 sections through untouched.
    sim::world::World w2;
    sim::eval::apply_world_content(w2, rc);
 assert(w2.survey_resolution_m() == 0.08);
    sim::viewer::MapEditor ed;
    ed.load_from(w2);
    ed.add_box(20.0, 20.0);
    assert(ed.save(path2));
    sim::eval::WorldFileContent rc2;
    assert(sim::eval::load_world_file(path2, rc2, &err));
    assert(rc2.boxes.size() == 2 && rc2.materials.size() == 2 &&
           rc2.zones.size() == 1 && rc2.patches.size() == 1);
 assert(rc2.survey_resolution_m == 0.08);
 assert(rc2.initial_pose.has_value() &&
 rc2.initial_pose->heading_sigma_rad == 0.06);
    // v1 tag fails closed (no silent partial load across versions).
    {
      std::FILE* fp = std::fopen(path2.c_str(), "wb");
      std::fputs(
          "{\"format\":\"lost-signal-world/1\",\"boxes\":[],\"places\":[]}",
          fp);
      std::fclose(fp);
    }
    rc2 = sim::eval::WorldFileContent{};
    assert(!sim::eval::load_world_file(path2, rc2, &err));
    assert(rc2.boxes.empty());
    // Unknown material + duplicate material fail closed at load.
    {
      sim::eval::WorldFileContent bad = content;
      bad.zones[0].material_id = "nope";
      assert(sim::eval::save_world_file(path2, bad, &err));
      sim::eval::WorldFileContent tmp;
      assert(!sim::eval::load_world_file(path2, tmp, &err));
      bad = content;
      bad.materials.push_back(bad.materials[0]);
      assert(sim::eval::save_world_file(path2, bad, &err));
      assert(!sim::eval::load_world_file(path2, tmp, &err));
 bad = content;
 bad.materials[0].surface_effect = "water";
 bad.materials[0].fluid_depth_m = 0.0;
 assert(sim::eval::save_world_file(path2, bad, &err));
 assert(!sim::eval::load_world_file(path2, tmp, &err));
 bad = content;
 bad.boxes[0].impact_material = sim::world::ImpactMaterial::kUnknown;
 assert(sim::eval::save_world_file(path2, bad, &err));
 assert(!sim::eval::load_world_file(path2, tmp, &err));
    }
    std::filesystem::remove(path);
    std::filesystem::remove(path2);
  }

  // Terrain handles: add/select/move/remove patches + zones; save keeps
  // every section (materials resolve at load, zones carry the id only).
  {
    sim::world::World w2;
    w2.set_materials({{"ice", 0.25, 0.0, 0.006, 1.0, 0.3}});
    sim::viewer::MapEditor ed;
    ed.load_from(w2);
    ed.add_patch(5.0, 5.0);
    assert(ed.selected_patch() == 0);
    ed.press(5.0, 5.0);
    assert(ed.selected_patch() == 0);
    ed.drag(6.0, 6.0);
    ed.release();
    ed.add_zone(0.0, 0.0, "ice");
    assert(ed.selected_zone() == 0);
    ed.press(0.0, 0.0);
    assert(ed.selected_zone() == 0);
    ed.remove_selected();
    // Reload-prove through a temp file: patch moved, zone removed.
    const std::string path =
        (std::filesystem::temp_directory_path() / "test_world_t.json")
            .string();
    assert(ed.save(path));
    sim::eval::WorldFileContent rc;
    std::string err;
    assert(sim::eval::load_world_file(path, rc, &err));
    assert(rc.patches.size() == 1 && rc.zones.empty());
    assert(rc.patches[0].x0 > 0.9 && rc.patches[0].grade_x == 0.2);
    // Zone with unknown material fails closed at load (save is unvalidating
    // by design; the loader owns the reference check). The save-time
    // validity proof catches it too.
    ed.add_zone(10.0, 10.0, "nope");
    assert(!ed.save(path));
    sim::eval::WorldFileContent bad;
    assert(!sim::eval::load_world_file(path, bad, &err));
    std::filesystem::remove(path);
  }

  std::printf("map_editor ok\n");
  return 0;
}
