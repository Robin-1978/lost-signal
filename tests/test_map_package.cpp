#include <cassert>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

#include "robot/iface/map.hpp"
#include "sim/eval/map_package.hpp"
#include "sim/providers/static_map.hpp"
#include "sim/world/world.hpp"

using namespace lost_signal;

namespace {

std::string tmp_path(const char* name) {
  return (std::filesystem::temp_directory_path() / name).string();
}

std::string read_bytes(const std::string& path) {
  std::ifstream in(path, std::ios::binary);
  std::ostringstream ss;
  ss << in.rdbuf();
  return ss.str();
}

}  // namespace

int main() {
  using sim::eval::MapPackage;

  // Survey a world: boxes + places through the live provider.
  sim::world::World w;
  sim::world::StaticBox b{1.0, 1.0, 2.0, 2.0, 0.5};
  w.set_static_boxes({b});
  w.add_place({"dock", 3.0, 0.0, 0.0});
  sim::providers::StaticMapProvider live(&w, -10.0, -10.0, 20.0, 20.0, 0.25);
  const robot::iface::MapSnapshot snap = live.active_snapshot();
  assert(snap.has_raster());
  assert(!snap.places.empty());

  // Package -> save -> load -> save is byte-identical.
  const std::string path = tmp_path("test_map_package.json");
  MapPackage pkg = sim::eval::package_from_snapshot("survey-1", snap);
  assert(pkg.map_id == "survey-1");
  assert(pkg.places.size() == 1 && pkg.places[0].place_id == "dock");
  std::string err;
  assert(sim::eval::save_map_package(path, pkg, &err));
  MapPackage back;
  assert(sim::eval::load_map_package(path, back, &err));
  assert(back.map_id == "survey-1");
  assert(back.occupancy == pkg.occupancy);
  assert(back.places.size() == 1 && back.places[0].x_m == 3.0);
  const std::string path2 = tmp_path("test_map_package_2.json");
  assert(sim::eval::save_map_package(path2, back, &err));
  assert(read_bytes(path) == read_bytes(path2));

  // Corruption fails closed with a checksum error (output untouched).
  {
    std::string text = read_bytes(path);
    const std::string before = text;
    std::size_t pos = text.find("100");
    assert(pos != std::string::npos);
    text[pos] = '0';  // flip an occupied cell to free: checksum must catch it
    {
      std::ofstream out(path, std::ios::binary | std::ios::trunc);
      out << text;
    }
    MapPackage evil;
    evil.map_id = "sentinel";
    assert(!sim::eval::load_map_package(path, evil, &err) && !err.empty());
    assert(evil.map_id == "sentinel");  // untouched on failure
    {
      std::ofstream out(path, std::ios::binary | std::ios::trunc);
      out << before;  // restore for the remaining tests
    }
  }

  // Wrong format version fails closed.
  {
    std::string text = read_bytes(path);
    const std::string tag = "lost-signal-map-package/1";
    std::size_t pos = text.find(tag);
    assert(pos != std::string::npos);
    text.replace(pos, tag.size(), "lost-signal-map-package/999");
    const std::string bad = tmp_path("test_map_package_bad.json");
    {
      std::ofstream out(bad, std::ios::binary | std::ios::trunc);
      out << text;
    }
    MapPackage evil;
    assert(!sim::eval::load_map_package(bad, evil, &err));
    std::filesystem::remove(bad);
  }

  // Save validates inputs: no id / mismatched dims fail without touching disk.
  {
    MapPackage bad_id = pkg;
    bad_id.map_id.clear();
    assert(!sim::eval::save_map_package(tmp_path("x.json"), bad_id, &err));
    MapPackage bad_dims = pkg;
    bad_dims.occupancy.pop_back();
    assert(!sim::eval::save_map_package(tmp_path("x.json"), bad_dims, &err));
    assert(!sim::eval::load_map_package(tmp_path("no-such-file.json"), bad_dims,
                                        &err));
  }

  // Package-backed provider serves the stored survey: identical raster and
  // places, selection by package id, unknown ids fail closed.
  {
    sim::providers::StaticMapProvider stored(pkg);
    const robot::iface::MapSnapshot got = stored.active_snapshot();
    assert(got.map_id == "survey-1");
    assert(got.width == snap.width && got.height == snap.height);
    assert(got.occupancy == snap.occupancy);
    assert(got.places.size() == 1 && got.places[0].place_id == "dock");
    assert(stored.select_map("survey-1"));
    assert(stored.select_map(""));
    assert(!stored.select_map("other-map"));
  }

  std::filesystem::remove(path);
  std::filesystem::remove(path2);
  std::printf("map_package ok\n");
  return 0;
}
