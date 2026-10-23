// Terrain + material import gate: authored facts drive the engine and the
// survey from one canonical function, and ROBOT sees only measurements.
// Pins: height math, engine contact geometry (raycast), per-zone friction
// effect + slip labels, survey elevation, environment grip, fail-closed
// refuses. Engine sections need MuJoCo; math/survey run anywhere.
#include <cassert>
#include <cmath>
#include <cstdio>

#include "robot/iface/task.hpp"
#include "sim/providers/static_map.hpp"
#include "sim/world/terrain.hpp"
#include "sim/world/world.hpp"

using namespace lost_signal;

namespace {

sim::world::World make_diff(double duty) {
  sim::world::World w;
  robot::iface::ActuatorCommand cmd;
  cmd.mode = robot::iface::ActuatorMode::kDiffDuty;
  cmd.stamp_ns = 0;
  cmd.expiry_ns = 1'000'000'000;
  cmd.drive[0] = cmd.drive[1] = duty;
  w.apply(cmd);
  return w;
}

}  // namespace

int main() {
  // Canonical height math (single source for engine + survey).
  {
    using namespace sim::world;
    std::vector<TerrainPatch> patches = {
        {"ramp", 2.0, -1.0, 6.0, 1.0, 0.0, 0.1, 0.0, 0.0, 0.0}};
    std::vector<SupportSlab> slabs;
    TerrainSample s = sample_terrain(patches, slabs, 3.0, 0.0);
    assert(std::abs(s.height_m - 0.1) < 1e-12);
    assert(std::abs(s.grade_x - 0.1) < 1e-12);
    assert(sample_terrain(patches, slabs, 0.0, 0.0).height_m == 0.0);
    // Later entries win; slabs override patches.
    patches.push_back(
        {"cap", 2.0, -1.0, 6.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0});
    assert(std::abs(sample_terrain(patches, slabs, 3.0, 0.0).height_m - 1.0) <
           1e-12);
    slabs.push_back({"deck", 3.0, 0.0, 0.5, 0.5, 0.0, 2.0, 0.0});
    assert(std::abs(sample_terrain(patches, slabs, 3.0, 0.0).height_m - 2.0) <
           1e-12);
    assert(std::abs(sample_terrain(patches, slabs, 0.0, 0.0).height_m) <
           1e-12);
    // Zone lookup: later wins, null outside.
    std::vector<MaterialZone> zones = {{"a", 0.0, 0.0, 5.0, 5.0, "ice"}};
    assert(material_zone_at(zones, 1.0, 1.0) != nullptr);
    assert(material_zone_at(zones, 9.0, 9.0) == nullptr);
    // Environment grip: interpolate, clamp ends, missing stays 1.0.
    std::vector<EnvironmentKeyframe> env = {
        {0.0, {{"ice", 1.0}}}, {10.0, {{"ice", 0.5}}}};
    assert(std::abs(environment_grip_at(env, "ice", 5.0) - 0.75) < 1e-12);
    assert(std::abs(environment_grip_at(env, "ice", 20.0) - 0.5) < 1e-12);
    assert(std::abs(environment_grip_at(env, "ice", -1.0) - 1.0) < 1e-12);
    assert(std::abs(environment_grip_at(env, "dry", 5.0) - 1.0) < 1e-12);
  }

  // Survey: elevation follows the authored function; materials unpublished.
  {
    sim::world::World w;
    w.set_terrain_patches(
        {{"ramp", 2.0, -1.0, 6.0, 1.0, 0.0, 0.1, 0.0, 0.0, 0.0}});
    sim::providers::StaticMapProvider maps(&w, -10.0, -10.0, 20.0, 20.0,
                                           0.25);
    const robot::iface::MapSnapshot snap = maps.active_snapshot();
    assert(snap.has_elevation());
    // Cell (3.0, 0.0) covers [3.0, 3.25): center height 0.1125.
    assert(std::abs(snap.elevation_at(3.0, 0.0) - 0.1125) < 0.003);
    assert(std::abs(snap.elevation_at(0.0, 0.0)) < 1e-9);  // flat floor
    assert(!snap.blocked(3.0, 0.0));  // terrain stays traversable unknown
  }

#ifdef LOST_SIGNAL_WITH_MUJOCO
  // Engine: ramp contact geometry matches the survey (raycast heights).
  {
    sim::world::World w = make_diff(0.0);
    w.set_terrain_patches(
        {{"ramp", 2.0, -1.0, 6.0, 1.0, 0.0, 0.1, 0.0, 0.0, 0.0}});
    w.step(0.02);  // builds the model
    double d = 0.0;
    assert(w.raycast(3.0, 0.0, 5.0, 0.0, 0.0, -1.0, 10.0, d));
    assert(std::abs(d - (5.0 - 0.1)) < 0.05);
    assert(w.raycast(5.0, 0.0, 5.0, 0.0, 0.0, -1.0, 10.0, d));
    assert(std::abs(d - (5.0 - 0.3)) < 0.05);
    // Slab deck: exact at the surveyed center.
    w.set_support_slabs({{"deck", 10.0, 0.0, 1.5, 1.0, 0.0, 0.3, 0.0}});
    w.step(0.02);
    assert(w.raycast(10.0, 0.0, 5.0, 0.0, 0.0, -1.0, 10.0, d));
    assert(std::abs(d - (5.0 - 0.3)) < 0.05);
  }

  // Material zones change the drive capability: ice travels shorter and
  // the slip label fires; the same duties on base ground stay quiet.
  {
    auto run = [](bool ice) {
      sim::world::World w = make_diff(1.0);
      if (ice) {
        w.set_materials({{"ice", 0.1, 0.005, 0.006, 1.0, 0.3}});
        w.set_material_zones({{"slick", -2.0, -2.0, 12.0, 2.0, "ice"}});
      }
      for (int i = 0; i < 100; ++i) w.step(0.02);
      return w;
    };
    sim::world::World base = run(false);
    sim::world::World slick = run(true);
    assert(slick.truth().body_x_m < base.truth().body_x_m);
    assert(slick.truth().drive_slipping[0] ||
           slick.truth().drive_slipping[1]);
    assert(!base.truth().drive_slipping[0] &&
           !base.truth().drive_slipping[1]);
  }

  // Fail-closed: roughness and bilinear cross-slope refuse the build
  // (frozen truth, never silently smoothed); unknown zone material too.
  {
    sim::world::World w = make_diff(0.5);
    w.set_terrain_patches(
        {{"rough", 2.0, -1.0, 6.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.05}});
    for (int i = 0; i < 10; ++i) w.step(0.02);
    assert(w.truth().forward_mps == 0.0);
    sim::world::World w2 = make_diff(0.5);
    w2.set_materials({{"ice", 0.25, 0.0, 0.006, 1.0, 0.3}});
    w2.set_material_zones({{"bad", 0.0, 0.0, 1.0, 1.0, "nope"}});
    for (int i = 0; i < 10; ++i) w2.step(0.02);
    assert(w2.truth().forward_mps == 0.0);
  }
#else
  // MuJoCo disabled: truth frozen, math above still pins the contract.
  {
    sim::world::World w = make_diff(1.0);
    w.step(0.02);
    assert(w.truth().forward_mps == 0.0);
  }
#endif

  std::printf("terrain ok\n");
  return 0;
}
