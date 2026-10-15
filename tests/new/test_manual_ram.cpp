// Manual ramming covers the dynamic-collision path (overlay tree, shipped
// files untouched): autonomous levels ignore stick input by design
// (set_manual_command is a no-op outside manual_calibration), and the entry
// governor now saves the hot build — so the only remaining driver of
// collision-dynamic is the human hand. Park on the patrol segment and let
// ping-pong deliver the mover; the governor must never fire here (manual
// bypasses decide_control: the driver outranks strategy by construction).
#include "game/content_catalog.hpp"
#include "game/mission_simulation.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>

#define CHECK(condition)                                                                                     \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n";                            \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

namespace fs = std::filesystem;

const char kRamMap[] = R"({
  "schema_version": 1,
  "id": "test_ram_map",
  "scene_3d": "scene.research_base",
  "size_m": [12.0, 12.0],
  "grid_resolution_m": 0.5,
  "obstacles": [],
  "surfaces": [],
  "markers": {
    "ram_start": [1.0, 1.0, 0.0],
    "ram_goal": [11.0, 11.0, 0.0]
  },
  "actors": [
    {
      "id": "ram_cart",
      "type": "patrol",
      "path": [[5.0, 5.0], [9.0, 5.0]],
      "speed_mps": 0.5,
      "radius_m": 0.45,
      "active_in": ["test_ram"],
      "collision": "fatal",
      "visual": {"model": "models/actors/forklift.glb", "ref_radius_m": 0.45}
    }
  ]
})";

const char kRamLevel[] = R"({
  "schema_version": 1,
  "id": "test_ram",
  "order": 99,
  "title": "Test Ram",
  "map": "test_ram_map",
  "control_mode": "manual_calibration",
  "localization_mode": "odometry",
  "chapter": 1,
  "prerequisites": [],
  "seed": 7,
  "objective": {
    "type": "reach",
    "spawn": "ram_start",
    "goal": "ram_goal",
    "goal_label": "Test Goal",
    "goal_visual": "exit",
    "time_limit_s": 120,
    "max_collisions": 0,
    "arrive_tolerance_m": 0.62
  },
  "loadout_rules": {"budget": 4, "required_slots": ["tires"]},
  "teaching": {"concept": "test", "prediction_prompt": "t", "knowledge_card": "t"},
  "score": {"safe_min_clearance_m": 0.3, "fast_time_s": 100.0, "efficient_energy_wh": 10.0},
  "rewards": ["ability.basic_motion"]
})";

int main() {
 const fs::path overlay = "/tmp/opencode/ram_assets";
 std::error_code error;
 fs::remove_all(overlay, error);
 fs::copy(LOST_SIGNAL_TEST_ASSETS, overlay,
          fs::copy_options::recursive | fs::copy_options::overwrite_existing, error);
 CHECK(!error);
 {
  std::ofstream out(overlay / "maps" / "test_ram_map.json", std::ios::trunc);
  CHECK(static_cast<bool>(out));
  out << kRamMap;
 }
 {
  std::ofstream out(overlay / "levels" / "test_ram.json", std::ios::trunc);
  CHECK(static_cast<bool>(out));
  out << kRamLevel;
 }
 {
  std::ifstream in(overlay / "presentation" / "boot.json");
  CHECK(static_cast<bool>(in));
  std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  std::string::size_type pos = text.find("\"presentation.boot\"");
  CHECK(pos != std::string::npos);
  text.replace(pos, std::string("\"presentation.boot\"").size(), "\"presentation.test_ram\"");
  pos = text.find("\"level\": \"boot\"");
  CHECK(pos != std::string::npos);
  text.replace(pos, std::string("\"level\": \"boot\"").size(), "\"level\": \"test_ram\"");
  pos = text.find("\"is_default\": true");
  CHECK(pos != std::string::npos);
  text.replace(pos, std::string("\"is_default\": true").size(), "\"is_default\": false");
  std::ofstream out(overlay / "presentation" / "test_ram.json", std::ios::trunc);
  CHECK(static_cast<bool>(out));
  out << text;
 }
 {
  std::ifstream in(overlay / "campaign.json");
  CHECK(static_cast<bool>(in));
  std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
  const auto levels_pos = text.find("\"levels\"");
  CHECK(levels_pos != std::string::npos);
  const auto open_pos = text.find("[", levels_pos);
  CHECK(open_pos != std::string::npos);
  const auto close_pos = text.find("]", open_pos);
  CHECK(close_pos != std::string::npos);
  text.insert(close_pos, ", \"test_ram\"");
  std::ofstream out(overlay / "campaign.json", std::ios::trunc);
  CHECK(static_cast<bool>(out));
  out << text;
 }
 const auto catalog = game::ContentCatalog::load(overlay.string());
 game::BuildSelection build;
 build.parts["tires"] = "tires.standard";
 build.style = 0.0;
 game::MissionSimulation ram(catalog, "test_ram", build, 7u);
 ram.start();
 // Drive at the patrol segment and stop on it (0.8 m < 0.9 m capture):
 // ping-pong guarantees the mover comes through. No parking precision
 // needed, no phase luck — the line itself is the trap.
 auto dist_to_seg = [](double px, double py) {
  const double t = std::max(0.0, std::min(1.0, (px - 5.0) / 4.0));
  return std::hypot(px - (5.0 + t * 4.0), py - 5.0);
 };
 bool parked = false;
 int steps = 0;
 while (ram.snapshot().status == game::MissionStatus::Running && steps < 6000) {
  const auto& snap = ram.snapshot();
  if (dist_to_seg(snap.robot.x, snap.robot.y) < 0.8) {
   parked = true;
   ram.set_manual_command(0.0, 0.0);
  } else {
   const double dx = 7.0 - snap.robot.x;
   const double dy = 5.0 - snap.robot.y;
   const double err = std::atan2(dy, dx) - snap.robot.theta;
   const double wrapped = std::atan2(std::sin(err), std::cos(err));
   ram.set_manual_command(0.5, std::clamp(3.0 * wrapped, -1.0, 1.0));
  }
  ram.step();
  ++steps;
 }
 std::cerr << "ram: parked=" << parked << " steps=" << steps << "\n";
 CHECK(parked);
 CHECK(ram.result().status == game::MissionStatus::Failed);
 CHECK(ram.result().failure_reason == "collision-dynamic");
 CHECK(ram.result().collisions >= 1);
 for (const auto& event : ram.events()) {
  CHECK(event.code != "governor_hold");
 }
 std::cout << "manual ram green\n";
 return 0;
}
