// Player hardware extension proof (DIY contract, headless): a third-party
// tire authored as JSON only (no C++ changes) flows through loader ->
// validator -> garage budget -> mission to completion. The overlay copies the
// shipped tree (player packs never edit shipped files), adds one file, and
// the catalog treats it identically. Algorithm components still need C++
// (factory seams documented in docs/diy.md) — hardware is data, algorithms
// are code, and this test pins exactly that boundary.

#include "game/campaign_progress.hpp"
#include "game/content_catalog.hpp"
#include "game/mission_simulation.hpp"

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

const char kPlayerTire[] = R"({
  "schema_version": 1,
  "slot": "tires",
  "components": [
    {
      "id": "tires.player_pro",
      "title": "Player Pro Tire",
      "description": "Third-party tire, standard-like behavior, authored without touching shipped files.",
      "cost": 1,
      "behavior": {
        "slip_scale": 1.0,
        "brake_decel_mps2": 3.0,
        "power_scale": 1.0,
        "friction_mu": 0.7,
        "rolling_resistance": 0.01
      },
      "visual": {
        "primitive_2d": "wheel",
        "size_m": [0.2, 0.1, 0.12],
        "color": "#202D32",
        "model_3d": "models/robots/ro1/wheel_standard.glb"
      }
    }
  ]
}
)";

int main() {
 // Overlay: copy the shipped tree once (player packs are additive).
 const fs::path overlay = "/tmp/opencode/player_pack_assets";
 std::error_code error;
 fs::remove_all(overlay, error);
 fs::copy(LOST_SIGNAL_TEST_ASSETS, overlay,
          fs::copy_options::recursive | fs::copy_options::overwrite_existing, error);
 CHECK(!error);
 {
  std::ofstream out(overlay / "components" / "player_tires.json", std::ios::trunc);
  CHECK(static_cast<bool>(out));
  out << kPlayerTire;
 }
 const auto catalog = game::ContentCatalog::load(overlay.string());
 // The player tire is a first-class component (loader + validator accept it).
 CHECK(catalog.components().count("tires.player_pro") == 1);
 CHECK(catalog.component("tires.player_pro").slot == "tires");
 // It passes the garage gate (budget/cost/slots) and drives a mission.
 auto build = catalog.presentation_for_level("boot").recommended_build;
 build.parts["tires"] = "tires.player_pro";
 game::MissionSimulation mission(catalog, "boot", build);
 mission.run_to_completion();
 CHECK(mission.result().status == game::MissionStatus::Succeeded);
 std::cerr << "player pack green (tire drives boot to completion)\n";
 return 0;
}
