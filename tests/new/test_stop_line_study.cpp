// Stop-line placement study (Phase-1, observe-only): where does DWA
// actually wait on the encounter levels, and is that inside a threat swept
// corridor (threat path +/- radii)? The pocket lesson claims DWA parks
// inside swept zones on the hold levels; signal_cross/recommended already
// showed clean waiting (14 steps, 0 inside) — the pathology is
// level-dependent, so this harness reports per-level instead of asserting
// one shape. Nothing consumes the result.
#include "sim/game/mission/content_catalog.hpp"
#include "sim/game/scenario/scenario_session.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#define CHECK(condition)                                                                                     \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n";                            \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

namespace {

double dist_point_segment(sim::Vec2 p, sim::Vec2 a, sim::Vec2 b) {
  const double dx = b.x - a.x;
  const double dy = b.y - a.y;
  const double len2 = dx * dx + dy * dy;
  double t = (len2 > 0.0) ? ((p.x - a.x) * dx + (p.y - a.y) * dy) / len2 : 0.0;
  t = std::max(0.0, std::min(1.0, t));
  return std::hypot(p.x - (a.x + t * dx), p.y - (a.y + t * dy));
}

struct LevelStudy {
  std::string level;
  long wait_steps{0};
  long wait_inside_swept{0};
  bool finished{false};
};

LevelStudy run_level(const mission::ContentCatalog& catalog, const std::string& level_id) {
  LevelStudy out;
  out.level = level_id;
  const auto build = catalog.presentation_for_level(level_id).recommended_build;
  const auto& map = catalog.map(catalog.level(level_id).map);
  std::vector<std::vector<sim::Vec2>> corridors;
  for (const auto& actor : map.actors) {
    const bool active =
      actor.active_in.empty() ||
      std::find(actor.active_in.begin(), actor.active_in.end(), level_id) != actor.active_in.end();
    if (active && actor.path.size() >= 2) corridors.push_back(actor.path);
  }
  mission::ScenarioSession mission(catalog, level_id, build);
  mission.start();
  int steps = 0;
  while (mission.snapshot().status == mission::MissionStatus::Running && steps < 60000) {
    mission.step();
    ++steps;
    const auto& snap = mission.snapshot();
    if (!snap.waiting_for_clearance) continue;
    ++out.wait_steps;
    const sim::Vec2 ego{snap.robot.x, snap.robot.y};
    for (const auto& corridor : corridors) {
      double nearest = 1e9;
      for (size_t i = 0; i + 1 < corridor.size(); ++i)
        nearest = std::min(nearest, dist_point_segment(ego, corridor[i], corridor[i + 1]));
      if (nearest < 0.9) {
        ++out.wait_inside_swept;
        break;
      }
    }
  }
  out.finished = mission.snapshot().status == mission::MissionStatus::Succeeded;
  return out;
}

} // namespace

int main() {
  const auto catalog = mission::ContentCatalog::load(LOST_SIGNAL_TEST_ASSETS);
  const std::vector<std::string> levels = {"signal_cross", "signal_hold", "signal_watch"};
  std::vector<LevelStudy> first_pass;
  for (const auto& level : levels) {
    first_pass.push_back(run_level(catalog, level));
    const auto& r = first_pass.back();
    std::cerr << r.level << ": finished=" << r.finished << " wait_steps=" << r.wait_steps
              << " inside_swept=" << r.wait_inside_swept << "\n";
  }
  // Hold/watch are the documented wait levels: waiting must exist.
  CHECK(first_pass[1].wait_steps > 0);
  CHECK(first_pass[2].wait_steps > 0);
  // Determinism: identical wait/inside counts on replay.
  for (size_t i = 0; i < levels.size(); ++i) {
    const auto replay = run_level(catalog, levels[i]);
    CHECK(replay.wait_steps == first_pass[i].wait_steps);
    CHECK(replay.wait_inside_swept == first_pass[i].wait_inside_swept);
    CHECK(replay.finished == first_pass[i].finished);
  }
  std::cout << "stop line study green\n";
  return 0;
}
