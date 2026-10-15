// Entry-window shadow, field replay (Phase-1, observe-only): step the real
// mission on signal_cross/recommended, feed live tracks + ego state into the
// pure assess_entry_window every step, and compare the shadow verdict stream
// against ground truth (min dynamic clearance, collisions). The mission and
// DWA consume nothing — this test only reads snapshots. If the window check
// is the right medicine for flagship myopia, Occupied episodes must precede
// close encounters and flip to Clear as the threat passes.
#include "algo/entry_shadow.hpp"
#include "game/content_catalog.hpp"
#include "game/mission_simulation.hpp"

#include <cmath>
#include <iostream>

#define CHECK(condition)                                                                                     \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n";                            \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

namespace {

struct ShadowStream {
  long assessed{0};
  long clear{0};
  long occupied{0};
  long uncertain{0};
  double first_occupied_s{-1.0};
  double last_occupied_s{-1.0};
  double first_clear_after_occupied_s{-1.0};
  double min_clear_margin_s{1e9};
};

// Ray-ray conflict: ego forward ray vs threat velocity ray, clamped to
// lookaheads. Parallel/degenrate -> midpoint fallback. Pure geometry on
// snapshot data; no content invented.
sim::Vec2 conflict_center(sim::Vec2 ego, sim::Vec2 ego_dir, sim::Vec2 threat, sim::Vec2 threat_vel) {
  const double tv = std::hypot(threat_vel.x, threat_vel.y);
  sim::Vec2 center{(ego.x + threat.x) * 0.5, (ego.y + threat.y) * 0.5};
  if (tv < 1e-6) return center;
  const sim::Vec2 v{threat_vel.x / tv, threat_vel.y / tv};
  const double denom = ego_dir.x * v.y - ego_dir.y * v.x;
  if (std::abs(denom) < 1e-6) return center;
  const double dx = threat.x - ego.x;
  const double dy = threat.y - ego.y;
  const double t = (dx * v.y - dy * v.x) / denom;
  const double s = (dx * ego_dir.y - dy * ego_dir.x) / denom;
  const double tc = std::max(0.0, std::min(t, 10.0));
  const double sc = std::max(0.0, std::min(s, 10.0));
  center = {(ego.x + ego_dir.x * tc + threat.x + v.x * sc) * 0.5,
            (ego.y + ego_dir.y * tc + threat.y + v.y * sc) * 0.5};
  return center;
}

ShadowStream run_shadow(game::MissionSimulation& mission) {
  ShadowStream stream;
  mission.start();
  int steps = 0;
  while (mission.snapshot().status == game::MissionStatus::Running && steps < 30000) {
    mission.step();
    ++steps;
    const auto& snap = mission.snapshot();
    const double ego_speed = std::abs(snap.robot.v);
    const sim::Vec2 ego{snap.robot.x, snap.robot.y};
    const sim::Vec2 dir{std::cos(snap.robot.theta), std::sin(snap.robot.theta)};
    for (const auto& track : snap.dynamic_obstacles) {
      const double age = snap.time_s - track.last_observed_s;
      if (age < 0.0 || age > sim::kEntryStaleBudgetS) continue;
      const sim::Vec2 tp{track.estimated_position.x, track.estimated_position.y};
      const sim::Vec2 tv{track.estimated_velocity.x, track.estimated_velocity.y};
      const auto center = conflict_center(ego, dir, tp, tv);
      const double half = 0.45 + track.radius_m + 0.2;
      if (std::hypot(center.x - ego.x, center.y - ego.y) > 10.0) continue;
      sim::WindowThreat threat;
      threat.valid = true;
      threat.dist_to_zone_m = std::hypot(tp.x - center.x, tp.y - center.y) - half;
      const double tvn = std::hypot(tv.x, tv.y);
      const double toward = (tv.x * (center.x - tp.x) + tv.y * (center.y - tp.y)) /
                            std::max(1e-6, std::hypot(center.x - tp.x, center.y - tp.y));
      threat.speed_mps = (toward > 0.0) ? tvn : 0.0;
      threat.age_s = age;
      const double ego_dist = std::max(0.0, std::hypot(center.x - ego.x, center.y - ego.y) - half);
      const auto a =
        sim::assess_entry_window(ego_dist, ego_speed, half, threat, sim::kEntryStaleBudgetS);
      ++stream.assessed;
      if (a.verdict == sim::EntryVerdict::kObservedClear) {
        ++stream.clear;
        if (stream.occupied > 0 && stream.first_clear_after_occupied_s < 0.0)
          stream.first_clear_after_occupied_s = snap.time_s;
        if (a.margin_s < stream.min_clear_margin_s) stream.min_clear_margin_s = a.margin_s;
      } else if (a.verdict == sim::EntryVerdict::kObservedOccupied) {
        ++stream.occupied;
        if (stream.first_occupied_s < 0.0) stream.first_occupied_s = snap.time_s;
        stream.last_occupied_s = snap.time_s;
      } else {
        ++stream.uncertain;
      }
    }
  }
  return stream;
}

} // namespace

int main() {
  const auto catalog = game::ContentCatalog::load(LOST_SIGNAL_TEST_ASSETS);
  const auto build = catalog.presentation_for_level("signal_cross").recommended_build;
  game::MissionSimulation first(catalog, "signal_cross", build);
  const auto s1 = run_shadow(first);
  CHECK(first.snapshot().status == game::MissionStatus::Succeeded);
  std::cerr << "assessed=" << s1.assessed << " clear=" << s1.clear << " occupied=" << s1.occupied
            << " uncertain=" << s1.uncertain << " first_occ=" << s1.first_occupied_s
            << " last_occ=" << s1.last_occupied_s
            << " clear_after_occ=" << s1.first_clear_after_occupied_s
            << " min_clear_margin=" << s1.min_clear_margin_s
            << " min_dyn_clear=" << first.result().min_dynamic_clearance_m
            << " collisions=" << first.result().collisions << "\n";
  // The shadow must see the encounter: tracks exist and the window fires.
  CHECK(s1.assessed > 0);
  CHECK(s1.occupied > 0);
  // Determinism: same content/seed replays the identical verdict stream.
  game::MissionSimulation second(catalog, "signal_cross", build);
  const auto s2 = run_shadow(second);
  CHECK(s2.assessed == s1.assessed && s2.clear == s1.clear && s2.occupied == s1.occupied &&
        s2.uncertain == s1.uncertain);
  CHECK(s2.first_occupied_s == s1.first_occupied_s);
  CHECK(second.result().min_dynamic_clearance_m == first.result().min_dynamic_clearance_m);
  std::cout << "entry shadow field green\n";
  return 0;
}
