// Entry-window shadow, field replay (Phase-1, observe-only): step the real
// mission on signal_cross/recommended, feed live tracks + ego state into the
// pure assess_entry_window every step, and compare the shadow verdict stream
// against ground truth (min dynamic clearance, collisions). The mission and
// DWA consume nothing — this test only reads snapshots. If the window check
// is the right medicine for flagship myopia, Occupied episodes must precede
// close encounters and flip to Clear as the threat passes.
#include "game/content_catalog.hpp"
#include "tests/new/shadow_field_helpers.hpp"

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
    for (const auto& track : snap.dynamic_obstacles) {
      const double age = snap.time_s - track.last_observed_s;
      const sim::Vec2 tp{track.estimated_position.x, track.estimated_position.y};
      const sim::Vec2 tv{track.estimated_velocity.x, track.estimated_velocity.y};
      const auto a = field::assess_pose_vs_track(ego, snap.robot.theta, ego_speed, tp, tv,
                                                 track.radius_m, age);
      if (!a.assessed) continue;
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
