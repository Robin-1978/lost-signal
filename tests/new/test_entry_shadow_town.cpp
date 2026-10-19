// Entry-window shadow over chapter-3 jobs (Phase-1, observe-only):
// town_freight + big_freight recommended carry carts and pedestrians the
// shadow has never looked at. Same reader as the cross/hold replays, new
// ground: verdict streams + determinism per level. Pins stay structural
// (assessed>0, replay-identical); the distributions are evidence, not gates.
#include "sim/game/mission/content_catalog.hpp"
#include "tests/new/shadow_field_helpers.hpp"

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

struct TownStream {
  long assessed{0};
  long clear{0};
  long occupied{0};
  long uncertain{0};
 long free_space_evidence{0};
 double maximum_observation_coverage{0.0};
};

TownStream run_level(mission::ScenarioSession& mission) {
  TownStream stream;
  mission.start();
  int steps = 0;
  while (mission.snapshot().status == mission::MissionStatus::Running && steps < 60000) {
    mission.step();
    ++steps;
    const auto& snap = mission.snapshot();
 if (snap.control.crossing_free_space_evidence_available)
 ++stream.free_space_evidence;
 stream.maximum_observation_coverage = std::max(
 stream.maximum_observation_coverage,
 snap.control.crossing_observation_coverage);
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
      if (a.verdict == sim::EntryVerdict::kObservedClear) ++stream.clear;
      else if (a.verdict == sim::EntryVerdict::kObservedOccupied) ++stream.occupied;
      else ++stream.uncertain;
    }
  }
  return stream;
}

} // namespace

int main() {
  const auto catalog = mission::ContentCatalog::load(LOST_SIGNAL_TEST_ASSETS);
  for (const auto* level : {"town_freight", "big_freight"}) {
    const auto build = catalog.presentation_for_level(level).recommended_build;
    mission::ScenarioSession first(catalog, level, build);
    const auto s1 = run_level(first);
    std::cerr << level << ": status=" << static_cast<int>(first.snapshot().status)
              << " assessed=" << s1.assessed << " clear=" << s1.clear << " occupied=" << s1.occupied
              << " uncertain=" << s1.uncertain
              << " min_dyn_clear=" << first.result().min_dynamic_clearance_m
              << " ped_contacts=" << first.result().pedestrian_contacts << "\n";
    CHECK(s1.assessed > 0);
    mission::ScenarioSession second(catalog, level, build);
    const auto s2 = run_level(second);
    CHECK(s2.assessed == s1.assessed && s2.clear == s1.clear && s2.occupied == s1.occupied &&
 s2.uncertain == s1.uncertain &&
 s2.free_space_evidence == s1.free_space_evidence &&
 s2.maximum_observation_coverage == s1.maximum_observation_coverage);
  }
  // P0 exit gate: town_freight must complete without timeout or collision.
 // Track loss remains unknown. The recommended 12 m LiDAR must instead
 // observe every authored traffic approach clear, with enough remaining
 // unseen-arrival time for the executable crossing trajectory.
  {
   const auto build = catalog.presentation_for_level("town_freight").recommended_build;
   mission::ScenarioSession mission(catalog, "town_freight", build);
 const auto stream = run_level(mission);
   CHECK(mission.result().status == mission::MissionStatus::Succeeded);
   CHECK(mission.result().collisions == 0);
 CHECK(mission.result().time_s <= catalog.level("town_freight").objective.time_limit_s);
 CHECK(mission.result().min_dynamic_clearance_m >= 0.5);
 CHECK(stream.free_space_evidence > 0);
 CHECK(stream.maximum_observation_coverage == 1.0);
  }
  std::cout << "entry shadow town green\n";
  return 0;
}
