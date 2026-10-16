// Stop-line what-if replay (Phase-1, observe-only): record a full
// signal_hold/recommended run, derive the stop-line candidate from the ego
// trail (braking + margin before corridor entry), then recompute every
// recorded step twice — actual ego pose vs hypothetical hold at the
// candidate. Question answered with numbers: how many Occupied steps would
// have been Clear, and how does the margin distribution move? The mission
// is only ever read, never steered.
#include "mission/content_catalog.hpp"
#include "tests/new/shadow_field_helpers.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
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

struct RecordedStep {
  double time_s{0.0};
  sim::Vec2 ego{0, 0};
  double heading{0.0};
  double speed{0.0};
  sim::Vec2 track_pos{0, 0};
  sim::Vec2 track_vel{0, 0};
  double track_radius{0.0};
  double age{0.0};
};

struct WhatIf {
  long steps{0};
  long actual_occupied{0};
  long hypo_clear_given_occupied{0};
  double actual_min_clear_margin{1e9};
  double hypo_min_margin{1e9};
  bool finished{false};
};

WhatIf run_whatif(const mission::ContentCatalog& catalog, const std::string& level_id,
                  sim::Vec2& candidate_out) {
  WhatIf out;
  const auto build = catalog.presentation_for_level(level_id).recommended_build;
  const auto& map = catalog.map(catalog.level(level_id).map);
  std::vector<std::vector<sim::Vec2>> corridors;
  for (const auto& actor : map.actors) {
    const bool active =
      actor.active_in.empty() ||
      std::find(actor.active_in.begin(), actor.active_in.end(), level_id) != actor.active_in.end();
    if (active && actor.path.size() >= 2) corridors.push_back(actor.path);
  }
  if (corridors.empty()) {
    std::cerr << "no threat corridors for " << level_id << "\n";
    std::abort();
  }
  mission::MissionSimulation mission(catalog, level_id, build);
  std::vector<RecordedStep> stream;
  std::vector<sim::Vec2> trail;
  mission.start();
  int steps = 0;
  while (mission.snapshot().status == mission::MissionStatus::Running && steps < 60000) {
    mission.step();
    ++steps;
    const auto& snap = mission.snapshot();
    trail.push_back({snap.robot.x, snap.robot.y});
    for (const auto& track : snap.dynamic_obstacles) {
      const double age = snap.time_s - track.last_observed_s;
      if (age < 0.0 || age > sim::kEntryStaleBudgetS) continue;
      RecordedStep rec;
      rec.time_s = snap.time_s;
      rec.ego = {snap.robot.x, snap.robot.y};
      rec.heading = snap.robot.theta;
      rec.speed = std::abs(snap.robot.v);
      rec.track_pos = {track.estimated_position.x, track.estimated_position.y};
      rec.track_vel = {track.estimated_velocity.x, track.estimated_velocity.y};
      rec.track_radius = track.radius_m;
      rec.age = age;
      stream.push_back(rec);
    }
  }
  out.finished = mission.snapshot().status == mission::MissionStatus::Succeeded;
  out.steps = static_cast<long>(stream.size());
  // Candidate: trail back (brake + margin) from first corridor entry.
  auto nearest_corridor = [&](sim::Vec2 p) {
    double nearest = 1e9;
    for (const auto& corridor : corridors)
      for (size_t i = 0; i + 1 < corridor.size(); ++i)
        nearest = std::min(nearest, dist_point_segment(p, corridor[i], corridor[i + 1]));
    return nearest;
  };
  size_t entry = trail.size();
  for (size_t i = 0; i < trail.size(); ++i) {
    if (nearest_corridor(trail[i]) < 0.9) {
      entry = i;
      break;
    }
  }
  if (entry >= trail.size()) return out; // ego never enters: nothing to hold.
  double walked = 0.0;
  size_t k = entry;
  while (k > 0 && walked < 1.48) { // v^2/2a + 1.0 for grip @ 2.0 m/s.
    walked += std::hypot(trail[k].x - trail[k - 1].x, trail[k].y - trail[k - 1].y);
    --k;
  }
  candidate_out = trail[k];
  // Recompute: actual pose vs hypothetical hold at the candidate with the
  // SAME speed (isolates the position effect: would being a braking distance
  // further back have cleared this step?). Stopped steps assess Uncertain in
  // both and drop out of the comparison naturally.
  long hypo_occupied_given_occupied = 0;
  for (const auto& rec : stream) {
    const auto actual = field::assess_pose_vs_track(rec.ego, rec.heading, rec.speed, rec.track_pos,
                                                    rec.track_vel, rec.track_radius, rec.age);
    if (!actual.assessed) continue;
    const auto hypo = field::assess_pose_vs_track(candidate_out, rec.heading, rec.speed,
                                                  rec.track_pos, rec.track_vel, rec.track_radius,
                                                  rec.age);
    if (!hypo.assessed) continue;
    if (actual.verdict == sim::EntryVerdict::kObservedClear)
      out.actual_min_clear_margin = std::min(out.actual_min_clear_margin, actual.margin_s);
    if (actual.verdict == sim::EntryVerdict::kObservedOccupied) {
      ++out.actual_occupied;
      if (hypo.verdict == sim::EntryVerdict::kObservedClear) {
        ++out.hypo_clear_given_occupied;
      } else if (hypo.verdict == sim::EntryVerdict::kObservedOccupied) {
        ++hypo_occupied_given_occupied;
      }
    }
    if (hypo.verdict == sim::EntryVerdict::kObservedClear)
      out.hypo_min_margin = std::min(out.hypo_min_margin, hypo.margin_s);
  }
  return out;
}

} // namespace

int main() {
  const auto catalog = mission::ContentCatalog::load(LOST_SIGNAL_TEST_ASSETS);
  sim::Vec2 candidate{0, 0};
  const auto r1 = run_whatif(catalog, "signal_hold", candidate);
  std::cerr << "hold: steps=" << r1.steps << " actual_occupied=" << r1.actual_occupied
            << " hypo_clear_given_occupied=" << r1.hypo_clear_given_occupied
            << " actual_min_clear_margin=" << r1.actual_min_clear_margin
            << " hypo_min_margin=" << r1.hypo_min_margin << " candidate=(" << candidate.x << ","
            << candidate.y << ")\n";
  CHECK(r1.finished);
  CHECK(r1.actual_occupied > 0);
  std::cerr << "conversion=" << r1.hypo_clear_given_occupied << "/" << r1.actual_occupied << "\n";
  sim::Vec2 candidate2{0, 0};
  const auto r2 = run_whatif(catalog, "signal_hold", candidate2);
  CHECK(r2.steps == r1.steps && r2.actual_occupied == r1.actual_occupied &&
        r2.hypo_clear_given_occupied == r1.hypo_clear_given_occupied);
  CHECK(candidate2.x == candidate.x && candidate2.y == candidate.y);
  std::cout << "stop whatif green\n";
  return 0;
}
