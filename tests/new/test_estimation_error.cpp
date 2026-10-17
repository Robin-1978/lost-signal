// Tracker estimation error vs ground truth (Gate A calibration basis):
// joins replay dynamic_obstacles (estimates) with actors (truth) by id and
// reports velocity/position error statistics, stratified by truth range.
// Evidence only — no bounds asserted (bounds would BE calibration, which is
// a separate, risk-decided slice). Deterministic: same seed, same numbers.

#include "mission/campaign_progress.hpp"
#include "mission/content_catalog.hpp"
#include "scenario/scenario_session.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <limits>
#include <map>
#include <string>

#define CHECK(condition)                                                                                     \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n";                           \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

namespace {

struct ErrorStats {
 int n{0};
 double sum_abs_vel{0.0};
 double sum_signed_speed{0.0};
 double max_abs_vel{0.0};
 double sum_abs_pos{0.0};
 double max_abs_pos{0.0};
 double sum_age{0.0};
 double max_age{0.0};
 void add(double abs_vel, double signed_speed, double abs_pos, double age) {
  ++n;
  sum_abs_vel += abs_vel;
  sum_signed_speed += signed_speed;
  max_abs_vel = std::max(max_abs_vel, abs_vel);
  sum_abs_pos += abs_pos;
  max_abs_pos = std::max(max_abs_pos, abs_pos);
  sum_age += age;
  max_age = std::max(max_age, age);
 }
};

void report(const char* label, const ErrorStats& s) {
 if (s.n == 0) {
  std::cerr << label << ": no samples\n";
  return;
 }
 std::cerr << label << ": n=" << s.n << " vel_abs_mean=" << s.sum_abs_vel / s.n
           << " vel_signed_mean=" << s.sum_signed_speed / s.n << " vel_abs_max=" << s.max_abs_vel
           << " pos_abs_mean=" << s.sum_abs_pos / s.n << " pos_abs_max=" << s.max_abs_pos
           << " age_mean=" << s.sum_age / s.n << " age_max=" << s.max_age << "\n";
}

// Threat ground-truth speed bins (Gate B DRAFT stratification; bins are
// reporting structure, not calibrated thresholds).
int speed_bin(double speed_mps) {
 if (speed_mps < 0.5) {
  return 0;
 }
 if (speed_mps < 1.5) {
  return 1;
 }
 return 2;
}

} // namespace

int main() {
 const auto catalog = mission::ContentCatalog::load(LOST_SIGNAL_TEST_ASSETS);
 // Multi-level, multi-seed accumulation (level.seed+0..2, mirroring
 // level_report --seeds): estimation evidence across encounters, not one
 // pinned trajectory. Exact values legitimately vary by seed (different
 // encounters), so asserts are structural; numbers are evidence for future
 // stratification, not a standard.
 const char* levels[2] = {"lost_signal", "signal_echo"};
 ErrorStats near[3];
 ErrorStats far[3];
 int runs = 0;
 for (const char* level_id : levels) {
  const auto build = catalog.presentation_for_level(level_id).recommended_build;
  uint64_t base = 0;
  {
   mission::ScenarioSession probe(catalog, level_id, build);
   base = probe.level().seed;
  }
  for (int s = 0; s < 3; ++s) {
   mission::ScenarioSession mission(catalog, level_id, build, base + static_cast<uint64_t>(s));
   mission.run_to_completion();
   ++runs;
   std::cerr << level_id << " seed+" << s << " status=" << static_cast<int>(mission.result().status)
             << "\n";
   for (const auto& snapshot : mission.replay()) {
    std::map<std::string, sim::Vec2> truth_vel;
    std::map<std::string, sim::Vec2> truth_pos;
    for (const auto& actor : snapshot.actors) {
     truth_vel[actor.id] = actor.velocity;
     truth_pos[actor.id] = actor.position;
    }
    for (const auto& obstacle : snapshot.dynamic_obstacles) {
     const auto velocity = truth_vel.find(obstacle.id);
     const auto position = truth_pos.find(obstacle.id);
     if (velocity == truth_vel.end() || position == truth_pos.end()) {
      continue;
     }
     const double abs_vel = std::hypot(obstacle.estimated_velocity.x - velocity->second.x,
                                       obstacle.estimated_velocity.y - velocity->second.y);
     const double signed_speed =
       std::hypot(obstacle.estimated_velocity.x, obstacle.estimated_velocity.y) -
       std::hypot(velocity->second.x, velocity->second.y);
     const double abs_pos = std::hypot(obstacle.estimated_position.x - position->second.x,
                                       obstacle.estimated_position.y - position->second.y);
     const double age = snapshot.time_s - obstacle.last_observed_s;
     CHECK(std::isfinite(abs_vel) && std::isfinite(abs_pos) && std::isfinite(age));
     const double range = std::hypot(position->second.x - snapshot.robot.x,
                                     position->second.y - snapshot.robot.y);
     const double true_speed =
       std::hypot(velocity->second.x, velocity->second.y);
     ErrorStats& cell = (range < 3.0 ? near : far)[speed_bin(true_speed)];
     cell.add(abs_vel, signed_speed, abs_pos, age);
    }
   }
  }
 }
 CHECK(runs == 6);
 const char* ranges[2] = {"near(<3m)", "far(>=3m)"};
 const char* bins[3] = {"slow(<0.5)", "mid(0.5-1.5)", "fast(>1.5)"};
 int total = 0;
 for (int r = 0; r < 2; ++r) {
  for (int b = 0; b < 3; ++b) {
   const ErrorStats& cell = (r == 0 ? near : far)[b];
   total += cell.n;
   char label[64];
   std::snprintf(label, sizeof(label), "est-error %s %s", ranges[r], bins[b]);
   report(label, cell);
  }
 }
 CHECK(total > 0);
 // Coverage, not calibration: two dynamic levels x three seeds, mature tracks
 // only. Numbers ground future perturbation amplitudes and speed
 // stratification; they are not a standard.
 std::cerr << "estimation-error evidence recorded over " << runs << " runs\n";
 return 0;
}
