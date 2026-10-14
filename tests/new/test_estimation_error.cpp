// Tracker estimation error vs ground truth (Gate A calibration basis):
// joins replay dynamic_obstacles (estimates) with actors (truth) by id and
// reports velocity/position error statistics, stratified by truth range.
// Evidence only — no bounds asserted (bounds would BE calibration, which is
// a separate, risk-decided slice). Deterministic: same seed, same numbers.

#include "game/campaign_progress.hpp"
#include "game/content_catalog.hpp"
#include "game/mission_simulation.hpp"

#include <cmath>
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

} // namespace

int main() {
 const auto catalog = game::ContentCatalog::load(LOST_SIGNAL_TEST_ASSETS);
 const auto build = catalog.presentation_for_level("lost_signal").recommended_build;
 game::MissionSimulation mission(catalog, "lost_signal", build);
 mission.run_to_completion();
 CHECK(mission.result().status == game::MissionStatus::Succeeded);

 ErrorStats near;  // truth range < 3 m: decision-relevant.
 ErrorStats far;
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
   if (range < 3.0) {
    near.add(abs_vel, signed_speed, abs_pos, age);
   } else {
    far.add(abs_vel, signed_speed, abs_pos, age);
   }
  }
 }
 CHECK(near.n > 0);
 CHECK(far.n > 0);
 report("est-error near(<3m)", near);
 report("est-error far(>=3m)", far);
 // Coverage, not calibration: single level, single pinned seed, mature
 // tracks only (estimates exist only after confirmation). These numbers
 // ground future perturbation amplitudes; they are not a standard.
 std::cerr << "estimation-error evidence recorded\n";
 return 0;
}
