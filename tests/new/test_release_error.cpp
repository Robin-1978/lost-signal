// Release-time error vs entry stratification (Gate B DRAFT method prep):
// per closed wait-episode, predicted release (first_pred at wait start) vs
// actual release (waited), signed error = waited - first_pred (+ = held
// longer than predicted, - = released early), stratified by fastest threat
// speed at wait start. Episodes opening with inf prediction carry no error
// (unassessable start) and are counted separately. Evidence only.

#include "sim/game/mission/campaign_progress.hpp"
#include "sim/game/mission/content_catalog.hpp"
#include "sim/game/scenario/scenario_session.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <limits>

#define CHECK(condition)                                                                                     \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n";                           \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

namespace {

int speed_bin(double speed_mps) {
 if (speed_mps < 0.5) {
  return 0;
 }
 if (speed_mps < 1.5) {
  return 1;
 }
 return 2;
}

struct Bin {
 int n{0};
 double sum_err{0.0};
 double max_over{0.0};   // waited longer than predicted (conservative side).
 double max_under{0.0};  // released earlier than predicted (dangerous side).
 double sum_waited{0.0};
};

} // namespace

int main() {
 const auto catalog = mission::ContentCatalog::load(LOST_SIGNAL_TEST_ASSETS);
 const char* levels[2] = {"lost_signal", "signal_echo"};
 Bin bins[3];
 int episodes = 0;
 int inf_first = 0;
 int cells[2][2] = {{0, 0}, {0, 0}};
 double cell_err[2][2] = {{0.0, 0.0}, {0.0, 0.0}};
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
   for (const auto& episode : mission.shadow_episodes()) {
    ++episodes;
    const int dur = episode.waited_s < 1.0 ? 0 : 1;
    const int fin = episode.first_pred_s == std::numeric_limits<double>::infinity() ? 0 : 1;
    ++cells[dur][fin];
    if (fin != 0) {
     cell_err[dur][fin] += episode.waited_s - episode.first_pred_s;
    }
    if (episode.first_pred_s == std::numeric_limits<double>::infinity()) {
     ++inf_first;
     continue;
    }
    CHECK(episode.first_pred_s >= 0.0 && episode.waited_s >= 0.0);
    const double err = episode.waited_s - episode.first_pred_s;
    Bin& bin = bins[speed_bin(episode.entry_threat_speed_mps)];
    ++bin.n;
    bin.sum_err += err;
    bin.max_over = std::max(bin.max_over, err);
    bin.max_under = std::min(bin.max_under, err);
    bin.sum_waited += episode.waited_s;
   }
  }
 }
 CHECK(episodes > 0);
 const char* names[3] = {"slow(<0.5)", "mid(0.5-1.5)", "fast(>1.5)"};
 // Micro vs strategic waits: micro (<1 s, DWA creep domain) vs sustained.
 // Release-error is only meaningful where a prediction actually engaged AND
 // the wait lasted long enough to matter; micro-waits stay with DWA+creep
 // (already safe: zero collisions, clearance pinned) and are monitored, not
 // optimized. Conflating the two cells is what made the first readout
 // all-negative.
 std::cerr << "wait cells [micro/sustained]x[inf/finite]: [" << cells[0][0] << "," << cells[0][1]
           << "][" << cells[1][0] << "," << cells[1][1] << "]\n";
 for (int dur = 0; dur < 2; ++dur) {
  for (int fin = 0; fin < 2; ++fin) {
   if (fin != 0 && cells[dur][fin] > 0) {
    std::cerr << "cell[" << dur << "][1] err_mean=" << cell_err[dur][fin] / cells[dur][fin] << "\n";
   }
  }
 }
 for (int b = 0; b < 3; ++b) {
  if (bins[b].n == 0) {
   std::cerr << "release-error " << names[b] << ": no finite-first episodes\n";
   continue;
  }
  std::cerr << "release-error " << names[b] << ": n=" << bins[b].n
            << " err_mean=" << bins[b].sum_err / bins[b].n << " over_max=" << bins[b].max_over
            << " under_min=" << bins[b].max_under
            << " waited_mean=" << bins[b].sum_waited / bins[b].n << "\n";
 }
 std::cerr << "episodes=" << episodes << " inf_first=" << inf_first << "\n";
 std::cerr << "release-error evidence recorded\n";
 return 0;
}
