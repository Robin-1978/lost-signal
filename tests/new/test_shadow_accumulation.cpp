// Shadow accumulation (Gate B evidence, mission untouched): departure
// episodes across levels x seeds into the 2x2 wait cells. signal_hold brings
// sustained waits (the empty sustained-finite cell); return_route exercises
// the RO-2/Ackermann production path. Read-only accessors only.

#include "game/campaign_progress.hpp"
#include "game/content_catalog.hpp"
#include "game/mission_simulation.hpp"

#include <iostream>
#include <limits>

#define CHECK(condition)                                                                                     \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n";                            \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

int main() {
 const auto catalog = game::ContentCatalog::load(LOST_SIGNAL_TEST_ASSETS);
 // signal_hold (seed 42, sustained corridor blockage) is the level that fills
 // the sustained-finite cell empty in lost_signal/signal_echo. return_route
 // (RO-2) was probed and produces zero waits — no distribution to accumulate,
 // so it stays out rather than pin a brittle zero.
 const char* levels[1] = {"signal_hold"};
 int total_episodes = 0;
 int total_sustained_finite = 0;
 for (const char* level_id : levels) {
  const auto build = catalog.presentation_for_level(level_id).recommended_build;
  uint64_t base = 0;
  {
   game::MissionSimulation probe(catalog, level_id, build);
   base = probe.level().seed;
  }
  int cells[2][2] = {{0, 0}, {0, 0}};
  double finite_err_sum = 0.0;
  int finite_n = 0;
  for (int s = 0; s < 3; ++s) {
   game::MissionSimulation mission(catalog, level_id, build, base + static_cast<uint64_t>(s));
   mission.run_to_completion();
   int seed_cells[2][2] = {{0, 0}, {0, 0}};
   for (const auto& episode : mission.shadow_episodes()) {
    ++total_episodes;
    CHECK(episode.start_s <= episode.end_s);
    CHECK(episode.waited_s >= 0.0);
    const int dur = episode.waited_s < 1.0 ? 0 : 1;
    const int fin = episode.first_pred_s == std::numeric_limits<double>::infinity() ? 0 : 1;
    ++cells[dur][fin];
    ++seed_cells[dur][fin];
    if (fin != 0) {
     finite_err_sum += episode.waited_s - episode.first_pred_s;
     ++finite_n;
    }
   }
   // Per-seed decomposition (Gate B stability readout): totals hide
   // lucky-seed dependence — the sustained-finite cell must not rest on
   // one phase.
   std::cerr << "  seed+" << s << " [" << seed_cells[0][0] << "," << seed_cells[0][1] << "]["
             << seed_cells[1][0] << "," << seed_cells[1][1] << "]\n";
  }
  std::cerr << level_id << " cells [micro/sus]x[inf/fin] = [" << cells[0][0] << "," << cells[0][1]
            << "][" << cells[1][0] << "," << cells[1][1] << "]";
  double finite_mean = finite_n > 0 ? finite_err_sum / finite_n : 0.0;
  std::cerr << " finite_err_mean=" << finite_mean << "\n";
  // Pinned distribution (fixed seeds, deterministic): micro-inf 35,
  // micro-finite 5, sustained-inf 4, sustained-finite 1. The last cell is
  // the Gate-B-relevant one — empty in the first two levels, filled here.
  CHECK(cells[0][0] == 35 && cells[0][1] == 5);
  CHECK(cells[1][0] == 4 && cells[1][1] == 1);
  CHECK(finite_n == 6);
  CHECK(finite_mean > -0.5 && finite_mean < 0.5);
  total_sustained_finite += cells[1][1];
 }
 CHECK(total_sustained_finite >= 1);
 CHECK(total_episodes > 0);
 std::cerr << "shadow accumulation green\n";
 return 0;
}
