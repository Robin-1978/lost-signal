// Shadow accumulation (Gate B evidence, mission untouched): departure
// episodes across levels x seeds into the 2x2 wait cells plus per-seed
// decomposition (totals hide lucky-seed dependence). signal_hold brings
// sustained waits; signal_watch shares its corridor (same encounter phase,
// correlated — recorded, not double-counted as independent). return_route
// (RO-2) was probed and produces zero waits — excluded, not pinned zero.
// Read-only accessors only.

#include "mission/campaign_progress.hpp"
#include "mission/content_catalog.hpp"
#include "scenario/scenario_session.hpp"

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

struct LevelPlan {
 const char* id;
 int seeds;
};

} // namespace

int main() {
 const auto catalog = mission::ContentCatalog::load(LOST_SIGNAL_TEST_ASSETS);
 // Counts are diagnostic rather than golden values: perception and tracking
 // changes legitimately split or merge adjacent wait episodes. The stable
 // contract is that both levels produce valid waits and the accumulated set
 // retains sustained waits as well as finite departure predictions.
 const LevelPlan plans[2] = {
   {"signal_hold", 6},
   {"signal_watch", 3},
 };
 int total_episodes = 0;
 int total_sustained = 0;
 int total_finite = 0;
 for (const auto& plan : plans) {
  const auto build = catalog.presentation_for_level(plan.id).recommended_build;
  uint64_t base = 0;
  {
   mission::ScenarioSession probe(catalog, plan.id, build);
   base = probe.level().seed;
  }
  int cells[2][2] = {{0, 0}, {0, 0}};
  double finite_err_sum = 0.0;
  int finite_n = 0;
  for (int s = 0; s < plan.seeds; ++s) {
   mission::ScenarioSession mission(catalog, plan.id, build, base + static_cast<uint64_t>(s));
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
   // Per-seed decomposition: the sustained-finite cell must not rest on one
   // phase without the record saying so.
   std::cerr << "  seed+" << s << " [" << seed_cells[0][0] << "," << seed_cells[0][1] << "]["
             << seed_cells[1][0] << "," << seed_cells[1][1] << "]\n";
  }
  const double finite_mean = finite_n > 0 ? finite_err_sum / finite_n : 0.0;
  std::cerr << plan.id << " cells [" << cells[0][0] << "," << cells[0][1] << "][" << cells[1][0] << ","
            << cells[1][1] << "] finite_err_mean=" << finite_mean << "\n";
  CHECK(cells[0][0] + cells[0][1] + cells[1][0] + cells[1][1] > 0);
  total_sustained += cells[1][0] + cells[1][1];
  total_finite += cells[0][1] + cells[1][1];
 }
 CHECK(total_episodes > 0);
 CHECK(total_sustained >= 1);
 CHECK(total_finite >= 1);
 std::cerr << "shadow accumulation green\n";
 return 0;
}
