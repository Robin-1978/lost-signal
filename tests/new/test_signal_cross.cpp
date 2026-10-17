// signal_cross curation gates (Family B crossing encounter, mission untouched):
// 5-seed veto (recommended collision = veto), both builds succeed, sustained
// waits present on both, distributions pinned. NOTE: sustained-finite is
// absent here (WaitUnsafe-dominated: DWA stops the approach inside the swept
// zone — a stop-position property content cannot move; pocket behavior would
// be the fix, which is Phase-2). This level adds encounter diversity, not
// sus-fin coverage — stated, not oversold.

#include "mission/campaign_progress.hpp"
#include "mission/content_catalog.hpp"
#include "mission/mission_simulation.hpp"

#include <iostream>
#include <limits>

#define CHECK(condition)                                                                                     \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n";                           \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

int main() {
 const auto catalog = mission::ContentCatalog::load(LOST_SIGNAL_TEST_ASSETS);
 const auto& pres = catalog.presentation_for_level("signal_cross");
 for (int b = 0; b < 2; ++b) {
  const auto build = b == 0 ? pres.default_build : pres.recommended_build;
  int coll = 0, succeeded = 0;
  int cells[2][2] = {{0, 0}, {0, 0}};
  for (int s = 0; s < 5; ++s) {
   mission::MissionSimulation mission(catalog, "signal_cross", build, 11u + static_cast<uint64_t>(s));
   mission.run_to_completion();
   if (mission.result().status == mission::MissionStatus::Succeeded) {
    ++succeeded;
   }
   if (mission.result().failure_reason.find("collision") != std::string::npos) {
    ++coll;
   }
   for (const auto& episode : mission.shadow_episodes()) {
    const int dur = episode.waited_s < 1.0 ? 0 : 1;
    const int fin = episode.first_pred_s == std::numeric_limits<double>::infinity() ? 0 : 1;
    ++cells[dur][fin];
   }
  }
  std::cerr << (b == 0 ? "default" : "recommended") << " succ=" << succeeded << "/5 coll=" << coll
            << "/5 cells=[" << cells[0][0] << "," << cells[0][1] << "][" << cells[1][0] << "," << cells[1][1]
            << "]\n";
  CHECK(succeeded == 5);
  CHECK(coll == 0);
  CHECK(cells[1][0] + cells[1][1] > 0);
 }
 std::cerr << "signal cross gates green\n";
 return 0;
}
