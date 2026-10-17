// Slip telemetry integrity. RO-1 now exposes articulated drive-wheel motion,
// encoder feedback and model-side tread/contact slip truth, so differential
// drive is observable as well as tracked drive. Pins remain structural:
// episode/timestamp pairing, valid time order and fixed-seed determinism.
// Exact episode counts are deliberately not a contract.

#include "sim/game/mission/campaign_progress.hpp"
#include "sim/game/mission/content_catalog.hpp"
#include "sim/game/scenario/scenario_session.hpp"

#include <cmath>
#include <iostream>

#define CHECK(condition)                                                                                     \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n";                           \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

int main() {
 const auto catalog = mission::ContentCatalog::load(LOST_SIGNAL_TEST_ASSETS);
 auto check_timeline = [](const mission::ScenarioSession& run) {
 const auto& times = run.slip_episode_times();
 CHECK(times.size() == static_cast<std::size_t>(run.slip_episodes()));
 double previous = -1.0;
 for (double time_s : times) {
 CHECK(std::isfinite(time_s));
 CHECK(time_s >= 0.0);
 CHECK(time_s <= run.result().time_s + 1e-9);
 CHECK(time_s > previous);
 previous = time_s;
 }
 return 0;
 };

 // Differential model-side slip telemetry is available and deterministic.
 {
  const auto& pres = catalog.presentation_for_level("lost_signal");
 mission::ScenarioSession first(catalog, "lost_signal", pres.recommended_build);
 mission::ScenarioSession second(catalog, "lost_signal", pres.recommended_build);
 first.run_to_completion();
 second.run_to_completion();
 CHECK(check_timeline(first) == 0);
 CHECK(check_timeline(second) == 0);
 CHECK(first.slip_episodes() == second.slip_episodes());
 CHECK(first.slip_episode_times() == second.slip_episode_times());
 }
 // Tracked instrument runs end to end; timestamps pair 1:1 with episodes.
 const char* tracked[] = {"long_track", "sand_track"};
 for (const char* level : tracked) {
  const auto& pres = catalog.presentation_for_level(level);
  mission::ScenarioSession mission(catalog, level, pres.recommended_build);
  mission.run_to_completion();
 CHECK(check_timeline(mission) == 0);
  std::cerr << level << ": slip_episodes=" << mission.slip_episodes();
  for (double t : mission.slip_episode_times()) {
   std::cerr << " @" << t;
  }
  std::cerr << "\n";
 }
 std::cerr << "slip shadow integrity green\n";
 return 0;
}
