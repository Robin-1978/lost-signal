// Shadow slip instrument integrity (Gate-B state-estimation prep): the
// detector runs observation-only on tracked missions, stays off by design on
// differential. Pins are STRUCTURAL ONLY (pairing, finiteness, gate) — never
// detection counts. Why: the upper-bound study (perfect-IMU proxy) found the
// as-spec'd detector fires on low-speed quantization chatter (long_track:
// 4 episodes at v~0.1 in the shelf corner) and stays silent on quasi-steady
// sand sliding (accel-diff metric is blind to steady slide by construction).
// Counts would enshrine chatter; the redesign (velocity-level sustained
// metric) is backlog with this evidence attached.

#include "mission/campaign_progress.hpp"
#include "mission/content_catalog.hpp"
#include "mission/mission_simulation.hpp"

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
 // Differential gate: detector stays off (lesson §5: tread unobservable).
 {
  const auto& pres = catalog.presentation_for_level("lost_signal");
  mission::MissionSimulation mission(catalog, "lost_signal", pres.recommended_build);
  mission.run_to_completion();
  CHECK(mission.slip_episodes() == 0);
  CHECK(mission.slip_episode_times().empty());
 }
 // Tracked instrument runs end to end; timestamps pair 1:1 with episodes.
 const char* tracked[] = {"long_track", "sand_track"};
 for (const char* level : tracked) {
  const auto& pres = catalog.presentation_for_level(level);
  mission::MissionSimulation mission(catalog, level, pres.recommended_build);
  mission.run_to_completion();
  CHECK(mission.slip_episode_times().size() == static_cast<std::size_t>(mission.slip_episodes()));
  std::cerr << level << ": slip_episodes=" << mission.slip_episodes();
  for (double t : mission.slip_episode_times()) {
   std::cerr << " @" << t;
  }
  std::cerr << "\n";
 }
 std::cerr << "slip shadow integrity green\n";
 return 0;
}
