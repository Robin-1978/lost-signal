#include "algo/entry_shadow.hpp"

#include <cmath>
#include <iostream>

#define CHECK(condition)                                                                                     \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n";                            \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

// Entry-window shadow contract (Phase-1, observe-only): the same geometry
// separates fast ego (clear) from slow ego (occupied) — the myopia case the
// current-distance check cannot see. No-data and stale-data never clear.
int main() {
  const double kHalf = 1.0; // 2 m conflict zone.
  // Crossing threat 8 m out at 0.55 m/s (town forklift class):
  // threat window [14.5, 18.2].
  sim::WindowThreat threat;
  threat.valid = true;
  threat.dist_to_zone_m = 8.0;
  threat.speed_mps = 0.55;
  threat.age_s = 0.05;
  // Fast ego: 8 m at 2.0 m/s -> transit [4.0, 5.0], clears 9 s before
  // the threat arrives.
  {
    const auto a = sim::assess_entry_window(8.0, 2.0, kHalf, threat);
    CHECK(a.verdict == sim::EntryVerdict::kObservedClear);
    CHECK(std::abs(a.ego_enter_s - 4.0) < 1e-9);
    CHECK(std::abs(a.ego_exit_s - 5.0) < 1e-9);
    CHECK(a.margin_s > 9.0);
  }
  // Slow ego, SAME geometry: 8 m at 0.5 m/s -> transit [16.0, 20.0];
  // threat [14.5, 18.2] overlaps -> occupied. Current-distance logic
  // says "go" in both cases; the window separates them. This is the unit.
  {
    const auto a = sim::assess_entry_window(8.0, 0.5, kHalf, threat);
    CHECK(a.verdict == sim::EntryVerdict::kObservedOccupied);
    CHECK(std::abs(a.ego_enter_s - 16.0) < 1e-9);
    CHECK(a.margin_s == 0.0);
  }
  // Threat already through: 6 m past the zone, still moving away.
  {
    sim::WindowThreat past = threat;
    past.dist_to_zone_m = -8.0; // 6 m past the 2 m zone.
    const auto a = sim::assess_entry_window(8.0, 0.5, kHalf, past);
    CHECK(a.verdict == sim::EntryVerdict::kObservedClear);
    CHECK(a.margin_s > 0.0);
  }
  // Stationary threat sitting inside the zone blocks every ego.
  {
    sim::WindowThreat sitting;
    sitting.valid = true;
    sitting.dist_to_zone_m = -0.5;
    sitting.speed_mps = 0.0;
    sitting.age_s = 0.0;
    const auto a = sim::assess_entry_window(8.0, 2.0, kHalf, sitting);
    CHECK(a.verdict == sim::EntryVerdict::kObservedOccupied);
  }
  // No-data and stale-data never default to go.
  {
    sim::WindowThreat none; // valid=false.
    CHECK(sim::assess_entry_window(8.0, 2.0, kHalf, none).verdict ==
          sim::EntryVerdict::kObservedUncertain);
    sim::WindowThreat stale = threat;
    stale.age_s = 0.31; // over the 0.30 s dropout budget.
    CHECK(sim::assess_entry_window(8.0, 2.0, kHalf, stale).verdict ==
          sim::EntryVerdict::kObservedUncertain);
  }
  // Degenerate ego inputs assess nothing (and never clear).
  {
    CHECK(sim::assess_entry_window(8.0, 0.0, kHalf, threat).verdict ==
          sim::EntryVerdict::kObservedUncertain);
    CHECK(sim::assess_entry_window(-1.0, 2.0, kHalf, threat).verdict ==
          sim::EntryVerdict::kObservedUncertain);
  }
  // Determinism: same inputs, bit-identical outputs.
  {
    const auto a = sim::assess_entry_window(8.0, 0.5, kHalf, threat);
    const auto b = sim::assess_entry_window(8.0, 0.5, kHalf, threat);
    CHECK(a.ego_enter_s == b.ego_enter_s && a.margin_s == b.margin_s &&
          a.verdict == b.verdict);
  }
  std::cout << "entry shadow green\n";
  return 0;
}
