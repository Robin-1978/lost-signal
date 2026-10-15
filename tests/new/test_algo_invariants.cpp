#include "algo/dynobs.hpp"
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

// Algorithm contracts (properties, not spot values): constant-speed loops
// close exactly, patrol mirrors, dwells account, seeds preserve length,
// and window touching counts as occupied. Success rates never appear here.
int main() {
  const std::vector<sim::Vec2> rect{{0, 0}, {4, 0}, {4, 3}, {0, 3}};
  // Wander: constant speed everywhere, exact period closure.
  {
    sim::Wander w{rect, 1.0, 0.25};
    for (const double t : {0.5, 1.7, 5.2, 13.9}) {
      const auto s = sim::wander_state(w, t);
      CHECK(std::abs(std::hypot(s.v.x, s.v.y) - 1.0) < 1e-9);
      CHECK(s.radius == 0.25);
    }
    const auto p0 = sim::wander_state(w, 0.0);
    for (const double t : {14.0, 28.0, 140.0}) {
      const auto s = sim::wander_state(w, t);
      CHECK(std::abs(s.p.x - p0.p.x) < 1e-9 && std::abs(s.p.y - p0.p.y) < 1e-9);
    }
  }
  // Patrol: mirror symmetry about the far end, velocity flips.
  {
    sim::Patrol pt{{0, 0}, {4, 0}, 2.0, 0.45};
    const auto there = sim::patrol_state(pt, 1.0);
    const auto back = sim::patrol_state(pt, 3.0);
    CHECK(there.p.x == 2.0 && back.p.x == 2.0);
    CHECK(there.v.x == 2.0 && back.v.x == -2.0);
    const auto start = sim::patrol_state(pt, 0.0);
    const auto period = sim::patrol_state(pt, 4.0);
    CHECK(start.p.x == period.p.x && start.p.y == period.p.y);
  }
  // Dwell: cycle = legs + waits; mid-wait velocity is exactly zero.
  {
    sim::Dwell d{rect, {2.0, 0.0, 0.0, 0.0}, 1.0, 0.25};
    const auto mid = sim::dwell_state(d, 1.0);
    CHECK(mid.p.x == 0.0 && mid.p.y == 0.0);
    CHECK(mid.v.x == 0.0 && mid.v.y == 0.0);
    const auto p0 = sim::dwell_state(d, 0.0);
    const auto cycled = sim::dwell_state(d, 16.0); // 14 m legs + 2 s wait.
    CHECK(std::abs(cycled.p.x - p0.p.x) < 1e-9 && std::abs(cycled.p.y - p0.p.y) < 1e-9);
  }
  // Seeded: rotation/reversal preserve the loop (same period closes).
  {
    for (const std::uint64_t seed : {0u, 1u, 7u, 33u, 1000u}) {
      sim::SeededWander w{rect, seed, 1.0, 0.25};
      const auto p0 = sim::seeded_wander_state(w, 0.0);
      const auto cycled = sim::seeded_wander_state(w, 14.0);
      CHECK(std::abs(cycled.p.x - p0.p.x) < 1e-9 && std::abs(cycled.p.y - p0.p.y) < 1e-9);
      const auto mid = sim::seeded_wander_state(w, 5.0);
      CHECK(std::abs(std::hypot(mid.v.x, mid.v.y) - 1.0) < 1e-9);
    }
  }
  // Window boundary: touching instants share the zone -> Occupied.
  // Ego [4,5] (8 m at 2 m/s, half 1); threat exits exactly at 4.0.
  {
    sim::WindowThreat touch;
    touch.valid = true;
    touch.dist_to_zone_m = 0.0;
    touch.speed_mps = 0.5;
    touch.age_s = 0.0;
    const auto a = sim::assess_entry_window(8.0, 2.0, 1.0, touch);
    CHECK(a.verdict == sim::EntryVerdict::kObservedOccupied);
    // A hair earlier (exit 3.9) is strictly disjoint -> Clear.
    sim::WindowThreat clear = touch;
    clear.dist_to_zone_m = -0.05;
    const auto b = sim::assess_entry_window(8.0, 2.0, 1.0, clear);
    CHECK(b.verdict == sim::EntryVerdict::kObservedClear);
    CHECK(std::abs(b.margin_s - 0.1) < 1e-9);
  }
  std::cout << "algo invariants green\n";
  return 0;
}
