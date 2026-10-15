#include "algo/entry_governor.hpp"

#include <iostream>

#define CHECK(condition)                                                                                     \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n";                            \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

// Governor contract: one-sided brake. Occupied engages now, sustained Clear
// releases, silence releases, and no input combination ever commands motion.
int main() {
  sim::GovernorState state;
  sim::GovernorInput idle;
  // Silent start: no hold, and silence never engages one.
  for (int i = 0; i < 100; ++i) CHECK(!govern_entry(state, idle).hold_active);
  // Occupied engages on the first fresh frame.
  sim::GovernorInput occ;
  occ.occupied_fresh = true;
  occ.stop_point = {3.0, 4.0};
  occ.stop_valid = true;
  const auto held = govern_entry(state, occ);
  CHECK(held.hold_active && held.stop_valid);
  CHECK(held.stop_point.x == 3.0 && held.stop_point.y == 4.0);
  // Uncertain gaps keep the hold (never invent a release).
  for (int i = 0; i < 10; ++i) CHECK(govern_entry(state, idle).hold_active);
  // A single Clear does not release (needs 25 sustained).
  sim::GovernorInput clear;
  clear.clear_fresh = true;
  CHECK(govern_entry(state, clear).hold_active);
  for (int i = 0; i < 23; ++i) CHECK(govern_entry(state, clear).hold_active);
  CHECK(!govern_entry(state, clear).hold_active);
  // Re-engage, then prolonged silence releases (tracks gone: DWA owns it).
  CHECK(govern_entry(state, occ).hold_active);
  for (int i = 0; i < 14; ++i) CHECK(govern_entry(state, idle).hold_active);
  CHECK(!govern_entry(state, idle).hold_active);
  // Conflict geometry: crossing rays meet mid-way; parallel falls back.
  const auto cross =
    sim::governor_conflict_center({0, 0}, {1, 0}, {4, -4}, {0, 1});
  CHECK(cross.x > 3.0 && cross.x < 5.0 && cross.y > -1.0 && cross.y < 1.0);
  const auto parallel =
    sim::governor_conflict_center({0, 0}, {1, 0}, {0, 2}, {1, 0});
  CHECK(parallel.x == 0.0 && parallel.y == 1.0);
  std::cout << "entry governor green\n";
  return 0;
}
