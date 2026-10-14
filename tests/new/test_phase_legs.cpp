// Three-leg pocket assessment (Phase-2 interfaces, pure): transit / hold /
// rejoin evaluated on one chained schedule, combined without dropping
// diagnoses. Transit has no arrival proof yet (honest Unknown); hold and
// rejoin are fully assessed. Mission stays untouched.

#include "algo/predictor.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

#define CHECK(condition)                                                                                     \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n";                           \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

namespace {

using V = sim::Feasibility::Verdict;
using R = sim::DepartReason;

std::vector<std::vector<sim::DynState>> parked_frames(int n, sim::Vec2 p, double r = 0.4) {
 return std::vector<std::vector<sim::DynState>>(static_cast<std::size_t>(n), {{{p.x, p.y}, {0, 0}, r}});
}

} // namespace

int main() {
 // Hold leg: whole interval swept, not endpoints.
 {
  const auto parked = parked_frames(21, {1.5, 0.0});
  // Safe hold at x=0 over [0, 2]: nearest approach 1.5 m, gap 0.72.
  const auto safe = sim::assess_hold({0.0, 0.0}, 0.0, 2.0, parked, 0.1, 0.38, 0.5);
  CHECK(safe.verdict == V::kFeasible && safe.reason == R::kClear);
  CHECK(std::abs(safe.min_gap_m - (1.5 - 0.4 - 0.38)) < 1e-9);
  CHECK(safe.end_s == 2.0);
  // Same spot, threat parked on it: staying kills.
  const auto overrun = sim::assess_hold({1.5, 0.0}, 0.0, 2.0, parked, 0.1, 0.38, 0.5);
  CHECK(overrun.verdict == V::kInfeasible && overrun.reason == R::kWaitUnsafe);
  // Zero-duration hold is a point check, not a free pass.
  const auto point_ok = sim::assess_hold({0.0, 0.0}, 1.0, 1.0, parked, 0.1, 0.38, 0.5);
  CHECK(point_ok.verdict == V::kFeasible);
  const auto point_bad = sim::assess_hold({1.5, 0.0}, 1.0, 1.0, parked, 0.1, 0.38, 0.5);
  CHECK(point_bad.verdict == V::kInfeasible);
  // Hold past the horizon is unprovable, not safe.
  const auto beyond = sim::assess_hold({0.0, 0.0}, 0.0, 5.0, parked, 0.1, 0.38, 0.5);
  CHECK(beyond.verdict == V::kUnknown && beyond.reason == R::kHorizonShort);
  // Threat leaves, then returns mid-hold: the return is caught.
  std::vector<std::vector<sim::DynState>> visit;
  for (int k = 0; k < 21; ++k) {
   const bool present = k < 5 || k >= 15;
   visit.push_back(present ? std::vector<sim::DynState>{{{0.0, 0.0}, {0.0, 0.0}, 0.4}}
                           : std::vector<sim::DynState>{{{9.0, 9.0}, {0.0, 0.0}, 0.4}});
  }
  // Threat leaves, then returns mid-hold: the hold starts after the first
  // visit (t=0.6), so only the return can refuse it — and it does.
  const auto returned = sim::assess_hold({0.0, 0.0}, 0.6, 2.0, visit, 0.1, 0.38, 0.5);
  CHECK(returned.verdict == V::kInfeasible && returned.reason == R::kWaitUnsafe);
  // Threat exactly at the spot at the hold-end instant: caught (joints are closed).
  std::vector<std::vector<sim::DynState>> arrive;
  for (int k = 0; k < 21; ++k) {
   arrive.push_back({{{9.0 - 9.0 * k * 0.1 / 2.0}, {0.0, 0.0}, 0.4}});
  }
  const auto endpoint = sim::assess_hold({0.0, 0.0}, 0.0, 2.0, arrive, 0.1, 0.38, 0.5);
  CHECK(endpoint.verdict == V::kInfeasible);
 }
 // Rejoin leg: hold-then-go with chained delay; zero cruise is Invalid.
 {
  const std::vector<sim::Vec2> lane{{0.0, 0.0}, {2.0, 0.0}};
  const auto open = parked_frames(40, {5.0, 5.0});
  const auto now = sim::assess_rejoin(lane, 0, 1.0, 0.0, open, 0.1, 0.38, 0.5);
  CHECK(now.verdict == V::kFeasible && now.reason == R::kClear);
  CHECK(std::abs(now.end_s - 2.0) < 1e-9);
  const auto stalled =
    sim::assess_rejoin(lane, 0, 0.0, 0.0, open, 0.1, 0.38, 0.5);
  CHECK(stalled.verdict == V::kUnknown && stalled.reason == R::kInvalid);
  const auto parked = parked_frames(40, {1.5, 0.0});
  const auto blocked = sim::assess_rejoin(lane, 0, 1.0, 0.0, parked, 0.1, 0.38, 0.5);
  CHECK(blocked.verdict == V::kInfeasible && blocked.reason == R::kBlockedInWindow);
  // Delay past any completable schedule: Unknown, not Feasible-later.
  const auto late = sim::assess_rejoin(lane, 0, 1.0, 3.9, open, 0.1, 0.38, 0.5);
  CHECK(late.verdict == V::kUnknown && late.reason == R::kHorizonShort);
 }
 // Transit leg: honestly Unknown until the A*+swept+caps slice lands, with a
 // nominal schedule for chaining.
 {
  const std::vector<sim::Vec2> transit{{0.0, 0.0}, {3.0, 4.0}};
  const auto leg = sim::assess_transit(transit, 0, 1.0);
  CHECK(leg.verdict == V::kUnknown && leg.reason == R::kUnassessed);
  CHECK(std::abs(leg.end_s - 5.0) < 1e-9);
  CHECK(leg.min_gap_m == std::numeric_limits<double>::infinity());
 }
 // Composition: chained schedule, no dropped diagnoses.
 {
  // Pocket 2 m off the threat lane; threat strolls the lane at 0.5 m/s.
  const std::vector<sim::Vec2> transit{{0.0, 2.0}, {3.0, 2.0}};
  const std::vector<sim::Vec2> rejoin{{3.0, 2.0}, {3.0, 0.0}, {6.0, 0.0}};
  std::vector<std::vector<sim::DynState>> frames;
  for (int k = 0; k < 121; ++k) {
   frames.push_back({{{5.0 - 0.5 * k * 0.1, 0.0}, {0.0, 0.0}, 0.4}});
  }
  // hold_s = 0: rejoin departs at transit end (t=3) straight into the threat.
  const auto rushed = sim::assess_pocket_wait(transit, 0, 1.0, 0.0, rejoin, 0, frames, 0.1, 0.38, 0.5);
  CHECK(rushed.transit.verdict == V::kUnknown);
  CHECK(rushed.rejoin.verdict == V::kInfeasible);
  CHECK(rushed.overall == V::kInfeasible);
  // hold_s = 2: chained departure at t=5, threat long past the descent.
  const auto patient =
    sim::assess_pocket_wait(transit, 0, 1.0, 2.0, rejoin, 0, frames, 0.1, 0.38, 0.5);
  CHECK(patient.hold.verdict == V::kFeasible);
  CHECK(patient.rejoin.verdict == V::kFeasible);
  CHECK(patient.overall == V::kUnknown);
  CHECK(std::abs(patient.rejoin.end_s - (3.0 + 2.0 + 5.0)) < 1e-9);
  std::cerr << "chained rejoin departs at t=5, ends t=10, gap=" << patient.rejoin.min_gap_m << "\n";
  // Threat parked on the pocket: hold dies, transit Unknown retained.
  const auto bad_spot = sim::assess_pocket_wait(transit, 0, 1.0, 2.0, rejoin, 0,
                                                parked_frames(121, {3.0, 2.0}), 0.1, 0.38, 0.5);
  CHECK(bad_spot.hold.verdict == V::kInfeasible);
  CHECK(bad_spot.hold.reason == R::kWaitUnsafe);
  CHECK(bad_spot.transit.verdict == V::kUnknown);
  CHECK(bad_spot.overall == V::kInfeasible);
 }
 std::cerr << "phase legs green\n";
 return 0;
}
