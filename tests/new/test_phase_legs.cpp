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
 // Transit leg with its own planning: A* geometry, executed trajectory checked.
 {
  sim::GridMap open(30, 30, 0.1);
  const auto empty40 = parked_frames(40, {50.0, 50.0});
  const auto empty61 = parked_frames(61, {50.0, 50.0});
  // Cell-center to cell-center: A* snaps nothing, no turns, end == drive time.
  const auto straight = sim::assess_transit(open, {0.05, 0.05}, 0.0, {1.95, 0.05}, 1.0, 2.0, true,
                                            empty61, 0.1, 0.38, 0.5);
  CHECK(straight.verdict == V::kFeasible && straight.reason == R::kClear);
  CHECK(std::abs(straight.end_s - 1.9) < 0.05);
  // 90-degree transit: the quarter spin is costed (pi/2 / 2 = 0.79 s).
  const auto turned = sim::assess_transit(open, {0.05, 0.05}, 0.0, {0.05, 1.95}, 1.0, 2.0, true,
                                          empty61, 0.1, 0.38, 0.5);
  CHECK(turned.verdict == V::kFeasible);
  CHECK(turned.end_s > 1.9 && turned.end_s < 3.0);
  CHECK(!turned.exec_traj.empty());
  // Same geometry, Ackermann: honestly Unknown (ChassisCaps has no curvature
  // execution model yet), nominal schedule kept for chaining.
  const auto ack =
    sim::assess_transit(open, {0.0, 0.0}, 0.0, {2.0, 0.0}, 1.0, 2.0, false, empty61, 0.1, 0.38, 0.5);
  CHECK(ack.verdict == V::kUnknown && ack.reason == R::kUnassessed);
  CHECK(ack.end_s < std::numeric_limits<double>::infinity());
  // Walled-off pocket: proven static infeasibility, not traffic, not unknown.
  sim::GridMap walled(30, 30, 0.1);
  walled.addBox(0.9, -1.0, 1.1, 3.0);
  const auto walled_off =
    sim::assess_transit(walled, {0.0, 0.0}, 0.0, {2.0, 0.0}, 1.0, 2.0, true, empty61, 0.1, 0.38, 0.5);
  CHECK(walled_off.verdict == V::kInfeasible && walled_off.reason == R::kStaticBlocked);
  // Threat crossing the leg as the ego arrives (spin-up turns put the ego at
  // x=1 near t=1.8; the threat reaches (1,0) at t=1.75): motion meets motion.
  std::vector<std::vector<sim::DynState>> crosser;
  for (int k = 0; k < 61; ++k) {
   crosser.push_back({{{1.0, 3.5 - 2.0 * k * 0.1}, {0.0, 0.0}, 0.4}});
  }
  const auto hit =
    sim::assess_transit(open, {0.0, 0.0}, 0.0, {2.0, 0.0}, 1.0, 2.0, true, crosser, 0.1, 0.38, 0.5);
  CHECK(hit.verdict == V::kInfeasible);
  CHECK(hit.min_gap_m < 0.5);
  // Checked evidence routes around a pillar: exec deviates off the straight line.
  sim::GridMap pillar(30, 30, 0.1);
  pillar.addBox(0.9, -0.1, 1.1, 0.1);
  const auto around =
    sim::assess_transit(pillar, {0.0, 0.0}, 0.0, {2.0, 0.0}, 1.0, 2.0, true, empty61, 0.1, 0.38, 0.5);
  CHECK(around.verdict == V::kFeasible);
  double max_dev = 0.0;
  for (const auto& sample : around.exec_traj) {
   max_dev = std::max(max_dev, std::abs(sample.p.y));
  }
  CHECK(max_dev > 0.05);
  // Already at the pocket: static spot check, zero execution.
  const auto home =
    sim::assess_transit(open, {1.0, 1.0}, 0.7, {1.0, 1.0}, 1.0, 2.0, true, empty61, 0.1, 0.38, 0.5);
  CHECK(home.verdict == V::kFeasible && home.end_s == 0.0);
  // Tight window on the same straight transit: undecidable, not unsafe.
  const auto tight =
    sim::assess_transit(open, {0.0, 0.0}, 0.0, {2.0, 0.0}, 1.0, 2.0, true,
                        parked_frames(21, {50.0, 50.0}), 0.1, 0.38, 0.5);
  CHECK(tight.verdict == V::kUnknown && tight.reason == R::kHorizonShort);
 }
 // Composition with planned legs on cell centers (no grid-snap jogs): pocket
 // 2 m off the threat lane; the lane threat strolls at 1 m/s. hold_s = 0
 // departs straight into it; hold_s = 2 chains the departure past it.
 // Identical geometry — only the chained delay differs.
 {
  sim::GridMap open(100, 100, 0.1);
  std::vector<std::vector<sim::DynState>> frames;
  for (int k = 0; k < 121; ++k) {
   frames.push_back({{{7.85 - 1.0 * k * 0.1, 0.0}, {0.0, 0.0}, 0.4}});
  }
  const std::vector<sim::Vec2> suffix{{6.0, 0.05}};
  const auto rushed =
    sim::assess_pocket_wait(open, {0.05, 2.05}, 0.0, {2.95, 2.05}, {2.95, 0.05}, suffix, 1.0, 2.0,
                            true, 0.0, frames, 0.1, 0.38, 0.5);
  CHECK(rushed.transit.verdict == V::kFeasible);
  CHECK(rushed.hold.verdict == V::kFeasible);
  CHECK(rushed.rejoin.verdict == V::kInfeasible);
  CHECK(rushed.overall == V::kInfeasible);
  const auto patient =
    sim::assess_pocket_wait(open, {0.05, 2.05}, 0.0, {2.95, 2.05}, {2.95, 0.05}, suffix, 1.0, 2.0,
                            true, 2.0, frames, 0.1, 0.38, 0.5);
  CHECK(patient.transit.verdict == V::kFeasible);
  CHECK(patient.hold.verdict == V::kFeasible);
  CHECK(patient.rejoin.verdict == V::kFeasible);
  CHECK(patient.rejoin.min_gap_m >= 0.5);
  CHECK(patient.overall == V::kFeasible);
  std::cerr << "chained pocket: transit_end=" << patient.transit.end_s
            << " rejoin_end=" << patient.rejoin.end_s << " gap=" << patient.rejoin.min_gap_m
            << "\n";
  // Threat parked on the pocket: transit drives into it (WaitUnsafe), hold
  // dies too, nothing masked.
  const auto bad_spot = sim::assess_pocket_wait(
    open, {0.05, 2.05}, 0.0, {2.95, 2.05}, {2.95, 0.05}, suffix, 1.0, 2.0, true, 2.0,
    parked_frames(121, {2.95, 2.05}), 0.1, 0.38, 0.5);
  CHECK(bad_spot.transit.verdict == V::kInfeasible);
  CHECK(bad_spot.hold.verdict == V::kInfeasible);
  CHECK(bad_spot.hold.reason == R::kWaitUnsafe);
  CHECK(bad_spot.overall == V::kInfeasible);
  // Ackermann transit: whole candidate withholds proof, diagnoses kept.
  const auto ack = sim::assess_pocket_wait(open, {0.05, 2.05}, 0.0, {2.95, 2.05}, {2.95, 0.05},
                                          suffix, 1.0, 2.0, false, 2.0, frames, 0.1, 0.38, 0.5);
  CHECK(ack.transit.verdict == V::kUnknown);
  CHECK(ack.overall == V::kUnknown);
 }
 std::cerr << "phase legs green\n";
 return 0;
}
