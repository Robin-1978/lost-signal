// Pocket/merge sampling (generation layer): bounded deterministic generation,
// full three-leg assessment per candidate, every verdict kept with reasons.
// Sampling never upgrades conclusions; truncation and Unknown are Incomplete.

#include "autonomy/planning/local/predictor.hpp"

#include <algorithm>
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
using S = sim::SearchVerdict;

sim::PocketSampleParams params() {
 sim::PocketSampleParams p;
 p.lateral_offsets_m = {1.0, -1.0, 2.0, -2.0};
 p.window_m = 10.0;
 p.min_downstream_m = 3.0;
 p.min_pocket_dist_m = 0.3;
 p.max_candidates = 8;
 return p;
}

std::vector<std::vector<sim::DynamicObjectState>> lane_frames() {
 std::vector<std::vector<sim::DynamicObjectState>> frames;
 for (int k = 0; k < 121; ++k) {
  frames.push_back({{{7.85 - 1.0 * k * 0.1, 0.0}, {0.0, 0.0}, 0.4}});
 }
 return frames;
}

const std::vector<sim::Vec2> kRoute{{0.05, 2.05}, {2.95, 2.05}, {2.95, 0.05}, {6.05, 0.05}};

} // namespace

int main() {
 // Main search: route dips to the threat lane; pockets sampled both sides.
 {
  sim::GridMap open(100, 100, 0.1);
  const auto search =
    sim::sample_pockets(open, kRoute, {0.05, 2.05}, 0.0, 1.0, sim::ChassisExecCaps{true, 2.0, 0.0, 0.0, 0.0, 0.0}, 2.0, params(), lane_frames(),
                        0.1, 0.38, 0.5);
  CHECK(!search.candidates.empty());
  CHECK(search.candidates.size() <= 8);
  int feasible = 0;
  int infeasible = 0;
  int unknown = 0;
  for (const auto& candidate : search.candidates) {
   // Every candidate carries its full three-leg diagnosis, refusals included.
   const auto overall = candidate.assessment.overall;
   if (overall == V::kFeasible) {
    ++feasible;
    CHECK(candidate.assessment.chain_min_gap_m >= 0.5);
   } else if (overall == V::kInfeasible) {
    ++infeasible;
   } else {
    ++unknown;
   }
   std::cerr << "pocket=(" << candidate.pocket.x << "," << candidate.pocket.y << ") merge=("
             << candidate.merge.x << "," << candidate.merge.y << ") overall=" << (int)overall
             << " chain_gap=" << candidate.assessment.chain_min_gap_m << "\n";
  }
  CHECK(feasible > 0);
  CHECK(infeasible > 0);
  CHECK(search.verdict == S::kFoundFeasible);
  std::cerr << "search: feasible=" << feasible << " infeasible=" << infeasible
            << " unknown=" << unknown << " truncated=" << search.truncated << "\n";
 }
 // Audit of the first Feasible pocket: the chain gap is the minimum over
 // assessed legs (here the rejoin leg binds), the schedule fits the window
 // end to end (turns + hold + merge), and Feasible required every leg —
 // nominal-model proof only, never a release license.
 {
  sim::GridMap open(100, 100, 0.1);
  const auto found =
    sim::sample_pockets(open, kRoute, {0.05, 2.05}, 0.0, 1.0, sim::ChassisExecCaps{true, 2.0, 0.0, 0.0, 0.0, 0.0}, 2.0, params(), lane_frames(),
                        0.1, 0.38, 0.5);
  const sim::PocketCandidate* first = nullptr;
  for (const auto& candidate : found.candidates) {
   if (candidate.assessment.overall == V::kFeasible) {
    first = &candidate;
    break;
   }
  }
  CHECK(first != nullptr);
  const auto& a = first->assessment;
  CHECK(a.transit.verdict == V::kFeasible && a.hold.verdict == V::kFeasible &&
        a.rejoin.verdict == V::kFeasible);
  CHECK(std::abs(a.chain_min_gap_m -
                 std::min({a.transit.min_gap_m, a.hold.min_gap_m, a.rejoin.min_gap_m})) < 1e-12);
  CHECK(a.rejoin.end_s <= 12.0 + 1e-9);
  CHECK(a.chain_min_gap_m == a.rejoin.min_gap_m);
  std::cerr << "audited feasible pocket: chain_gap=" << a.chain_min_gap_m
            << " (rejoin binds), rejoin_end=" << a.rejoin.end_s << "\n";
 }
 // Cap semantics: truncation with no proof is Incomplete. (Truncation, not
 // Unknown legs, drives this verdict now that Ackermann transit is real.)
 {
  sim::GridMap open(100, 100, 0.1);
  auto capped = params();
  capped.max_candidates = 2;
  const auto search =
    sim::sample_pockets(open, kRoute, {0.05, 2.05}, 0.0, 1.0, sim::ChassisExecCaps{false, 0.0, 0.62, 0.55, 2.0, 0.35}, 2.0, capped, lane_frames(),
                        0.1, 0.38, 0.5);
  CHECK(search.candidates.size() == 2);
  CHECK(search.truncated);
  CHECK(search.verdict == S::kIncomplete);
 }
 // All-refused is proven per-candidate refusal, not a space proof: wall
 // between start and every eastern pocket, transits StaticBlocked.
 {
  sim::GridMap split(100, 100, 0.1);
  split.addBox(0.9, -1.0, 1.1, 11.0);
  const std::vector<sim::Vec2> west{{0.05, 4.05}, {0.05, 2.05}, {0.05, 0.05}};
  const auto search =
    sim::sample_pockets(split, west, {0.05, 2.05}, 0.0, 1.0, sim::ChassisExecCaps{true, 2.0, 0.0, 0.0, 0.0, 0.0}, 2.0, params(), lane_frames(),
                        0.1, 0.38, 0.5);
  CHECK(!search.candidates.empty());
  CHECK(search.verdict == S::kAllRefused);
  for (const auto& candidate : search.candidates) {
   CHECK(candidate.assessment.overall == V::kInfeasible);
   CHECK(candidate.assessment.transit.reason == R::kStaticBlocked);
  }
 }
 // Degenerate sampling inputs search nothing, incompletely.
 {
  sim::GridMap open(100, 100, 0.1);
  auto none = params();
  none.max_candidates = 0;
  CHECK(sim::sample_pockets(open, kRoute, {0.05, 2.05}, 0.0, 1.0, sim::ChassisExecCaps{true, 2.0, 0.0, 0.0, 0.0, 0.0}, 2.0, none,
                            lane_frames(), 0.1, 0.38, 0.5)
          .verdict == S::kIncomplete);
  CHECK(sim::sample_pockets(open, {}, {0.05, 2.05}, 0.0, 1.0, sim::ChassisExecCaps{true, 2.0, 0.0, 0.0, 0.0, 0.0}, 2.0, params(),
                            lane_frames(), 0.1, 0.38, 0.5)
          .verdict == S::kIncomplete);
 }
 std::cerr << "pocket search green\n";
 return 0;
}
