// Joint perturbation gate (pre-release, behavior still frozen): the frozen
// nominal pocket chain against prediction bias and execution noise, separately
// then jointly, on the common time axis. Perturbations enter the INPUTS
// (threat velocity estimate, ego timed samples) and every verdict is
// recomputed — never a fixed constant subtracted from a nominal gap.
// Passing here would still not bound real-world error (margin calibration is
// a later, separate slice); failing here blocks release discussion.

#include "autonomy/planning/local/predictor.hpp"

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

std::vector<std::vector<sim::DynamicObjectState>> lane(double x0, double v, int n = 121) {
 std::vector<std::vector<sim::DynamicObjectState>> frames;
 for (int k = 0; k < n; ++k) {
  frames.push_back({{{x0 + v * k * 0.1, 0.0}, {0.0, 0.0}, 0.4}});
 }
 return frames;
}

// Deterministic execution noise: lateral sine over the timed samples (no RNG,
// same binary replays bit-identically). Models tracking deviation, not sensor
// noise: the CHECKED trajectory is the noisy one.
std::vector<sim::TimedPose> noisy_exec(const std::vector<sim::TimedPose>& exec, double amp_m) {
 std::vector<sim::TimedPose> out = exec;
 for (std::size_t i = 0; i < out.size(); ++i) {
  out[i].p.y += amp_m * std::sin(1.7 * static_cast<double>(i));
 }
 return out;
}

const std::vector<sim::Vec2> kSuffix{{6.0, 0.05}};

sim::PocketAssessment pocket(const std::vector<std::vector<sim::DynamicObjectState>>& frames) {
 sim::GridMap open(100, 100, 0.1);
 const sim::ChassisExecCaps spin{true, 2.0, 0.0, 0.0, 0.0, 0.0};
 return sim::assess_pocket_wait(open, {0.05, 2.05}, 0.0, {2.95, 2.05}, {2.95, 0.05}, kSuffix, 1.0,
                                spin, 2.0, frames, 0.1, 0.38, 0.5);
}

} // namespace

int main() {
 // 1. Frozen nominal baseline (control group): everything later compares here.
 {
  const auto nominal = pocket(lane(7.85, -1.0));
  CHECK(nominal.overall == V::kFeasible);
  CHECK(std::abs(nominal.chain_min_gap_m - 0.6696) < 1e-3);
  CHECK(std::abs(nominal.rejoin.end_s - 9.95) < 1e-9);
  std::cerr << "nominal frozen: chain=" << nominal.chain_min_gap_m << "\n";
 }
 // 2. Bias only (estimator thinks the threat 30% slower, so it lingers):
 // transit and hold are untouched (threat far away then), rejoin walks into
 // it. First failing leg is named, not just the overall.
 {
  const auto biased = pocket(lane(7.85, -0.7));
  CHECK(biased.overall == V::kInfeasible);
  CHECK(biased.transit.verdict == V::kFeasible);
  CHECK(biased.hold.verdict == V::kFeasible);
  CHECK(biased.rejoin.verdict == V::kInfeasible);
  CHECK(biased.rejoin.reason == R::kBlockedInWindow);
  CHECK(biased.chain_min_gap_m < 0.0);
  std::cerr << "bias-only: rejoin fails first, chain=" << biased.chain_min_gap_m << "\n";
 }
 // 3. Noise only (0.15 m tracking weave on the transit execution): the gap
 // shrinks measurably (1.27 -> ~1.13) yet stays feasible — noise recorded,
 // not fatal, and rejoin is untouched by construction.
 {
  const auto nominal = pocket(lane(7.85, -1.0));
  const double noisy_gap =
    sim::timed_gap(noisy_exec(nominal.transit.exec_traj, 0.15), lane(7.85, -1.0), 0.1, 0.38);
  CHECK(noisy_gap < nominal.transit.min_gap_m - 0.05);
  CHECK(noisy_gap >= 0.5);
  std::cerr << "noise-only transit: " << nominal.transit.min_gap_m << " -> " << noisy_gap << "\n";
 }
 // 4. Joint (biased frames everywhere + noisy transit execution): still
 // refused, still by rejoin first; transit survives both. Each perturbation
 // hits its own object — effects separate, verdict recomputed.
 {
  const auto frames = lane(7.85, -0.7);
  const auto joint = pocket(frames);
  CHECK(joint.overall == V::kInfeasible);
  CHECK(joint.rejoin.verdict == V::kInfeasible);
  const double noisy_transit =
    sim::timed_gap(noisy_exec(joint.transit.exec_traj, 0.15), frames, 0.1, 0.38);
  CHECK(joint.transit.verdict == V::kFeasible);
  CHECK(noisy_transit < joint.transit.min_gap_m);
  CHECK(noisy_transit >= 0.5);
  std::cerr << "joint: refused by rejoin, noisy transit still feasible at " << noisy_transit << "\n";
 }
 // 4b. FALSE-GO TRIPWIRE (open hole, Gate C must close it): the SAME slow
 // threat (nominal Infeasible, chain -0.73) with an OPTIMISTIC 60% speedup
 // bias flips to Feasible (+1.14). The pure assessor is correct given its
 // inputs — garbage in, verdict out — so no assess-level fix exists: the
 // defense must live in consumption (estimator confidence gating the verdict,
 // or margin covering the bias error). If this case ever stops flipping,
 // something changed the assessor or the anchor — investigate, don't cheer:
 // the tripwire guards the hole's SHAPE, and closing it belongs to Gate C
 // wiring design, not to this harness.
 {
  const auto nominal_slow = pocket(lane(7.85, -0.7));
  CHECK(nominal_slow.overall == V::kInfeasible);
  CHECK(nominal_slow.chain_min_gap_m < 0.0);
  const auto optimistic = pocket(lane(7.85, -0.7 * 1.6));
  std::cerr << "false-go: nominal chain=" << nominal_slow.chain_min_gap_m << " -> optimistic chain="
            << optimistic.chain_min_gap_m << " overall=" << static_cast<int>(optimistic.overall)
            << "\n";
  CHECK(optimistic.overall == V::kFeasible);
  CHECK(optimistic.chain_min_gap_m > 1.0);
 }
 // 5. Threshold boundary: the sampler's first Feasible pocket sits at chain
 // 0.5015 — a mere 5% velocity bias flips the SAME pocket to 0.3754. Nominal
 // pass at the threshold carries zero robustness; this case stays a boundary
 // sentinel, never a success story.
 {
  sim::GridMap open(100, 100, 0.1);
  sim::PocketSampleParams prm;
  prm.lateral_offsets_m = {1.0, -1.0, 2.0, -2.0};
  prm.window_m = 10.0;
  prm.min_downstream_m = 3.0;
  prm.min_pocket_dist_m = 0.3;
  prm.max_candidates = 8;
  const std::vector<sim::Vec2> route{{0.05, 2.05}, {2.95, 2.05}, {2.95, 0.05}, {6.05, 0.05}};
  const auto nominal_search =
    sim::sample_pockets(open, route, {0.05, 2.05}, 0.0, 1.0, sim::ChassisExecCaps{true, 2.0, 0.0, 0.0, 0.0, 0.0}, 2.0, prm, lane(7.85, -1.0),
                        0.1, 0.38, 0.5);
  const sim::PocketCandidate* anchor = nullptr;
  for (const auto& candidate : nominal_search.candidates) {
   if (candidate.assessment.overall == V::kFeasible) {
    anchor = &candidate;
    break;
   }
  }
  CHECK(anchor != nullptr);
  CHECK(std::abs(anchor->assessment.chain_min_gap_m - 0.5015) < 1e-3);
  const auto biased_search =
    sim::sample_pockets(open, route, {0.05, 2.05}, 0.0, 1.0, sim::ChassisExecCaps{true, 2.0, 0.0, 0.0, 0.0, 0.0}, 2.0, prm, lane(7.85, -0.95),
                        0.1, 0.38, 0.5);
  const sim::PocketCandidate* same = nullptr;
  for (const auto& candidate : biased_search.candidates) {
   if (std::hypot(candidate.pocket.x - anchor->pocket.x, candidate.pocket.y - anchor->pocket.y) <
       0.2) {
    same = &candidate;
    break;
   }
  }
  CHECK(same != nullptr);
  CHECK(same->assessment.overall == V::kInfeasible);
  CHECK(same->assessment.chain_min_gap_m < 0.5);
  std::cerr << "boundary: " << anchor->assessment.chain_min_gap_m << " -> "
            << same->assessment.chain_min_gap_m << " under 5% bias\n";
 }
 // 6. Window edge: a 4 m traversal with H=3.9 is undecidable and with H=4.0
 // is GoNow — one frame flips the verdict, so window sizing is evidence too.
 {
  const std::vector<sim::Vec2> path{{0.0, 0.0}, {4.0, 0.0}};
  const auto short_open = std::vector<std::vector<sim::DynamicObjectState>>(40, {{{50.0, 50.0}, {0.0, 0.0}, 0.4}});
  const auto long_open = std::vector<std::vector<sim::DynamicObjectState>>(41, {{{50.0, 50.0}, {0.0, 0.0}, 0.4}});
  CHECK(sim::assess_departure(path, 0, 1.0, short_open, 0.1, 0.38, 0.5).reason == R::kHorizonShort);
  CHECK(sim::assess_departure(path, 0, 1.0, long_open, 0.1, 0.38, 0.5).reason == R::kGoNow);
 }
 std::cerr << "perturbation gate green\n";
 return 0;
}
