// SIL offline closed-loop replay (gate 2) + Phase-2 feasibility interface
// (candidate kinds frozen, time feasibility assessable, spatial kinds honestly
// Unknown).
//
// Replay: same time axis drives ego and threat along an arbitrary polyline
// path. Each cycle rebuilds prediction frames from the current threat state
// (constant-velocity estimate, optionally biased), recomputes t_dep for the
// remaining suffix (replan every cycle, never latched), and either holds or
// advances the ego at cruise. Records the predicted release, the actual
// departure, and the true minimum gap from ground-truth positions (10x
// subsampled measurement).
//
// Miss (quality) vs hard fail (safety) are separate verdicts:
//   hard fail = collision (min_gap < 0) or goal never reached;
//   miss      = min_gap < margin (margin eaten) or |depart - initial_pred|
//               > 0.5 s (release-time error beyond the miss threshold).
// A run can be safe yet a miss (conservative bias costs time); a miss is a
// prediction-quality verdict, not a crash report.

#include "algo/predictor.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#define CHECK(condition)                                                                                     \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n";                           \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

namespace {

constexpr double kCruiseMps = 1.0;
constexpr double kDtS = 0.1;
constexpr double kHorizonS = 10.0;
constexpr double kEgoRadiusM = 0.38;
constexpr double kThreatRadiusM = 0.4;
constexpr double kMarginM = 0.5;
constexpr double kMissTimeS = 0.5;
constexpr double kPi = 3.14159265358979323846;

struct ReplayResult {
 bool reached{false};
 double initial_pred_s{std::numeric_limits<double>::infinity()};
 double depart_s{-1.0};
 double min_gap{std::numeric_limits<double>::infinity()};
};

double poly_length(const std::vector<sim::Vec2>& path) {
 double length = 0.0;
 for (std::size_t i = 0; i + 1 < path.size(); ++i) {
  length += std::hypot(path[i + 1].x - path[i].x, path[i + 1].y - path[i].y);
 }
 return length;
}

sim::Vec2 path_at(const std::vector<sim::Vec2>& path, double s) {
 double walked = 0.0;
 for (std::size_t i = 0; i + 1 < path.size(); ++i) {
  const double leg = std::hypot(path[i + 1].x - path[i].x, path[i + 1].y - path[i].y);
  if (leg < 1e-9) {
   continue;
  }
  if (walked + leg >= s) {
   const double t = std::clamp((s - walked) / leg, 0.0, 1.0);
   return {path[i].x + (path[i + 1].x - path[i].x) * t,
           path[i].y + (path[i + 1].y - path[i].y) * t};
  }
  walked += leg;
 }
 return path.back();
}

// Remaining suffix from arc distance s: current position plus every vertex
// ahead of it.
std::vector<sim::Vec2> path_suffix(const std::vector<sim::Vec2>& path, double s) {
 std::vector<sim::Vec2> suffix{path_at(path, s)};
 double walked = 0.0;
 for (std::size_t i = 0; i + 1 < path.size(); ++i) {
  const double leg = std::hypot(path[i + 1].x - path[i].x, path[i + 1].y - path[i].y);
  walked += leg;
  if (walked > s + 1e-9) {
   suffix.push_back(path[i + 1]);
  }
 }
 return suffix;
}

// True same-time gap between two linearly moving points, densely subsampled
// measurement (miss <= ~2 cm at these speeds; assertions keep 10x clearance).
double measured_gap(sim::Vec2 e0, sim::Vec2 e1, sim::Vec2 t0, sim::Vec2 t1, double radii) {
 double gap = std::numeric_limits<double>::infinity();
 for (int i = 0; i <= 10; ++i) {
  const double a = static_cast<double>(i) / 10.0;
  const double dx = (e0.x + (e1.x - e0.x) * a) - (t0.x + (t1.x - t0.x) * a);
  const double dy = (e0.y + (e1.y - e0.y) * a) - (t0.y + (t1.y - t0.y) * a);
  gap = std::min(gap, std::hypot(dx, dy) - radii);
 }
 return gap;
}

ReplayResult run_replay(const std::vector<sim::Vec2>& path, sim::Vec2 threat_p0,
                        sim::Vec2 threat_v_true, sim::Vec2 threat_v_est, double horizon_s,
                        double noise_amp_m = 0.0) {
 ReplayResult result;
 const int frames = static_cast<int>(horizon_s / kDtS) + 1;
 const double total = poly_length(path);
 double ego_s = 0.0;
 double time_s = 0.0;
 sim::Vec2 threat = threat_p0;
 // Execution noise: deterministic lateral weave on the TRUE ego positions
 // only. Planning (remaining path, departure) stays blind to it — exactly
 // the separation the joint gate requires. Zero by default (nominal runs).
 auto weave = [&](double t) {
 return noise_amp_m * std::sin(2.0 * kPi * t / 2.0);
 };
 for (int cycle = 0; cycle < 600 && ego_s < total - 1e-9; ++cycle) {
  const auto remaining = path_suffix(path, ego_s);
  std::vector<std::vector<sim::DynState>> predicted;
  for (int k = 0; k < frames; ++k) {
   predicted.push_back(
     {{{threat.x + threat_v_est.x * k * kDtS, threat.y + threat_v_est.y * k * kDtS}, {0.0, 0.0},
       kThreatRadiusM}});
  }
  const double dep =
    sim::earliest_departure(remaining, 0, kCruiseMps, predicted, kDtS, kEgoRadiusM, kMarginM);
  if (cycle == 0) {
   result.initial_pred_s = dep;
  }
  sim::Vec2 ego0 = path_at(path, ego_s);
  ego0.y += weave(time_s);
  if (dep == 0.0) {
   if (result.depart_s < 0.0) {
    result.depart_s = time_s;
   }
   ego_s = std::min(total, ego_s + kCruiseMps * kDtS);
  }
  sim::Vec2 ego1 = path_at(path, ego_s);
  ego1.y += weave(time_s + kDtS);
  const sim::Vec2 next{threat.x + threat_v_true.x * kDtS, threat.y + threat_v_true.y * kDtS};
  result.min_gap =
    std::min(result.min_gap, measured_gap(ego0, ego1, threat, next, kThreatRadiusM + kEgoRadiusM));
  threat = next;
  time_s += kDtS;
 }
 result.reached = ego_s >= total - 1e-9;
 return result;
}

bool is_miss(const ReplayResult& r) {
 if (!r.reached || r.min_gap < 0.0) {
  return true;
 }
 if (r.min_gap < kMarginM) {
  return true;
 }
 if (r.depart_s < 0.0 || std::abs(r.depart_s - r.initial_pred_s) > kMissTimeS + 1e-9) {
  return true;
 }
 return false;
}

const std::vector<sim::Vec2> kStraight{{0.0, 0.0}, {6.0, 0.0}};

} // namespace

int main() {
 // Feasibility interface: Proceed/WaitInPlace decided by time assessment;
 // spatial kinds honestly Unknown with their own reason.
 {
  const std::vector<sim::Vec2> line{{0.0, 0.0}, {2.0, 0.0}};
  const std::vector<std::vector<sim::DynState>> open(40, {{{5.0, 5.0}, {0.0, 0.0}, 0.4}});
  const auto go = sim::check_candidate(sim::LocalCandidate::kProceed, line, 0, 1.0, open, 0.1,
                                       0.38, 0.5);
  CHECK(go.verdict == sim::Feasibility::Verdict::kFeasible);
  CHECK(go.reason == sim::DepartReason::kGoNow && go.wait_s == 0.0 && go.min_gap_m >= 0.5);
  const auto hold =
    sim::check_candidate(sim::LocalCandidate::kWaitInPlace, line, 0, 1.0, open, 0.1, 0.38, 0.5);
  CHECK(hold.verdict == sim::Feasibility::Verdict::kFeasible);
  std::vector<std::vector<sim::DynState>> clearing;
  for (int k = 0; k < 40; ++k) {
   clearing.push_back(k < 10 ? std::vector<sim::DynState>{{{1.5, 0.0}, {0.0, 0.0}, 0.4}}
                             : std::vector<sim::DynState>{{{5.0, 5.0}, {0.0, 0.0}, 0.4}});
  }
  // Clearing corridor: departing now is refused (too early), waiting is feasible.
  const auto early =
    sim::check_candidate(sim::LocalCandidate::kProceed, line, 0, 1.0, clearing, 0.1, 0.38, 0.5);
  CHECK(early.verdict == sim::Feasibility::Verdict::kInfeasible);
  CHECK(early.reason == sim::DepartReason::kWait);
  const auto patient = sim::check_candidate(sim::LocalCandidate::kWaitInPlace, line, 0, 1.0,
                                            clearing, 0.1, 0.38, 0.5);
  CHECK(patient.verdict == sim::Feasibility::Verdict::kFeasible);
  CHECK(patient.wait_s > 0.0 && patient.min_gap_m >= 0.5);
  // Parked threat: both time kinds infeasible, same cause.
  const std::vector<std::vector<sim::DynState>> parked(40, {{{1.5, 0.0}, {0.0, 0.0}, 0.4}});
  CHECK(sim::check_candidate(sim::LocalCandidate::kProceed, line, 0, 1.0, parked, 0.1, 0.38, 0.5)
          .verdict == sim::Feasibility::Verdict::kInfeasible);
  CHECK(sim::check_candidate(sim::LocalCandidate::kWaitInPlace, line, 0, 1.0, parked, 0.1, 0.38,
                             0.5)
          .reason == sim::DepartReason::kBlockedInWindow);
  // Short window on a long path: Unknown, never Infeasible.
  const std::vector<sim::Vec2> long_line{{0.0, 0.0}, {10.0, 0.0}};
  const std::vector<std::vector<sim::DynState>> short_open(21, {{{50.0, 50.0}, {0.0, 0.0}, 0.4}});
  CHECK(sim::check_candidate(sim::LocalCandidate::kProceed, long_line, 0, 1.0, short_open, 0.1,
                             0.38, 0.5)
          .verdict == sim::Feasibility::Verdict::kUnknown);
  // Spatial kinds: Unknown with their own reason, no borrowed verdicts.
  for (const auto kind :
       {sim::LocalCandidate::kWaitAtPocket, sim::LocalCandidate::kRejoin, sim::LocalCandidate::kDetour}) {
   const auto pending = sim::check_candidate(kind, line, 0, 1.0, open, 0.1, 0.38, 0.5);
   CHECK(pending.verdict == sim::Feasibility::Verdict::kUnknown);
   CHECK(pending.reason == sim::DepartReason::kUnassessed);
   CHECK(pending.wait_s == std::numeric_limits<double>::infinity());
  }
  // Degenerate input stays Unknown/Invalid, never Feasible.
  CHECK(sim::check_candidate(sim::LocalCandidate::kProceed, {}, 0, 1.0, open, 0.1, 0.38, 0.5)
          .reason == sim::DepartReason::kInvalid);
 }
 // Nominal crosser, perfect estimation: prediction credible, countdown exact.
 {
  const ReplayResult r = run_replay(kStraight, {3.0, 2.5}, {0.0, -1.0}, {0.0, -1.0}, kHorizonS);
  CHECK(r.reached);
  CHECK(r.initial_pred_s > 0.0 && r.initial_pred_s < std::numeric_limits<double>::infinity());
  CHECK(r.min_gap >= kMarginM);
  CHECK(!is_miss(r));
  std::cerr << "nominal: pred=" << r.initial_pred_s << " depart=" << r.depart_s
            << " gap=" << r.min_gap << "\n";
 }
 // Conservative bias (threat faster than estimated): still safe, but the
 // initial prediction overshoots and replanning drifts — a quality miss in
 // the conservative direction, costing time instead of margin.
 {
  const ReplayResult nominal =
    run_replay(kStraight, {3.0, 2.5}, {0.0, -1.0}, {0.0, -1.0}, kHorizonS);
  const ReplayResult r = run_replay(kStraight, {3.0, 2.5}, {0.0, -1.0}, {0.0, -0.7}, kHorizonS);
  CHECK(r.reached);
  CHECK(r.min_gap >= 0.0);
  CHECK(is_miss(r));
  CHECK(r.depart_s >= nominal.depart_s - 1e-9);
  std::cerr << "conservative-bias: pred=" << r.initial_pred_s << " depart=" << r.depart_s
            << " gap=" << r.min_gap << "\n";
 }
 // Adversarial bias (threat slower than estimated): departs far too early,
 // eats the whole margin without technically touching. This is why estimator
 // uncertainty needs its own margin budget — exact arithmetic on wrong
 // inputs still misses.
 {
  const ReplayResult r = run_replay(kStraight, {3.0, 2.5}, {0.0, -1.0}, {0.0, -1.6}, kHorizonS);
  CHECK(r.reached);
  CHECK(r.min_gap >= 0.0);
  CHECK(is_miss(r));
  CHECK(r.min_gap < kMarginM);
  std::cerr << "adversarial-bias: pred=" << r.initial_pred_s << " depart=" << r.depart_s
            << " gap=" << r.min_gap << "\n";
 }
 // Head-on through the waiting spot: at t=0 the function reports infinity —
 // correctly refusing to bless a wait during which the threat drives through
 // the ego. The replay (which waits anyway) gets run over at t=2 s. Waiting
 // here is a Phase-2 (where to wait) problem; Phase-1 must not claim it.
 {
  const ReplayResult r = run_replay(kStraight, {4.0, 0.0}, {-2.0, 0.0}, {-2.0, 0.0}, kHorizonS);
  CHECK(r.initial_pred_s == std::numeric_limits<double>::infinity());
  CHECK(r.min_gap < 0.0);
  CHECK(is_miss(r));
  std::cerr << "headon-wait-overrun: pred=inf gap=" << r.min_gap << "\n";
 }
 // Positive lateral scenario (scenario-matrix anchor): the lane ahead is
 // lethally blocked for an immediate departure (gap -0.33), but the start
 // pocket sits 2 m off the threat lane, so waiting is safe and the full
 // traverse-descend-merge executes with margin intact.
 {
  const std::vector<sim::Vec2> pocket{{0.0, 2.0}, {3.0, 2.0}, {3.0, 0.0}, {6.0, 0.0}};
  const ReplayResult r = run_replay(pocket, {5.0, 0.0}, {-0.5, 0.0}, {-0.5, 0.0}, 12.0);
  CHECK(r.reached);
  CHECK(r.initial_pred_s > 0.0 && r.initial_pred_s < std::numeric_limits<double>::infinity());
  CHECK(r.min_gap >= kMarginM);
  CHECK(!is_miss(r));
  std::cerr << "pocket-positive: pred=" << r.initial_pred_s << " depart=" << r.depart_s
            << " gap=" << r.min_gap << "\n";
 }
 // Pocket joint replay (Gate A): biased estimate and blind execution noise,
 // truth remeasured on the common axis. Estimator 40% fast departs at 0.2
 // into the threat's tail and survives on 0.0006 — boundary sentinel, pinned
 // from both sides so any drift fails loudly instead of silently.
 {
  const std::vector<sim::Vec2> pocket{{0.0, 2.0}, {3.0, 2.0}, {3.0, 0.0}, {6.0, 0.0}};
  const ReplayResult hot = run_replay(pocket, {5.0, 0.0}, {-0.5, 0.0}, {-0.7, 0.0}, 12.0);
  CHECK(hot.reached);
  CHECK(hot.min_gap >= kMarginM && hot.min_gap < kMarginM + 0.05);
  CHECK(!is_miss(hot));
  std::cerr << "pocket-hot-bias: pred=" << hot.initial_pred_s << " depart=" << hot.depart_s
            << " gap=" << hot.min_gap << "\n";
  // Estimator 20% slow: waits out the threat, safe but late (time miss).
  const ReplayResult late = run_replay(pocket, {5.0, 0.0}, {-0.5, 0.0}, {-0.4, 0.0}, 12.0);
  CHECK(late.reached);
  CHECK(late.min_gap >= 0.0);
  CHECK(is_miss(late));
  std::cerr << "pocket-late-bias: pred=" << late.initial_pred_s << " depart=" << late.depart_s
            << " gap=" << late.min_gap << "\n";
  // Nominal estimate with 0.1 m blind tracking weave: survives, recorded.
  const ReplayResult wobble = run_replay(pocket, {5.0, 0.0}, {-0.5, 0.0}, {-0.5, 0.0}, 12.0, 0.1);
  CHECK(wobble.reached);
  CHECK(wobble.min_gap >= kMarginM);
  CHECK(!is_miss(wobble));
  std::cerr << "pocket-weave: pred=" << wobble.initial_pred_s << " depart=" << wobble.depart_s
            << " gap=" << wobble.min_gap << "\n";
  // Observed-max stress (est-error test: vel_abs_max ~= 1.9 m/s near field):
  // the margin is eaten with no collision. Amplitude grounded in observation,
  // verdict still miss — current 0.5 m does not cover observed-max error.
  const ReplayResult stressed = run_replay(pocket, {5.0, 0.0}, {-0.5, 0.0}, {-2.4, 0.0}, 12.0);
  CHECK(stressed.reached);
  CHECK(stressed.min_gap >= 0.0);
  CHECK(is_miss(stressed));
  CHECK(stressed.min_gap < kMarginM);
  std::cerr << "pocket-max-stress: pred=" << stressed.initial_pred_s
            << " depart=" << stressed.depart_s << " gap=" << stressed.min_gap << "\n";
 }
 // Hold-unsafe under a sign-flipped estimate: "wait 0.6 then go" meets the
 // threat driving through the wait spot at t=2. Truth, not estimate, decides.
 {
  const ReplayResult hit = run_replay(kStraight, {4.0, 0.0}, {-2.0, 0.0}, {0.5, 0.0}, kHorizonS);
  CHECK(hit.min_gap < 0.0);
  CHECK(is_miss(hit));
  std::cerr << "hold-unsafe-flip: pred=" << hit.initial_pred_s << " depart=" << hit.depart_s
            << " gap=" << hit.min_gap << "\n";
 }
 // Horizon too short for the whole traversal: open field, still infinity.
 // Unknown is not safe (SOTIF Area 3): no proof fits in the window.
 {
  const std::vector<sim::Vec2> long_path{{0.0, 0.0}, {10.0, 0.0}};
  const std::vector<std::vector<sim::DynState>> open(21, {{{50.0, 50.0}, {0.0, 0.0}, 0.4}});
  CHECK(sim::earliest_departure(long_path, 0, 1.0, open, 0.1, 0.38, 0.5) ==
        std::numeric_limits<double>::infinity());
 }
 std::cerr << "departure replay matrix green\n";
 return 0;
}
