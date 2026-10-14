// SIL offline closed-loop replay (gate 2) for earliest_departure.
//
// Same time axis drives ego and threat: each cycle rebuilds prediction frames
// from the current threat state (constant-velocity estimate, optionally
// biased), recomputes t_dep for the remaining path (replan every cycle, never
// latched), and either holds or advances the ego at cruise. Records the
// predicted release, the actual departure, and the true minimum gap from
// ground-truth positions (10x subsampled measurement).
//
// Miss (quality) vs hard fail (safety) are separate verdicts:
//   hard fail = collision (min_gap < 0) or goal never reached;
//   miss      = min_gap < margin (margin eaten) or |depart - initial_pred|
//               > 0.5 s (release-time error beyond the miss threshold).
// A run can be safe yet a miss (conservative bias costs time); a miss is a
// prediction-quality verdict, not a crash report.

#include "algo/predictor.hpp"

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

struct ReplayResult {
 bool reached{false};
 double initial_pred_s{std::numeric_limits<double>::infinity()};
 double depart_s{-1.0};
 double min_gap{std::numeric_limits<double>::infinity()};
};

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

ReplayResult run_replay(sim::Vec2 threat_p0, sim::Vec2 threat_v_true, sim::Vec2 threat_v_est,
                        double path_length = 6.0) {
 ReplayResult result;
 const int frames = static_cast<int>(kHorizonS / kDtS) + 1;
 double ego_s = 0.0;
 double time_s = 0.0;
 sim::Vec2 threat = threat_p0;
 for (int cycle = 0; cycle < 300 && ego_s < path_length - 1e-9; ++cycle) {
  const std::vector<sim::Vec2> remaining{{ego_s, 0.0}, {path_length, 0.0}};
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
  const sim::Vec2 ego0{ego_s, 0.0};
  if (dep == 0.0) {
   if (result.depart_s < 0.0) {
    result.depart_s = time_s;
   }
   ego_s = std::min(path_length, ego_s + kCruiseMps * kDtS);
  }
  const sim::Vec2 ego1{ego_s, 0.0};
  const sim::Vec2 next{threat.x + threat_v_true.x * kDtS, threat.y + threat_v_true.y * kDtS};
  result.min_gap =
    std::min(result.min_gap, measured_gap(ego0, ego1, threat, next, kThreatRadiusM + kEgoRadiusM));
  threat = next;
  time_s += kDtS;
 }
 result.reached = ego_s >= path_length - 1e-9;
 return result;
}

bool is_miss(const ReplayResult& r) {
 if (!r.reached || r.min_gap < 0.0) {
  return true;
 }
 if (r.min_gap < kMarginM) {
  return true;
 }
 if (r.depart_s < 0.0 ||
     std::abs(r.depart_s - r.initial_pred_s) > kMissTimeS + 1e-9) {
  return true;
 }
 return false;
}

} // namespace

int main() {
 // Nominal crosser, perfect estimation: prediction credible, countdown exact.
 {
  const ReplayResult r = run_replay({3.0, 2.5}, {0.0, -1.0}, {0.0, -1.0});
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
  const ReplayResult nominal = run_replay({3.0, 2.5}, {0.0, -1.0}, {0.0, -1.0});
  const ReplayResult r = run_replay({3.0, 2.5}, {0.0, -1.0}, {0.0, -0.7});
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
  const ReplayResult r = run_replay({3.0, 2.5}, {0.0, -1.0}, {0.0, -1.6});
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
  const ReplayResult r = run_replay({4.0, 0.0}, {-2.0, 0.0}, {-2.0, 0.0});
  CHECK(r.initial_pred_s == std::numeric_limits<double>::infinity());
  CHECK(r.min_gap < 0.0);
  CHECK(is_miss(r));
  std::cerr << "headon-wait-overrun: pred=inf gap=" << r.min_gap << "\n";
 }
 // Horizon too short for the whole traversal: open field, still infinity.
 // Unknown is not safe (SOTIF Area 3): no proof fits in the window.
 {
  const std::vector<sim::Vec2> long_path{{0.0, 0.0}, {10.0, 0.0}};
  const std::vector<std::vector<sim::DynState>> open(21, {{{50.0, 50.0}, {0.0, 0.0}, 0.4}});
  CHECK(sim::earliest_departure(long_path, 0, 1.0, open, 0.1, kEgoRadiusM, kMarginM) ==
        std::numeric_limits<double>::infinity());
 }
 std::cerr << "departure replay matrix green\n";
 return 0;
}
