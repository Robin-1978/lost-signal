// Estimator replay + latency validation (observation only).
// A: per-source latency distributions (synthetic deterministic streams).
// B: same trajectory x {0,20,50,100,200ms} -> gap-error table (margin evidence).
// C: dropout x {0,50,100,300ms} x policies -> status/confidence/position error.

#include "autonomy/localization/estimator_replay.hpp"

#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

#define CHECK(condition)                                                                                     \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n";                            \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

namespace {

// Deterministic jittered stream: base + (k mod span) * step.
std::vector<double> make_stream(std::size_t n, double base_s, double span_step_s, int span) {
  std::vector<double> out;
  for (std::size_t k = 0; k < n; ++k) {
    out.push_back(base_s + static_cast<double>(k % static_cast<std::size_t>(span)) * span_step_s);
  }
  return out;
}

} // namespace

int main() {
  // A. Time consistency: three sources, 200 samples each.
  {
    const auto imu = make_stream(200, 0.003, 0.0005, 5);    // ~3-5 ms
    const auto wheel = make_stream(200, 0.006, 0.001, 5);   // ~6-10 ms
    const auto laser = make_stream(200, 0.040, 0.010, 5);   // ~40-80 ms
    const auto si = sim::latency_stats(imu);
    const auto sw = sim::latency_stats(wheel);
    const auto sl = sim::latency_stats(laser);
    std::cerr << "imu:   n=" << si.n << " mean=" << si.mean_s << " p50=" << si.p50_s << " p95=" << si.p95_s
              << " p99=" << si.p99_s << " max=" << si.max_s << "\n";
    std::cerr << "wheel: n=" << sw.n << " mean=" << sw.mean_s << " p50=" << sw.p50_s << " p95=" << sw.p95_s
              << " p99=" << sw.p99_s << " max=" << sw.max_s << "\n";
    std::cerr << "laser: n=" << sl.n << " mean=" << sl.mean_s << " p50=" << sl.p50_s << " p95=" << sl.p95_s
              << " p99=" << sl.p99_s << " max=" << sl.max_s << "\n";
    CHECK(si.n == 200 && sw.n == 200 && sl.n == 200);
    CHECK(si.p95_s < sw.p95_s && sw.p95_s < sl.p95_s); // ordering, not thresholds
    CHECK(si.max_s < 0.010 && sw.max_s < 0.015 && sl.max_s < 0.100);
    const auto empty = sim::latency_stats({});
    CHECK(empty.n == 0 && empty.max_s == 0.0);
  }
  // B. Latency eats margin: ego 1 m/s along +x for 4 s, threat crossing x=3
  // at 2 m/s (near-miss: closest approach 1.0 m minus radii). Same threat,
  // five latencies.
  {
    std::vector<sim::TimedPose> ego;
    for (int k = 0; k <= 200; ++k) {
      ego.push_back({k * 0.02, {k * 0.02 * 1.0, 0.0}});
    }
    std::vector<std::vector<sim::DynamicObjectState>> frames;
    for (int f = 0; f <= 60; ++f) {
      const double t = f * 0.1;
      frames.push_back({{{3.0, -4.0 + 2.0 * t}, {0.0, 2.0}, 0.4}});
    }
    const std::vector<double> lags{0.0, 0.02, 0.05, 0.10, 0.20};
    const auto rows = sim::gap_vs_latency(ego, frames, 0.1, 0.38, lags);
    CHECK(rows.size() == 5);
    for (const auto& r : rows) {
      std::cerr << "latency=" << r.latency_s * 1000.0 << "ms gap=" << r.gap_m
                << " err=" << r.gap_error_m << "\n";
    }
    CHECK(std::abs(rows[0].gap_error_m) < 1e-9);
    for (const auto& r : rows) {
      CHECK(std::isfinite(r.gap_m) && std::isfinite(r.gap_error_m));
    }
    // Pinned table (deterministic): latency eats ~0.9x itself in gap error
    // here (threat 2 m/s geometry). Monotonic non-decreasing, and POSITIVE:
    // stale ego sits behind truth, the threat passes ahead, the planner sees
    // MORE margin than exists — the optimistic (dangerous) direction. A
    // future margin cites this table; this slice sets none.
    CHECK(rows[0].gap_m > 0.10 && rows[0].gap_m < 0.13);
    for (std::size_t i = 1; i < rows.size(); ++i) {
      CHECK(rows[i].gap_error_m >= rows[i - 1].gap_error_m);
      CHECK(rows[i].gap_error_m > 0.0);
    }
    CHECK(rows[1].gap_error_m > 0.010 && rows[1].gap_error_m < 0.030);
    CHECK(rows[2].gap_error_m > 0.045 && rows[2].gap_error_m < 0.065);
    CHECK(rows[3].gap_error_m > 0.080 && rows[3].gap_error_m < 0.100);
    CHECK(rows[4].gap_error_m > 0.170 && rows[4].gap_error_m < 0.190);
  }
  // C. Dropout at the stream END (the only kind that matters at query time:
  // a mid-stream gap heals once the stream resumes; a gap covering the query
  // is what the consumer actually decides on). Window [query-drop, query).
  {
    std::vector<sim::TimedPose> truth;
    for (int k = 0; k <= 100; ++k) {
      truth.push_back({k * 0.02, {k * 0.02 * 1.0, 0.0}});
    }
    const double query = 2.0;
    const double staleness = 0.1;
    auto run = [&](double drop, sim::DropoutPolicy policy) {
      return sim::dropout_observe(truth, 0.05, query - drop, drop, query, staleness, policy);
    };
    for (double drop : {0.0, 0.05, 0.15, 0.30}) {
      const auto hold = run(drop, sim::DropoutPolicy::kHoldLast);
      const auto cv = run(drop, sim::DropoutPolicy::kPredictConstantVelocity);
      const auto flag = run(drop, sim::DropoutPolicy::kFlagInvalid);
      std::cerr << "drop=" << drop * 1000.0 << "ms hold(st=" << static_cast<int>(hold.status)
                << " conf=" << hold.confidence << " err=" << hold.position_error_m << ")"
                << " cv(st=" << static_cast<int>(cv.status) << " err=" << cv.position_error_m << ")"
                << " flag(st=" << static_cast<int>(flag.status) << " conf=" << flag.confidence << ")\n";
    }
    // No dropout: fresh everywhere. 0.3 s past a 0.1 s staleness budget:
    // hold/cv go stale (never silently fresh), flag refuses. Within budget
    // (50 ms) the mean is trusted as-is: Fresh with 0.06 m residual — the
    // budget's honest cost, not hidden.
    CHECK(run(0.0, sim::DropoutPolicy::kHoldLast).status == sim::EstimateStatus::kFresh);
    CHECK(run(0.05, sim::DropoutPolicy::kHoldLast).status == sim::EstimateStatus::kFresh);
    const auto stale = run(0.30, sim::DropoutPolicy::kHoldLast);
    CHECK(stale.status == sim::EstimateStatus::kPredictedStale);
    // Hold freezes: error ~= dropout x speed (1 m/s); CV rides the stream
    // velocity and stays near truth. The spread IS the policy comparison.
    CHECK(stale.position_error_m > 0.20 && stale.position_error_m < 0.40);
    const auto cv = run(0.30, sim::DropoutPolicy::kPredictConstantVelocity);
    CHECK(cv.status == sim::EstimateStatus::kPredictedStale);
    CHECK(cv.position_error_m < 0.05);
    const auto refused = run(0.30, sim::DropoutPolicy::kFlagInvalid);
    CHECK(refused.status == sim::EstimateStatus::kInvalid);
    CHECK(refused.confidence == 0.0);
    CHECK(!std::isfinite(refused.position_error_m));
  }
  std::cerr << "estimator replay green\n";
  return 0;
}
