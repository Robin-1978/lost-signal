// Slip residual observer tests (design slice, no wiring).
// Headline: the steady sand slide the accel-diff detector missed
// (wheel 1.5 vs body 0.8) reads 0.467 Observable here; the creep corner
// it false-alarmed on (0.1 m/s) refuses as Unobservable.

#include "algo/slip_observer.hpp"

#include <cmath>
#include <iostream>

#define CHECK(condition)                                                                                     \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n";                            \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

namespace {

sim::PropagatedEstimate make_body(double vx, double vy, double w, sim::EstimateStatus status) {
  sim::PropagatedEstimate view;
  view.status = status;
  view.estimate.stamp_s = 10.0;
  view.estimate.latency_s = 0.05;
  view.estimate.state = {0.0, 0.0, 0.0, vx, vy, w};
  for (std::size_t i = 0; i < 36; ++i) {
    view.estimate.covariance[i] = (i % 7 == 0) ? 0.01 : 0.0;
  }
  view.estimate.wheel_valid = true;
  view.estimate.imu_valid = true;
  return view;
}

} // namespace

int main() {
  // Steady sand slide: wheel 1.5, body 0.8 — the old metric's blind spot.
  {
    const auto obs = sim::observe_slip({1.5, 0.0}, make_body(0.8, 0.0, 0.0, sim::EstimateStatus::kFresh));
    std::cerr << "sand: ratio=" << obs.longitudinal_ratio << " conf=" << obs.confidence
              << " quality=" << sim::slip_quality_name(obs.quality) << "\n";
    CHECK(obs.quality == sim::SlipQuality::kObservable);
    CHECK(std::abs(obs.longitudinal_ratio - 0.7 / 1.5) < 1e-9);
    CHECK(std::abs(obs.confidence - 1.0 / 1.06) < 1e-9);
    CHECK(obs.yaw_residual_rads == 0.0);
  }
  // Clean rolling: zero residual, full confidence.
  {
    const auto obs = sim::observe_slip({1.0, 0.1}, make_body(1.0, 0.0, 0.1, sim::EstimateStatus::kFresh));
    CHECK(obs.quality == sim::SlipQuality::kObservable);
    CHECK(obs.longitudinal_ratio == 0.0);
    CHECK(obs.yaw_residual_rads == 0.0);
  }
  // Creep corner (v ~ 0.1, the old detector's chatter zone): refused.
  {
    const auto obs = sim::observe_slip({0.1, 0.0}, make_body(0.05, 0.0, 0.0, sim::EstimateStatus::kFresh));
    CHECK(obs.quality == sim::SlipQuality::kUnobservableLowSpeed);
    CHECK(obs.confidence == 0.0);
  }
  // Locked wheel (body outruns wheels): negative, denominator floored.
  {
    const auto obs = sim::observe_slip({0.2, 0.0}, make_body(1.0, 0.0, 0.0, sim::EstimateStatus::kFresh));
    CHECK(obs.quality == sim::SlipQuality::kObservable);
    CHECK(std::abs(obs.longitudinal_ratio - (0.2 - 1.0) / 0.5) < 1e-9);
  }
  // Yaw skid: straight wheels, spinning body.
  {
    const auto obs = sim::observe_slip({1.0, 0.0}, make_body(1.0, 0.0, 0.3, sim::EstimateStatus::kFresh));
    CHECK(obs.quality == sim::SlipQuality::kObservable);
    CHECK(std::abs(obs.yaw_residual_rads - (-0.3)) < 1e-9);
  }
  // Stale estimator: residual computed, confidence halved, quality says so.
  {
    const auto obs =
      sim::observe_slip({1.5, 0.0}, make_body(0.8, 0.0, 0.0, sim::EstimateStatus::kPredictedStale));
    CHECK(obs.quality == sim::SlipQuality::kStaleEstimator);
    CHECK(std::abs(obs.longitudinal_ratio - 0.7 / 1.5) < 1e-9);
    CHECK(std::abs(obs.confidence - 0.5 / 1.06) < 1e-9);
  }
  // Invalid estimator: refused, never a number dressed as one.
  {
    sim::PropagatedEstimate empty;
    const auto obs = sim::observe_slip({1.5, 0.0}, empty);
    CHECK(obs.quality == sim::SlipQuality::kInvalid);
    CHECK(obs.confidence == 0.0);
  }
  std::cerr << "slip observer green\n";
  return 0;
}
