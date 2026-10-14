// Slip residual observer tests (design slice, no wiring).
// Headline: the steady sand slide the accel-diff detector missed
// (wheel 1.5 vs body 0.8) reads 0.467 Observable here; the creep corner
// it false-alarmed on (0.1 m/s) refuses as Unobservable.

#include "algo/slip_observer.hpp"

#include "hw/chassis/ackermann.hpp"
#include "hw/chassis/motor_drive.hpp"

#include <cmath>
#include <iostream>
#include <vector>

#define CHECK(condition)                                                                                     \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n";                            \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

namespace {

sim::AckermannDrive make_ro2() {
  sim::AckermannParams params;
  params.wheel_base_m = 0.62;
  params.wheel_radius_m = 0.1;
  params.track_width_m = 0.58;
  params.max_steer_rad = 0.55;
  params.max_steer_rate_radps = 2.0;
  params.servo_response_s = 0.35;
  params.max_drive_speed_mps = 2.4;
  params.max_drive_accel_mps2 = 6.0;
  params.stall_force_N = 54.0;
  params.free_speed_mps = 3.0;
  params.mass_kg = 26.0;
  params.pwm_limit = 1.0;
  params.integral_limit = 2.0;
  params.encoder_counts_per_revolution = 2048;
  const sim::WheelPidGains gains = sim::tune_wheel_pid(13.0, 54.0, 3.0, 1.2);
  return sim::AckermannDrive(params, gains, 1.0 / (0.35 * 2.0));
}

// Mission-equivalent contact (mission_simulation.cpp wheel_ground):
// mu = tire_mu * ground_mu / 0.8, rr = tire_rr + ground_rr, standard tire.
sim::GroundContact paved_contact() { return {0.7 * 1.0 / 0.8, 0.01 + 0.0, 6.0}; }
sim::GroundContact sand_contact() { return {0.7 * 0.55 / 0.8, 0.01 + 0.03, 6.0}; }

struct ProductionSample {
  double wheel_measured_mps; // encoder-quantized production signal.
  double body_true_mps;
  bool contact_slipping;
};

std::vector<ProductionSample> roll_straight(sim::GroundContact ground, double target_v, double seconds) {
  std::vector<ProductionSample> out;
  sim::AckermannDrive car = make_ro2();
  sim::DiffState state{0.0, 0.0, 0.0, 0.0, 0.0};
  for (int k = 0; k < static_cast<int>(seconds / 0.02); ++k) {
    state = car.step(state, {target_v, 0.0}, 0.02, ground);
    const auto& drive = car.telemetry().drive;
    out.push_back({drive.measured_mps, state.v, drive.slipping});
  }
  return out;
}

sim::PropagatedEstimate fresh_body(double v) {
  sim::PropagatedEstimate view;
  view.status = sim::EstimateStatus::kFresh;
  view.estimate.stamp_s = 0.0;
  view.estimate.latency_s = 0.05;
  view.estimate.state = {0.0, 0.0, 0.0, v, 0.0, 0.0};
  for (std::size_t i = 0; i < 36; ++i) {
    view.estimate.covariance[i] = (i % 7 == 0) ? 0.01 : 0.0;
  }
  view.estimate.wheel_valid = true;
  view.estimate.imu_valid = true;
  return view;
}

} // namespace

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
  // Production streams (no mission wiring): RO-2 straight accel 0->1.5 m/s,
  // wheel input = encoder-quantized production telemetry, body = Fresh truth
  // stream. Straight maneuvers never exceed the contact traction cap in this
  // envelope (slipping flag stays 0 — probed, not assumed), so this section
  // pins the NO-FALSE-POSITIVE side: quantization must not fake slip above
  // the floor, and the creep zone must refuse on the real quantized signal.
  // The positive-slip production case awaits a slipping-capable maneuver
  // (tracked scrub / grade / locked-wheel braking); physics is NOT tuned to
  // manufacture it — the synthetic 1.5/0.8 above remains the design proof.
  for (const auto& surface : {std::pair<const char*, sim::GroundContact>{"paved", paved_contact()},
                              std::pair<const char*, sim::GroundContact>{"sand", sand_contact()}}) {
    const auto samples = roll_straight(surface.second, 1.5, 5.0);
    int observables = 0;
    double ss_abs_sum = 0.0;
    int ss_n = 0;
    int slip_flags = 0;
    for (std::size_t i = 0; i < samples.size(); ++i) {
      if (samples[i].contact_slipping) {
        ++slip_flags;
      }
      const auto obs =
        sim::observe_slip({samples[i].wheel_measured_mps, 0.0}, fresh_body(samples[i].body_true_mps));
      if (i * 0.02 > 4.0) { // steady-state last second only (spool-up excluded).
        CHECK(obs.quality == sim::SlipQuality::kObservable);
        ss_abs_sum += std::abs(obs.longitudinal_ratio);
        ++ss_n;
      }
      if (obs.quality == sim::SlipQuality::kObservable) {
        ++observables;
      }
    }
    const double ss_mean = ss_abs_sum / std::max(1, ss_n);
    std::cerr << surface.first << ": steady_abs_ratio=" << ss_mean << " observables=" << observables
              << "/" << samples.size() << " slip_flags=" << slip_flags << "\n";
    CHECK(slip_flags == 0);
    CHECK(ss_mean < 0.03);
  }
  // Creep with real quantization (0.1 m/s demand): every sample refuses.
  {
    const auto samples = roll_straight(sand_contact(), 0.1, 3.0);
    for (const auto& sample : samples) {
      const auto obs =
        sim::observe_slip({sample.wheel_measured_mps, 0.0}, fresh_body(sample.body_true_mps));
      CHECK(obs.quality == sim::SlipQuality::kUnobservableLowSpeed);
      CHECK(obs.confidence == 0.0);
    }
    std::cerr << "creep: " << samples.size() << " samples all refused\n";
  }
  std::cerr << "slip observer green\n";
  return 0;
}
