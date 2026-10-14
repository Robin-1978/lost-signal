// Slip residual observer tests (design slice, no wiring).
// Headline: the steady sand slide the accel-diff detector missed
// (wheel 1.5 vs body 0.8) reads 0.467 Observable here; the creep corner
// it false-alarmed on (0.1 m/s) refuses as Unobservable.

#include "algo/slip_observer.hpp"

#include "hw/chassis/ackermann.hpp"
#include "hw/chassis/motor_drive.hpp"
#include "hw/chassis/tracked.hpp"

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
sim::GroundContact water_contact() { return {0.7 * 0.35 / 0.8, 0.01 + 0.02, 1.7375}; }

sim::TrackedDrive make_ro3() {
  sim::TrackedParams params;
  params.wheel_base_m = 0.55;
  params.wheel_radius_m = 0.1;
  params.track_length_m = 0.7;
  params.max_drive_speed_mps = 2.4;
  params.max_drive_accel_mps2 = 6.0;
  params.stall_force_N = 54.0;
  params.free_speed_mps = 3.0;
  params.mass_kg = 30.0;
  params.yaw_inertia_kgm2 = 3.0;
  params.max_yaw_rate_rads = 1.5;
  params.pwm_limit = 1.0;
  params.integral_limit = 2.0;
  params.encoder_counts_per_revolution = 2048;
  params.wheel_equiv_mass_kg = 3.0;
  const sim::WheelPidGains gains = sim::tune_wheel_pid(15.0, 54.0, 3.0, 1.2);
  return sim::TrackedDrive(params, gains);
}

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
  // Excitation matrix (data only, no detector): each maneuver reports its
  // residual envelope {n_obs, max|ratio|, steady mean|yawres|}. Contacts are
  // mission-equivalent; nothing is tuned to manufacture slip. Grade launch is
  // replaced by sand max launch (no height fields in sim — the highest
  // sustained torque demand inside the envelope, stated proxy).
  // Longitudinal truth note: tracked/ackermann telemetry reports body-implied
  // tread speeds as "actual", so the pinned numbers below are the OBSERVED
  // residual envelope (input to future confidence calibration), not a
  // truth-spread decomposition — quantization vs shear cannot be separated
  // from these signals, and the table does not pretend otherwise.
  {
    // 1. Hard launch 0->2.4 on water (RO-2): transient residual bounded.
    sim::AckermannDrive car = make_ro2();
    sim::DiffState state{0.0, 0.0, 0.0, 0.0, 0.0};
    double max_ratio = 0.0;
    int n_obs = 0;
    for (int k = 0; k < 250; ++k) {
      state = car.step(state, {2.4, 0.0}, 0.02, water_contact());
      const auto obs = sim::observe_slip({car.telemetry().drive.measured_mps, 0.0},
                                         fresh_body(state.v));
      if (obs.quality == sim::SlipQuality::kObservable) {
        max_ratio = std::max(max_ratio, std::abs(obs.longitudinal_ratio));
        ++n_obs;
      }
    }
    std::cerr << "launch: n_obs=" << n_obs << " max_ratio=" << max_ratio << "\n";
    CHECK(n_obs > 200);
    CHECK(max_ratio < 0.06);
  }
  {
    // 2. Emergency stop 2.4->0 on water (RO-2): negative residual bounded,
    // wheels never lock in this envelope.
    sim::AckermannDrive car = make_ro2();
    sim::DiffState state{0.0, 0.0, 0.0, 0.0, 0.0};
    for (int k = 0; k < 150; ++k) {
      state = car.step(state, {2.4, 0.0}, 0.02, water_contact());
    }
    double min_ratio = 0.0;
    int n_obs = 0;
    for (int k = 0; k < 150; ++k) {
      state = car.step(state, {0.0, 0.0}, 0.02, water_contact());
      const auto obs = sim::observe_slip({car.telemetry().drive.measured_mps, 0.0},
                                         fresh_body(state.v));
      if (obs.quality == sim::SlipQuality::kObservable) {
        min_ratio = std::min(min_ratio, obs.longitudinal_ratio);
        ++n_obs;
      }
    }
    std::cerr << "estop: n_obs=" << n_obs << " min_ratio=" << min_ratio << "\n";
    CHECK(n_obs > 0);
    CHECK(min_ratio > -0.03);
  }
  {
    // 3. Sand max launch 0->2.4 (RO-2, grade proxy): bounded, contact quiet.
    sim::AckermannDrive car = make_ro2();
    sim::DiffState state{0.0, 0.0, 0.0, 0.0, 0.0};
    double max_ratio = 0.0;
    int n_obs = 0, slip_flags = 0;
    for (int k = 0; k < 250; ++k) {
      state = car.step(state, {2.4, 0.0}, 0.02, sand_contact());
      if (car.telemetry().drive.slipping) {
        ++slip_flags;
      }
      const auto obs = sim::observe_slip({car.telemetry().drive.measured_mps, 0.0},
                                         fresh_body(state.v));
      if (obs.quality == sim::SlipQuality::kObservable) {
        max_ratio = std::max(max_ratio, std::abs(obs.longitudinal_ratio));
        ++n_obs;
      }
    }
    std::cerr << "sand-launch: n_obs=" << n_obs << " max_ratio=" << max_ratio
              << " slip_flags=" << slip_flags << "\n";
    CHECK(slip_flags == 0);
    CHECK(max_ratio < 0.09);
  }
  {
    // 4. Tracked scrub turn v=1 w=0.8 on sand (RO-3): longitudinal quiet,
    // yaw channel sees the shear — tread-implied yaw (~0.22) vs body yaw
    // (~0.05), sustained. The contact slipping flag stays quiet (it is
    // force-cap based); the shear is kinematic, and the residual is where
    // it shows. Body yaw chatters step-to-step (scrub stick-slip, recorded
    // as truth-side behavior, not asserted as slip).
    sim::TrackedDrive car = make_ro3();
    sim::DiffState state{0.0, 0.0, 0.0, 0.0, 0.0};
    double ss_ratio = 0.0, ss_yaw = 0.0;
    int n_ss = 0, n_obs = 0;
    for (int k = 0; k < 250; ++k) {
      state = car.step(state, {1.0, 0.8}, 0.02, sand_contact());
      const auto& tm = car.telemetry();
      const double wheel_v = (tm.left.measured_mps + tm.right.measured_mps) / 2.0;
      const double wheel_w = (tm.right.measured_mps - tm.left.measured_mps) / 0.55;
      const auto obs = sim::observe_slip({wheel_v, wheel_w}, fresh_body(state.v));
      // Body yaw truth for the yaw channel: fresh_body carries w=0, so
      // re-observe with true yaw here for calibration honesty.
      sim::PropagatedEstimate truth_yaw = fresh_body(state.v);
      truth_yaw.estimate.state[5] = state.w;
      const auto obs_yaw = sim::observe_slip({wheel_v, wheel_w}, truth_yaw);
      if (obs.quality == sim::SlipQuality::kObservable) {
        ++n_obs;
      }
      if (k * 0.02 >= 3.0 && obs_yaw.quality == sim::SlipQuality::kObservable) {
        ss_ratio += std::abs(obs_yaw.longitudinal_ratio);
        ss_yaw += std::abs(obs_yaw.yaw_residual_rads);
        ++n_ss;
      }
    }
    ss_ratio /= std::max(1, n_ss);
    ss_yaw /= std::max(1, n_ss);
    std::cerr << "scrub: n_obs=" << n_obs << " steady_ratio=" << ss_ratio << " steady_yawres=" << ss_yaw
              << "\n";
    CHECK(n_obs > 200);
    CHECK(ss_ratio < 0.03);
    CHECK(ss_yaw > 0.10 && ss_yaw < 0.25);
  }
  {
    // 5. Spin in place (RO-3, sand): negative control — v~0 on both sides,
    // nothing observable, nothing reported.
    sim::TrackedDrive car = make_ro3();
    sim::DiffState state{0.0, 0.0, 0.0, 0.0, 0.0};
    int n_obs = 0;
    for (int k = 0; k < 100; ++k) {
      state = car.step(state, {0.0, 1.0}, 0.02, sand_contact());
      const auto& tm = car.telemetry();
      const double wheel_v = (tm.left.measured_mps + tm.right.measured_mps) / 2.0;
      const auto obs = sim::observe_slip({wheel_v, 0.0}, fresh_body(state.v));
      if (obs.quality == sim::SlipQuality::kObservable) {
        ++n_obs;
      }
    }
    std::cerr << "spin: n_obs=" << n_obs << " (negative control)\n";
    CHECK(n_obs == 0);
  }
  std::cerr << "slip observer green\n";
  return 0;
}
