// Estimator chain (offline, mission untouched): production chassis ->
// quantized/noisy streams -> linear CV filter -> slip residual observer.
// Answers: does the residual envelope from the direct-truth slice survive
// with the filter in the loop? No behavior, no release — chain evidence.

#include "algo/estimator_filter.hpp"
#include "algo/slip_observer.hpp"
#include "algo/state_estimator.hpp"
#include "hw/chassis/ackermann.hpp"
#include "hw/chassis/motor_drive.hpp"
#include "hw/chassis/tracked.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>

#define CHECK(condition)                                                                                     \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n";                            \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

namespace {

std::uint64_t rng_state{0xABCDEF01u};
double rand_uniform() {
  rng_state = rng_state * 6364136223846793005u + 1442695040888963407u;
  return static_cast<double>(rng_state >> 33) / static_cast<double>(1u << 31);
}
double randn() {
  const double u1 = std::max(1e-9, rand_uniform());
  const double u2 = rand_uniform();
  return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * 3.141592653589793 * u2);
}

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
  return sim::AckermannDrive(params, sim::tune_wheel_pid(13.0, 54.0, 3.0, 1.2), 1.0 / (0.35 * 2.0));
}

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
  return sim::TrackedDrive(params, sim::tune_wheel_pid(15.0, 54.0, 3.0, 1.2));
}

// Mission-equivalent contacts (standard tire 0.7/0.01).
sim::GroundContact sand_contact() { return {0.7 * 0.55 / 0.8, 0.01 + 0.03, 2.475}; }

sim::StateEstimate fresh_filter() {
  sim::StateEstimate estimate;
  estimate.stamp_s = 0.0;
  for (std::size_t i = 0; i < 36; ++i) {
    estimate.covariance[i] = 0.0;
  }
  estimate.covariance[0] = 1.0;
  estimate.covariance[7] = 1.0;
  estimate.covariance[14] = 0.5;
  estimate.covariance[21] = 0.5;
  estimate.covariance[28] = 0.5;
  estimate.covariance[35] = 0.1;
  estimate.wheel_valid = true;
  estimate.imu_valid = true;
  return estimate;
}

// Filter output passes through the holder (stamp discipline is part of the
// chain, not bypassed): query at the filter's own stamp with a wide budget.
sim::PropagatedEstimate holder_view(sim::EstimateHolder& holder, const sim::StateEstimate& estimate) {
  holder.push(estimate);
  return holder.at(estimate.stamp_s, 1.0, sim::DropoutPolicy::kHoldLast);
}

} // namespace

int main() {
  // Q covers chassis maneuver accel (stated test choice, as in filter test).
  constexpr double kDt = 0.02;
  constexpr double kQ = 0.5 * 0.5;
  constexpr double kQw = 0.1 * 0.1;
  // 1. Straight sand launch (RO-2): wheel-speed + laser-pos + yaw streams.
  {
    sim::AckermannDrive car = make_ro2();
    sim::DiffState state{0.0, 0.0, 0.0, 0.0, 0.0};
    sim::StateEstimate f = fresh_filter();
    sim::EstimateHolder holder;
    double max_ratio = 0.0;
    int n_obs = 0, n_stale = 0;
    for (int k = 0; k < 250; ++k) {
      state = car.step(state, {2.4, 0.0}, kDt, sand_contact());
      sim::filter_predict(f, kDt, kQ, kQw);
      const double measured = car.telemetry().drive.measured_mps;
      sim::filter_update_velocity(f, measured, 0.0, 0.03 * 0.03);
      if (k % 4 == 0) { // laser-like position at 12.5 Hz.
        sim::filter_update_position(f, state.x + randn() * 0.02, state.y + randn() * 0.02,
                                    0.02 * 0.02);
      }
      sim::filter_update_yaw(f, state.theta + randn() * 0.01, 0.01 * 0.01);
      const auto obs = sim::observe_slip({measured, 0.0}, holder_view(holder, f));
      if (obs.quality == sim::SlipQuality::kObservable) {
        max_ratio = std::max(max_ratio, std::abs(obs.longitudinal_ratio));
        ++n_obs;
      } else if (obs.quality == sim::SlipQuality::kStaleEstimator) {
        ++n_stale;
      }
    }
    std::cerr << "chain-launch: n_obs=" << n_obs << " n_stale=" << n_stale << " max_ratio=" << max_ratio
              << " body_v=" << state.v << "\n";
    CHECK(n_obs > 200);
    CHECK(max_ratio < 0.03);
  }
  // 2. Tracked scrub turn (RO-3): yaw shear visible through the filter.
  {
    sim::TrackedDrive car = make_ro3();
    sim::DiffState state{0.0, 0.0, 0.0, 0.0, 0.0};
    sim::StateEstimate f = fresh_filter();
    sim::EstimateHolder holder2;
    double ss_yaw = 0.0, ss_ratio = 0.0;
    int n_ss = 0;
    for (int k = 0; k < 250; ++k) {
      state = car.step(state, {1.0, 0.8}, kDt, sand_contact());
      const auto& tm = car.telemetry();
      const double wheel_v = (tm.left.measured_mps + tm.right.measured_mps) / 2.0;
      const double wheel_w = (tm.right.measured_mps - tm.left.measured_mps) / 0.55;
      sim::filter_predict(f, kDt, kQ, kQw);
      sim::filter_update_velocity(f, wheel_v * std::cos(f.state[2]), wheel_v * std::sin(f.state[2]),
                                  0.05 * 0.05);
      if (k % 4 == 0) {
        sim::filter_update_position(f, state.x + randn() * 0.02, state.y + randn() * 0.02,
                                    0.02 * 0.02);
      }
      sim::filter_update_yaw(f, state.theta + randn() * 0.01, 0.01 * 0.01);
      const auto obs = sim::observe_slip({wheel_v, wheel_w}, holder_view(holder2, f));
      if (k * kDt >= 3.0 && obs.quality == sim::SlipQuality::kObservable) {
        ss_yaw += std::abs(obs.yaw_residual_rads);
        ss_ratio += std::abs(obs.longitudinal_ratio);
        ++n_ss;
      }
    }
    ss_yaw /= std::max(1, n_ss);
    ss_ratio /= std::max(1, n_ss);
    std::cerr << "chain-scrub: n_ss=" << n_ss << " steady_yawres=" << ss_yaw
              << " steady_ratio=" << ss_ratio << "\n";
    CHECK(n_ss > 80);
    CHECK(ss_yaw > 0.10 && ss_yaw < 0.30);
    CHECK(ss_ratio < 0.03);
  }
  std::cerr << "estimator chain green\n";
  return 0;
}
