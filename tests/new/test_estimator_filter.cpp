// Estimator filter tests (design slice, no wiring).
// Synthetic truth + known-noise streams: the filter must beat raw
// measurements (RMSE), stay consistent (3-sigma), grow covariance in
// dropout and shrink it on update, refuse bad inputs, wrap yaw.

#include "robot/autonomy/localization/estimator_filter.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
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

// Deterministic PRNG (LCG, fixed seed — no rand()).
std::uint64_t rng_state{0x12345678u};
double rand_uniform() {
  rng_state = rng_state * 6364136223846793005u + 1442695040888963407u;
  return static_cast<double>(rng_state >> 33) / static_cast<double>(1u << 31);
}
double randn() {
  // Box-Muller from the LCG stream.
  const double u1 = std::max(1e-9, rand_uniform());
  const double u2 = rand_uniform();
  return std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * 3.141592653589793 * u2);
}

sim::StateEstimate fresh_filter() {
  sim::StateEstimate estimate;
  estimate.stamp_s = 0.0;
  for (std::size_t i = 0; i < 36; ++i) {
    estimate.covariance[i] = 0.0;
  }
  estimate.covariance[0] = 1.0;
  estimate.covariance[7] = 1.0;
  estimate.covariance[14] = 0.5;
  estimate.covariance[21] = 0.1;
  estimate.covariance[28] = 0.1;
  estimate.covariance[35] = 0.05;
  estimate.wheel_valid = true;
  estimate.imu_valid = true;
  return estimate;
}

double trace_of(const sim::StateEstimate& e) {
  return e.covariance[0] + e.covariance[7] + e.covariance[14] + e.covariance[21] + e.covariance[28] +
         e.covariance[35];
}

} // namespace

int main() {
  // Truth: accelerate 0->1.2 m/s over 3 s with a gentle turn (w 0->0.2).
  // Streams: position R=0.02^2 (laser-like), velocity R=0.03^2 (wheel-like),
  // yaw R=0.01^2 (IMU-like). Q covers the TRUTH maneuver (stated test choice,
  // not production tuning): peak accel 0.4 m/s^2 + centripetal v*w ~= 0.24 ->
  // sigma 0.5; yaw accel 0.067 rad/s^2 -> sigma 0.1. Production Q comes from
  // identification (how much accel the CV model misses per maneuver class);
  // the filter owns no numbers, only machinery.
  constexpr double kDt = 0.02;
  constexpr double kQ = 0.5 * 0.5;
  constexpr double kQw = 0.1 * 0.1;
  constexpr double kRp = 0.02 * 0.02;
  constexpr double kRv = 0.03 * 0.03;
  constexpr double kRy = 0.01 * 0.01;
  double tx = 0.0, ty = 0.0, th = 0.0, tv = 0.0, tw = 0.0;
  sim::StateEstimate f = fresh_filter();
  double raw_sq = 0.0, filt_sq = 0.0;
  int consistent = 0, total = 0;
  for (int k = 0; k < 300; ++k) {
    tv = std::min(1.2, tv + 1.2 / 150.0);
    tw = std::min(0.2, tw + 0.2 / 150.0);
    tx += tv * std::cos(th) * kDt;
    ty += tv * std::sin(th) * kDt;
    th += tw * kDt;
    sim::filter_predict(f, kDt, kQ, kQw);
    const double zx = tx + randn() * 0.02;
    const double zy = ty + randn() * 0.02;
    const double zvx = tv * std::cos(th) + randn() * 0.03;
    const double zvy = tv * std::sin(th) + randn() * 0.03;
    const double zt = th + randn() * 0.01;
    sim::filter_update_position(f, zx, zy, kRp);
    sim::filter_update_velocity(f, zvx, zvy, kRv);
    sim::filter_update_yaw(f, zt, kRy);
    raw_sq += (zx - tx) * (zx - tx) + (zy - ty) * (zy - ty);
    filt_sq += (f.state[0] - tx) * (f.state[0] - tx) + (f.state[1] - ty) * (f.state[1] - ty);
    const double sx = std::sqrt(std::max(1e-12, f.covariance[0]));
    const double sy = std::sqrt(std::max(1e-12, f.covariance[7]));
    if (std::abs(f.state[0] - tx) <= 3.0 * sx && std::abs(f.state[1] - ty) <= 3.0 * sy) {
      ++consistent;
    }
    ++total;
  }
  const double raw_rmse = std::sqrt(raw_sq / total);
  const double filt_rmse = std::sqrt(filt_sq / total);
  std::cerr << "raw_rmse=" << raw_rmse << " filt_rmse=" << filt_rmse << " consistent=" << consistent
            << "/" << total << " trace=" << trace_of(f) << "\n";
  CHECK(filt_rmse < raw_rmse);
  CHECK(filt_rmse < 0.015);
  CHECK(consistent >= 270);
  // Dropout growth then update shrink (same filter, continued).
  {
    const double before = trace_of(f);
    for (int k = 0; k < 50; ++k) {
      sim::filter_predict(f, kDt, kQ, kQw);
    }
    const double grown = trace_of(f);
    sim::filter_update_position(f, f.state[0], f.state[1], kRp);
    sim::filter_update_velocity(f, f.state[3], f.state[4], kRv);
    const double shrunk = trace_of(f);
    std::cerr << "dropout: before=" << before << " grown=" << grown << " shrunk=" << shrunk << "\n";
    CHECK(grown > before);
    CHECK(grown > 0.1);
    CHECK(shrunk < grown);
    CHECK(shrunk < 0.05);
  }
  // Refusal: bad R / NaN z leave the state untouched.
  {
    sim::StateEstimate g = fresh_filter();
    const sim::StateEstimate snapshot = g;
    sim::filter_update_position(g, 5.0, 5.0, 0.0);
    sim::filter_update_position(g, 5.0, 5.0, -1.0);
    sim::filter_update_velocity(g, std::numeric_limits<double>::quiet_NaN(),
                                std::numeric_limits<double>::quiet_NaN(), kRv);
    sim::filter_update_yaw(g, 1.0, 0.0);
    sim::filter_predict(g, -0.5, kQ, kQw);
    for (std::size_t i = 0; i < 6; ++i) {
      CHECK(g.state[i] == snapshot.state[i]);
    }
    for (std::size_t i = 0; i < 36; ++i) {
      CHECK(g.covariance[i] == snapshot.covariance[i]);
    }
  }
  // Yaw wrap: state near +pi, measurement near -pi — converges short way.
  {
    sim::StateEstimate g = fresh_filter();
    g.state[2] = 3.1;
    for (int k = 0; k < 20; ++k) {
      sim::filter_update_yaw(g, -3.1, kRy);
    }
    std::cerr << "wrapped theta=" << g.state[2] << "\n";
    CHECK(std::abs(std::abs(g.state[2]) - 3.14159265) < 0.1);
  }
  // Determinism: same seed stream twice, bit-identical.
  {
    rng_state = 0x12345678u;
    sim::StateEstimate a = fresh_filter();
    for (int k = 0; k < 10; ++k) {
      sim::filter_predict(a, kDt, kQ, kQw);
      sim::filter_update_position(a, randn(), randn(), kRp);
    }
    rng_state = 0x12345678u;
    sim::StateEstimate b = fresh_filter();
    for (int k = 0; k < 10; ++k) {
      sim::filter_predict(b, kDt, kQ, kQw);
      sim::filter_update_position(b, randn(), randn(), kRp);
    }
    for (std::size_t i = 0; i < 6; ++i) {
      CHECK(a.state[i] == b.state[i]);
    }
    for (std::size_t i = 0; i < 36; ++i) {
      CHECK(a.covariance[i] == b.covariance[i]);
    }
  }
  std::cerr << "estimator filter green\n";
  return 0;
}
