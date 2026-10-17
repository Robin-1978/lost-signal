#include "robot/autonomy/localization/estimator_filter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>

namespace sim {

namespace {

using Matrix6 = std::array<double, 36>;

double& at(Matrix6& m, std::size_t r, std::size_t c) { return m[r * 6 + c]; }
double at_const(const Matrix6& m, std::size_t r, std::size_t c) { return m[r * 6 + c]; }

// P = F * P * F' for the CV transition (x += vx*dt, y += vy*dt, th += w*dt).
// Only the affected rows/cols are touched; the rest is identity.
Matrix6 predict_covariance(const Matrix6& p, double dt) {
  Matrix6 out = p;
  // Position rows pick up velocity covariance: F = I + dt * d(pos)/d(vel).
  // New P[i][j] for i in {0,1,2} with driver d in {3,4,5}:
  // P[i][j] += dt * P[d][j]; then P[j][i] += dt * P[j][d] via symmetry pass.
  const std::size_t pos[3] = {0, 1, 2};
  const std::size_t vel[3] = {3, 4, 5};
  for (int a = 0; a < 3; ++a) {
    const std::size_t i = pos[a];
    const std::size_t d = vel[a];
    for (std::size_t j = 0; j < 6; ++j) {
      out[i * 6 + j] += dt * at_const(p, d, j);
    }
  }
  for (int a = 0; a < 3; ++a) {
    const std::size_t i = pos[a];
    const std::size_t d = vel[a];
    for (std::size_t j = 0; j < 6; ++j) {
      out[j * 6 + i] += dt * at_const(out, j, d);
    }
  }
  return out;
}

// Add discrete CV process noise per axis pair (pos p, vel v):
// Q = q * [[dt^3/3, dt^2/2], [dt^2/2, dt]] (velocity block is q*dt, NOT
// q*dt^2 — a 50x underweight at 0.02 s that starves the velocity variance
// and was caught by the same consistency pin as the aliasing bug).
void add_axis_noise(Matrix6& p, std::size_t pos, std::size_t vel, double q, double dt) {
  const double t2 = dt * dt;
  const double t3 = t2 * dt;
  at(p, pos, pos) += q * t3 / 3.0;
  at(p, pos, vel) += q * t2 / 2.0;
  at(p, vel, pos) += q * t2 / 2.0;
  at(p, vel, vel) += q * dt;
}

// Scalar Kalman update of state element k with measurement z, variance r.
void scalar_update(StateEstimate& estimate, std::size_t k, double z, double r) {
  if (!(r > 0.0) || !std::isfinite(z)) {
    return;
  }
  Matrix6& p = estimate.covariance;
  const double s = at(p, k, k) + r;
  if (!(s > 0.0) || !std::isfinite(s)) {
    return;
  }
  double gain[6];
  for (std::size_t j = 0; j < 6; ++j) {
    gain[j] = at(p, k, j) / s;
  }
  const double innov = z - estimate.state[k];
  estimate.state[k] += gain[k] * innov;
  // Rank-1 update P -= K * H * P (H picks row k). Row k MUST be snapshotted:
  // the double loop overwrites it at i == k, and later rows would otherwise
  // read the corrected row (in-place aliasing that silently breaks PSD —
  // caught by the consistency pin, invisible while cross terms are small).
  double row_k[6];
  for (std::size_t j = 0; j < 6; ++j) {
    row_k[j] = at_const(p, k, j);
  }
  for (std::size_t i = 0; i < 6; ++i) {
    for (std::size_t j = 0; j < 6; ++j) {
      at(p, i, j) -= gain[i] * row_k[j];
    }
  }
  for (std::size_t i = 0; i < 6; ++i) {
    for (std::size_t j = i + 1; j < 6; ++j) {
      const double sym = 0.5 * (at(p, i, j) + at(p, j, i));
      at(p, i, j) = sym;
      at(p, j, i) = sym;
    }
  }
}

} // namespace

void filter_predict(StateEstimate& estimate, double dt_s, double q_accel, double q_yaw_accel) {
  if (!(dt_s > 0.0) || !std::isfinite(dt_s)) {
    return;
  }
  estimate.state[0] += estimate.state[3] * dt_s;
  estimate.state[1] += estimate.state[4] * dt_s;
  estimate.state[2] += estimate.state[5] * dt_s;
  estimate.stamp_s += dt_s;
  Matrix6 p = predict_covariance(estimate.covariance, dt_s);
  const double q = std::max(0.0, q_accel);
  const double qw = std::max(0.0, q_yaw_accel);
  add_axis_noise(p, 0, 3, q, dt_s);
  add_axis_noise(p, 1, 4, q, dt_s);
  add_axis_noise(p, 2, 5, qw, dt_s);
  estimate.covariance = p;
}

void filter_update_position(StateEstimate& estimate, double z_x, double z_y, double r_xy) {
  scalar_update(estimate, 0, z_x, r_xy);
  scalar_update(estimate, 1, z_y, r_xy);
}

void filter_update_velocity(StateEstimate& estimate, double z_vx, double z_vy, double r_v) {
  scalar_update(estimate, 3, z_vx, r_v);
  scalar_update(estimate, 4, z_vy, r_v);
}

void filter_update_yaw(StateEstimate& estimate, double z_theta, double r_theta) {
  if (!std::isfinite(z_theta)) {
    return;
  }
  // Wrap innovation the short way before the scalar update.
  Matrix6& p = estimate.covariance;
  if (!(r_theta > 0.0)) {
    return;
  }
  const double s = at(p, 2, 2) + r_theta;
  if (!(s > 0.0) || !std::isfinite(s)) {
    return;
  }
  double gain[6];
  for (std::size_t j = 0; j < 6; ++j) {
    gain[j] = at(p, 2, j) / s;
  }
  double innov = z_theta - estimate.state[2];
  innov = std::atan2(std::sin(innov), std::cos(innov));
  estimate.state[2] += gain[2] * innov;
  // Same row-k snapshot requirement as scalar_update above.
  double row_2[6];
  for (std::size_t j = 0; j < 6; ++j) {
    row_2[j] = at_const(p, 2, j);
  }
  for (std::size_t i = 0; i < 6; ++i) {
    for (std::size_t j = 0; j < 6; ++j) {
      at(p, i, j) -= gain[i] * row_2[j];
    }
  }
  for (std::size_t i = 0; i < 6; ++i) {
    for (std::size_t j = i + 1; j < 6; ++j) {
      const double sym = 0.5 * (at(p, i, j) + at(p, j, i));
      at(p, i, j) = sym;
      at(p, j, i) = sym;
    }
  }
}

} // namespace sim
