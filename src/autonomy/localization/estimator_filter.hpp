#pragma once

#include "autonomy/localization/state_estimator.hpp"

namespace sim {

// Estimator filter (design slice, no producers/consumers wired).
// The contract froze time management (holder) with an explicit gap: NO
// covariance growth — inventing process noise there would be tuning disguised
// as math. This module is where growth lives, with the discipline inverted:
// every noise number is a caller-supplied argument, never a hidden constant.
// Content-derived values (IMU accelerometer specs) may flow in from JSON at
// the call site; the filter itself owns no numbers.
//
// Model: constant-velocity planar motion, which is LINEAR in the 6D state
// [x, y, theta, vx, vy, w] (x += vx*dt, theta += w*dt). No EKF, no
// linearization error — the day the model goes nonlinear (bias states,
// contact coupling), this file gets a stated replacement, not a silent one.
// Process noise enters as continuous white noise on (ax, ay, alpha) with
// caller-given spectral densities; discrete Q via the textbook CV form
// (dt^3/3, dt^2/2, dt per axis). Measurement updates are sequential scalar
// Kalman updates (position x/y, velocity vx/vy, yaw) with caller-given R —
// equivalent to the vector form for diagonal R, simpler to audit.
// Velocity here is the BODY estimate the slip observer will consume;
// wheel twist stays an independent input to the residual (never fused in).

void filter_predict(StateEstimate& estimate, double dt_s, double q_accel, double q_yaw_accel);

// Scalar updates. Innovation on yaw is wrapped (short way around).
// Non-positive R or non-finite z: update skipped (refusal, not clamping).
void filter_update_position(StateEstimate& estimate, double z_x, double z_y, double r_xy);
void filter_update_velocity(StateEstimate& estimate, double z_vx, double z_vy, double r_v);
void filter_update_yaw(StateEstimate& estimate, double z_theta, double r_theta);

} // namespace sim
