#pragma once

// SIM-owned tire contact (World internals, never ROBOT input).
// Rolling resistance in the same Coulomb+viscous form the ROBOT feedforward
// closes against (R = rr*load*tanh(v/s) + c*v): one formula, both sides.
// Traction-limit wheelspin needs wheel-vs-body slip state and belongs to
// the force-driven bodies (S3); this module owns rolling drag + stiction
// for the rigid-follow plant only.

namespace lost_signal::sim::world {

// Standard gravity: physics, not a tuned constant.
constexpr double kGravityMps2 = 9.81;

struct WheelContactParams {
  double rolling_resist = 0.01;  // combined tire+ground Coulomb coefficient
  double load_per_wheel_N = 0.0;  // normal load (vehicle mass * g / wheels)
  double coulomb_smoothing_mps = 0.05;  // tanh shoulder; stiction owns below
  double viscous_drag_N_per_mps = 0.6;
};

// Signed resistance ON the wheel: opposes the given motion, zero at rest.
// Odd in v (tanh + linear are odd), so reversing the wheel reverses the
// force exactly.
double contact_resistance_N(double wheel_mps, const WheelContactParams& params);

// Coulomb capacity for stiction: rr * load. Applied forces within this hold
// a slow wheel exactly at zero (no creep); beyond it the patch breaks away.
double contact_static_capacity_N(const WheelContactParams& params);

}  // namespace lost_signal::sim::world
