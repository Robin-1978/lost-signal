// Stage-2 pins for the feedforward mapping: units, signs, limits, and the
// dynamics chain (lead -> wheel accel -> resistance -> curve -> PWM).
// Numbers are hand-computable from the documented formulas.

#include "hw/feedforward.hpp"

#include <cmath>
#include <iostream>

#define CHECK(condition)                                                                                     \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n";                           \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

namespace {

sim::FeedforwardParams params() {
 sim::FeedforwardParams p;
 p.wheel_mass_kg = 3.0;
 p.stall_force_N = 54.0;
 p.free_speed_mps = 3.0;
 p.rolling_resist = 0.02;
 p.load_per_unit_N = 24.0 * 9.81 / 2.0;
 return p;
}

} // namespace

int main() {
 // Lead: error over T_lead, per axis, no coupling.
 {
  const auto lead = sim::lead_accel(1.0, 0.6, 0.2, 0.0, 1.0 / 6.0);
  CHECK(std::abs(lead.a_mps2 - 2.4) < 1e-9);
  CHECK(std::abs(lead.alpha_radps2 - 1.2) < 1e-9);
  const auto still = sim::lead_accel(0.5, 0.5, 0.0, 0.0, 1.0 / 6.0);
  CHECK(still.a_mps2 == 0.0 && still.alpha_radps2 == 0.0);
 }
 // Rest at zero speed: no accel, no resistance -> zero, unsaturated.
 {
  const auto out = sim::wheel_feedforward(0.0, 0.0, params());
  CHECK(out.pwm_ff == 0.0 && !out.saturated);
 }
 // Forward accel at cruise: hand value.
 // R = 0.02*117.72*tanh(1/0.05) + 0.6*1 = 2.3544 + 0.6 = 2.9544;
 // F = 3*2 + 2.9544 = 8.9544; curve = 1-1/3 = 2/3; denom = 54*2/3 = 36.
 {
  const auto out = sim::wheel_feedforward(2.0, 1.0, params());
  CHECK(std::abs(out.pwm_ff - 8.9544 / 36.0) < 1e-9);
  CHECK(!out.saturated);
 }
 // Braking demand goes negative (sign preserved through resistance).
 {
  const auto out = sim::wheel_feedforward(-4.0, 1.0, params());
  CHECK(out.pwm_ff < 0.0 && !out.saturated);
 }
 // Stalled against a wall: demand far beyond authority clamps with flag.
 {
  const auto out = sim::wheel_feedforward(60.0, 0.0, params());
  CHECK(out.pwm_ff == 1.0 && out.saturated);
  const auto rev = sim::wheel_feedforward(-60.0, 0.0, params());
  CHECK(rev.pwm_ff == -1.0 && rev.saturated);
 }
 // Past free speed at rest demand: resistance still demands force the motor
 // cannot give -> saturated at full deflection (demand unmet, reported).
 {
  const auto out = sim::wheel_feedforward(0.0, 3.5, params());
  CHECK(out.pwm_ff == 1.0 && out.saturated);
 }
 // Past free speed with demand: saturated at full deflection, never NaN.
 {
  const auto out = sim::wheel_feedforward(2.0, 3.5, params());
  CHECK(out.pwm_ff == 1.0 && out.saturated);
  CHECK(std::isfinite(out.pwm_ff));
 }
 // Twin map: pure yaw gives antisymmetric wheel accels; straight gives equal.
 {
  const auto spin = sim::twist_feedforward({0.0, 2.0}, 0.0, 0.0, 0.52, params());
  CHECK(std::abs(spin.left.pwm_ff + spin.right.pwm_ff) < 1e-9);
  CHECK(spin.left.pwm_ff < 0.0 && spin.right.pwm_ff > 0.0);
  const auto straight = sim::twist_feedforward({1.0, 0.0}, 0.5, 0.0, 0.52, params());
  CHECK(std::abs(straight.left.pwm_ff - straight.right.pwm_ff) < 1e-12);
 }
 std::cerr << "feedforward mapping green\n";
 return 0;
}
