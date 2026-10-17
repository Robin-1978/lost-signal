#include "sim/hardware/vehicle/contact.hpp"

#include <algorithm>
#include <cmath>

namespace sim {
namespace {

// Slip model constants: linear range edge and the low-speed denominator floor.
// Both are method constants (tire character), not per-content data.
constexpr double kSlipRef = 0.12;
constexpr double kSlipSpeedFloorMps = 0.3;

} // namespace

WheelContactResult step_driven_wheel(double wheel_mps, double body_mps, double pwm,
                                     const WheelContactParams& params, double dt) {
 const double curve = std::max(
 0.0, 1.0 - std::abs(wheel_mps) / std::max(0.05, params.free_speed_mps));
 const double motor_force = pwm * params.stall_force_N * curve;
 const double denom = std::max(kSlipSpeedFloorMps, std::abs(body_mps));
 const double stiffness = std::max(0.0, params.traction_limit_N) / (kSlipRef * denom);
 const double mass = std::max(0.5, params.wheel_equiv_mass_kg);
 const double wheel_free =
   (mass * wheel_mps / dt + motor_force + stiffness * body_mps) / (mass / dt + stiffness);
 const double force_free = stiffness * (wheel_free - body_mps);
 // Traction cap: the ground cannot return more than mu*N. Past the cap the
 // tread breaks away — the body gets the capped force, the wheel keeps the
 // remainder as spin. This is what makes the "saturated" in linear-saturated
 // true, identically for every chassis sharing this module.
 const double limit = std::max(0.0, params.traction_limit_N);
 if (std::abs(force_free) <= limit) {
  const double slip = (wheel_free - body_mps) / denom;
  return {wheel_free, force_free, slip, std::abs(slip) > kSlipRef};
 }
 const double capped = std::copysign(limit, force_free);
 const double wheel_capped = wheel_mps + (motor_force - capped) * dt / mass;
 const double slip = (wheel_capped - body_mps) / denom;
 return {wheel_capped, capped, slip, true};
}

} // namespace sim
