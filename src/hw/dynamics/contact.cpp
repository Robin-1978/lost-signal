#include "contact.hpp"

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
 const double curve = std::max(0.0, 1.0 - std::abs(wheel_mps) / std::max(0.05, params.free_speed_mps));
 const double motor_force = pwm * params.stall_force_N * curve;
 const double denom = std::max(kSlipSpeedFloorMps, std::abs(body_mps));
 const double stiffness = std::max(0.0, params.traction_limit_N) / (kSlipRef * denom);
 const double mass = std::max(0.5, params.wheel_equiv_mass_kg);
 const double wheel_new =
   (mass * wheel_mps / dt + motor_force + stiffness * body_mps) / (mass / dt + stiffness);
 const double slip = (wheel_new - body_mps) / denom;
 const double body_force = stiffness * (wheel_new - body_mps);
 return {wheel_new, body_force, slip, std::abs(slip) > kSlipRef};
}

} // namespace sim
