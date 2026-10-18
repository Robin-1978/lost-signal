#include "sim/hardware/vehicle/contact.hpp"

#include "sim/hardware/actuators/electric_drive.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sim {
namespace {

// Slip model constants: linear range edge and the low-speed denominator floor.
// Both are method constants (tire character), not per-content data.
constexpr double kSlipRef = 0.12;
constexpr double kSlipSpeedFloorMps = 0.3;

} // namespace

WheelContactResult step_driven_wheel(double wheel_mps, double body_mps, double pwm,
                                     const WheelContactParams& params, double dt) {
 if (!(dt > 0.0) || !std::isfinite(dt))
 throw std::invalid_argument("wheel contact step must be finite and positive");
 const double denom = std::max(kSlipSpeedFloorMps, std::abs(body_mps));
 const double stiffness = std::max(0.0, params.traction_limit_N) / (kSlipRef * denom);
 const double mass = std::max(0.5, params.wheel_equiv_mass_kg);
 const double limit = std::max(0.0, params.traction_limit_N);
 double remaining = dt;
 double contact_impulse_Ns = 0.0;
 double slip = 0.0;
 bool slipped = false;
 // The electrical/contact time scale is shorter than the outer 20 ms
 // control period. Re-evaluate the four-quadrant motor curve and tire force
 // at 1 ms so a braking wheel that crosses zero does not use a stale torque
 // for the rest of the public step.
 while (remaining > 1e-12) {
 const double h = std::min(0.001, remaining);
 const double motor_force = hardware::dc_motor_force_N(
 pwm, wheel_mps, params.stall_force_N, params.free_speed_mps);
 const double wheel_free =
 (mass * wheel_mps / h + motor_force + stiffness * body_mps) / (mass / h + stiffness);
 const double force_free = stiffness * (wheel_free - body_mps);
 double body_force = force_free;
 if (std::abs(force_free) <= limit) {
 wheel_mps = wheel_free;
 } else {
 body_force = std::copysign(limit, force_free);
 wheel_mps += (motor_force - body_force) * h / mass;
 slipped = true;
 }
 slip = (wheel_mps - body_mps) / denom;
 slipped = slipped || std::abs(slip) > kSlipRef;
 contact_impulse_Ns += body_force * h;
 remaining -= h;
 }
 return {wheel_mps, contact_impulse_Ns / dt, slip, slipped};
}

} // namespace sim
