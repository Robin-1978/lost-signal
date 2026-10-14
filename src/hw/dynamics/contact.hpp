#pragma once

#include "algo/math.hpp"

#include <algorithm>
#include <cmath>

namespace sim {

// Tire-ground contact physics (quarter-car-lite). This module answers one
// question: given a tread speed, a body speed and a motor command, what force
// does the ground return? It belongs to neither chassis: differential and
// ackermann share it so tire and ground content means the same thing on every
// vehicle. Chassis code owns actuators and body integration; this module owns
// the contact patch in between.
struct WheelContactParams {
 double stall_force_N{54.0};
 double free_speed_mps{3.0};
 double traction_limit_N{80.0};
 double wheel_equiv_mass_kg{3.0};
};

struct WheelContactResult {
 double wheel_mps{0.0};
 double body_force_N{0.0};
 double slip_ratio{0.0};
 bool slipping{false};
};

WheelContactResult step_driven_wheel(double wheel_mps, double body_mps, double pwm,
                                     const WheelContactParams& params, double dt);

} // namespace sim
