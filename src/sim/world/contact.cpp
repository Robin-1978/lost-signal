#include "sim/world/contact.hpp"

#include <algorithm>
#include <cmath>

namespace lost_signal::sim::world {

double contact_resistance_N(double wheel_mps,
                            const WheelContactParams& params) {
  const double load = std::max(0.0, params.load_per_wheel_N);
  const double coulomb = std::max(0.0, params.rolling_resist) * load *
                         std::tanh(wheel_mps / std::max(1e-4, params.coulomb_smoothing_mps));
  const double viscous =
      std::max(0.0, params.viscous_drag_N_per_mps) * wheel_mps;
  return -(coulomb + viscous);  // force ON the wheel: opposes the motion
}

double contact_static_capacity_N(const WheelContactParams& params) {
  return std::max(0.0, params.rolling_resist) *
         std::max(0.0, params.load_per_wheel_N);
}

}  // namespace lost_signal::sim::world
