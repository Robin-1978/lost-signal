#include "sim/hardware/vehicle/planar_diff_drive.hpp"

#include "robot/core/math.hpp"

#include <cmath>
#include <stdexcept>

namespace sim {

DiffState PlanarDiffDrivePlant::step(const DiffState& state, const DiffCmd& command,
 double dt_s) const {
 if (dt_s <= 0.0)
 throw std::invalid_argument("planar differential plant step must be positive");

 DiffState next = state;
 const double desired_v = clamp(command.target_v, -params_.max_reverse_mps,
 params_.max_forward_mps);
 const double desired_w = clamp(command.target_w, -params_.max_yaw_rate_radps,
 params_.max_yaw_rate_radps);
 next.v = state.v + clamp(desired_v - state.v,
 -params_.max_acceleration_mps2 * dt_s,
 params_.max_acceleration_mps2 * dt_s);
 next.w = state.w + clamp(desired_w - state.w,
 -params_.max_yaw_acceleration_radps2 * dt_s,
 params_.max_yaw_acceleration_radps2 * dt_s);
 next.x = state.x + next.v * std::cos(state.theta) * dt_s;
 next.y = state.y + next.v * std::sin(state.theta) * dt_s;
 next.theta = wrap_angle(state.theta + next.w * dt_s);
 return next;
}

} // namespace sim
