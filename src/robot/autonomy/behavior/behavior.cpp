#include "robot/autonomy/behavior/behavior.hpp"

#include <cmath>

namespace lost_signal::robot::autonomy {

WaitOnIntrusionBehavior::WaitOnIntrusionBehavior(double hold_radius_m)
    : hold_radius_m_(hold_radius_m) {}

BehaviorDecision WaitOnIntrusionBehavior::decide(
    const EstimatedState& est, const PerceptionResult& perception,
    const platform::ChassisLimits& limits) {
  if (!est.valid || !perception.scan_ok) {
    return {true, 0.0, "unknown-evidence-hold"};
  }
  for (const auto& t : perception.tracks) {
    double d = std::hypot(t.x_m - est.x_m, t.y_m - est.y_m);
    if (d < hold_radius_m_) {
      return {true, 0.0, "track-intrusion-hold"};
    }
  }
  return {false, limits.max_forward_mps, "clear"};
}

}  // namespace lost_signal::robot::autonomy
