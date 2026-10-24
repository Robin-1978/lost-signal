#include "robot/autonomy/behavior/behavior.hpp"

#include <algorithm>
#include <cmath>

namespace lost_signal::robot::autonomy {

WaitOnIntrusionBehavior::WaitOnIntrusionBehavior(double hold_radius_m,
 double prediction_horizon_s)
 : hold_radius_m_(std::max(0.0, hold_radius_m)),
 prediction_horizon_s_(std::max(0.0, prediction_horizon_s)) {}

BehaviorDecision WaitOnIntrusionBehavior::decide(
    const EstimatedState& est, const PerceptionResult& perception,
    const platform::ChassisLimits& limits) {
  if (!est.valid || !perception.scan_ok) {
    return {true, 0.0, "unknown-evidence-hold"};
  }
 const double c = std::cos(est.yaw_rad);
 const double s = std::sin(est.yaw_rad);
 const double ego_vx = est.forward_mps * c;
 const double ego_vy = est.forward_mps * s;
 const double ego_radius = std::max(0.0, limits.footprint_radius_m);
 for (const auto& t : perception.tracks) {
 const double dx = t.x_m - est.x_m;
 const double dy = t.y_m - est.y_m;
 const double radius = ego_radius + t.occupied_radius_m();
 const double d0 = std::hypot(dx, dy);
 // A close contact is a hold even before the track has enough history for
 // prediction. The larger legacy radius is only a forward-path envelope.
 if (d0 <= radius) {
 return {true, 0.0, "track-contact-hold"};
 }
 const double lateral = std::abs(-s * dx + c * dy);
 const double longitudinal = c * dx + s * dy;
 const double rvx = t.vx_mps - ego_vx;
 const double rvy = t.vy_mps - ego_vy;
 const bool in_forward_envelope =
 longitudinal >= -radius &&
 longitudinal <= hold_radius_m_ + std::max(0.0, limits.max_forward_mps) *
 prediction_horizon_s_ + radius &&
 lateral <= radius;
 if (!in_forward_envelope) continue;

 // Anything already in the commanded forward corridor is an occupied
 // conflict zone. Objects outside that corridor (for example, behind or
 // beside the base) do not stop the robot. Velocity prediction below also
 // catches a moving object before it enters this near corridor.
 if (longitudinal >= 0.0 && longitudinal <= hold_radius_m_ + radius) {
 return {true, 0.0, "track-conflict-hold"};
 }

 // Constant-velocity relative prediction. The closest point over the
 // horizon catches an approaching crossing object without stopping for a
 // stationary object behind the robot.
 const double vv = rvx * rvx + rvy * rvy;
 double tau = 0.0;
 if (vv > 1e-9) {
 tau = std::clamp(-(dx * rvx + dy * rvy) / vv,
 0.0, prediction_horizon_s_);
 }
 const double px = dx + rvx * tau;
 const double py = dy + rvy * tau;
 if (std::hypot(px, py) <= radius) {
 return {true, 0.0, "track-conflict-hold"};
 }
  }
  return {false, limits.max_forward_mps, "clear"};
}

}  // namespace lost_signal::robot::autonomy
