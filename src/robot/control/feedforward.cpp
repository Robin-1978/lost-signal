#include "robot/control/feedforward.hpp"

#include <algorithm>
#include <cmath>

namespace sim {
namespace {

// Mirrors of motor_drive.cpp:15-16 (single source of those values lives
// there; duplicated here with pointer so the FF mapping cannot silently
// diverge from the drives it assists).
constexpr double kCoulombWidthMps = 0.05;
constexpr double kViscousNPerMps = 0.6;
constexpr double kCurveFloorMps = 0.05;

} // namespace

BodyAccelDes lead_accel(double v_target_mps, double v_meas_mps, double w_target_radps,
                        double w_meas_radps, double t_lead_s) {
 const double lead = std::max(1e-6, t_lead_s);
 return {(v_target_mps - v_meas_mps) / lead, (w_target_radps - w_meas_radps) / lead};
}

FeedforwardOut wheel_feedforward(double wheel_accel_des_mps2, double wheel_speed_meas_mps,
                                 const FeedforwardParams& params) {
 const double curve =
   std::max(0.0, 1.0 - std::abs(wheel_speed_meas_mps) / std::max(kCurveFloorMps, params.free_speed_mps));
 const double authority = params.stall_force_N * curve;
 const double resistance = params.rolling_resist * params.load_per_unit_N *
                             std::tanh(wheel_speed_meas_mps / kCoulombWidthMps) +
                           kViscousNPerMps * wheel_speed_meas_mps;
 const double force = params.wheel_mass_kg * wheel_accel_des_mps2 + resistance;
 if (!(authority > 1e-12)) {
  // No force authority (at/past free speed): hold zero unless force is
  // demanded, in which case report saturation at full deflection.
  if (std::abs(force) <= 1e-12) {
   return {0.0, false};
  }
  return {std::copysign(1.0, force), true};
 }
 const double raw = force / authority;
 if (std::abs(raw) <= 1.0) {
  return {raw, false};
 }
 return {std::copysign(1.0, raw), true};
}

TwistFeedforward twist_feedforward(const BodyAccelDes& desired, double v_meas_mps,
                                   double w_meas_mps, double wheel_base_m,
                                   const FeedforwardParams& params) {
 // Same linear coefficients as DifferentialKinematics::twist_to_wheels (differentiation
 // passes through unchanged): written out so this unit never depends on
 // chassis step code.
 const double half = std::max(1e-9, wheel_base_m) / 2.0;
 const double left_accel = desired.a_mps2 - desired.alpha_radps2 * half;
 const double right_accel = desired.a_mps2 + desired.alpha_radps2 * half;
 const double left_speed = v_meas_mps - w_meas_mps * half;
 const double right_speed = v_meas_mps + w_meas_mps * half;
 TwistFeedforward out;
 out.left = wheel_feedforward(left_accel, left_speed, params);
 out.right = wheel_feedforward(right_accel, right_speed, params);
 return out;
}

} // namespace sim
