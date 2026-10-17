#pragma once

#include <vector>

namespace sim {

// Feedforward mapping, vehicle side (stages 2-3 of the agreed FF route).
// Pure functions: desired accelerations + measured speeds + hardware data in,
// candidate PWM out. NEVER touches actuators, PIDs, or mission state —
// stage 3 calls these from the mission shadow path only.
//
// Chain (all linear maps reused, never re-derived):
//   body accel desire -> wheel accel desire (same coefficients as
//     DifferentialKinematics::twist_to_wheels: differentiation is linear)
//   wheel force = wheel_mass * a + R(v) with R the same Coulomb+viscous form
//     as the drives (motor_drive.cpp:75-86, ackermann.cpp:117-118,
//     tracked.cpp:77-80): R = rr*load*tanh(v/0.05) + 0.6*v
//   pwm = F / (stall * curve(v)), curve(v) = max(0, 1-|v|/max(0.05, free))
//     (motor_drive.cpp:76-77, contact.cpp:18-19)
//
// Honest boundaries (stage-4/offline evaluates, never assumed):
// - Tracked yaw scrub torque is NOT modeled (residual stays with PID).
// - Ackermann steering servo stays P-only (drive-longitudinal FF only).
// - No deadzone anywhere (repo has none; real deadzone is sim2real
//   calibration per docs/sim2real.md).
// - T_lead is a caller input, not a constant here. Default rationale lives
//   with the caller: no motor lag is modeled, so T_lead = 1/wc of the
//   assisted closed loop (motor_drive.cpp:17, wc=6), sensitivity-scanned in
//   stage 4, identified on hardware last.

struct FeedforwardParams {
 double wheel_mass_kg{3.0};  // contact wheel inertia actually accelerated.
 double stall_force_N{54.0};
 double free_speed_mps{3.0};
 double rolling_resist{0.0}; // live combined tire+ground rr.
 // Normal load per driven unit (N): m*g/2 twin-unit, m*g single-unit.
 double load_per_unit_N{0.0};
};

struct FeedforwardOut {
 double pwm_ff{0.0}; // clamped to [-1, 1].
 bool saturated{false}; // demand exceeded force authority (incl. zero curve).
};

struct BodyAccelDes {
 double a_mps2{0.0};
 double alpha_radps2{0.0};
};

// Lead compensator on the assisted loop: (target - measured) / T_lead.
BodyAccelDes lead_accel(double v_target_mps, double v_meas_mps, double w_target_radps,
                        double w_meas_radps, double t_lead_s);

// One driven unit: force from desired accel + resistance, mapped through the
// speed curve. Above free speed the curve is zero: demand without authority
// reports saturated (never NaN, never silent).
FeedforwardOut wheel_feedforward(double wheel_accel_des_mps2, double wheel_speed_meas_mps,
                                 const FeedforwardParams& params);

struct TwistFeedforward {
 FeedforwardOut left;
 FeedforwardOut right;
};

// Twin-unit chassis (differential, tracked): body desire + measured body
// twist, mapped through the same linear coefficients as twist_to_wheels.
TwistFeedforward twist_feedforward(const BodyAccelDes& desired, double v_meas_mps,
                                   double w_meas_mps, double wheel_base_m,
                                   const FeedforwardParams& params);

} // namespace sim
