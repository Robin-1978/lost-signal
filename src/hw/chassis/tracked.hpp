#pragma once

#include "diff_drive.hpp"
#include "motor_drive.hpp"
#include "hw/dynamics/contact.hpp"
#include "autonomy/control/pid.hpp"

namespace sim {

// Tracked drive parameters. Everything is per-vehicle hardware from robot
// JSON; loop gains derive from the same wheel rule as differential drive
// (a track drive unit is a wheel to the speed loop). The one tracked-only
// number is the yaw-rate spec: tracks scrub to turn, so top spin rate is a
// vendor capability, not something the controller may assume.
struct TrackedParams {
 double wheel_base_m{0.55};
 double wheel_radius_m{0.1};
 double track_length_m{0.7};
 double max_drive_speed_mps{2.0};
 double max_drive_accel_mps2{4.0};
 double stall_force_N{54.0};
 double free_speed_mps{3.0};
 double mass_kg{30.0};
 double yaw_inertia_kgm2{3.0};
 double max_yaw_rate_rads{1.5};
 double pwm_limit{1.0};
 double integral_limit{2.0};
 int encoder_counts_per_revolution{2048};
 double wheel_equiv_mass_kg{3.0};
 double support_width_m{0.55};
};

// Tracked chassis: two skid-steered tracks. Longitudinally each track is a
// driven wheel on the shared contact (tread state, traction cap), so slip
// is observable on the encoders from birth. Laterally tracks do not roll:
// turning scrubs soil, modeled as a yaw resistance moment (Coulomb from track
// length plus small viscous), not as a lateral velocity state — a full
// planar track-soil model is deferred the same way Pacejka is. Twist
// interface like differential (spins in place, with scrub); odometry during
// turns lies, which is the teaching point, not a bug.
class TrackedDrive : public ChassisModel {
 public:
 TrackedDrive(TrackedParams params, WheelPidGains drive_gains);

 void reset();
 DiffState step(const DiffState& state, const DiffCmd& command, double dt,
                const GroundContact& ground) override;
 const ChassisTelemetry& telemetry() const override { return telemetry_; }
 ChassisCaps caps() const override {
 ChassisCaps output;
 output.geometry.support_width_m = params_.support_width_m;
 output.geometry.support_length_m = params_.track_length_m;
 output.max_yaw_rate_radps = params_.max_yaw_rate_rads;
 // A long rectangular contact patch sweeps its front corner outside the
 // centerline while pivoting. Reserve that real overhang in global planning
 // so the track does not clip shelving even when its circular footprint
 // radius alone would fit.
 output.planning_clearance_margin_m = 0.0;
 output.rigid_body_cruise_scale = 0.65;
 output.tread_slip_observable = true;
 return output;
 }

 private:
 double encoder_measure(double wheel_mps, double dt) const;

 TrackedParams params_;
 PID left_pid_;
 PID right_pid_;
 double left_tread_mps_{0.0};
 double right_tread_mps_{0.0};
 ChassisTelemetry telemetry_;
};

} // namespace sim
