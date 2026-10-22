#include "robot/autonomy/diagnostics/chassis_diagnostics.hpp"

namespace sim::autonomy {

void ChassisDiagnosticsSystem::reset() {
  braking_.reset();
  slip_detector_.reset();
  previous_tread_speed_mps_ = 0.0;
  previous_body_speed_mps_ = 0.0;
  have_previous_slip_sample_ = false;
}

const BrakingMeasurement& ChassisDiagnosticsSystem::update_braking(
    double time_s, Vec2 position, double speed_mps, double target_speed_mps) {
  return braking_.update(time_s, position, speed_mps, target_speed_mps);
}

bool ChassisDiagnosticsSystem::update_slip(bool tread_observable,
                                            double tread_speed_mps,
                                            double body_speed_mps,
                                            double dt_s) {
  if (!tread_observable || dt_s <= 0.0) {
    return slip_detector_.detected();
  }

  if (have_previous_slip_sample_) {
    const double encoder_accel =
        (tread_speed_mps - previous_tread_speed_mps_) / dt_s;
    const double body_accel =
        (body_speed_mps - previous_body_speed_mps_) / dt_s;
    slip_detector_.step(encoder_accel, body_accel, dt_s);
  }

  previous_tread_speed_mps_ = tread_speed_mps;
  previous_body_speed_mps_ = body_speed_mps;
  have_previous_slip_sample_ = true;
  return slip_detector_.detected();
}

bool ChassisDiagnosticsSystem::update_slip_from_sensors(
 const EncoderObservation& encoder, const ImuObservation& imu, double dt_s) {
 if (dt_s <= 0.0) {
 return slip_detector_.detected();
 }
 if (have_previous_slip_sample_) {
 const double encoder_accel =
 (encoder.linear_velocity_mps - previous_tread_speed_mps_) / dt_s;
 slip_detector_.step(encoder_accel, imu.forward_acceleration_mps2, dt_s);
 }
 previous_tread_speed_mps_ = encoder.linear_velocity_mps;
 previous_body_speed_mps_ = 0.0;
 have_previous_slip_sample_ = true;
 return slip_detector_.detected();
}

}  // namespace sim::autonomy
