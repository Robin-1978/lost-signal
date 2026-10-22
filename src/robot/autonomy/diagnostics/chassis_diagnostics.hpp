#pragma once

#include "robot/autonomy/diagnostics/vehicle_measurement.hpp"
#include "robot/autonomy/localization/slip_detect.hpp"
#include "robot/runtime/observation.hpp"

namespace sim::autonomy {

// ROBOT-owned, sensor-facing chassis diagnostics.  The class deliberately
// contains only state that must persist between control ticks.  Mission code
// may consume the published measurements, but cannot own the detector's
// history or feed simulator truth into it.
class ChassisDiagnosticsSystem {
 public:
  void reset();

  const BrakingMeasurement& update_braking(double time_s, Vec2 position,
                                           double speed_mps,
                                           double target_speed_mps);

  // Both speeds are post-step sensed quantities.  When tread sensing is not
  // available, the detector is left unchanged and reports its last state.
  bool update_slip(bool tread_observable, double tread_speed_mps,
                   double body_speed_mps, double dt_s);
 // Sensor-only path: compare encoder-derived acceleration with the measured
 // IMU specific-force acceleration. No plant contact or simulator state is
 // accepted by this API.
 bool update_slip_from_sensors(const EncoderObservation& encoder,
 const ImuObservation& imu, double dt_s);

  const BrakingMeasurement& braking() const { return braking_.braking(); }
  bool slip_detected() const { return slip_detector_.detected(); }
  int slip_episodes() const { return slip_detector_.episodes(); }

 private:
  VehicleMeasurementSystem braking_;
  SlipDetector slip_detector_;
  double previous_tread_speed_mps_{0.0};
  double previous_body_speed_mps_{0.0};
  bool have_previous_slip_sample_{false};
};

}  // namespace sim::autonomy
