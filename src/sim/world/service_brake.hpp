#pragma once

// SIM-owned service-brake actuator (World internals, never ROBOT input).
// A mechanical service-brake actuator: owns actuator lag and requested shaft
// torque only. Ported verbatim from the main-stack service_brake; tire
// contact remains the sole owner of normal load and friction limits (S2).
// The plant reports passive capacity; the World resolves the signed joint
// force from shaft velocity so the brake can only dissipate, never drive.

namespace lost_signal::sim::world {

struct ServiceBrakeParams {
  double maximum_torque_Nm{0.0};
  double apply_time_s{0.12};
  double release_time_s{0.08};
  double low_speed_smoothing_radps{0.6};
};

struct ServiceBrakeTelemetry {
  double requested{0.0};
  double applied{0.0};
  // Positive passive capacity offered to the physics owner. Actual signed
  // joint torque is resolved from instantaneous shaft velocity there.
  double available_torque_Nm{0.0};
};

class ServiceBrakePlant {
 public:
  explicit ServiceBrakePlant(ServiceBrakeParams params = ServiceBrakeParams{});

  void reset();
  double step(double normalized_request, double dt_s);
  const ServiceBrakeTelemetry& telemetry() const { return telemetry_; }

 private:
  ServiceBrakeParams params_;
  ServiceBrakeTelemetry telemetry_;
};

}  // namespace lost_signal::sim::world
