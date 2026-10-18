#pragma once

namespace sim::hardware {

struct ServiceBrakeParams {
 double maximum_torque_Nm{0.0};
 double apply_time_s{0.12};
 double release_time_s{0.08};
 double low_speed_smoothing_radps{0.6};
};

struct ServiceBrakeTelemetry {
 double requested{0.0};
 double applied{0.0};
 // Positive passive capacity offered to the physics provider. Actual signed
 // joint torque is resolved from instantaneous shaft velocity there.
 double available_torque_Nm{0.0};
};

// A mechanical service-brake actuator. It owns actuator lag and requested
// shaft torque only. Tire contact remains the sole owner of normal load,
// friction limits, wheel slip, and the resulting vehicle deceleration.
class ServiceBrakePlant {
 public:
 explicit ServiceBrakePlant(ServiceBrakeParams params);

 void reset();
 double step(double normalized_request, double dt_s);
 const ServiceBrakeTelemetry& telemetry() const { return telemetry_; }

 private:
 ServiceBrakeParams params_;
 ServiceBrakeTelemetry telemetry_;
};

} // namespace sim::hardware
