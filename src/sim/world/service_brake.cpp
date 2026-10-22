#include "sim/world/service_brake.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace lost_signal::sim::world {

ServiceBrakePlant::ServiceBrakePlant(ServiceBrakeParams params)
    : params_(params) {
  if (!std::isfinite(params_.maximum_torque_Nm) ||
      !std::isfinite(params_.apply_time_s) ||
      !std::isfinite(params_.release_time_s) ||
      !std::isfinite(params_.low_speed_smoothing_radps) ||
      params_.maximum_torque_Nm < 0.0 || params_.apply_time_s <= 0.0 ||
      params_.release_time_s <= 0.0 ||
      params_.low_speed_smoothing_radps <= 0.0) {
    throw std::invalid_argument("invalid service-brake actuator parameters");
  }
}

void ServiceBrakePlant::reset() { telemetry_ = {}; }

double ServiceBrakePlant::step(double normalized_request, double dt_s) {
  if (!std::isfinite(normalized_request) || !std::isfinite(dt_s) ||
      dt_s <= 0.0) {
    throw std::invalid_argument(
        "service-brake inputs must be finite and dt positive");
  }
  telemetry_.requested = std::clamp(normalized_request, 0.0, 1.0);
  const double response_s = telemetry_.requested > telemetry_.applied
                                ? params_.apply_time_s
                                : params_.release_time_s;
  const double maximum_delta = dt_s / response_s;
  telemetry_.applied += std::clamp(telemetry_.requested - telemetry_.applied,
                                   -maximum_delta, maximum_delta);

  telemetry_.available_torque_Nm =
      telemetry_.applied * params_.maximum_torque_Nm;
  return telemetry_.available_torque_Nm;
}

}  // namespace lost_signal::sim::world
