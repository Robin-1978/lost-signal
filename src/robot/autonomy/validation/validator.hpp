#pragma once

#include <cstddef>
#include <limits>
#include <string>

#include "robot/autonomy/estimation/estimator.hpp"
#include "robot/autonomy/perception/perception.hpp"
#include "robot/autonomy/planning/local_planner.hpp"
#include "robot/core/geometry.hpp"
#include "robot/iface/observation.hpp"

namespace lost_signal::robot::autonomy {

// Independent safety authority. Validates the exact selected candidate
// against observed evidence; a planner score is never a safety argument.
struct ValidationResult {
  bool accepted = false;
  std::string reason;
};

// Clearance policy, derived from the chassis — never a bare literal:
// need = footprint_radius + extra_margin + braking distance v^2/2a.
// The clearance check runs on the FORWARD sector only (sensor-frame heading
// +/- half_angle): a side or rear return must never veto forward motion —
// the rollout's own collision check already covers the swept path, and the
// independent safety monitor owns the all-around contact latch.
struct ClearanceValidatorConfig {
  double footprint_radius_m = 0.4;
  double extra_margin_m = 0.1;
  double brake_decel_mps2 = 2.0;
  double forward_half_angle_rad = 0.5;
  // Sensor yaw in base_link (mount calibration). Forward-sector beams are
  // measured relative to it; native bring-ups with rotated mounts set this
  // instead of silently validating the wrong sector.
  double sensor_yaw_rad = 0.0;
};

// IValidator: candidate + estimate + perception + raw scan -> verdict.
// Missing or stale scan is unknown evidence -> reject motion (hold ok).
class IValidator {
 public:
  virtual ~IValidator() = default;
  virtual ValidationResult validate(const LocalCandidate& candidate,
                                    const EstimatedState& est,
                                    const PerceptionResult& perception,
                                    const iface::ObservationFrame& obs) = 0;
};

// v1: holds always pass; motion needs footprint + margin + braking distance
// of clearance. Conservative and explicit.
class ClearanceValidator : public IValidator {
 public:
  explicit ClearanceValidator(ClearanceValidatorConfig config = {})
      : config_(config) {}

  ValidationResult validate(const LocalCandidate& candidate,
                            const EstimatedState& /*est*/,
                            const PerceptionResult& perception,
                            const iface::ObservationFrame& obs) override {
    if (!perception.scan_ok || !obs.scan_valid || obs.ranges_m.empty()) {
      if (candidate.forward_mps == 0.0 && candidate.yaw_rate_radps == 0.0) {
        return {true, "hold without scan"};
      }
      return {false, "no scan evidence for motion"};
    }
    float min_range = std::numeric_limits<float>::infinity();
    const std::size_t n = obs.ranges_m.size();
    for (std::size_t i = 0; i < n; ++i) {
      // Even beam layout over [-pi, pi); forward is index n/2.
      const double angle =
          -core::kPi + 2.0 * core::kPi * static_cast<double>(i) / n;
      if (std::abs(core::wrap_angle(angle - config_.sensor_yaw_rad)) >
          config_.forward_half_angle_rad) {
        continue;
      }
      if (obs.ranges_m[i] < min_range) min_range = obs.ranges_m[i];
    }
    if (!std::isfinite(min_range)) {
      return {false, "no forward returns"};
    }
    double speed = candidate.forward_mps < 0 ? -candidate.forward_mps
                                             : candidate.forward_mps;
    // Footprint plus margin plus braking distance: the distance needed to
    // stop from the candidate speed at the configured deceleration.
    const double need_m = config_.footprint_radius_m +
                          config_.extra_margin_m +
                          (speed * speed) / (2.0 * config_.brake_decel_mps2);
    if (candidate.forward_mps == 0.0 && candidate.yaw_rate_radps == 0.0) {
      return {true, "hold"};
    }
    if (min_range < need_m) return {false, "insufficient clearance"};
    return {true, "clear"};
  }

 private:
  ClearanceValidatorConfig config_;
};

}  // namespace lost_signal::robot::autonomy
