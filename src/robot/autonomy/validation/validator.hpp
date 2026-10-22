#pragma once

#include <string>

#include "robot/autonomy/estimation/estimator.hpp"
#include "robot/autonomy/perception/perception.hpp"
#include "robot/autonomy/planning/local_planner.hpp"
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
struct ClearanceValidatorConfig {
  double footprint_radius_m = 0.4;
  double extra_margin_m = 0.1;
  double brake_decel_mps2 = 2.0;
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
    float min_range = obs.ranges_m[0];
    for (float r : obs.ranges_m) min_range = r < min_range ? r : min_range;
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
