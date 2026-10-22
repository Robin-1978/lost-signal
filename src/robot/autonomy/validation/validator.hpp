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

// v1: holds always pass; motion needs min-range clearance beyond the
// commanded stopping distance. Conservative and explicit.
class ClearanceValidator : public IValidator {
 public:
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
    double need_m = 0.5 + speed * 1.0;  // footprint + one decision period
    if (candidate.forward_mps == 0.0 && candidate.yaw_rate_radps == 0.0) {
      return {true, "hold"};
    }
    if (min_range < need_m) return {false, "insufficient clearance"};
    return {true, "clear"};
  }
};

}  // namespace lost_signal::robot::autonomy
