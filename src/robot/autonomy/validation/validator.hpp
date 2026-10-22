#pragma once

#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include "robot/autonomy/estimation/estimator.hpp"
#include "robot/autonomy/perception/perception.hpp"
#include "robot/autonomy/planning/local_planner.hpp"
#include "robot/core/geometry.hpp"
#include "robot/iface/observation.hpp"

namespace lost_signal::robot::autonomy {

// Trajectory verdict for the exact selected candidate. A planner score is
// never a safety argument; the rollout is re-checked against raw evidence.
enum class TrajectoryVerdict {
  kAccepted = 0,
  // Minimum-risk escape: the candidate starts inside the soft margin, but
  // its rollout gains clearance where braking-and-holding would strand the
  // base inside the margin. Accepted, flagged for selection ordering
  // (a fully margin-compliant candidate always outranks an escape).
  kMinimumRiskEscape = 1,
  kStaticCollision = 2,       // swept footprint intersects observed geometry
  kDynamicCollision = 3,       // rollout meets a predicted track
  kUnsafeEmergencyStop = 4,    // braking tail from candidate speed doesn't fit
  kInvalidStructure = 5,       // infeasible candidate / missing evidence
};

inline const char* trajectory_verdict_name(TrajectoryVerdict verdict) {
  switch (verdict) {
    case TrajectoryVerdict::kAccepted:
      return "accepted";
    case TrajectoryVerdict::kMinimumRiskEscape:
      return "minimum-risk-escape";
    case TrajectoryVerdict::kStaticCollision:
      return "static-collision";
    case TrajectoryVerdict::kDynamicCollision:
      return "dynamic-collision";
    case TrajectoryVerdict::kUnsafeEmergencyStop:
      return "unsafe-emergency-stop";
    case TrajectoryVerdict::kInvalidStructure:
      return "invalid-structure";
  }
  return "unknown";
}

struct ValidationResult {
  bool accepted = false;  // kAccepted || kMinimumRiskEscape
  TrajectoryVerdict verdict = TrajectoryVerdict::kInvalidStructure;
  // MRE only: separation gained by the escape over holding still.
  double escape_clearance_gain_m = 0.0;
  // Closest approach of the swept footprint to any observed endpoint.
  double min_clearance_m = std::numeric_limits<double>::infinity();
  std::string reason;
};

// Clearance policy, derived from the chassis — never a bare literal:
// need = footprint_radius + extra_margin + braking distance v^2/2a.
// The sector check runs on the FORWARD sector only (sensor-frame heading
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
  // Rollout horizon for swept-path checks.
  double rollout_horizon_s = 2.0;
  double rollout_dt_s = 0.1;
  // MRE escape cap: only slow creep may be admitted as a minimum-risk
  // escape. A fast candidate inside the margin is rejected, never escaped.
  double escape_speed_cap_mps = 0.3;
  double escape_gain_floor_m = 0.15;
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

// Trajectory validator: constant-twist rollout of the exact candidate,
// swept against raw scan endpoints (map-frame) and predicted tracks, with
// a braking-tail fit check. Holds always pass; motion needs footprint plus
// margin plus braking distance, or a minimum-risk escape when already
// inside the margin and the rollout gains clearance. Conservative and
// explicit; without a valid estimate it falls back to the forward-sector
// check (dead-reckoning start, never a silent full validation).
class ClearanceValidator : public IValidator {
 public:
  explicit ClearanceValidator(ClearanceValidatorConfig config = {})
      : config_(config) {}

  ValidationResult validate(const LocalCandidate& candidate,
                            const EstimatedState& est,
                            const PerceptionResult& perception,
                            const iface::ObservationFrame& obs) override {
    ValidationResult out;
    if (!candidate.feasible) {
      out.reason = "infeasible candidate";
      return out;
    }
    const bool hold = candidate.forward_mps == 0.0 &&
                      candidate.yaw_rate_radps == 0.0;
    if (!perception.scan_ok || !obs.scan_valid || obs.ranges_m.empty()) {
      if (hold) {
        out.accepted = true;
        out.verdict = TrajectoryVerdict::kAccepted;
        out.reason = "hold without scan";
        return out;
      }
      out.reason = "no scan evidence for motion";
      return out;
    }
    // Forward-sector minimum range (braking-tail check below).
    float sector_min = std::numeric_limits<float>::infinity();
    const std::size_t n = obs.ranges_m.size();
    for (std::size_t i = 0; i < n; ++i) {
      // Even beam layout over [-pi, pi); forward is index n/2.
      const double angle =
          -core::kPi + 2.0 * core::kPi * static_cast<double>(i) / n;
      if (std::abs(core::wrap_angle(angle - config_.sensor_yaw_rad)) >
          config_.forward_half_angle_rad) {
        continue;
      }
      if (obs.ranges_m[i] < sector_min) sector_min = obs.ranges_m[i];
    }
    if (!std::isfinite(sector_min)) {
      if (hold) {
        out.accepted = true;
        out.verdict = TrajectoryVerdict::kAccepted;
        out.reason = "hold";
        return out;
      }
      out.reason = "no forward returns";
      return out;
    }
    if (hold) {
      out.accepted = true;
      out.verdict = TrajectoryVerdict::kAccepted;
      out.reason = "hold";
      return out;
    }
    const double speed = std::abs(candidate.forward_mps);
    // Footprint plus margin plus braking distance: the distance needed to
    // stop from the candidate speed at the configured deceleration.
    const double need_m = config_.footprint_radius_m +
                          config_.extra_margin_m +
                          (speed * speed) / (2.0 * config_.brake_decel_mps2);
    if (!est.valid) {
      // Dead-reckoning start: sector check only, same as the v1 gate.
      if (sector_min < need_m) {
        out.verdict = TrajectoryVerdict::kUnsafeEmergencyStop;
        out.reason = "insufficient clearance";
        return out;
      }
      out.accepted = true;
      out.verdict = TrajectoryVerdict::kAccepted;
      out.min_clearance_m = sector_min;
      out.reason = "clear";
      return out;
    }
    // Full trajectory validation from a valid estimate.
    // Endpoints in the map frame: raw evidence, independent of the mapper.
    std::vector<core::Vec2> endpoints;
    endpoints.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
      const float range = obs.ranges_m[i];
      if (!(range > 0.0f) || !std::isfinite(range)) continue;
      const double angle = -core::kPi + 2.0 * core::kPi * i / n + est.yaw_rad;
      endpoints.push_back(
          {est.x_m + range * std::cos(angle), est.y_m + range * std::sin(angle)});
    }
    // Rollout the exact candidate twist; sample the swept footprint.
    const int steps = std::max(
        1, static_cast<int>(std::ceil(config_.rollout_horizon_s / config_.rollout_dt_s)));
    double min_clear = std::numeric_limits<double>::infinity();
    double end_clear = std::numeric_limits<double>::infinity();
    bool touches = false;
    for (int s = 1; s <= steps; ++s) {
      const double t = s * config_.rollout_dt_s;
      // Exact constant-twist integration: straight line or arc.
      double ax = est.x_m, ay = est.y_m;
      if (std::abs(candidate.yaw_rate_radps) >= 1e-9) {
        const double r = candidate.forward_mps / candidate.yaw_rate_radps;
        const double d_yaw = candidate.yaw_rate_radps * t;
        ax = est.x_m +
             r * (std::sin(est.yaw_rad + d_yaw) - std::sin(est.yaw_rad));
        ay = est.y_m -
             r * (std::cos(est.yaw_rad + d_yaw) - std::cos(est.yaw_rad));
      } else {
        ax = est.x_m + candidate.forward_mps * t * std::cos(est.yaw_rad);
        ay = est.y_m + candidate.forward_mps * t * std::sin(est.yaw_rad);
      }
      double sample_min = std::numeric_limits<double>::infinity();
      for (const auto& e : endpoints) {
        const double d = std::hypot(e.x - ax, e.y - ay);
        if (d < sample_min) sample_min = d;
        if (d < config_.footprint_radius_m) touches = true;
      }
      min_clear = std::min(min_clear, sample_min);
      if (s == steps) end_clear = sample_min;
      // Dynamic: tracks propagated linearly to the sample time.
      for (const auto& track : perception.tracks) {
        const double tx = track.x_m + track.vx_mps * t;
        const double ty = track.y_m + track.vy_mps * t;
        if (std::hypot(tx - ax, ty - ay) <
            config_.footprint_radius_m + track.occupied_radius_m()) {
          out.verdict = TrajectoryVerdict::kDynamicCollision;
          out.min_clearance_m = min_clear;
          out.reason = "dynamic collision on rollout";
          return out;
        }
      }
      if (touches) break;
    }
    out.min_clearance_m = min_clear;
    if (touches) {
      // Inside the footprint already? Then holding strands the base;
      // a slow rollout that gains clearance is the minimum-risk escape.
      const double now_clear = current_clearance(endpoints, est);
      if (now_clear < need_m && speed <= config_.escape_speed_cap_mps &&
          end_clear > now_clear + config_.escape_gain_floor_m) {
        out.accepted = true;
        out.verdict = TrajectoryVerdict::kMinimumRiskEscape;
        out.escape_clearance_gain_m = end_clear - now_clear;
        out.reason = "minimum-risk escape gains clearance";
        return out;
      }
      out.verdict = TrajectoryVerdict::kStaticCollision;
      out.reason = "static collision on rollout";
      return out;
    }
    if (sector_min < need_m) {
      // Path ahead is clear on the rollout but the braking tail doesn't
      // fit in the observed opening: stopping from this speed needs more
      // room than the sector shows.
      out.verdict = TrajectoryVerdict::kUnsafeEmergencyStop;
      out.reason = "insufficient clearance";
      return out;
    }
    out.accepted = true;
    out.verdict = TrajectoryVerdict::kAccepted;
    out.reason = "clear";
    return out;
  }

 private:
  static double current_clearance(const std::vector<core::Vec2>& endpoints,
                                  const EstimatedState& est) {
    double c = std::numeric_limits<double>::infinity();
    for (const auto& e : endpoints) {
      c = std::min(c, std::hypot(e.x - est.x_m, e.y - est.y_m));
    }
    return c;
  }

  ClearanceValidatorConfig config_;
};

}  // namespace lost_signal::robot::autonomy
