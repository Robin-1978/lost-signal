#pragma once

#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

#include "robot/autonomy/estimation/estimator.hpp"
#include "robot/autonomy/mapping/elevation.hpp"
#include "robot/autonomy/mapping/terrain_math.hpp"
#include "robot/autonomy/perception/perception.hpp"
#include "robot/autonomy/planning/local_planner.hpp"
#include "robot/core/geometry.hpp"
#include "robot/iface/map.hpp"
#include "robot/iface/observation.hpp"
#include "robot/core/deskew.hpp"
#include "robot/core/transform.hpp"

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
  kTerrainInfeasible = 6,      // surveyed grade beyond climb/side capability
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
    case TrajectoryVerdict::kTerrainInfeasible:
      return "terrain-infeasible";
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
 // Sensor mount lives in the TF tree; set via set_lidar_mount().
  // Rollout horizon for swept-path checks.
  double rollout_horizon_s = 2.0;
  double rollout_dt_s = 0.1;
  // MRE escape cap: only slow creep may be admitted as a minimum-risk
  // escape. A fast candidate inside the margin is rejected, never escaped.
  double escape_speed_cap_mps = 0.3;
 // Minimum measured clearance improvement for a low-speed escape. This is
 // deliberately below the normal margin: when already inside the footprint
 // envelope, requiring a full margin gain deadlocks the robot at corners.
 // The speed cap and monotonic gain remain hard gates.
 double escape_gain_floor_m = 0.05;
 // A beam this far from its static reference is a foreground return.
 // Perception predicts and tracks it; the static rollout must not also
 // treat the same endpoint as fixed geometry.
 double foreground_threshold_m = 0.30;
  // Rollover envelope (dynamic stability on side-slopes, master's form):
  // support width shares the traversal belief's support envelope (one
  // doctrine); COM height follows the declared plant loadout via
  // set_plant_geometry (payload-folded, description never truth) — the
  // struct defaults below are the pre-wiring stock assumption, kept so
  // direct-constructed validators behave exactly as before.
  // Risk 1.0 = static tip-over; the cap holds margin below it.
  double support_width_m = 0.435;
  double com_height_m = 0.30;
  double max_dynamic_rollover_risk = 0.8;
};

// Surveyed-elevation context for terrain-aware validation (navigation
// prior, never live physics). Null/absent map degrades to the planar
// behavior exactly: every return is a potential obstacle.
struct SurveyContext {
  const iface::MapSnapshot* map = nullptr;
  // Lidar plane above the body origin (TF mount height, fanned out by the
  // coordinator; a horizontal beam meets an up-ramp where the surface
  // crosses this plane).
  double sensor_height_m = 0.35;
  // Body origin above ground at spawn (chassis ride height, measured 0.10
  // on the engine): the estimate dead-reckons z above the spawn plane, so
  // the absolute sensor height is est.z + mount + ride.
 double ride_height_m = 0.10;
 // Spatial scale used to decide whether the terrain prior is resolvable.
 // Filled from the declared chassis footprint.
 double footprint_radius_m = 0.4;
  // Endpoint-vs-survey agreement to call a return surveyed ground.
  double ground_tol_m = 0.15;
  // Steeper along-track grade is unclimbable; steeper side-slope risks
  // rollover (same caps as route choice, one doctrine).
  double max_grade = 0.5;
  double max_cross_grade = 0.4;

  double sensor_z(const EstimatedState& est) const {
    return est.z_m + sensor_height_m + ride_height_m;
  }
  // Live elevation overlay (mapper output, measurement heights where
  // confident, NaN elsewhere). The validator prefers live where present
  // and falls back to the survey prior — same ground doctrine, two
  // sources. Live confirms ground only, never invents obstacles: obstacle
  // evidence stays raw-scan. Empty (unattached) disables live exactly.
  LiveElevationView live;
  // Ground test for one endpoint: live surface meets the sensor plane, else
  // the survey prior decides. Both sources share the tolerance.
  bool is_ground(double end_x_m, double end_y_m, double sensor_z_m) const {
    return endpoint_is_live_ground(
        live, end_x_m, end_y_m, sensor_z_m, ground_tol_m, [&] {
          return map != nullptr &&
                 endpoint_is_survey_ground(*map, end_x_m, end_y_m,
                                           sensor_z_m, ground_tol_m);
        });
  }
  // Context carries evidence when the estimate is valid and either source
  // is present (live alone suffices: survey gaps/revisions still validate).
  bool usable(const EstimatedState& est) const {
 // Elevation is a navigation prior.  Applying a sharp finite-difference
 // grade gate while the pose is uncertain turns localization drift into a
 // false terrain veto (and can strand the vehicle).  Keep raw scan safety
 // active, but let the validator use surveyed grade only once the fused
 // estimate is trustworthy enough to associate the footprint with the
 // survey cell.
 if (!est.valid || !std::isfinite(est.position_sigma_m) ||
 !std::isfinite(est.heading_sigma_rad) ||
 !((map != nullptr && map->has_elevation()) || live.attached())) {
 return false;
 }
 const double survey_scale = std::max(
 1e-3, std::max(map != nullptr && map->resolution_m > 0.0
 ? map->resolution_m : 0.0, footprint_radius_m));
 const double projected_sigma = est.position_sigma_m +
 survey_scale * std::sin(std::min(
 std::abs(est.heading_sigma_rad), 0.5 * core::kPi));
 return projected_sigma <= survey_scale;
  }
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
 // Lidar mount from the TF tree (coordinator fans out). Default ignore.
 virtual void set_lidar_mount(const core::RigidTransform& mount) {
 (void)mount;
 }
 virtual void set_lidar_yaw(double yaw_rad) { (void)yaw_rad; }
  // Declared plant geometry for stability envelopes (support width + COM
  // height, description from the machine file, never measured truth).
  // Default ignore (stock envelope); the concrete validator re-resolves.
  virtual void set_plant_geometry(double support_width_m, double com_height_m) {
    (void)support_width_m;
    (void)com_height_m;
  }
  // Survey-aware validation (default ignores the survey: planar behavior).
  // The concrete validator classifies surveyed-ground returns out of the
  // obstacle set and gates the rollout on the surveyed grade.
  virtual ValidationResult validate(const LocalCandidate& candidate,
                                    const EstimatedState& est,
                                    const PerceptionResult& perception,
                                    const iface::ObservationFrame& obs,
                                    const SurveyContext& survey) {
    (void)survey;
    return validate(candidate, est, perception, obs);
  }
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

 void set_lidar_mount(const core::RigidTransform& mount) override {
 base_from_lidar_ = mount;
 }
 void set_lidar_yaw(double yaw_rad) override {
 if (std::isfinite(yaw_rad)) {
 base_from_lidar_ = {{0.0, 0.0, 0.0},
 core::quaternion_from_rpy(0.0, 0.0, yaw_rad)};
 }
 }
  // Declared plant envelope (support width + COM height replace the
  // stock-chassis assumption; non-positive inputs are ignored, never
  // partially applied — the envelope stays internally consistent).
  void set_plant_geometry(double support_width_m, double com_height_m) override {
    if (support_width_m > 0.0) config_.support_width_m = support_width_m;
    if (com_height_m > 0.0) config_.com_height_m = com_height_m;
  }
  ValidationResult validate(const LocalCandidate& candidate,
                            const EstimatedState& est,
                            const PerceptionResult& perception,
                            const iface::ObservationFrame& obs) override {
    return validate_impl(candidate, est, perception, obs, nullptr);
  }
  // Survey-aware validation: endpoints consistent with surveyed ground are
  // not obstacles, and the rollout is gated on the surveyed grade (same
  // caps as route choice). Needs a valid estimate (sensor height rides on
  // it); otherwise degrades to the planar behavior exactly.
  ValidationResult validate(const LocalCandidate& candidate,
                            const EstimatedState& est,
                            const PerceptionResult& perception,
                            const iface::ObservationFrame& obs,
                            const SurveyContext& survey) override {
    const SurveyContext* ctx =
        survey.usable(est) ? &survey : nullptr;
    return validate_impl(candidate, est, perception, obs, ctx);
  }

  ValidationResult validate_impl(const LocalCandidate& candidate,
                                 const EstimatedState& est,
                                 const PerceptionResult& perception,
                                 const iface::ObservationFrame& obs,
                                 const SurveyContext* survey) {
    ValidationResult out;
    if (!candidate.feasible) {
      out.reason = "infeasible candidate";
      return out;
    }
    const bool hold = candidate.forward_mps == 0.0 &&
                      candidate.yaw_rate_radps == 0.0 &&
                      candidate.lateral_mps == 0.0;
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
    // Forward-sector minimum range (braking-tail check below). Surveyed
    // ground returns are not obstacle evidence (counted separately so an
    // all-ground sector reads open, never blind). Endpoints are deskewed
    // to the stamp pose with the measured twist (same helper as
    // perception/mapper, one doctrine).
    float sector_min = std::numeric_limits<float>::infinity();
    std::size_t forward_beams = 0, forward_ground = 0;
    const std::size_t n = obs.ranges_m.size();
    const bool has_toff =
        obs.time_offset_valid && obs.time_offsets_s.size() == n;
    const core::DeskewTwist twist{est.forward_mps, est.yaw_rate_radps};
 const core::RigidTransform map_from_base{
 {est.x_m, est.y_m, est.z_m},
 core::quaternion_from_rpy(est.roll_rad, est.pitch_rad, est.yaw_rad)};
 const core::RigidTransform map_from_sensor =
 core::compose(map_from_base, base_from_lidar_);
 const core::Vec3 sensor_p = map_from_sensor.translation_m;
    for (std::size_t i = 0; i < n; ++i) {
      // Use the observation's explicit scan layout (one definition shared
        // with perception and mapping).
        const double angle = iface::scan_beam_angle_rad(obs, i);
 const core::Vec3 beam_dir = core::rotate_vector(
 base_from_lidar_.rotation, {std::cos(angle), std::sin(angle), 0.0});
 const double base_angle = std::atan2(beam_dir.y, beam_dir.x);
 if (std::abs(core::wrap_angle(base_angle)) >
          config_.forward_half_angle_rad) {
        continue;
      }
      const float range = obs.ranges_m[i];
      if (!(range > 0.0f) || !std::isfinite(range)) continue;
      ++forward_beams;
      const double toff = has_toff ? obs.time_offsets_s[i] : 0.0;
      const core::Vec2 local = core::deskew_beam(range, angle, toff, twist);
 const core::Vec3 off = core::rotate_vector(
 map_from_sensor.rotation, {local.x, local.y, 0.0});
 const double ex = sensor_p.x + off.x;
 const double ey = sensor_p.y + off.y;
      if (survey != nullptr) {
        if (survey->is_ground(ex, ey, survey->sensor_z(est))) {
          ++forward_ground;
          continue;
        }
      }
      if (range < sector_min) sector_min = range;
    }
    if (!std::isfinite(sector_min)) {
      if (hold) {
        out.accepted = true;
        out.verdict = TrajectoryVerdict::kAccepted;
        out.reason = "hold";
        return out;
      }
      if (survey != nullptr && forward_ground > 0 &&
          forward_beams == forward_ground) {
        // Every forward return is surveyed ground: open sector, infinite
        // clearance. Falls through (the braking-need check below passes).
      } else {
        out.reason = "no forward returns";
        return out;
      }
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
    // Surveyed ground returns are excluded (same classifier as the sector
    // above, so the two cannot disagree on what counts as ground).
    std::vector<core::Vec2> endpoints;
    endpoints.reserve(n);
 const bool has_static_ref =
 obs.scan_static_valid && obs.static_ranges_m.size() == n;
 auto tracked_foreground = [&](double x, double y) {
 for (const auto& track : perception.tracks) {
 if (std::hypot(track.x_m - x, track.y_m - y) <=
 track.occupied_radius_m() + 0.5) {
 return true;
 }
 }
 return false;
 };
    for (std::size_t i = 0; i < n; ++i) {
      const float range = obs.ranges_m[i];
      if (!(range > 0.0f) || !std::isfinite(range)) continue;
      const double angle = iface::scan_beam_angle_rad(obs, i);
      const double toff = has_toff ? obs.time_offsets_s[i] : 0.0;
      const core::Vec2 local = core::deskew_beam(range, angle, toff, twist);
 const core::Vec3 off = core::rotate_vector(
 map_from_sensor.rotation, {local.x, local.y, 0.0});
 const double ex = sensor_p.x + off.x;
 const double ey = sensor_p.y + off.y;
 if (has_static_ref) {
 const float expected = obs.static_ranges_m[i];
 const bool foreground = std::isfinite(expected) && expected > 0.0f &&
 std::abs(range - expected) > config_.foreground_threshold_m;
 // A foreground return is removed from the static rollout only after the
 // perception layer has a matching predicted track. An unmatched return
 // remains a conservative static obstacle; missing association is not a
 // license to drive through raw evidence.
 if (foreground && tracked_foreground(ex, ey)) continue;
 }
      if (survey != nullptr &&
          survey->is_ground(ex, ey, survey->sensor_z(est))) {
        continue;
      }
      endpoints.push_back({ex, ey});
    }
    // Rollout the exact candidate twist; sample the swept footprint.
    const int steps = std::max(
        1, static_cast<int>(std::ceil(config_.rollout_horizon_s / config_.rollout_dt_s)));
    double min_clear = std::numeric_limits<double>::infinity();
    double end_clear = std::numeric_limits<double>::infinity();
    bool touches = false;
        // Keep the previous center: the footprint sweeps the segment between
        // rollout samples, so a thin obstacle cannot disappear between them.
        double prev_x = est.x_m;
        double prev_y = est.y_m;
        for (int s = 1; s <= steps; ++s) {
      const double t = s * config_.rollout_dt_s;
      // Exact constant-twist integration, body twist (v, lateral) rotated
      // into the map frame. Matches the planner's rollout convention.
      const double lat = candidate.lateral_mps;
      double ax = est.x_m, ay = est.y_m;
      if (std::abs(candidate.yaw_rate_radps) >= 1e-9) {
        const double w = candidate.yaw_rate_radps;
        const double h1 = est.yaw_rad + w * t;
        ax = est.x_m + (candidate.forward_mps * (std::sin(h1) - std::sin(est.yaw_rad)) +
                        lat * (std::cos(h1) - std::cos(est.yaw_rad))) / w;
        ay = est.y_m + (-candidate.forward_mps * (std::cos(h1) - std::cos(est.yaw_rad)) +
                        lat * (std::sin(h1) - std::sin(est.yaw_rad))) / w;
      } else {
        ax = est.x_m + (candidate.forward_mps * std::cos(est.yaw_rad) -
                        lat * std::sin(est.yaw_rad)) * t;
        ay = est.y_m + (candidate.forward_mps * std::sin(est.yaw_rad) +
                        lat * std::cos(est.yaw_rad)) * t;
      }
      double sample_min = std::numeric_limits<double>::infinity();
      const double vx = ax - prev_x;
                const double vy = ay - prev_y;
                const double seg2 = vx * vx + vy * vy;
                for (const auto& e : endpoints) {
                    const double d = std::hypot(e.x - ax, e.y - ay);
                    const double u =
                        seg2 > 1e-12
                            ? std::clamp(((e.x - prev_x) * vx +
                                          (e.y - prev_y) * vy) / seg2,
                                         0.0, 1.0)
                            : 0.0;
                    const double qx = prev_x + u * vx;
                    const double qy = prev_y + u * vy;
                    const double swept_d = std::hypot(e.x - qx, e.y - qy);
                    if (d < sample_min) sample_min = d;
                    if (swept_d < sample_min) sample_min = swept_d;
                    if (swept_d < config_.footprint_radius_m) touches = true;
                }
                min_clear = std::min(min_clear, sample_min);
                prev_x = ax;
                prev_y = ay;
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
      // Surveyed grade gates (same caps as route choice, one doctrine):
      // unclimbable along-track grade or rollover-risk cross-slope rejects
      // the rollout. Unknown survey reads flat and passes (the raw-scan
      // veto above still owns unmapped obstacles).
      if (survey != nullptr) {
        const double heading_t =
            est.yaw_rad + candidate.yaw_rate_radps * t;
        const double along =
            survey_longitudinal_grade(*survey->map, ax, ay, heading_t);
        if (std::abs(along) > survey->max_grade) {
          out.verdict = TrajectoryVerdict::kTerrainInfeasible;
          out.min_clearance_m = min_clear;
          out.reason = "grade exceeds capability";
          return out;
        }
        const double cross =
            survey_lateral_grade(*survey->map, ax, ay, heading_t);
        if (std::abs(cross) > survey->max_cross_grade) {
          out.verdict = TrajectoryVerdict::kTerrainInfeasible;
          out.min_clearance_m = min_clear;
          out.reason = "cross-slope exceeds capability";
          return out;
        }
        // Dynamic rollover envelope: surface tilt plus cornering load over
        // the support envelope (master's stability-risk form). Straight and
        // slow on a legal slope passes; carrying speed into a turn on the
        // same slope trips the cap. Lateral load is v*w for the constant
        // candidate twist (rotating-frame exact; yaw accel is zero).
        const double lateral_accel_mps2 =
            candidate.forward_mps * candidate.yaw_rate_radps;
        const double risk = dynamic_rollover_risk(
            cross, lateral_accel_mps2, config_.support_width_m,
            config_.com_height_m);
        if (risk > config_.max_dynamic_rollover_risk) {
          out.verdict = TrajectoryVerdict::kTerrainInfeasible;
          out.min_clearance_m = min_clear;
          out.reason = "dynamic rollover risk";
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
 // Lidar mount from the TF tree (default identity).
 core::RigidTransform base_from_lidar_{
 {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 1.0}};
};

}  // namespace lost_signal::robot::autonomy
