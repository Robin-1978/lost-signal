#pragma once

// ROBOT-owned traversal belief (motion evidence + survey elevation prior,
// never SIM truth). The survey publishes elevation only — friction,
// contact, and wetness stay unpublished — so the model holds tire priors
// from its own hardware contract and estimates only what it can observe:
// roughness, from the IMU vertical-excitation observer below (main's
// RasterTraversalModel deadband form, adapted to our estimate/message
// types), and traction, from measured slip events latched per cell
// (beyond main: main prices the survey prior + roughness only; slipped-on
// cells cost more here until the memory expires). Cost and feasibility
// feed route choice (A* roughness hook) and task evidence; the speed
// envelope stays with the slip governor (measured slip, not roughness —
// bumpy but gripped ground must not read as slip).

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <vector>

#include "robot/autonomy/estimation/estimator.hpp"
#include "robot/autonomy/mapping/terrain_math.hpp"
#include "robot/core/geometry.hpp"
#include "robot/iface/map.hpp"

namespace lost_signal::robot::autonomy {

struct TraversalConfig {
  // Unknown terrain starts deliberately coarse: a prior for the planner,
  // not a claim about the surface. Motion evidence revises it per cell.
  double default_roughness_m = 0.02;
  double roughness_observation_floor_m = 0.005;
  double roughness_observation_ceiling_m = 0.20;
  double roughness_observation_deadband_mps2 = 0.35;
  double roughness_observation_min_speed_mps = 0.20;
  double roughness_belief_time_constant_s = 1.5;
  double roughness_belief_full_confidence_s = 2.0;
  double roughness_belief_max_age_s = 3.0;
  double roughness_support_base_m = 0.435;
  // Segment cost/feasibility on the surveyed grade (mirrors the A* prior
  // so route choice cannot disagree with itself across layers).
  double max_grade = 0.5;  // steeper rise/run is impassable
  double max_cross_grade = 0.4;  // steeper side-slope is impassable
  double grade_cost_per_m = 2.0;
  double roughness_cost_per_m = 8.0;
  // Traction memory pricing: slipped-on cells cost more per meter until
  // the memory expires (same spatial-staleness doctrine as roughness, own
  // age so the two beliefs tune independently).
  double slip_cost_per_m = 8.0;
  double slip_belief_max_age_s = 3.0;
};

struct TraversalCost {
  bool feasible = false;
  double cost = std::numeric_limits<double>::infinity();  // per segment
};

class TraversalModel {
 public:
  explicit TraversalModel(TraversalConfig config = {}) : config_(config) {}

  void set_config(TraversalConfig config) { config_ = config; }
  void bind(iface::MapSnapshot map) {
    map_ = std::move(map);
    beliefs_.clear();
    latest_observation_time_s_ = 0.0;
    have_observation_time_ = false;
  }
  bool ready() const { return map_.has_raster(); }
  std::uint64_t revision() const { return map_.revision; }

  // Roughness belief update from LSRI motion observations (estimate +
  // vertical specific force). No simulator contact or authored terrain
  // value is accepted here. Vertical excitation beyond the gravity
  // projection (from the ROBOT attitude estimate) and a noise deadband is
  // attributed to relief, inverted through the bump model at the measured
  // speed. Rising relief latches fast; returning to smooth is slow, so one
  // quiet sample cannot erase a real bump.
  void observe_motion(const EstimatedState& est, double imu_up_mps2,
                      double stamp_s, double dt_s) {
    if (!ready() || !(dt_s > 0.0)) return;
    if (std::isfinite(stamp_s)) {
      if (!have_observation_time_ || stamp_s > latest_observation_time_s_) {
        latest_observation_time_s_ = stamp_s;
      }
      have_observation_time_ = true;
    }
    if (!est.valid || !est.attitude_valid ||
        !std::isfinite(est.forward_mps) ||
        std::abs(est.forward_mps) <
            config_.roughness_observation_min_speed_mps ||
        !std::isfinite(imu_up_mps2) ||
        !(config_.roughness_belief_time_constant_s > 0.0) ||
        !(config_.roughness_belief_full_confidence_s > 0.0)) {
      return;
    }
    const int gx = static_cast<int>(
        std::floor((est.x_m - map_.origin_x_m) / map_.resolution_m));
    const int gy = static_cast<int>(
        std::floor((est.y_m - map_.origin_y_m) / map_.resolution_m));
    if (gx < 0 || gy < 0 || gx >= static_cast<int>(map_.width) ||
        gy >= static_cast<int>(map_.height)) {
      return;
    }
    // Specific force minus the gravity projection from the ROBOT attitude
    // estimate (never accel-derived: sustained body accel would eat its
    // own evidence — same discipline as the grip governor's survey grade).
    const double gravity_expected =
        kTerrainGravityMps2 * std::cos(est.roll_rad) * std::cos(est.pitch_rad);
    // Instrument floor (fixed white-noise allowance; the device model
    // documents bounded noise, the observer stays above it).
    constexpr double kInstrumentSigmaMps2 = 0.05;
    const double residual =
        std::max(0.0, std::abs(imu_up_mps2 - gravity_expected) -
                          std::max(config_.roughness_observation_deadband_mps2,
                                   3.0 * kInstrumentSigmaMps2));
    const double support_base =
        std::max(0.05, config_.roughness_support_base_m);
    const double wavenumber = 2.0 * core::kPi / support_base;
    const double speed_sq =
        std::max(0.04, est.forward_mps * est.forward_mps);
    const double measured = std::clamp(
        residual / std::max(1e-9, wavenumber * wavenumber * speed_sq),
        config_.roughness_observation_floor_m,
        config_.roughness_observation_ceiling_m);
    const std::uint64_t key =
        static_cast<std::uint64_t>(static_cast<std::uint32_t>(gy)) << 32 |
        static_cast<std::uint64_t>(static_cast<std::uint32_t>(gx));
    auto [it, inserted] = beliefs_.try_emplace(key);
    RoughnessBelief& belief = it->second;
    if (inserted) {
      belief.roughness_m = config_.default_roughness_m;
      belief.confidence = 0.0;
    }
    double alpha =
        1.0 - std::exp(-dt_s / config_.roughness_belief_time_constant_s);
    if (measured > belief.roughness_m) {
      alpha = std::max(alpha, 1.0 - std::exp(
                                    -dt_s / (0.35 * config_
                                                        .roughness_belief_time_constant_s)));
    }
    belief.roughness_m = (1.0 - alpha) * belief.roughness_m + alpha * measured;
    belief.confidence =
        std::min(1.0, belief.confidence +
                          dt_s / config_.roughness_belief_full_confidence_s);
    belief.observed_at_s = latest_observation_time_s_;
  }

  // Roughness belief at a point: observed value + confidence, or the
  // default prior at zero confidence (unknown, expired, or unbound map).
  double roughness_at(double x_m, double y_m,
                      double* confidence = nullptr) const {
    if (confidence != nullptr) *confidence = 0.0;
    if (!ready()) return config_.default_roughness_m;
    const int gx = static_cast<int>(
        std::floor((x_m - map_.origin_x_m) / map_.resolution_m));
    const int gy = static_cast<int>(
        std::floor((y_m - map_.origin_y_m) / map_.resolution_m));
    if (gx < 0 || gy < 0 || gx >= static_cast<int>(map_.width) ||
        gy >= static_cast<int>(map_.height)) {
      return config_.default_roughness_m;
    }
    const std::uint64_t key =
        static_cast<std::uint64_t>(static_cast<std::uint32_t>(gy)) << 32 |
        static_cast<std::uint64_t>(static_cast<std::uint32_t>(gx));
    const auto it = beliefs_.find(key);
    if (it == beliefs_.end()) return config_.default_roughness_m;
    if (have_observation_time_ &&
        latest_observation_time_s_ - it->second.observed_at_s >
            config_.roughness_belief_max_age_s) {
      return config_.default_roughness_m;  // stale: back to the prior
    }
    if (confidence != nullptr) *confidence = it->second.confidence;
    return it->second.roughness_m;
  }

  // Traction memory: latch measured slip per cell (the grip governor's
  // fire verdict + severity, never re-thresholded here). Worst latches
  // (like roughness rises fast): genuine traction loss persists, and the
  // age cap expires the memory back to the prior. Slip prices route cost
  // only — ice is traversable, never infeasible (the DWA gate owns the
  // binary veto on grade, never on traction).
  void observe_slip(double x_m, double y_m, double severity01,
                    double stamp_s) {
    if (!ready() || !std::isfinite(x_m) || !std::isfinite(y_m) ||
        !std::isfinite(stamp_s)) {
      return;
    }
    const int gx = static_cast<int>(
        std::floor((x_m - map_.origin_x_m) / map_.resolution_m));
    const int gy = static_cast<int>(
        std::floor((y_m - map_.origin_y_m) / map_.resolution_m));
    if (gx < 0 || gy < 0 || gx >= static_cast<int>(map_.width) ||
        gy >= static_cast<int>(map_.height)) {
      return;
    }
    if (std::isfinite(stamp_s)) {
      if (!have_observation_time_ || stamp_s > latest_observation_time_s_) {
        latest_observation_time_s_ = stamp_s;
      }
      have_observation_time_ = true;
    }
    const std::uint64_t key =
        static_cast<std::uint64_t>(static_cast<std::uint32_t>(gy)) << 32 |
        static_cast<std::uint64_t>(static_cast<std::uint32_t>(gx));
    auto [it, inserted] = beliefs_.try_emplace(key);
    RoughnessBelief& belief = it->second;
    if (inserted) {
      belief.roughness_m = config_.default_roughness_m;
      belief.confidence = 0.0;
    }
    belief.slip_01 =
        std::max(belief.slip_01, std::clamp(severity01, 0.0, 1.0));
    belief.slip_observed_at_s = latest_observation_time_s_;
  }

  // Traction memory at a point: worst latched severity, or 0 (unknown,
  // expired, or unbound map). Same grid and staleness doctrine as
  // roughness, independent age.
  double slip_at(double x_m, double y_m, double* confidence = nullptr) const {
    if (confidence != nullptr) *confidence = 0.0;
    if (!ready()) return 0.0;
    const int gx = static_cast<int>(
        std::floor((x_m - map_.origin_x_m) / map_.resolution_m));
    const int gy = static_cast<int>(
        std::floor((y_m - map_.origin_y_m) / map_.resolution_m));
    if (gx < 0 || gy < 0 || gx >= static_cast<int>(map_.width) ||
        gy >= static_cast<int>(map_.height)) {
      return 0.0;
    }
    const std::uint64_t key =
        static_cast<std::uint64_t>(static_cast<std::uint32_t>(gy)) << 32 |
        static_cast<std::uint64_t>(static_cast<std::uint32_t>(gx));
    const auto it = beliefs_.find(key);
    if (it == beliefs_.end()) return 0.0;
    if (have_observation_time_ &&
        latest_observation_time_s_ - it->second.slip_observed_at_s >
            config_.slip_belief_max_age_s) {
      return 0.0;  // stale: back to the prior
    }
    // A latched fire event is decisive evidence (unlike accumulated
    // roughness): fresh memory reports full confidence.
    if (confidence != nullptr) *confidence = 1.0;
    return it->second.slip_01;
  }

  // Segment cost for route choice: distance times (1 + uphill grade cost
  // + cross-slope cost + believed-roughness cost + traction-memory cost).
  // Infeasible (cliff or
  // side-slope beyond the caps) reads infinite cost, never a number.
  TraversalCost cost(double x0_m, double y0_m, double x1_m, double y1_m,
                     double yaw_rad) const {
    TraversalCost out;
    const double dist = std::hypot(x1_m - x0_m, y1_m - y0_m);
    if (!(dist > 1e-9)) {
      out.feasible = true;
      out.cost = 0.0;
      return out;
    }
    if (!feasible(x0_m, y0_m, x1_m, y1_m, yaw_rad)) return out;
    const float h0 = map_.has_elevation() ? map_.elevation_at(x0_m, y0_m)
                                          : std::numeric_limits<float>::quiet_NaN();
    const float h1 = map_.has_elevation() ? map_.elevation_at(x1_m, y1_m)
                                          : std::numeric_limits<float>::quiet_NaN();
    const double rise =
        (std::isfinite(h0) && std::isfinite(h1))
            ? std::max(0.0, static_cast<double>(h1 - h0))
            : 0.0;
    const double rough =
        roughness_at(0.5 * (x0_m + x1_m), 0.5 * (y0_m + y1_m));
    const double cross = std::abs(
        survey_lateral_grade(map_, 0.5 * (x0_m + x1_m), 0.5 * (y0_m + y1_m),
                             yaw_rad));
    const double slip = slip_at(0.5 * (x0_m + x1_m), 0.5 * (y0_m + y1_m));
    out.feasible = true;
    out.cost = dist * (1.0 + config_.grade_cost_per_m * rise / dist +
                       config_.roughness_cost_per_m * rough + cross +
                       config_.slip_cost_per_m * slip);
    return out;
  }

  // Segment feasibility on the surveyed grade: cliff and side-slope caps.
  // Unknown elevation is feasible (optimistic sampling; the validator owns
  // the conservative veto on raw evidence). The along-track grade reads
  // through the fixed survey-resolution window (same quantity the validator
  // gates on, one doctrine): raw endpoint differences over short rollout
  // steps would alias the survey's nearest-cell stairsteps into phantom
  // cliffs.
  bool feasible(double x0_m, double y0_m, double x1_m, double y1_m,
                double yaw_rad) const {
    if (!ready() || !map_.has_elevation()) return true;
    const double mx = 0.5 * (x0_m + x1_m), my = 0.5 * (y0_m + y1_m);
    if (std::abs(survey_longitudinal_grade(map_, mx, my, yaw_rad)) >
        config_.max_grade) {
      return false;
    }
    const double cross = std::abs(survey_lateral_grade(map_, mx, my, yaw_rad));
    return cross <= config_.max_cross_grade;
  }

 private:
  struct RoughnessBelief {
    double roughness_m = 0.0;
    double confidence = 0.0;
    double observed_at_s = 0.0;
    // Traction memory: worst latched slip severity in [0, 1] + its clock.
    // Same cell, same spatial-staleness doctrine, independent age.
    double slip_01 = 0.0;
    double slip_observed_at_s = 0.0;
  };

  TraversalConfig config_;
  iface::MapSnapshot map_;
  std::unordered_map<std::uint64_t, RoughnessBelief> beliefs_;
  double latest_observation_time_s_ = 0.0;
  bool have_observation_time_ = false;
};

}  // namespace lost_signal::robot::autonomy
