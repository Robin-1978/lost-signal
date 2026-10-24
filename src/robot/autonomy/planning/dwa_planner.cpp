#include "robot/autonomy/planning/dwa_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

namespace lost_signal::robot::autonomy {

namespace {

// --- Scalar command-window helpers (same semantics as the main-stack ones).

struct CommandWindow {
  double minimum{0.0};
  double maximum{0.0};
};

CommandWindow next_window(double current, double desired_min,
                          double desired_max, double max_delta) {
  return {std::max(desired_min, current - max_delta),
          std::min(desired_max, current + max_delta)};
}

CommandWindow next_velocity_window(double current, double desired_min,
                                   double desired_max, double max_accel_delta,
                                   double max_decel_delta) {
  double lo = current >= 0.0 ? current - max_decel_delta
                             : current - max_accel_delta;
  double hi = current >= 0.0 ? current + max_accel_delta
                             : current + max_decel_delta;
  lo = std::max(desired_min, lo);
  hi = std::min(desired_max, hi);
  if (lo > hi) {  // standstill band keeps the window non-empty
    lo = hi = std::clamp(0.0, desired_min, desired_max);
  }
  return {lo, hi};
}

double velocity_step_toward(double current, double target, double accel,
                            double decel, double dt) {
  if (dt <= 0.0) return current;
  if (target > current) return std::min(target, current + accel * dt);
  return std::max(target, current - decel * dt);
}

std::vector<double> command_samples(CommandWindow w, double spacing,
                                    std::vector<double> preferred = {}) {
  std::vector<double> out;
  if (w.maximum < w.minimum) return out;
  for (double v = w.minimum; v <= w.maximum + 1e-9; v += spacing)
    out.push_back(v);
  out.push_back(w.maximum);
  for (double p : preferred) {
    if (p >= w.minimum - 1e-9 && p <= w.maximum + 1e-9) out.push_back(p);
  }
  std::sort(out.begin(), out.end());
  out.erase(std::unique(out.begin(), out.end(),
                        [](double a, double b) {
                          return std::abs(a - b) < 1e-9;
                        }),
            out.end());
  return out;
}

// --- Path projection (route progress critic).

struct PathProjection {
  double distance_m{std::numeric_limits<double>::infinity()};
  double along_m{0.0};
  double tangent_rad{0.0};
};

PathProjection project_to_path(core::Vec2 point,
                               const std::vector<core::Vec2>& path) {
  PathProjection best;
  if (path.empty()) return best;
  if (path.size() == 1) {
    best.distance_m =
        std::hypot(point.x - path.front().x, point.y - path.front().y);
    return best;
  }
  double accumulated_m = 0.0;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    core::Vec2 delta{path[i + 1].x - path[i].x, path[i + 1].y - path[i].y};
    const double length_m = std::hypot(delta.x, delta.y);
    if (length_m < 1e-9) continue;
    const double projection = std::clamp(
        ((point.x - path[i].x) * delta.x + (point.y - path[i].y) * delta.y) /
            (length_m * length_m),
        0.0, 1.0);
    core::Vec2 closest{path[i].x + projection * delta.x,
                       path[i].y + projection * delta.y};
    const double distance_m =
        std::hypot(point.x - closest.x, point.y - closest.y);
    if (distance_m < best.distance_m) {
      best.distance_m = distance_m;
      best.along_m = accumulated_m + projection * length_m;
      best.tangent_rad = std::atan2(delta.y, delta.x);
    }
    accumulated_m += length_m;
  }
  return best;
}

struct RouteTarget {
 core::Vec2 position;
 double yaw_rad = 0.0;
 int gear = 0;
};

// Route lookahead: closest route point, then advance lookahead_m along it.
// Preserve the planner's body heading and gear annotation at the selected
// segment so a Hybrid-A* reverse decision is not lost in local sampling.
RouteTarget route_goal(const Route& route, double x, double y,
 double lookahead_m) {
  std::vector<core::Vec2> pts;
  pts.reserve(route.points.size());
  for (const auto& p : route.points) pts.push_back({p.x_m, p.y_m});
  PathProjection proj = project_to_path({x, y}, pts);
  double target_along = proj.along_m + lookahead_m;
  double accumulated_m = 0.0;
  for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
    const double seg =
        std::hypot(pts[i + 1].x - pts[i].x, pts[i + 1].y - pts[i].y);
    if (accumulated_m + seg >= target_along && seg > 1e-9) {
      const double s = (target_along - accumulated_m) / seg;
 return {{pts[i].x + s * (pts[i + 1].x - pts[i].x),
 pts[i].y + s * (pts[i + 1].y - pts[i].y)},
 route.points[i].yaw_rad, route.points[i].gear};
    }
    accumulated_m += seg;
  }
 return {pts.back(), route.points.back().yaw_rad,
 route.points.back().gear};
}

}  // namespace

DwaPlanner::DwaPlanner(DwaPlannerConfig config) : config_(config) {}

void DwaPlanner::reset() {
 previous_w_ = 0.0;
 previous_lateral_ = 0.0;
}

LocalPlan DwaPlanner::plan(const EstimatedState& est,
                           const PerceptionResult& perception,
                           const LocalMap& map, const Route& route,
                           const platform::ChassisLimits& limits) {
  LocalPlan out;
  if (!est.valid || !perception.scan_ok || !map.valid) return out;
  if (!route.valid || route.points.empty()) return out;
  const double v_max = limits.max_forward_mps;
  const double w_max = limits.max_yaw_rate_radps;
  if (!(v_max > 0.0) || !(w_max > 0.0)) return out;
 const double dt = config_.dt_s;
 const double predict_T = config_.predict_time_s;
 const double accel = config_.accel_mps2;
 const double decel = config_.decel_mps2 > 0.0 ? config_.decel_mps2 : accel;
 if (dt <= 0.0) return out;
  // Holonomic lateral sampling, gated by the chassis envelope: diff and
  // Ackermann bases carry max_lateral_mps == 0 and never sample sideways.
 // The previous command supplies the lateral state between measurements.
  std::vector<double> lateral_samples{0.0};
  if (limits.max_lateral_mps > 0.0) {
 const double lateral_delta = std::max(0.0, config_.lateral_accel_mps2) * dt;
 const CommandWindow lateral_window = next_window(
 previous_lateral_, -limits.max_lateral_mps, limits.max_lateral_mps,
 lateral_delta);
 lateral_samples = command_samples(lateral_window, 0.25,
 {previous_lateral_, 0.0, limits.max_lateral_mps,
 -limits.max_lateral_mps});
  }

  // Dynamic window from measured twist: reachable by the NEXT decision, not
  // by the end of the horizon. Never publish the rollout's terminal target.
  // Reverse is a first-class motion primitive: blocked-forward escapes,
 // rear goals, and tight docks need negative velocity. Symmetric with the
 // chassis forward limit (clamp_command already accepts the full range).
 const auto velocity_window = next_velocity_window(
 est.forward_mps, -v_max, v_max, accel * dt, decel * dt);
 const auto velocity_samples = command_samples(
 velocity_window, 0.25, {est.forward_mps, 0.0, v_max, -v_max});

 const RouteTarget target =
 route_goal(route, est.x_m, est.y_m, config_.lookahead_m);
 const core::Vec2 goal = target.position;
 const double heading_target = std::isfinite(target.yaw_rad)
 ? target.yaw_rad
 : std::atan2(goal.y - est.y_m, goal.x - est.x_m);
  const double goal_directed_yaw_rate = core::clamp(
      core::wrap_angle(heading_target - est.yaw_rad) /
          std::max(dt, predict_T),
      -w_max, w_max);

  // Predict dynamic tracks once per decision; rollouts index into frames.
  std::vector<PredictInput> inputs;
  for (const auto& t : perception.tracks)
    inputs.push_back(predict_input_from_track(t, est.stamp_ns * 1e-9));
  const double now_s = est.stamp_ns * 1e-9;
  const auto frames =
      predict_frames(inputs, now_s, predict_T, dt, /*max_age_s=*/5.0);
  const double ego_radius =
      limits.footprint_radius_m + config_.safety_margin_m;

  auto dynamic_hit = [&](double x, double y, double t) {
    if (frames.empty()) return false;
    std::size_t k = static_cast<std::size_t>(t / dt);
    if (k >= frames.size()) k = frames.size() - 1;
    for (const auto& s : frames[k]) {
      const double d = std::hypot(x - s.x_m, y - s.y_m);
      if (d < ego_radius + s.occupied_radius_m()) return true;
    }
    return false;
  };

  std::vector<core::Vec2> reference;
  reference.reserve(route.points.size());
  for (const auto& p : route.points) reference.push_back({p.x_m, p.y_m});

  LocalCandidate best;
  best.feasible = false;
  double best_score = -1e18;

  for (double velocity : velocity_samples) {
 if (target.gear < 0 && velocity > 1e-9) continue;
 if (target.gear > 0 && velocity < -1e-9) continue;
    const auto angular_window = next_window(est.yaw_rate_radps, -w_max, w_max,
                                            config_.yaw_accel_radps2 * dt);
    const auto angular_samples = command_samples(
        angular_window, 0.3,
        {est.yaw_rate_radps, 0.0, goal_directed_yaw_rate});
    for (double angular : angular_samples) {
 // Ackermann candidates are bounded by steering curvature and cannot turn
 // in place. Differential and holonomic bases leave this envelope at zero.
 if (limits.max_curvature_inv_m > 0.0) {
 const double max_angular = std::abs(velocity) *
 limits.max_curvature_inv_m;
 if (std::abs(angular) > max_angular + 1e-9) continue;
 }
      for (double lateral : lateral_samples) {
      double x = est.x_m, y = est.y_m, heading = est.yaw_rad;
      double rollout_v = est.forward_mps, rollout_w = est.yaw_rate_radps;
      std::vector<core::Vec2> samples{{x, y}};
      double free_time = predict_T;
      bool collision = false;
      for (double time = 0.0; time < predict_T; time += dt) {
        const double step_dt = std::min(dt, predict_T - time);
        const double previous_x = x;
        const double previous_y = y;
        // Preserve the measured twist during the response delay, then apply
        // physical limits. Without this, a nominal turn can look free while
        // the real chassis still translates.
        const double responsive_dt =
            std::max(0.0, time + step_dt -
                              std::max(time, config_.response_delay_s));
        rollout_v = velocity_step_toward(rollout_v, velocity, accel, decel,
                                         responsive_dt);
        rollout_w += core::clamp(angular - rollout_w,
                                 -config_.yaw_accel_radps2 * responsive_dt,
                                 config_.yaw_accel_radps2 * responsive_dt);
        // Body-frame twist (v, lateral) rotated into the map frame.
        x += (rollout_v * std::cos(heading) - lateral * std::sin(heading)) *
             step_dt;
        y += (rollout_v * std::sin(heading) + lateral * std::cos(heading)) *
             step_dt;
        heading += rollout_w * step_dt;
        const double sample_time = time + step_dt;
        samples.push_back({x, y});
        if (!map.segment_is_traversable(previous_x, previous_y, x, y) ||
            dynamic_hit(x, y, sample_time) ||
            (gate_ && !gate_(previous_x, previous_y, x, y, heading))) {
          collision = true;
          free_time = time;
          break;
        }
      }
      const double goal_score =
          -std::hypot(goal.x - x, goal.y - y);
      const double heading_score =
          std::cos(core::wrap_angle(heading_target - heading));
      // Clearance is a bounded preference among collision-free rollouts,
      // normalized by one cell and capped so open space never outweighs
      // route progress. The hard veto stays with the validator.
      const double normalized_clearance =
          std::clamp(map.clearance(x, y) / std::max(1e-9, map.resolution_m),
                     0.0, 1.0);
      constexpr double kClearanceRangeS = 0.15;
      const double clearance_score =
          collision ? free_time
                    : predict_T + 1.0 + kClearanceRangeS * normalized_clearance;
      double distance_sum_m = 0.0;
      for (const auto& s : samples)
        distance_sum_m += project_to_path(s, reference).distance_m;
      const auto start_projection = project_to_path({est.x_m, est.y_m}, reference);
      const auto end_projection = project_to_path({x, y}, reference);
      const double path_score =
          -distance_sum_m / static_cast<double>(samples.size()) +
          0.35 * std::cos(core::wrap_angle(end_projection.tangent_rad - heading));
      const double progress_score =
          end_projection.along_m - start_projection.along_m;
      const bool angular_reversal = angular * previous_w_ < -0.04;
      const double oscillation_penalty = angular_reversal ? 0.5 : 0.0;
      const double angular_effort_penalty = 0.025 * std::abs(angular);
      const double score =
          config_.weights.w_goal * (goal_score + 0.7 * heading_score) +
          config_.weights.w_clear * clearance_score +
          config_.weights.w_path * path_score +
          config_.weights.w_progress * progress_score +
          config_.weights.w_speed * velocity -
          config_.weights.w_oscillation * oscillation_penalty -
          angular_effort_penalty;
      if (!collision && score > best_score) {
        best_score = score;
        best.forward_mps = velocity;
        best.yaw_rate_radps = angular;
        best.lateral_mps = lateral;
        best.cost = -score;
        best.feasible = true;
      }
      }  // lateral
    }
  }
  if (!best.feasible) return out;
  previous_w_ = best.yaw_rate_radps;
 previous_lateral_ = best.lateral_mps;
  out.has_plan = true;
  out.best = best;
  return out;
}

}  // namespace lost_signal::robot::autonomy
