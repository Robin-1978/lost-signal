#include "robot/autonomy/behavior/behavior.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lost_signal::robot::autonomy {
namespace {

struct RoutePath {
 std::vector<core::Vec2> points;
 std::vector<double> progress_m;
};

RoutePath route_path_from_estimate(const Route& route,
 const EstimatedState& est, double fallback_length_m) {
 RoutePath path;
 path.points.push_back({est.x_m, est.y_m});
 if (!route.valid || route.points.empty()) {
 const double length = std::max(0.0, fallback_length_m);
 path.points.push_back({est.x_m + length * std::cos(est.yaw_rad),
 est.y_m + length * std::sin(est.yaw_rad)});
 } else if (route.points.size() == 1) {
 path.points.push_back({route.points.front().x_m,
 route.points.front().y_m});
 } else {
 std::size_t best_segment = 0;
 double best_fraction = 0.0;
 double best_distance2 = std::numeric_limits<double>::infinity();
 for (std::size_t i = 0; i + 1 < route.points.size(); ++i) {
 const core::Vec2 a{route.points[i].x_m, route.points[i].y_m};
 const core::Vec2 b{route.points[i + 1].x_m, route.points[i + 1].y_m};
 const core::Vec2 delta{b.x - a.x, b.y - a.y};
 const double length2 = delta.x * delta.x + delta.y * delta.y;
 if (length2 <= 1e-12) continue;
 const double fraction = std::clamp(
 ((est.x_m - a.x) * delta.x + (est.y_m - a.y) * delta.y) /
 length2, 0.0, 1.0);
 const double dx = est.x_m - (a.x + fraction * delta.x);
 const double dy = est.y_m - (a.y + fraction * delta.y);
 const double distance2 = dx * dx + dy * dy;
 if (distance2 < best_distance2) {
 best_distance2 = distance2;
 best_segment = i;
 best_fraction = fraction;
 }
 }
 const auto& a = route.points[best_segment];
 const auto& b = route.points[best_segment + 1];
 const core::Vec2 projection{
 a.x_m + best_fraction * (b.x_m - a.x_m),
 a.y_m + best_fraction * (b.y_m - a.y_m)};
 if (std::hypot(projection.x - est.x_m, projection.y - est.y_m) > 1e-6) {
 path.points.push_back(projection);
 }
 for (std::size_t i = best_segment + 1; i < route.points.size(); ++i) {
 const core::Vec2 point{route.points[i].x_m, route.points[i].y_m};
 if (std::hypot(point.x - path.points.back().x,
 point.y - path.points.back().y) > 1e-6) {
 path.points.push_back(point);
 }
 }
 }
 if (path.points.size() == 1) path.points.push_back(path.points.front());
 path.progress_m.resize(path.points.size(), 0.0);
 for (std::size_t i = 1; i < path.points.size(); ++i) {
 path.progress_m[i] = path.progress_m[i - 1] + std::hypot(
 path.points[i].x - path.points[i - 1].x,
 path.points[i].y - path.points[i - 1].y);
 }
 return path;
}

core::Vec2 point_at_progress(const RoutePath& path, double progress_m) {
 if (path.points.empty()) return {};
 const double progress = std::max(0.0, progress_m);
 for (std::size_t i = 1; i < path.points.size(); ++i) {
 if (progress > path.progress_m[i]) continue;
 const double span = path.progress_m[i] - path.progress_m[i - 1];
 const double fraction = span > 1e-12
 ? (progress - path.progress_m[i - 1]) / span : 0.0;
 return {
 path.points[i - 1].x +
 fraction * (path.points[i].x - path.points[i - 1].x),
 path.points[i - 1].y +
 fraction * (path.points[i].y - path.points[i - 1].y)};
 }
 return path.points.back();
}

double distance_to_path_prefix(const RoutePath& path, core::Vec2 point,
 double prefix_m) {
 double best = std::numeric_limits<double>::infinity();
 for (std::size_t i = 1; i < path.points.size(); ++i) {
 const double begin = path.progress_m[i - 1];
 if (begin > prefix_m) break;
 const double segment = path.progress_m[i] - begin;
 const double allowed = std::min(segment, std::max(0.0, prefix_m - begin));
 const core::Vec2 a = path.points[i - 1];
 const core::Vec2 full_delta{
 path.points[i].x - a.x, path.points[i].y - a.y};
 const double fraction = segment > 1e-12 ? allowed / segment : 0.0;
 const core::Vec2 b{a.x + fraction * full_delta.x,
 a.y + fraction * full_delta.y};
 const core::Vec2 delta{b.x - a.x, b.y - a.y};
 const double length2 = delta.x * delta.x + delta.y * delta.y;
 const double u = length2 > 1e-12
 ? std::clamp(((point.x - a.x) * delta.x +
 (point.y - a.y) * delta.y) / length2, 0.0, 1.0)
 : 0.0;
 best = std::min(best, std::hypot(
 point.x - (a.x + u * delta.x),
 point.y - (a.y + u * delta.y)));
 }
 return best;
}

} // namespace

WaitOnIntrusionBehavior::WaitOnIntrusionBehavior(
 RouteConflictBehaviorConfig config)
 : config_(config) {
 config_.near_route_hold_m = std::max(0.0, config_.near_route_hold_m);
 config_.prediction_horizon_s = std::max(0.0, config_.prediction_horizon_s);
 config_.brake_deceleration_mps2 =
 std::max(1e-6, config_.brake_deceleration_mps2);
 config_.extra_clearance_m = std::max(0.0, config_.extra_clearance_m);
}

BehaviorDecision WaitOnIntrusionBehavior::decide(
    const EstimatedState& est, const PerceptionResult& perception,
 const PredictionBatch& prediction,
 const Route& route,
    const platform::ChassisLimits& limits) {
  if (!est.valid || !perception.scan_ok) {
    return {true, 0.0, "unknown-evidence-hold"};
  }
 const double ego_radius = std::max(0.0, limits.footprint_radius_m);
 const double nominal_speed = std::max(0.0, limits.max_forward_mps);
 const double fallback_length = std::max(
 config_.near_route_hold_m,
 nominal_speed * config_.prediction_horizon_s);
 const RoutePath path = route_path_from_estimate(
 route, est, fallback_length);
 for (const auto& t : perception.tracks) {
 const double dx = t.x_m - est.x_m;
 const double dy = t.y_m - est.y_m;
 const double radius = ego_radius + t.occupied_radius_m();
 const double d0 = std::hypot(dx, dy);
 // A close contact is a hold even before the track has enough history for
 // prediction. The larger legacy radius is only a forward-path envelope.
 if (d0 <= radius) {
 return {true, 0.0, "track-contact-hold"};
 }
 // An immature track has no defensible future velocity yet. Hold only when
 // its measured envelope already occupies the actual route prefix; a return
 // along the current heading but away from a turning route is not a conflict.
 if (!t.prediction_ready && distance_to_path_prefix(
 path, {t.x_m, t.y_m}, config_.near_route_hold_m + radius) <= radius) {
 return {true, 0.0, "track-conflict-hold"};
 }
  }

 // Consume the same typed hypotheses as the local planners and validator.
 // The ego path is time-parameterized along the active route at the declared
 // maximum speed. Each interval uses continuous relative-motion clearance,
 // so a target crossing between prediction frames cannot disappear.
 const double available_horizon = prediction.future_times_s.empty()
 ? 0.0 : prediction.future_times_s.back();
 const double horizon = std::min(
 config_.prediction_horizon_s, available_horizon);
 if (!(horizon > 0.0) || !(nominal_speed > 0.0) ||
 prediction.future_times_s.size() < 2) {
 return {false, limits.max_forward_mps, "clear"};
 }

 PredictionBatch mature = prediction;
 mature.hypotheses.erase(std::remove_if(
 mature.hypotheses.begin(), mature.hypotheses.end(),
 [&](const PredictedHypothesis& hypothesis) {
 const auto current = std::find_if(
 perception.tracks.begin(), perception.tracks.end(),
 [&](const DynamicTrack& track) {
 return track.id == hypothesis.track_id;
 });
 return current == perception.tracks.end() || !current->prediction_ready;
 }), mature.hypotheses.end());

 const core::Vec2 ego_position{est.x_m, est.y_m};
 const PredictedClearance hold_clearance = minimum_predicted_clearance(
 mature, ego_position, ego_position, 0.0, horizon,
 ego_radius + config_.extra_clearance_m);
 const bool hold_is_safe = hold_clearance.clearance_m > 0.0;

 PredictedClearance first_conflict;
 for (std::size_t i = 1; i < prediction.future_times_s.size(); ++i) {
 const double t0 = prediction.future_times_s[i - 1];
 const double t1 = prediction.future_times_s[i];
 if (t0 >= horizon) break;
 const double interval_end = std::min(t1, horizon);
 const PredictedClearance clearance = minimum_predicted_clearance(
 mature, point_at_progress(path, nominal_speed * t0),
 point_at_progress(path, nominal_speed * interval_end),
 t0, interval_end, ego_radius + config_.extra_clearance_m);
 if (clearance.clearance_m <= 0.0 &&
 std::isfinite(clearance.time_s)) {
 first_conflict = clearance;
 break;
 }
 }
 if (!std::isfinite(first_conflict.time_s)) {
 return {false, limits.max_forward_mps, "clear"};
 }

 const double conflict_progress_m = std::min(
 path.progress_m.back(), nominal_speed * first_conflict.time_s);
 if (!hold_is_safe) {
 // A one-sided behavior constraint must not trap the robot where a target
 // is forecast to hit it. Leave candidate choice to DWA/TEB and the
 // independent validator, which can select a retreat or lateral escape.
 return {false, limits.max_forward_mps, "route-conflict-no-safe-hold"};
 }
 const double speed = std::max(0.0, est.forward_mps);
 const double braking_distance_m = speed * speed /
 (2.0 * config_.brake_deceleration_mps2);
 const double reaction_distance_m = speed * std::max(
 0.0, prediction.frame_dt_s);
 if (conflict_progress_m <= braking_distance_m + reaction_distance_m) {
 return {true, 0.0, "route-conflict-hold"};
 }
 const double safe_speed = std::sqrt(
 2.0 * config_.brake_deceleration_mps2 * conflict_progress_m);
 return {false, std::min(limits.max_forward_mps, safe_speed),
 "route-conflict-speed-cap"};
}

}  // namespace lost_signal::robot::autonomy
