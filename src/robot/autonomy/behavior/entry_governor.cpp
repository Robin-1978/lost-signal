#include "robot/autonomy/behavior/entry_governor.hpp"
#include "robot/autonomy/planning/local/predictor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sim {

GovernorOutput govern_entry(GovernorState& state, const GovernorInput& input) {
  const bool fresh = input.occupied_fresh || input.clear_fresh;
  if (input.occupied_fresh) {
    state.hold_active = true;
    state.clear_streak = 0;
    state.silent_streak = 0;
 if (input.stop_valid) {
 state.stop_point = input.stop_point;
 state.stop_direction = input.stop_direction;
 state.stop_valid = true;
 } else {
 // Fresh occupied evidence may invalidate a previously safe waiting pose.
 // Keeping the old wall would command the robot to remain where the updated
 // forecast says it will be struck. HOLD remains active, but with no wall the
 // shared local planner must select among independently validated actions.
 state.stop_valid = false;
 }
  } else if (input.clear_fresh) {
    state.silent_streak = 0;
    if (state.hold_active) {
      ++state.clear_streak;
      if (state.clear_streak >= kReleaseClears) {
        state.hold_active = false;
        state.clear_streak = 0;
 state.stop_valid = false;
      }
    }
  } else {
    // Stale/uncertain: keep state (never invent a verdict), but do not hold
    // the brake on silence forever — tracks gone means DWA native owns it.
    ++state.silent_streak;
    state.clear_streak = 0;
    if (state.silent_streak >= kReleaseSilent) {
      state.hold_active = false;
 state.stop_valid = false;
    }
  }
  GovernorOutput out;
  out.hold_active = state.hold_active;
 // A hold without a valid stop point is still reported (DWA clamps to zero),
 // but once a valid virtual wall exists it persists for the hold lifetime.
 out.stop_point = state.stop_point;
 out.stop_direction = state.stop_direction;
 out.stop_valid = state.hold_active && state.stop_valid;
  (void)fresh;
  return out;
}

DynamicEntryAssessment assess_dynamic_entry_window(
 const EntryWindowRequest& request) {
 DynamicEntryAssessment result;
 if (request.route.size() < 2 || !(request.cruise_speed_mps > 0.0) ||
 !(request.frame_dt_s > 0.0) || request.predicted_frames.size() < 2 ||
 request.ego_radius_m < 0.0 || request.clearance_m < 0.0 ||
 !(request.braking_deceleration_mps2 > 0.0)) {
 return result;
 }
 result.prediction_evidence_available = std::any_of(
 request.predicted_frames.begin(), request.predicted_frames.end(),
 [](const auto& frame) { return !frame.empty(); });
 if (!result.prediction_evidence_available)
 return result;

 const double horizon_s = request.frame_dt_s *
 static_cast<double>(request.predicted_frames.size() - 1);
 const double maximum_route_length_m =
 request.cruise_speed_mps * horizon_s;
 std::vector<Vec2> local_route{request.route.front()};
 double remaining_m = maximum_route_length_m;
 for (std::size_t index = 0;
 index + 1 < request.route.size() && remaining_m > 1e-9; ++index) {
 const Vec2 from = request.route[index];
 const Vec2 to = request.route[index + 1];
 const double segment_m = std::hypot(to.x - from.x, to.y - from.y);
 if (segment_m <= 1e-9)
 continue;
 if (segment_m <= remaining_m + 1e-9) {
 local_route.push_back(to);
 remaining_m -= segment_m;
 } else {
 const double blend = remaining_m / segment_m;
 local_route.push_back({
 from.x + blend * (to.x - from.x),
 from.y + blend * (to.y - from.y),
 });
 remaining_m = 0.0;
 }
 }
 if (local_route.size() < 2)
 return result;

 const auto evidence = predicted_gap_evidence(
 local_route, 0, request.cruise_speed_mps,
 request.predicted_frames, request.frame_dt_s,
 request.ego_radius_m, request.clearance_m);
 result.conflict = evidence.conflict;
 result.minimum_gap_m = evidence.min_gap_m;
 result.first_conflict_s = evidence.first_conflict_s;
 result.first_conflict_position = evidence.first_conflict_position;
 result.governor.occupied_fresh = evidence.conflict;
 result.governor.clear_fresh = !evidence.conflict;
 if (!evidence.conflict)
 return result;

 // Convert first conflict time back into route progress, then reserve both
 // braking distance and the largest predicted target occupancy. This stop
 // geometry is derived from live ROBOT predictions and the ego route; it
 // does not need a mapped crossing or an authored stop position.
 const double conflict_progress_m =
 request.cruise_speed_mps * evidence.first_conflict_s;
 double route_progress_m = 0.0;
 std::size_t entry = 0;
 while (entry + 1 < local_route.size() &&
 route_progress_m < conflict_progress_m) {
 route_progress_m += std::hypot(
 local_route[entry + 1].x - local_route[entry].x,
 local_route[entry + 1].y - local_route[entry].y);
 ++entry;
 }
 double dynamic_envelope_m = 0.0;
 for (const auto& frame : request.predicted_frames) {
 for (const auto& future : frame) {
 dynamic_envelope_m = std::max(
 dynamic_envelope_m,
 request.ego_radius_m + future.occupied_radius_m() +
 request.clearance_m);
 }
 }
 const double braking_distance_m =
 request.cruise_speed_mps * request.cruise_speed_mps /
 (2.0 * request.braking_deceleration_mps2);
 const double required_backoff_m = braking_distance_m + dynamic_envelope_m;
 double walked_m = 0.0;
 std::size_t stop = entry;
 while (stop > 0 && walked_m < required_backoff_m) {
 walked_m += std::hypot(
 local_route[stop].x - local_route[stop - 1].x,
 local_route[stop].y - local_route[stop - 1].y);
 --stop;
 }
 const auto hold_is_swept = [&](Vec2 point) {
 for (const auto& frame : request.predicted_frames) {
 for (const auto& future : frame) {
 const double required_m = request.ego_radius_m +
 future.occupied_radius_m() + request.clearance_m;
 if (std::hypot(point.x - future.p.x, point.y - future.p.y) <
 required_m)
 return true;
 }
 }
 return false;
 };
 while (stop > 0 && hold_is_swept(local_route[stop]))
 --stop;
 if (hold_is_swept(local_route[stop])) {
 // No point on the available upstream route is a safe waiting trajectory.
 // Preserve occupied_fresh but publish no virtual wall. This is not passage
 // permission: every executable candidate still faces the same dynamic and
 // emergency-stop validators.
 result.governor.stop_valid = false;
 return result;
 }
 result.governor.stop_point = local_route[stop];
 if (stop + 1 < local_route.size()) {
 const Vec2 delta{
 local_route[stop + 1].x - local_route[stop].x,
 local_route[stop + 1].y - local_route[stop].y,
 };
 const double length_m = std::hypot(delta.x, delta.y);
 if (length_m > 1e-9) {
 result.governor.stop_direction = {
 delta.x / length_m, delta.y / length_m};
 }
 } else if (stop > 0) {
 const Vec2 delta{
 local_route[stop].x - local_route[stop - 1].x,
 local_route[stop].y - local_route[stop - 1].y,
 };
 const double length_m = std::hypot(delta.x, delta.y);
 if (length_m > 1e-9) {
 result.governor.stop_direction = {
 delta.x / length_m, delta.y / length_m};
 }
 }
 result.governor.stop_valid = true;
 result.safe_hold_available = true;
 result.stop_distance_m = 0.0;
 for (std::size_t index = 0; index < stop; ++index) {
 result.stop_distance_m += std::hypot(
 local_route[index + 1].x - local_route[index].x,
 local_route[index + 1].y - local_route[index].y);
 }
 return result;
}

GovernorOutput merge_entry_constraints(
 const GovernorOutput& first, const GovernorOutput& second,
 const std::vector<Vec2>& route) {
 if (!first.hold_active)
 return second;
 if (!second.hold_active)
 return first;
 if (!first.stop_valid)
 return first;
 if (!second.stop_valid)
 return second;
 if (route.empty())
 return first;

 const auto progress_to = [&](Vec2 point) {
 double best_distance_sq = std::numeric_limits<double>::infinity();
 double best_progress_m = std::numeric_limits<double>::infinity();
 double walked_m = 0.0;
 if (route.size() == 1) {
 const double dx = point.x - route.front().x;
 const double dy = point.y - route.front().y;
 return std::pair<double, double>{dx * dx + dy * dy, 0.0};
 }
 for (std::size_t index = 0; index + 1 < route.size(); ++index) {
 const Vec2 from = route[index];
 const Vec2 delta{
 route[index + 1].x - from.x,
 route[index + 1].y - from.y,
 };
 const double length_sq = delta.x * delta.x + delta.y * delta.y;
 if (length_sq <= 1e-18)
 continue;
 const double blend = std::clamp(
 ((point.x - from.x) * delta.x + (point.y - from.y) * delta.y) /
 length_sq,
 0.0, 1.0);
 const Vec2 projected{from.x + blend * delta.x, from.y + blend * delta.y};
 const double dx = point.x - projected.x;
 const double dy = point.y - projected.y;
 const double distance_sq = dx * dx + dy * dy;
 const double length_m = std::sqrt(length_sq);
 const double progress_m = walked_m + blend * length_m;
 if (distance_sq < best_distance_sq - 1e-18 ||
 (std::abs(distance_sq - best_distance_sq) <= 1e-18 &&
 progress_m < best_progress_m)) {
 best_distance_sq = distance_sq;
 best_progress_m = progress_m;
 }
 walked_m += length_m;
 }
 return std::pair<double, double>{best_distance_sq, best_progress_m};
 };
 const auto first_progress = progress_to(first.stop_point);
 const auto second_progress = progress_to(second.stop_point);
 if (second_progress.first < first_progress.first - 1e-12)
 return second;
 if (first_progress.first < second_progress.first - 1e-12)
 return first;
 return second_progress.second < first_progress.second ? second : first;
}

Vec2 governor_conflict_center(Vec2 ego, Vec2 ego_dir, Vec2 threat, Vec2 threat_vel) {
  const double tv = std::hypot(threat_vel.x, threat_vel.y);
  Vec2 center{(ego.x + threat.x) * 0.5, (ego.y + threat.y) * 0.5};
  if (tv < 1e-6) return center;
  const Vec2 v{threat_vel.x / tv, threat_vel.y / tv};
  const double denom = ego_dir.x * v.y - ego_dir.y * v.x;
  if (std::abs(denom) < 1e-6) return center;
  const double dx = threat.x - ego.x;
  const double dy = threat.y - ego.y;
  const double t = (dx * v.y - dy * v.x) / denom;
  const double s = (dx * ego_dir.y - dy * ego_dir.x) / denom;
  const double tc = std::max(0.0, std::min(t, 10.0));
  const double sc = std::max(0.0, std::min(s, 10.0));
  center = {(ego.x + ego_dir.x * tc + threat.x + v.x * sc) * 0.5,
            (ego.y + ego_dir.y * tc + threat.y + v.y * sc) * 0.5};
  return center;
}

} // namespace sim
