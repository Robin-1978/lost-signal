#include "robot/autonomy/behavior/crossing_behavior.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sim {
namespace {

constexpr double kStopLineToleranceM = 0.10;
constexpr double kReliableAwaySpeedMps = 0.25;
constexpr double kSpeedSearchStepMps = 0.10;

double distance(Vec2 a, Vec2 b) {
 return std::hypot(a.x - b.x, a.y - b.y);
}

bool inside(Vec2 point, const ConflictZone& zone) {
 return point.x >= zone.min.x && point.x <= zone.max.x && point.y >= zone.min.y && point.y <= zone.max.y;
}

bool segment_intersects(Vec2 from, Vec2 to, const ConflictZone& zone) {
 // Liang-Barsky clipping tests the continuous route segment. Checking route
 // vertices alone misses a narrow zone whenever both grid points lie just
 // outside opposite edges.
 double enter = 0.0;
 double leave = 1.0;
 const double dx = to.x - from.x;
 const double dy = to.y - from.y;
 const auto clip = [&](double p, double q) {
  if (std::abs(p) <= 1e-12)
   return q >= 0.0;
  const double ratio = q / p;
  if (p < 0.0)
   enter = std::max(enter, ratio);
  else
   leave = std::min(leave, ratio);
  return enter <= leave + 1e-12;
 };
 return clip(-dx, from.x - zone.min.x) && clip(dx, zone.max.x - from.x) &&
  clip(-dy, from.y - zone.min.y) && clip(dy, zone.max.y - from.y);
}

std::size_t physical_route_index(const std::vector<RoutePoint>& route, Vec2 ego_position) {
 std::size_t nearest = 0;
 double nearest_distance = std::numeric_limits<double>::infinity();
 for (std::size_t index = 0; index < route.size(); ++index) {
 const double candidate = distance(route[index].position, ego_position);
 if (candidate + 1e-12 < nearest_distance) {
 nearest = index;
 nearest_distance = candidate;
 }
 }
 return nearest;
}

struct RouteProjection {
 double along_m{0.0};
 double lateral_m{std::numeric_limits<double>::infinity()};
};

RouteProjection project_to_route(const std::vector<RoutePoint>& route, Vec2 point) {
 RouteProjection best;
 if (route.empty())
 return best;
 if (route.size() == 1)
 return {0.0, distance(route.front().position, point)};
 double accumulated_m = 0.0;
 for (std::size_t index = 0; index + 1 < route.size(); ++index) {
 const Vec2 from = route[index].position;
 const Vec2 delta{
 route[index + 1].position.x - from.x,
 route[index + 1].position.y - from.y,
 };
 const double length_m = std::hypot(delta.x, delta.y);
 if (length_m <= 1e-9)
 continue;
 const double blend = std::clamp(
 ((point.x - from.x) * delta.x + (point.y - from.y) * delta.y) /
 (length_m * length_m), 0.0, 1.0);
 const Vec2 projected{
 from.x + blend * delta.x,
 from.y + blend * delta.y,
 };
 const double lateral_m = distance(point, projected);
 if (lateral_m + 1e-12 < best.lateral_m) {
 best.lateral_m = lateral_m;
 best.along_m = accumulated_m + blend * length_m;
 }
 accumulated_m += length_m;
 }
 return best;
}

double route_distance_to_point(const std::vector<RoutePoint>& route, Vec2 ego_position,
 Vec2 target, double tolerance_m) {
 if (route.empty())
 return 0.0;
 const auto ego = project_to_route(route, ego_position);
 const auto goal = project_to_route(route, target);
 const double remaining = ego.lateral_m + std::max(0.0, goal.along_m - ego.along_m);
 return std::max(0.0, remaining - tolerance_m);
}

double distance_to_box(Vec2 point, const ConflictZone& zone) {
 const double dx = std::max({zone.min.x - point.x, 0.0, point.x - zone.max.x});
 const double dy = std::max({zone.min.y - point.y, 0.0, point.y - zone.max.y});
 return std::hypot(dx, dy);
}

bool safe_waiting_spot(Vec2 point,
 const std::vector<std::vector<DynamicObjectState>>& predicted_frames,
 double ego_radius_m, double margin_m) {
 for (const auto& frame : predicted_frames) {
 for (const auto& actor : frame) {
 const double clearance = distance(point, actor.p) - ego_radius_m - actor.occupied_radius_m();
 if (clearance < margin_m)
 return false;
 }
 }
 return true;
}

double crossing_wrap_angle(double angle) {
 while (angle > 3.14159265358979323846)
 angle -= 2.0 * 3.14159265358979323846;
 while (angle < -3.14159265358979323846)
 angle += 2.0 * 3.14159265358979323846;
 return angle;
}

std::vector<Vec2> simplify_polyline(const std::vector<Vec2>& input) {
 std::vector<Vec2> output;
 for (const auto point : input) {
 if (!output.empty() && distance(output.back(), point) <= 1e-6)
 continue;
 while (output.size() >= 2) {
 const Vec2 a = output[output.size() - 2];
 const Vec2 b = output.back();
 const double first = std::atan2(b.y - a.y, b.x - a.x);
 const double second = std::atan2(point.y - b.y, point.x - b.x);
 if (std::abs(crossing_wrap_angle(second - first)) > 1e-3)
 break;
 output.pop_back();
 }
 output.push_back(point);
 }
 return output;
}

// Time-parameterize the route with a bounded longitudinal acceleration and
// yaw rate. RO-1 starts a controlled crossing from rest, turns to the first
// leg in place, then follows subsequent corners while translating. A corner
// may stretch its adjacent leg but can never make execution faster than the
// longitudinal profile. This is intentionally a behavior-level envelope;
// the local planner still validates and executes every control sample.
std::vector<TimedPose> crossing_trajectory(const std::vector<Vec2>& raw_path,
 double start_heading_rad, double start_speed_mps, double target_speed_mps,
 double sample_dt_s, const CrossingExecutionCaps& caps) {
 const auto path = simplify_polyline(raw_path);
 if (path.empty() || target_speed_mps <= 0.0 || caps.max_accel_mps2 <= 0.0 ||
 caps.max_decel_mps2 <= 0.0 || caps.max_yaw_rate_radps <= 0.0) {
 return {};
 }
 std::vector<TimedPose> trajectory{{0.0, path.front()}};
 if (path.size() == 1)
 return trajectory;
 double time_s = 0.0;
 double speed_mps = std::clamp(std::abs(start_speed_mps), 0.0, target_speed_mps);
 double heading_rad = start_heading_rad;
 const double dt = std::max(1e-3, sample_dt_s);
 for (std::size_t segment = 0; segment + 1 < path.size(); ++segment) {
 const Vec2 from = path[segment];
 const Vec2 to = path[segment + 1];
 const double length = distance(from, to);
 if (length <= 1e-9)
 continue;
 const double desired_heading = std::atan2(to.y - from.y, to.x - from.x);
 const double yaw_time = std::abs(crossing_wrap_angle(desired_heading - heading_rad)) /
 caps.max_yaw_rate_radps;
 if (segment == 0 && yaw_time > 1e-9) {
 time_s += yaw_time;
 trajectory.push_back({time_s, from});
 }

 const double accel = speed_mps <= target_speed_mps ? caps.max_accel_mps2 : caps.max_decel_mps2;
 const double signed_accel = speed_mps <= target_speed_mps ? accel : -accel;
 const double target_delta = target_speed_mps - speed_mps;
 const double ramp_time = std::abs(target_delta) / accel;
 const double ramp_distance = speed_mps * ramp_time + 0.5 * signed_accel * ramp_time * ramp_time;
 double motion_time = 0.0;
 bool reaches_target = ramp_distance <= length + 1e-9;
 if (reaches_target) {
 motion_time = ramp_time + std::max(0.0, length - ramp_distance) / target_speed_mps;
 } else {
 const double discriminant = std::max(0.0, speed_mps * speed_mps + 2.0 * signed_accel * length);
 motion_time = signed_accel < 0.0
 ? (speed_mps - std::sqrt(discriminant)) / accel
 : (std::sqrt(discriminant) - speed_mps) / accel;
 }
 // Later heading changes happen over the adjoining motion. Stretching the
 // leg is a conservative kinematic envelope for a circular differential
 // footprint and preserves continuous position for the swept-gap oracle.
 const double duration = segment == 0 ? motion_time : std::max(motion_time, yaw_time);
 const double time_scale = motion_time > 1e-9 ? duration / motion_time : 1.0;
 const double start_time = time_s;
 for (double local_time = dt; local_time < motion_time - 1e-9; local_time += dt) {
 double walked = 0.0;
 if (local_time <= ramp_time) {
 walked = speed_mps * local_time + 0.5 * signed_accel * local_time * local_time;
 } else {
 walked = ramp_distance + target_speed_mps * (local_time - ramp_time);
 }
 const double alpha = std::clamp(walked / length, 0.0, 1.0);
 trajectory.push_back({start_time + local_time * time_scale,
 {from.x + (to.x - from.x) * alpha, from.y + (to.y - from.y) * alpha}});
 }
 time_s = start_time + duration;
 trajectory.push_back({time_s, to});
 if (reaches_target)
 speed_mps = target_speed_mps;
 else
 speed_mps = std::sqrt(std::max(0.0, speed_mps * speed_mps + 2.0 * signed_accel * length));
 heading_rad = desired_heading;
 }
 return trajectory;
}

DepartureAssessment assess_crossing_trajectory(
 const std::vector<TimedPose>& trajectory, double transit_speed_mps,
 const std::vector<std::vector<DynamicObjectState>>& predicted_frames,
 double frame_dt_s, const CrossingSafetyEnvelope& safety) {
 if (trajectory.empty() || predicted_frames.empty() || predicted_frames.front().empty())
 return {std::numeric_limits<double>::infinity(), DepartReason::kInvalid};
 const double horizon_s = static_cast<double>(predicted_frames.size() - 1) *
 std::max(1e-6, frame_dt_s);
 if (trajectory.back().t_s > horizon_s + 1e-9)
 return {std::numeric_limits<double>::infinity(), DepartReason::kHorizonShort};
 (void)transit_speed_mps;
 // Time uncertainty is a time-domain contract. Sweep early and late ego
 // execution against the same actor forecast instead of converting it into
 // speed * time and adding that distance twice to an already latency-aware
 // spatial clearance.
 double gap = std::numeric_limits<double>::infinity();
 const double shift_step_s = std::max(1e-6, frame_dt_s);
 for (double shift_s = -kCrossingTimeMarginS;
 shift_s <= kCrossingTimeMarginS + 1e-9; shift_s += shift_step_s) {
 auto shifted = trajectory;
 for (auto& pose : shifted)
 pose.t_s += shift_s;
 gap = std::min(gap, timed_gap(shifted, predicted_frames, frame_dt_s, safety.ego_radius_m));
 }
 const double required_gap = safety.clearance_m;
 if (gap >= required_gap)
 return {0.0, DepartReason::kGoNow};
 const std::vector<TimedPose> waiting{{0.0, trajectory.front().p}, {horizon_s, trajectory.front().p}};
 if (timed_gap(waiting, predicted_frames, frame_dt_s, safety.ego_radius_m) < required_gap)
 return {std::numeric_limits<double>::infinity(), DepartReason::kWaitUnsafe};
 return {std::numeric_limits<double>::infinity(), DepartReason::kBlockedInWindow};
}

} // namespace

CrossingReadiness assess_crossing_readiness(
 Vec2 ego_position, double ego_speed_mps, double ego_heading_rad,
 Vec2 stop_point, Vec2 stop_direction,
 const CrossingSafetyEnvelope& safety) {
 const double direction_norm = std::hypot(stop_direction.x, stop_direction.y);
 CrossingReadiness out;
 if (!(direction_norm > 1e-9) || !std::isfinite(direction_norm))
 return out;
 stop_direction.x /= direction_norm;
 stop_direction.y /= direction_norm;
 out.stop_longitudinal_m =
 (ego_position.x - stop_point.x) * stop_direction.x +
 (ego_position.y - stop_point.y) * stop_direction.y;
 out.stop_lateral_m =
 -(ego_position.x - stop_point.x) * stop_direction.y +
 (ego_position.y - stop_point.y) * stop_direction.x;
 out.heading_error_rad = crossing_wrap_angle(
 ego_heading_rad - std::atan2(stop_direction.y, stop_direction.x));
 const double position_tolerance = kStopLineToleranceM + safety.stop_position_tolerance_m;
 out.position_ready = std::hypot(out.stop_longitudinal_m, out.stop_lateral_m) <=
 position_tolerance;
 out.speed_ready = std::abs(ego_speed_mps) <= 0.10;
 out.heading_ready = std::abs(out.heading_error_rad) <=
 safety.stop_heading_tolerance_rad;
 return out;
}

CrossingEvaluation evaluate_crossing(const std::vector<ConflictZone>& zones,
                                     const std::vector<RoutePoint>& route, std::size_t path_index,
 Vec2 ego_position, double ego_heading_rad, double ego_speed_mps,
                                     const std::vector<std::vector<DynamicObjectState>>& predicted_frames,
 double frame_dt_s, const CrossingSafetyEnvelope& safety,
 const CrossingExecutionCaps& execution) {
 CrossingEvaluation out;
 if (route.empty() || zones.empty())
  return out;
 // A pursuit/lookahead index may run several nodes ahead of a lagging body.
 // Crossing relevance therefore follows the closest physical route point;
 // path_index is only retained as caller evidence and never grants clearance.
 (void)path_index;
 const std::size_t start = physical_route_index(route, ego_position);
 const ConflictZone* selected_zone = nullptr;
 std::size_t selected_entry = route.size();
 std::size_t selected_exit = route.size();
 std::size_t selected_distance = route.size();
 for (const auto& zone : zones) {
 const double route_relevance_distance = safety.ego_radius_m +
 safety.clearance_m + std::max(0.0, zone.participant_radius_m);
 const ConflictZone relevance_zone{
 zone.id,
 {zone.min.x - route_relevance_distance, zone.min.y - route_relevance_distance},
 {zone.max.x + route_relevance_distance, zone.max.y + route_relevance_distance},
 zone.participant_radius_m,
 };
 const bool body_inside_zone =
 distance_to_box(ego_position, zone) <=
 safety.ego_radius_m + std::max(0.0, zone.participant_radius_m);
  std::size_t scan = 0;
  while (scan < route.size()) {
   const auto route_intersects_at = [&](std::size_t index) {
 // A route need not pass through the mathematical center-line box to
 // conflict with traffic. Relevance uses the Minkowski envelope of both
 // participants plus the required clearance; stop/exit placement below
 // still measures Euclidean distance from the canonical semantic box.
 if (inside(route[index].position, relevance_zone))
     return true;
    return index + 1 < route.size() &&
 segment_intersects(route[index].position, route[index + 1].position,
 relevance_zone);
   };
   while (scan < route.size() && !route_intersects_at(scan))
    ++scan;
   if (scan == route.size())
    break;
   // The entry point is the first route vertex reached after the continuous
   // segment enters the zone. It may itself lie outside a narrow box.
   const std::size_t entry = std::min(scan + 1, route.size() - 1);
   while (scan + 1 < route.size() && route_intersects_at(scan + 1))
    ++scan;
   std::size_t exit = std::min(scan + 1, route.size() - 1);
   // The maneuver ends at a safe refuge, not at the mathematical box edge.
   // assess_departure holds its endpoint for the rest of the horizon, so the
   // endpoint must clear the zone by both footprints and the safety margin.
 const double zone_clear_distance = safety.ego_radius_m +
 safety.clearance_m + std::max(0.0, zone.participant_radius_m);
 while (exit + 1 < route.size() &&
 distance_to_box(route[exit].position, zone) < zone_clear_distance)
    ++exit;
   if (exit >= start || body_inside_zone) {
    const std::size_t route_distance = body_inside_zone ? 0 : (entry > start ? entry - start + 1 : 1);
    if (!selected_zone || route_distance < selected_distance ||
        (route_distance == selected_distance && entry < selected_entry)) {
     selected_zone = &zone;
     selected_entry = entry;
     selected_exit = exit;
     selected_distance = route_distance;
    }
    if (body_inside_zone || exit >= start)
     break;
   }
   ++scan;
  }
 }
 if (!selected_zone)
  return out;
 const double clear_distance = safety.ego_radius_m + safety.clearance_m +
 std::max(0.0, selected_zone->participant_radius_m);

 std::size_t stop = selected_entry;
 // Derive the stop line from the canonical route, not the controller's
 // moving lookahead index. Its world position therefore remains fixed while
 // the robot approaches and waits.
 // Clearance is Euclidean footprint separation from the conflict region,
 // not arc length walked backwards along the route.  On a diagonal approach
 // those differ by up to sqrt(2); using arc length placed the virtual wall
 // inside the swept vehicle envelope even though the route distance looked
 // sufficient.  Reserve the controller tolerance in the same geometric
 // metric used to determine physical conflict occupancy.
 while (stop > 0 && distance_to_box(route[stop].position, *selected_zone) <
 clear_distance + kStopLineToleranceM + safety.stop_position_tolerance_m) {
  --stop;
 }
 // The semantic box describes the intersection of participant reference
 // paths. Its declared participant radius makes the fixed stop safe before a
 // track matures. Forecast uncertainty remains time indexed in the hold check
 // below; folding its horizon maximum into geometry would count it twice.
 // A valid stop line must also be a safe hold pose over the complete
 // prediction window. Back up along the canonical route until waiting itself
 // satisfies the same footprint and margin contract used for traversal.
 while (stop > 0 && !safe_waiting_spot(
 route[stop].position, predicted_frames, safety.ego_radius_m,
 safety.clearance_m + safety.stop_position_tolerance_m)) {
 --stop;
 }
 // Global search vertices are discrete, but a behavior stop line is a
 // continuous world-space pose. Once an upstream vertex is safe and the next
 // one is unsafe, bisect that route segment to keep the stop as close to the
 // crossing as the geometric and predicted-hold contracts permit. This
 // avoids quantizing visibility and safety to the global grid resolution.
 const auto stop_pose_safe = [&](Vec2 point) {
 return distance_to_box(point, *selected_zone) >=
 clear_distance + kStopLineToleranceM + safety.stop_position_tolerance_m &&
 safe_waiting_spot(point, predicted_frames, safety.ego_radius_m,
 safety.clearance_m + safety.stop_position_tolerance_m);
 };
 Vec2 continuous_stop = route[stop].position;
 if (stop + 1 < route.size() && stop_pose_safe(continuous_stop) &&
 !stop_pose_safe(route[stop + 1].position)) {
 double safe_blend = 0.0;
 double unsafe_blend = 1.0;
 for (int iteration = 0; iteration < 32; ++iteration) {
 const double blend = 0.5 * (safe_blend + unsafe_blend);
 const Vec2 candidate{
 route[stop].position.x +
 blend * (route[stop + 1].position.x - route[stop].position.x),
 route[stop].position.y +
 blend * (route[stop + 1].position.y - route[stop].position.y),
 };
 if (stop_pose_safe(candidate))
 safe_blend = blend;
 else
 unsafe_blend = blend;
 }
 continuous_stop = {
 route[stop].position.x +
 safe_blend * (route[stop + 1].position.x - route[stop].position.x),
 route[stop].position.y +
 safe_blend * (route[stop + 1].position.y - route[stop].position.y),
 };
 }

 std::vector<Vec2> crossing_path{ego_position};
 for (std::size_t index = start; index <= selected_exit && index < route.size(); ++index) {
  if (distance(crossing_path.back(), route[index].position) > 1e-6)
   crossing_path.push_back(route[index].position);
 }

 out.relevant = true;
 // Controller path progress is not body occupancy: lookahead may advance
 // while the vehicle is laterally displaced. Physical geometry is the only
 // valid source for entering a conflict zone or reaching its stop line.
 out.body_in_conflict =
 distance_to_box(ego_position, *selected_zone) <= safety.ego_radius_m;
 Vec2 stop_direction{1.0, 0.0};
 if (stop + 1 < route.size()) {
  stop_direction = {
    route[stop + 1].position.x - route[stop].position.x,
    route[stop + 1].position.y - route[stop].position.y,
  };
 } else if (stop > 0) {
  stop_direction = {
    route[stop].position.x - route[stop - 1].position.x,
    route[stop].position.y - route[stop - 1].position.y,
  };
 }
 const double direction_norm = std::max(1e-9, std::hypot(stop_direction.x, stop_direction.y));
 stop_direction.x /= direction_norm;
 stop_direction.y /= direction_norm;
 const double longitudinal = (ego_position.x - continuous_stop.x) * stop_direction.x +
 (ego_position.y - continuous_stop.y) * stop_direction.y;
 const double lateral =
 -(ego_position.x - continuous_stop.x) * stop_direction.y +
 (ego_position.y - continuous_stop.y) * stop_direction.x;
 const double stop_pose_tolerance = kStopLineToleranceM + safety.stop_position_tolerance_m;
 out.stop_reached = std::hypot(longitudinal, lateral) <= stop_pose_tolerance;
 out.zone_id = selected_zone->id;
 out.entry_index = selected_entry;
 out.exit_index = selected_exit;
 out.stop_index = stop;
 out.stop_point = continuous_stop;
 out.stop_direction = stop_direction;
 out.exit_point = route[selected_exit].position;
 if (selected_exit + 1 < route.size()) {
 out.exit_direction = {
 route[selected_exit + 1].position.x - route[selected_exit].position.x,
 route[selected_exit + 1].position.y - route[selected_exit].position.y,
 };
 } else if (selected_exit > 0) {
 out.exit_direction = {
 route[selected_exit].position.x - route[selected_exit - 1].position.x,
 route[selected_exit].position.y - route[selected_exit - 1].position.y,
 };
 }
 const double exit_norm = std::max(1e-9, std::hypot(out.exit_direction.x, out.exit_direction.y));
 out.exit_direction.x /= exit_norm;
 out.exit_direction.y /= exit_norm;
 out.body_cleared = start >= selected_exit &&
 (ego_position.x - out.exit_point.x) * out.exit_direction.x +
 (ego_position.y - out.exit_point.y) * out.exit_direction.y >= -kStopLineToleranceM;
 // Braking distance follows the route rather than the infinite tangent
 // plane. On a curved approach, a point many metres away may project beyond
 // that plane even though the vehicle has not reached the crossing.
 out.stop_distance_m = route_distance_to_point(
 route, ego_position, continuous_stop, safety.stop_position_tolerance_m);
 out.transit_speed_mps = execution.preferred_speed_mps;
 out.prediction_evidence_available = !predicted_frames.empty() &&
 !predicted_frames.front().empty();
 // Gap acceptance also needs an intent guard. A mathematically open window
 // must not release on the first noisy samples of an actor reversal. Only
 // actors whose predicted envelope can reach this zone participate, and each
 // must be moving reliably outwards. Requiring its complete spatial envelope
 // to clear the zone as well would duplicate the time-indexed swept-gap gate
 // below and can deadlock a periodic actor whose endpoint is close to the
 // semantic box even though it will be clear when ego arrives.
 out.relevant_threats_moving_away = out.prediction_evidence_available;
 if (out.prediction_evidence_available) {
 for (std::size_t actor_index = 0; actor_index < predicted_frames.front().size(); ++actor_index) {
 const auto& current = predicted_frames.front()[actor_index];
 const double actor_envelope =
 current.occupied_radius_m() + safety.ego_radius_m + safety.clearance_m;
 bool can_reach_zone = false;
 for (const auto& frame : predicted_frames) {
 if (actor_index < frame.size() && distance_to_box(frame[actor_index].p, *selected_zone) <= actor_envelope) {
 can_reach_zone = true;
 break;
 }
 }
 if (!can_reach_zone)
 continue;
 const Vec2 closest{
 std::clamp(current.p.x, selected_zone->min.x, selected_zone->max.x),
 std::clamp(current.p.y, selected_zone->min.y, selected_zone->max.y),
 };
 const double outward_x = current.p.x - closest.x;
 const double outward_y = current.p.y - closest.y;
 const double outward_norm = std::max(1e-9, std::hypot(outward_x, outward_y));
 const double outward_speed =
 (current.v.x * outward_x + current.v.y * outward_y) / outward_norm;
 if (outward_speed < kReliableAwaySpeedMps) {
 out.relevant_threats_moving_away = false;
 break;
 }
 }
 }
 // Conflict-zone release needs time separation as well as footprint
 // separation. Express the time buffer in the same metric used by the swept
 // gap oracle so the decision stays independent of actor implementation.
 const double preferred_speed = std::clamp(execution.preferred_speed_mps, 0.0, execution.max_speed_mps);
 const double maximum_speed = std::max(preferred_speed, execution.max_speed_mps);
 bool any_covered = false;
 double best_gap = -std::numeric_limits<double>::infinity();
 out.departure = {std::numeric_limits<double>::infinity(), DepartReason::kInvalid};
 for (double speed = preferred_speed;;
 speed = std::min(maximum_speed, speed + kSpeedSearchStepMps)) {
 auto trajectory = crossing_trajectory(crossing_path, ego_heading_rad, ego_speed_mps,
 speed, frame_dt_s, execution);
 const auto assessment = assess_crossing_trajectory(
 trajectory, speed, predicted_frames, frame_dt_s, safety);
 double candidate_gap = std::numeric_limits<double>::infinity();
 if (trajectory.empty()) {
 candidate_gap = -std::numeric_limits<double>::infinity();
 } else {
 for (double shift_s = -kCrossingTimeMarginS;
 shift_s <= kCrossingTimeMarginS + 1e-9; shift_s += std::max(1e-6, frame_dt_s)) {
 auto shifted = trajectory;
 for (auto& pose : shifted)
 pose.t_s += shift_s;
 candidate_gap = std::min(candidate_gap,
 timed_gap(shifted, predicted_frames, frame_dt_s, safety.ego_radius_m));
 }
 }
 const double candidate_required_gap = safety.clearance_m;
 if (!trajectory.empty()) {
 const double horizon_s = static_cast<double>(predicted_frames.size() - 1) *
 std::max(1e-6, frame_dt_s);
 any_covered = any_covered || trajectory.back().t_s <= horizon_s + 1e-9;
 if (candidate_gap > best_gap) {
 best_gap = candidate_gap;
 out.transit_speed_mps = speed;
 out.execution_trajectory = trajectory;
 out.execution_duration_s = trajectory.back().t_s;
 out.execution_min_gap_m = candidate_gap;
 out.execution_required_gap_m = candidate_required_gap;
 }
 }
 if (assessment.reason == DepartReason::kGoNow) {
 out.transit_speed_mps = speed;
 out.execution_trajectory = std::move(trajectory);
 out.execution_duration_s = out.execution_trajectory.back().t_s;
 out.execution_min_gap_m = candidate_gap;
 out.execution_required_gap_m = candidate_required_gap;
 out.departure = assessment;
 break;
 }
 if (out.departure.reason == DepartReason::kInvalid ||
 (out.departure.reason == DepartReason::kHorizonShort && assessment.reason != DepartReason::kHorizonShort)) {
 out.departure = assessment;
 }
 if (speed >= maximum_speed - 1e-9)
 break;
 }
 if (!any_covered && out.departure.reason != DepartReason::kInvalid)
 out.departure = {std::numeric_limits<double>::infinity(), DepartReason::kHorizonShort};
 return out;
}

CrossingOutput govern_crossing(CrossingState& state, const CrossingInput& input, const std::string& zone_id) {
 if (state.phase == CrossingPhase::Committed) {
 if (input.body_cleared) {
 state.phase = CrossingPhase::Cleared;
 return {state.phase, false, false, state.stop_point, state.stop_direction, false, state.exit_point,
 state.exit_valid, state.transit_speed_mps};
 }
 return {state.phase, false, true, state.stop_point, state.stop_direction, false, state.exit_point,
 state.exit_valid, state.transit_speed_mps};
 }
 if (!input.relevant) {
 state = {};
 return {};
 }
 if (!zone_id.empty() && !state.zone_id.empty() && zone_id != state.zone_id)
  state = {};
 if (!zone_id.empty())
  state.zone_id = zone_id;
 if (input.stop_valid && !state.stop_valid) {
  state.stop_point = input.stop_point;
 state.stop_direction = input.stop_direction;
  state.stop_valid = true;
 }
 if (input.exit_valid && !state.exit_valid) {
 state.exit_point = input.exit_point;
 state.exit_direction = input.exit_direction;
 state.exit_valid = true;
 }
 if (input.transit_speed_mps > 0.0)
 state.transit_speed_mps = input.transit_speed_mps;
 if (input.body_in_conflict) {
  state.phase = CrossingPhase::Committed;
  state.clear_time_s = 0.0;
 state.clear_observation_samples = 0;
 return {state.phase, false, true, state.stop_point, state.stop_direction, false, state.exit_point, state.exit_valid,
 state.transit_speed_mps};
 }
 state.phase = CrossingPhase::Holding;
 if (input.ready_to_commit && input.safe_to_commit) {
  state.clear_time_s += std::max(0.0, input.dt_s);
 ++state.clear_observation_samples;
 if (state.clear_observation_samples >= kCrossingCommitConfirmSamples) {
   state.phase = CrossingPhase::Committed;
   state.clear_time_s = 0.0;
 state.clear_observation_samples = 0;
 return {state.phase, false, true, state.stop_point, state.stop_direction, false, state.exit_point, state.exit_valid,
 state.transit_speed_mps};
  }
 } else {
  state.clear_time_s = 0.0;
 state.clear_observation_samples = 0;
 }
 return {state.phase, true, false, state.stop_point, state.stop_direction, state.stop_valid, state.exit_point, state.exit_valid,
 state.transit_speed_mps};
}

void CrossingBehaviorSystem::reset() {
 state_ = {};
 cleared_zone_id_.clear();
}

CrossingBehaviorDecision CrossingBehaviorSystem::assess(
 const std::vector<CrossingZoneSpec>& zones,
 const std::vector<RoutePoint>& route, std::size_t path_index,
 Vec2 ego_position, double ego_speed_mps, double ego_heading_rad,
 const std::vector<std::vector<DynamicObjectState>>& predicted_frames,
 double frame_dt_s, const CrossingSafetyEnvelope& safety,
 const CrossingExecutionCaps& execution, double decision_dt_s) {
 std::vector<ConflictZone> geometry;
 geometry.reserve(zones.size());
 for (const auto& zone : zones)
 geometry.push_back(zone.geometry);

 CrossingBehaviorDecision decision;
 decision.has_active_zone = !zones.empty();
 decision.assessment = evaluate_crossing(
 geometry, route, path_index, ego_position, ego_heading_rad, ego_speed_mps,
 predicted_frames, frame_dt_s, safety, execution);

 const auto has_control_authority = [&](const std::string& id) {
 const auto match = std::find_if(zones.begin(), zones.end(), [&](const auto& zone) {
 return zone.geometry.id == id;
 });
 return match != zones.end() && match->authority == CrossingAuthority::Control;
 };

 if (decision.assessment.relevant && decision.assessment.zone_id == cleared_zone_id_) {
 decision.control.phase = CrossingPhase::Cleared;
 return decision;
 }
 if (!decision.assessment.relevant) {
 decision.control_authority = has_control_authority(state_.zone_id);
 const auto previous_phase = state_.phase;
 CrossingInput input;
 if (state_.phase == CrossingPhase::Committed && state_.exit_valid) {
 const Vec2 exit_delta{
 ego_position.x - state_.exit_point.x,
 ego_position.y - state_.exit_point.y,
 };
 const double longitudinal = exit_delta.x * state_.exit_direction.x +
 exit_delta.y * state_.exit_direction.y;
 const double lateral = -exit_delta.x * state_.exit_direction.y +
 exit_delta.y * state_.exit_direction.x;
 const double corridor_half_width = safety.ego_radius_m + safety.clearance_m +
 safety.stop_position_tolerance_m;
 input.body_cleared = longitudinal >= -0.10 && std::abs(lateral) <= corridor_half_width;
 }
 decision.control = govern_crossing(state_, input);
 if (decision.control.phase == CrossingPhase::Cleared)
 cleared_zone_id_ = state_.zone_id;
 decision.phase_changed = decision.control.phase != previous_phase;
 return decision;
 }

 decision.control_authority = has_control_authority(decision.assessment.zone_id);
 if (!decision.control_authority) {
 const auto previous_phase = state_.phase;
 decision.control = govern_crossing(state_, {});
 decision.phase_changed = decision.control.phase != previous_phase;
 return decision;
 }

 CrossingInput input;
 input.relevant = true;
 input.body_in_conflict = decision.assessment.body_in_conflict;
 input.body_cleared = decision.assessment.body_cleared;
 // Once Holding starts, admission and braking share the immutable stop pose
 // that was proven safe. Fresh predictions may change the release decision,
 // but they cannot move the virtual wall underneath a stopping vehicle.
 const Vec2 ready_stop = state_.stop_valid ? state_.stop_point : decision.assessment.stop_point;
 const Vec2 ready_direction = state_.stop_valid ? state_.stop_direction : decision.assessment.stop_direction;
 decision.readiness = assess_crossing_readiness(
 ego_position, ego_speed_mps, ego_heading_rad, ready_stop, ready_direction, safety);
 input.ready_to_commit = decision.readiness.ready();
 input.safe_to_commit = decision.assessment.departure.reason == DepartReason::kGoNow &&
 decision.assessment.prediction_evidence_available &&
 decision.assessment.relevant_threats_moving_away;
 decision.ready_to_commit = input.ready_to_commit;
 decision.safe_to_commit = input.safe_to_commit;
 input.stop_point = decision.assessment.stop_point;
 input.stop_direction = decision.assessment.stop_direction;
 input.stop_valid = true;
 input.exit_point = decision.assessment.exit_point;
 input.exit_direction = decision.assessment.exit_direction;
 input.exit_valid = true;
 input.transit_speed_mps = decision.assessment.transit_speed_mps;
 input.dt_s = decision_dt_s;
 const auto previous_phase = state_.phase;
 decision.control = govern_crossing(state_, input, decision.assessment.zone_id);
 if (state_.stop_valid) {
 // The state machine latches the virtual wall. Distance must be evaluated to
 // that same point even when a new prediction would choose an earlier stop;
 // mixing a fresh distance with the latched pose can produce a false zero.
 decision.assessment.stop_distance_m = route_distance_to_point(
 route, ego_position, state_.stop_point, safety.stop_position_tolerance_m);
 }
 if (decision.control.phase == CrossingPhase::Cleared)
 cleared_zone_id_ = state_.zone_id;
 decision.phase_changed = decision.control.phase != previous_phase;
 return decision;
}

} // namespace sim
