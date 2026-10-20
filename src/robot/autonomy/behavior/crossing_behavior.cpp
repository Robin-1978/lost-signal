#include "robot/autonomy/behavior/crossing_behavior.hpp"
#include "robot/autonomy/planning/path_smoothing.hpp"

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

double point_segment_distance(Vec2 point, Vec2 from, Vec2 to) {
 const Vec2 delta{to.x - from.x, to.y - from.y};
 const double length_sq = delta.x * delta.x + delta.y * delta.y;
 if (length_sq <= 1e-12)
 return distance(point, from);
 const double blend = std::clamp(
 ((point.x - from.x) * delta.x + (point.y - from.y) * delta.y) /
 length_sq, 0.0, 1.0);
 return distance(point, {from.x + blend * delta.x, from.y + blend * delta.y});
}

bool point_observed_free(Vec2 point, const std::vector<ObservedRay2D>& rays) {
 for (const auto& ray : rays) {
 const Vec2 delta{ray.endpoint.x - ray.origin.x,
 ray.endpoint.y - ray.origin.y};
 const double length = std::hypot(delta.x, delta.y);
 if (length <= 1e-9 || !(ray.angular_resolution_rad > 0.0))
 continue;
 const Vec2 relative{point.x - ray.origin.x, point.y - ray.origin.y};
 const double along = (relative.x * delta.x + relative.y * delta.y) / length;
 if (along <= 0.0 || along > length + 1e-9 ||
 (ray.physical_return && along >= length - 1e-9)) {
 continue;
 }
 const double cross = std::abs(relative.x * delta.y - relative.y * delta.x);
 const double angle_error = std::atan2(cross, along * length);
 // Coverage reports whether the point lies in the angular sampling cell of
 // an unobstructed measurement. It is diagnostic only: angular spacing is
 // neither optical beam width nor a guarantee that a small object between
 // rays would have been detected.
 if (angle_error <= 0.5 * ray.angular_resolution_rad + 1e-12)
 return true;
 }
 return false;
}

struct ApproachObservation {
 bool available{false};
 bool clear{false};
 double coverage{0.0};
 double unseen_arrival_lower_bound_s{0.0};
};

ApproachObservation observe_approaches(
 const ConflictZone& zone, const std::vector<ObservedRay2D>& rays,
 double visibility_stamp_s, double evaluation_stamp_s,
 double execution_duration_s) {
 ApproachObservation result;
 if (zone.observation_approaches.empty() || rays.empty() ||
 !std::isfinite(visibility_stamp_s)) {
 return result;
 }
 result.available = true;
 const double evidence_age_s = std::isfinite(evaluation_stamp_s)
 ? evaluation_stamp_s - visibility_stamp_s
 : 0.0;
 if (evidence_age_s < -1e-9)
 return {};
 (void)execution_duration_s;
 std::size_t samples = 0;
 std::size_t covered = 0;
 // Numerical integration spacing for the diagnostic percentage. This is not
 // a map cell size, participant footprint, safety margin or detectability
 // threshold, and it never participates in admission.
 constexpr double sample_spacing = 0.10;
 for (const auto& approach : zone.observation_approaches) {
 if (approach.centerline.size() < 2 || !(approach.half_width_m > 0.0)) {
 result.available = false;
 return result;
 }
 for (std::size_t segment = 0; segment + 1 < approach.centerline.size(); ++segment) {
 const Vec2 from = approach.centerline[segment];
 const Vec2 to = approach.centerline[segment + 1];
 const Vec2 delta{to.x - from.x, to.y - from.y};
 const double segment_length = std::hypot(delta.x, delta.y);
 if (segment_length <= 1e-9)
 continue;
 const Vec2 normal{-delta.y / segment_length, delta.x / segment_length};
 const int longitudinal_samples = std::max(
 1, static_cast<int>(std::ceil(segment_length / sample_spacing)));
 const int lateral_samples = std::max(
 1, static_cast<int>(std::ceil(
 2.0 * approach.half_width_m / sample_spacing)));
 for (int longitudinal = 0; longitudinal <= longitudinal_samples; ++longitudinal) {
 if (segment > 0 && longitudinal == 0)
 continue;
 const double along = static_cast<double>(longitudinal) /
 static_cast<double>(longitudinal_samples);
 const Vec2 centre{
 from.x + along * delta.x,
 from.y + along * delta.y,
 };
 for (int lateral = 0; lateral <= lateral_samples; ++lateral) {
 const double across = lateral_samples == 0
 ? 0.0
 : -approach.half_width_m +
 2.0 * approach.half_width_m * static_cast<double>(lateral) /
 static_cast<double>(lateral_samples);
 const Vec2 sample{
 centre.x + across * normal.x,
 centre.y + across * normal.y,
 };
 ++samples;
 const bool sample_visible = point_observed_free(sample, rays);
 if (sample_visible)
 ++covered;
 }
 }
 }
 }
 result.coverage = samples > 0
 ? static_cast<double>(covered) / static_cast<double>(samples)
 : 0.0;
 // Visibility is useful evidence for tracking and diagnostics, but without
 // an externally enforced traffic bound it cannot prove how soon an unseen
 // real-world participant may enter. It therefore never grants admission by
 // itself. The decision layer must continue using live tracks, uncertainty,
 // receding-horizon validation and the independent collision monitor.
 result.clear = false;
 result.unseen_arrival_lower_bound_s = 0.0;
 return result;
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

struct RouteProjection {
 double along_m{0.0};
 double lateral_m{std::numeric_limits<double>::infinity()};
 std::size_t segment_index{0};
 double segment_blend{0.0};
};

RouteProjection project_to_route(const std::vector<RoutePoint>& route, Vec2 point) {
 RouteProjection best;
 if (route.empty())
 return best;
 if (route.size() == 1)
 return {0.0, distance(route.front().position, point), 0, 0.0};
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
 // At a shared route vertex both adjacent segments have zero lateral error.
 // Prefer the later along-route projection so a conflict whose exit is that
 // vertex is treated as cleared rather than selected again from behind.
 const double along_m = accumulated_m + blend * length_m;
 if (lateral_m + 1e-12 < best.lateral_m ||
 (std::abs(lateral_m - best.lateral_m) <= 1e-12 &&
 (along_m > best.along_m + 1e-12 ||
 (std::abs(along_m - best.along_m) <= 1e-12 &&
 index > best.segment_index)))) {
 best.lateral_m = lateral_m;
 best.along_m = along_m;
 best.segment_index = index;
 best.segment_blend = blend;
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

bool safe_from_conflict_geometry(Vec2 point, const ConflictZone& zone,
 double required_center_distance_m) {
 if (distance_to_box(point, zone) < required_center_distance_m)
 return false;
 // A semantic conflict box marks the shared crossing itself. Its observable
 // approaches describe where an unseen participant can arrive from and are
 // therefore also unsafe places to hold. This geometric reserve is stable
 // when a finite-horizon track is temporarily far away or absent.
 for (const auto& approach : zone.observation_approaches) {
 for (std::size_t index = 0; index + 1 < approach.centerline.size(); ++index) {
 if (point_segment_distance(
 point, approach.centerline[index], approach.centerline[index + 1]) <
 required_center_distance_m + std::max(0.0, approach.half_width_m)) {
 return false;
 }
 }
 }
 return true;
}

bool overlaps_observation_approach(
 const DynamicObjectState& target, const ConflictZone& zone) {
 const double target_radius_m = target.occupied_radius_m();
 for (const auto& approach : zone.observation_approaches) {
 for (std::size_t index = 0; index + 1 < approach.centerline.size(); ++index) {
 if (point_segment_distance(
 target.p, approach.centerline[index], approach.centerline[index + 1]) <=
 std::max(0.0, approach.half_width_m) + target_radius_m) {
 return true;
 }
 }
 }
 return false;
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

double initial_trajectory_heading(
 const std::vector<TimedPose>& trajectory, double fallback_heading_rad) {
 for (std::size_t index = 1; index < trajectory.size(); ++index) {
 const Vec2 delta{
 trajectory[index].p.x - trajectory[index - 1].p.x,
 trajectory[index].p.y - trajectory[index - 1].p.y,
 };
 if (std::hypot(delta.x, delta.y) > 1e-6)
 return std::atan2(delta.y, delta.x);
 }
 return fallback_heading_rad;
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
 caps.max_decel_mps2 <= 0.0 || caps.max_yaw_rate_radps <= 0.0 ||
 caps.max_yaw_accel_radps2 <= 0.0) {
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
 const double yaw_distance =
 std::abs(crossing_wrap_angle(desired_heading - heading_rad));
 const double yaw_ramp_distance = caps.max_yaw_rate_radps * caps.max_yaw_rate_radps /
 caps.max_yaw_accel_radps2;
 // Rest-to-rest angular reachability. Crossing admission starts only after
 // the body has settled at the hold pose, so an instantaneous jump to the
 // yaw-rate limit would prove a window the actuator cannot execute.
 const double yaw_time = yaw_distance <= yaw_ramp_distance
 ? 2.0 * std::sqrt(yaw_distance / caps.max_yaw_accel_radps2)
 : 2.0 * caps.max_yaw_rate_radps / caps.max_yaw_accel_radps2 +
 (yaw_distance - yaw_ramp_distance) / caps.max_yaw_rate_radps;
 if (yaw_time > 1e-9) {
 time_s += yaw_time;
 trajectory.push_back({time_s, from});
 }

 // An intermediate polyline corner is a stop-turn-go contract. The final
 // point is different: it is the first route pose whose complete footprint
 // has cleared the conflict, not a docking target. Requiring zero speed there
 // lengthens the predicted conflict occupancy and disagrees with execution,
 // which hands control back to ordinary route following as soon as the body
 // clears the exit. Reach the final point at the admitted transit speed (or
 // the largest speed reachable over a short final leg), while retaining a
 // physical stop at every intermediate sharp corner.
 const bool final_segment = segment + 2 == path.size();
 double terminal_speed_mps = final_segment ? target_speed_mps : 0.0;
 const double minimum_transition_distance_m = terminal_speed_mps >= speed_mps
 ? (terminal_speed_mps * terminal_speed_mps - speed_mps * speed_mps) /
 (2.0 * caps.max_accel_mps2)
 : (speed_mps * speed_mps - terminal_speed_mps * terminal_speed_mps) /
 (2.0 * caps.max_decel_mps2);
 if (minimum_transition_distance_m > length + 1e-9) {
 if (!final_segment)
 return {};
 terminal_speed_mps = std::sqrt(std::max(
 0.0, speed_mps * speed_mps + 2.0 * caps.max_accel_mps2 * length));
 }
 const double peak_squared =
 (2.0 * length + speed_mps * speed_mps / caps.max_accel_mps2 +
 terminal_speed_mps * terminal_speed_mps / caps.max_decel_mps2) /
 (1.0 / caps.max_accel_mps2 + 1.0 / caps.max_decel_mps2);
 const double peak_speed = std::min(
 target_speed_mps, std::sqrt(std::max(0.0, peak_squared)));
 const double accel_time = std::max(
 0.0, (peak_speed - speed_mps) / caps.max_accel_mps2);
 const double accel_distance = std::max(
 0.0, (peak_speed * peak_speed - speed_mps * speed_mps) /
 (2.0 * caps.max_accel_mps2));
 const double decel_time = std::max(
 0.0, (peak_speed - terminal_speed_mps) / caps.max_decel_mps2);
 const double decel_distance =
 std::max(0.0,
 (peak_speed * peak_speed - terminal_speed_mps * terminal_speed_mps) /
 (2.0 * caps.max_decel_mps2));
 const double cruise_distance = std::max(
 0.0, length - accel_distance - decel_distance);
 const double cruise_time = peak_speed > 1e-9
 ? cruise_distance / peak_speed
 : 0.0;
 const double motion_time = accel_time + cruise_time + decel_time;
 // A generic differential contract cannot assume that a sharp route corner
 // is followed while translating. Budget a rest-to-rest in-place turn at
 // every corner, then translate the leg. A future continuous-curvature
 // tracker may provide a tighter proved envelope, but behavior must not
 // admit a timing window from motion the current controller cannot execute.
 // A new translation phase starts from rest after every explicit corner.
 // Budget the same delivered controller response used by the emergency-stop
 // validator; an ideal zero-delay acceleration profile otherwise runs ahead
 // of the physical base before it reaches the conflict region.
 time_s += std::max(0.0, caps.actuator_response_delay_s);
 trajectory.push_back({time_s, from});
 const double start_time = time_s;
 for (double local_time = dt; local_time < motion_time - 1e-9; local_time += dt) {
 double walked = 0.0;
 if (local_time <= accel_time) {
 walked = speed_mps * local_time +
 0.5 * caps.max_accel_mps2 * local_time * local_time;
 } else if (local_time <= accel_time + cruise_time) {
 walked = accel_distance + peak_speed * (local_time - accel_time);
 } else {
 const double braking_time = local_time - accel_time - cruise_time;
 walked = accel_distance + cruise_distance +
 peak_speed * braking_time -
 0.5 * caps.max_decel_mps2 * braking_time * braking_time;
 }
 const double alpha = std::clamp(walked / length, 0.0, 1.0);
 trajectory.push_back({start_time + local_time,
 {from.x + (to.x - from.x) * alpha, from.y + (to.y - from.y) * alpha}});
 }
 time_s = start_time + motion_time;
 trajectory.push_back({time_s, to});
 speed_mps = terminal_speed_mps;
 heading_rad = desired_heading;
 }
 return trajectory;
}

// Time-parameterize a fully filleted centreline without inventing stops at
// its arc samples. Linear acceleration/deceleration bounds are applied with
// forward/backward passes. Curvature limits speed by yaw rate, and curvature
// transitions receive a conservative finite-difference yaw-acceleration cap.
// The local planner still validates the resulting tracking tube and physical
// stopping tail on every control cycle.
std::vector<TimedPose> continuous_crossing_trajectory(
 const FilletPath& path, double start_heading_rad, double start_speed_mps,
 double target_speed_mps, const CrossingExecutionCaps& caps) {
 if (!path.valid || !path.fully_smoothed || path.points.size() < 2 ||
 path.points.size() != path.curvature_inv_m.size() ||
 !(target_speed_mps > 0.0) || !(caps.max_accel_mps2 > 0.0) ||
 !(caps.max_decel_mps2 > 0.0) || !(caps.max_yaw_rate_radps > 0.0) ||
 !(caps.max_yaw_accel_radps2 > 0.0)) {
 return {};
 }
 std::vector<double> leg_m(path.points.size() - 1, 0.0);
 std::vector<double> speed(path.points.size(), target_speed_mps);
 for (std::size_t index = 0; index + 1 < path.points.size(); ++index) {
 leg_m[index] = distance(path.points[index], path.points[index + 1]);
 if (leg_m[index] <= 1e-9)
 return {};
 }
 for (std::size_t index = 0; index < speed.size(); ++index) {
 const double curvature = std::abs(path.curvature_inv_m[index]);
 if (curvature > 1e-9)
 speed[index] = std::min(speed[index], caps.max_yaw_rate_radps / curvature);
 }
 // A circular fillet has a step in curvature at each tangent. Bound the
 // reference speed around that step so |delta omega| / delta t cannot exceed
 // the declared angular acceleration at the geometric sample scale.
 for (std::size_t index = 0; index + 1 < speed.size(); ++index) {
 const double curvature_change = std::abs(
 path.curvature_inv_m[index + 1] - path.curvature_inv_m[index]);
 if (curvature_change <= 1e-9)
 continue;
 const double transition_cap = std::sqrt(std::max(
 0.0, caps.max_yaw_accel_radps2 * leg_m[index] / curvature_change));
 speed[index] = std::min(speed[index], transition_cap);
 speed[index + 1] = std::min(speed[index + 1], transition_cap);
 }
 speed.front() = std::min(speed.front(),
 std::clamp(std::abs(start_speed_mps), 0.0, target_speed_mps));
 for (std::size_t index = 1; index < speed.size(); ++index) {
 speed[index] = std::min(speed[index], std::sqrt(std::max(
 0.0, speed[index - 1] * speed[index - 1] +
 2.0 * caps.max_accel_mps2 * leg_m[index - 1])));
 }
 for (std::size_t index = speed.size() - 1; index > 0; --index) {
 speed[index - 1] = std::min(speed[index - 1], std::sqrt(std::max(
 0.0, speed[index] * speed[index] +
 2.0 * caps.max_decel_mps2 * leg_m[index - 1])));
 }

 const Vec2 first_delta{
 path.points[1].x - path.points[0].x,
 path.points[1].y - path.points[0].y,
 };
 const double first_heading = std::atan2(first_delta.y, first_delta.x);
 const double yaw_distance = std::abs(crossing_wrap_angle(
 first_heading - start_heading_rad));
 const double yaw_ramp_distance = caps.max_yaw_rate_radps *
 caps.max_yaw_rate_radps / caps.max_yaw_accel_radps2;
 const double yaw_time = yaw_distance <= yaw_ramp_distance
 ? 2.0 * std::sqrt(yaw_distance / caps.max_yaw_accel_radps2)
 : 2.0 * caps.max_yaw_rate_radps / caps.max_yaw_accel_radps2 +
 (yaw_distance - yaw_ramp_distance) / caps.max_yaw_rate_radps;
 double time_s = yaw_time + std::max(0.0, caps.actuator_response_delay_s);
 std::vector<TimedPose> trajectory{{0.0, path.points.front()}};
 if (time_s > 1e-9)
 trajectory.push_back({time_s, path.points.front()});
 for (std::size_t index = 0; index + 1 < path.points.size(); ++index) {
 const double average_speed = 0.5 * (speed[index] + speed[index + 1]);
 if (average_speed <= 1e-9)
 return {};
 time_s += leg_m[index] / average_speed;
 trajectory.push_back({time_s, path.points[index + 1]});
 }
 return trajectory;
}

bool smoothed_path_is_static_feasible(
 const FilletPath& path, const CrossingPathFeasibility& path_feasible) {
 if (!path.valid || !path.fully_smoothed || !path_feasible)
 return false;
 for (std::size_t index = 0; index + 1 < path.points.size(); ++index) {
 if (!path_feasible(path.points[index], path.points[index + 1]))
 return false;
 }
 return true;
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
 const double required_gap = safety.clearance_m +
 std::max(0.0, safety.tracking_error_m);
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
 const double position_error = std::hypot(
 out.stop_longitudinal_m, out.stop_lateral_m);
 out.position_ready = position_error <= position_tolerance;
 out.position_settled = position_error <= 0.5 * position_tolerance;
 out.speed_ready = std::abs(ego_speed_mps) <= 0.10;
 out.heading_ready = std::abs(out.heading_error_rad) <=
 safety.stop_heading_tolerance_rad;
 return out;
}

CrossingPoseApproach crossing_pose_approach(
 Vec2 ego_position, double ego_heading_rad, Vec2 stop_point,
 Vec2 stop_direction, bool force_reverse,
 double drive_axle_x_m, double acquisition_radius_m) {
 const double route_heading = std::atan2(stop_direction.y, stop_direction.x);
 const Vec2 control_position{
 ego_position.x + drive_axle_x_m * std::cos(ego_heading_rad),
 ego_position.y + drive_axle_x_m * std::sin(ego_heading_rad),
 };
 const Vec2 control_target{
 stop_point.x + drive_axle_x_m * std::cos(route_heading),
 stop_point.y + drive_axle_x_m * std::sin(route_heading),
 };
 const double control_distance = distance(control_position, control_target);
 if (force_reverse)
 return {true, route_heading, false, control_distance};
 if (control_distance <= std::max(0.0, acquisition_radius_m))
 return {false, route_heading, true, control_distance};
 const double bearing = std::atan2(
 control_target.y - control_position.y,
 control_target.x - control_position.x);
 const double reverse_heading = crossing_wrap_angle(bearing + kPi);
 const double forward_turn = std::abs(crossing_wrap_angle(
 bearing - ego_heading_rad));
 const double reverse_turn = std::abs(crossing_wrap_angle(
 reverse_heading - ego_heading_rad));
 // A small angular preference prevents an exactly sideways target from
 // changing gear due only to floating-point noise.
 constexpr double kGearHeadingHysteresisRad = 0.05;
 const bool reverse = reverse_turn + kGearHeadingHysteresisRad < forward_turn;
 return {reverse, reverse ? reverse_heading : bearing, false, control_distance};
}

CrossingEvaluation evaluate_crossing(const std::vector<ConflictZone>& zones,
                                     const std::vector<RoutePoint>& route, std::size_t path_index,
 Vec2 ego_position, double ego_heading_rad, double ego_speed_mps,
                                     const std::vector<std::vector<DynamicObjectState>>& predicted_frames,
 double frame_dt_s, const CrossingSafetyEnvelope& safety,
 const CrossingExecutionCaps& execution,
 const std::vector<ObservedRay2D>& visibility_rays,
 double visibility_stamp_s, double evaluation_stamp_s,
 bool committed_continuation,
 CrossingPathFeasibility path_feasible) {
 CrossingEvaluation out;
 if (route.empty() || zones.empty())
  return out;
 // A pursuit/lookahead index may run several nodes ahead of a lagging body.
 // Crossing relevance therefore follows the closest physical route point;
 // path_index is only retained as caller evidence and never grants clearance.
 (void)path_index;
 const auto physical_projection = project_to_route(route, ego_position);
 const std::size_t start = std::min(
 physical_projection.segment_index, route.size() - 1);
 const ConflictZone* selected_zone = nullptr;
 std::size_t selected_entry = route.size();
 std::size_t selected_exit = route.size();
 std::size_t selected_distance = route.size();
 for (const auto& zone : zones) {
 const double route_relevance_distance = safety.ego_radius_m +
 safety.clearance_m;
 const ConflictZone relevance_zone{
 zone.id,
 {zone.min.x - route_relevance_distance, zone.min.y - route_relevance_distance},
 {zone.max.x + route_relevance_distance, zone.max.y + route_relevance_distance},
 {},
 };
 const bool body_inside_zone =
 distance_to_box(ego_position, zone) <=
 safety.ego_radius_m;
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
 // route_intersects_at(scan) also examines the segment ending at scan + 1.
 // After the loop, route[scan] can already be the first vertex outside the
 // relevance envelope. Starting from scan + 1 skipped that valid refuge and
 // needlessly lengthened the proved maneuver by one complete grid edge.
 // The clearance loop below remains the physical authority and advances from
 // scan whenever that vertex has not cleared the full footprint envelope.
 std::size_t exit = scan;
   // The maneuver ends at a safe refuge, not at the mathematical box edge.
   // assess_departure holds its endpoint for the rest of the horizon, so the
   // endpoint must clear the zone by both footprints and the safety margin.
 const double zone_clear_distance = safety.ego_radius_m +
 safety.clearance_m;
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

 // Keep only hypotheses whose swept occupancy reaches this crossing or one
 // of its mapped approaches. A different moving object elsewhere in the map
 // remains the generic local validator's responsibility, but it cannot move
 // this crossing's fixed hold pose several metres upstream.
 std::vector<std::size_t> relevant_actor_indices;
 if (!predicted_frames.empty()) {
 for (std::size_t actor_index = 0;
 actor_index < predicted_frames.front().size(); ++actor_index) {
 bool can_reach_zone = false;
 for (const auto& frame : predicted_frames) {
 if (actor_index >= frame.size())
 continue;
 const double actor_envelope = frame[actor_index].occupied_radius_m() +
 safety.ego_radius_m + safety.clearance_m;
 if (distance_to_box(frame[actor_index].p, *selected_zone) <= actor_envelope ||
 overlaps_observation_approach(frame[actor_index], *selected_zone)) {
 can_reach_zone = true;
 break;
 }
 }
 if (can_reach_zone)
 relevant_actor_indices.push_back(actor_index);
 }
 }
 std::vector<std::vector<DynamicObjectState>> relevant_predicted_frames;
 relevant_predicted_frames.reserve(predicted_frames.size());
 for (const auto& frame : predicted_frames) {
 auto& relevant_frame = relevant_predicted_frames.emplace_back();
 relevant_frame.reserve(relevant_actor_indices.size());
 for (const auto actor_index : relevant_actor_indices) {
 if (actor_index < frame.size())
 relevant_frame.push_back(frame[actor_index]);
 }
 }
 const double clear_distance = safety.ego_radius_m + safety.clearance_m +
 0.0;

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
 const double stop_geometry_clearance_m =
 clear_distance + kStopLineToleranceM + safety.stop_position_tolerance_m +
 std::max(0.0, safety.stop_plane_tracking_tolerance_m);
 // The stop line is infrastructure geometry and must remain invariant under
 // prediction horizon, observation cadence and a participant's current
 // phase. Dynamic occupancy still validates the complete stationary wait
 // and every approach/escape trajectory below. Folding a finite forecast
 // into this geometry makes a short-horizon computer stop closer to traffic
 // than a long-horizon computer, then continuously relocates the control
 // boundary as forecasts change.
 const auto stop_pose_safe = [&](Vec2 point) {
 return safe_from_conflict_geometry(
 point, *selected_zone, stop_geometry_clearance_m);
 };
 while (stop > 0 && !stop_pose_safe(route[stop].position)) {
  --stop;
 }
 // The semantic box describes stable shared-space geometry. Live participant
 // shape and forecast uncertainty remain time indexed in perception and the
 // hold check below; a map must not invent either quantity.
 // Global search vertices are discrete, but a behavior stop line is a
 // continuous world-space pose. Once an upstream vertex is safe and the next
 // one is unsafe, bisect that route segment to keep the stop as close to the
 // crossing as the geometric contract permits. This avoids quantizing
 // visibility and safety to the global grid resolution.
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

 // A conflict zone on the eventual global route is not yet an active
 // behavior constraint. Activate it only inside a route-distance preview
 // that covers both the available forecast and a full delayed stop from the
 // platform limit. This mirrors behavior-module request distances in road
 // stacks and prevents a distant virtual stop plane from constraining an
 // unrelated local corner many metres upstream.
 const double approach_distance_m = route_distance_to_point(
 route, ego_position, continuous_stop,
 safety.stop_position_tolerance_m);
 // Activation follows the speed this behavior will actually authorize. A
 // high-speed motor's physical ceiling is not the route policy and would
 // activate a low-speed crossing tens of metres too early.
 const double speed_mps = std::clamp(
 execution.preferred_speed_mps, 0.0,
 std::max(0.0, execution.max_speed_mps));
 const double delayed_stop_m =
 speed_mps * std::max(0.0, execution.actuator_response_delay_s) +
 (execution.max_decel_mps2 > 0.0
 ? speed_mps * speed_mps / (2.0 * execution.max_decel_mps2)
 : std::numeric_limits<double>::infinity());
 const double activation_distance_m = std::max({
 4.0,
 speed_mps * std::max(0.0, execution.activation_horizon_s),
 delayed_stop_m,
 });
 const bool selected_body_inside =
 distance_to_box(ego_position, *selected_zone) <=
 safety.ego_radius_m;
if (execution.activation_horizon_s > 0.0 &&
 !committed_continuation && !selected_body_inside &&
 approach_distance_m > activation_distance_m + 1e-9) {
 return out;
 }

 // Admission proves the maneuver from the fixed hold pose. The body may be
 // anywhere inside the declared stop-position tolerance when the gate opens;
 // using that measured offset as a new first waypoint makes the first route
 // tangent (and therefore the timed proof) jump with contact/localization
 // noise. The tracking tube below already reserves that bounded offset.
 // Initial admission is proved from the immutable stop pose. If a fresh
 // forecast invalidates an already committed schedule before entry, a
 // replacement must instead start at the measured current state; rebuilding
 // it from the old stop would replay elapsed motion and compare actors at the
 // wrong times.
 std::vector<Vec2> crossing_path{
 committed_continuation ? ego_position : continuous_stop};
 // Continue from the next vertex of the physically closest route segment.
 // Re-inserting that segment's start vertex can point behind the measured
 // body and make admission prove a reverse jog that the forward route
 // follower will never execute.
 const std::size_t first_future_vertex = route.size() > 1
 ? std::min(start + 1, route.size() - 1)
 : start;
 for (std::size_t index = first_future_vertex;
 index <= selected_exit && index < route.size(); ++index) {
 // Grid planners may place a sharp waypoint at the centre of a crossing.
 // Interpreting that raster corner as stop-turn-go parks the robot directly
 // in cross traffic. The semantic conflict envelope is shared traversable
 // space, so collapse its internal raster vertices into one continuous chord
 // to the proved refuge. Corners before the envelope remain explicit and may
 // still use the conservative stop-turn-go contract.
 // The admitted centreline may legally consume its tracking tube. A raster
 // vertex whose tube overlaps the conflict envelope therefore cannot be a
 // stop-turn-go waypoint: stopping there would place some valid tracking
 // states inside traffic even though the mathematical vertex is just
 // outside. Collapse such grid artefacts into the continuous chord and
 // finish all required alignment at the upstream hold pose.
 if (index < selected_exit &&
 distance_to_box(route[index].position, *selected_zone) <
 clear_distance + std::max(0.0, safety.tracking_error_m)) {
 continue;
 }
 if (distance(crossing_path.back(), route[index].position) > 1e-6)
 crossing_path.push_back(route[index].position);
 }

 // Grid search preserves obstacle clearance but may leave short raster
 // corners in open space. Stop-turn-go at each of those vertices needlessly
 // lengthens occupancy of a dynamic conflict corridor. Prune only segments
 // that the current deployable map independently proves traversable; this is
 // the same map-side contract available on a real robot, not simulator
 // world truth.
 std::vector<std::vector<Vec2>> crossing_paths;
 if (path_feasible && crossing_path.size() > 2) {
 std::vector<Vec2> pruned{crossing_path.front()};
 std::size_t current = 0;
 while (current + 1 < crossing_path.size()) {
 std::size_t next = crossing_path.size() - 1;
 while (next > current + 1 &&
 !path_feasible(crossing_path[current], crossing_path[next])) {
 --next;
 }
 pruned.push_back(crossing_path[next]);
 current = next;
 }
 // Static line feasibility makes this chord a valid candidate; it does not
 // prove that the chord has better same-time clearance from moving traffic.
 // Evaluate it alongside the canonical route below instead of replacing the
 // route and silently discarding its potentially safer interaction timing.
 bool differs = pruned.size() != crossing_path.size();
 for (std::size_t index = 0; !differs && index < pruned.size(); ++index) {
 differs = distance(pruned[index], crossing_path[index]) > 1e-9;
 }
 if (differs)
 crossing_paths.push_back(std::move(pruned));
 }
 crossing_paths.push_back(crossing_path);

 out.relevant = true;
 // Controller path progress is not body occupancy: lookahead may advance
 // while the vehicle is laterally displaced. Physical geometry is the only
 // valid source for entering a conflict zone or reaching its stop line.
 out.body_in_conflict =
 distance_to_box(ego_position, *selected_zone) <= safety.ego_radius_m;
 out.no_return_started =
 distance_to_box(ego_position, *selected_zone) <=
 safety.ego_radius_m + safety.clearance_m;
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
 // A forecast is relevant when its swept envelope reaches the conflict region
 // or occupies one of the mapped incoming corridors. The latter matters for
 // a target currently moving away: its constant-velocity forecast may never
 // touch the box, yet it is the observed participant on that approach and its
 // complete timed trajectory is useful evidence. A remote track elsewhere in
 // the map remains unrelated and cannot stand in for an occluded approach.
 out.prediction_evidence_available = !relevant_actor_indices.empty();
 // Gap acceptance also needs an intent guard. A mathematically open window
 // must not release on the first noisy samples of an actor reversal. Only
 // actors whose predicted envelope can reach this zone participate, and each
 // must be moving reliably outwards. Requiring its complete spatial envelope
 // to clear the zone as well would duplicate the time-indexed swept-gap gate
 // below and can deadlock a periodic actor whose endpoint is close to the
 // semantic box even though it will be clear when ego arrives.
 out.relevant_threats_moving_away = out.prediction_evidence_available;
 out.prediction_models_ready = out.prediction_evidence_available;
 if (out.prediction_evidence_available) {
 for (const auto actor_index : relevant_actor_indices) {
 const auto& current = predicted_frames.front()[actor_index];
 if (!current.prediction_ready) {
 out.prediction_models_ready = false;
 out.relevant_threats_moving_away = false;
 continue;
 }
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
 bool feasible_found = false;
 double best_gap = -std::numeric_limits<double>::infinity();
 out.departure = {std::numeric_limits<double>::infinity(), DepartReason::kInvalid};
 const auto consider_trajectory = [&](std::vector<TimedPose> trajectory,
 double speed, bool continuous) {
 DepartureAssessment assessment{
 std::numeric_limits<double>::infinity(), DepartReason::kInvalid};
 // The canonical stop-turn-go path needs the same full-segment map proof as
 // an optional smoothed chord. A global route can become obstructed by the
 // transient local layer after it was generated; checking only smoothing
 // candidates would let behavior admit a trajectory that the executor must
 // reject on its first cycle.
 if (path_feasible) {
 for (std::size_t index = 0; index + 1 < trajectory.size(); ++index) {
 if (distance(trajectory[index].p, trajectory[index + 1].p) > 1e-9 &&
 !path_feasible(trajectory[index].p, trajectory[index + 1].p)) {
 trajectory.clear();
 break;
 }
 }
 }
 if (out.prediction_evidence_available) {
 assessment = assess_crossing_trajectory(
 trajectory, speed, relevant_predicted_frames, frame_dt_s, safety);
 } else if (!trajectory.empty()) {
 const auto observed = observe_approaches(
 *selected_zone, visibility_rays, visibility_stamp_s,
 evaluation_stamp_s, trajectory.back().t_s);
 out.free_space_evidence_available = observed.available;
 out.observed_approaches_clear = observed.clear;
 out.observation_stamp_s = visibility_stamp_s;
 out.observation_coverage = observed.coverage;
 out.unseen_arrival_lower_bound_s = observed.unseen_arrival_lower_bound_s;
 }
 double candidate_gap = std::numeric_limits<double>::infinity();
 if (trajectory.empty()) {
 candidate_gap = -std::numeric_limits<double>::infinity();
 } else if (out.prediction_evidence_available) {
 for (double shift_s = -kCrossingTimeMarginS;
 shift_s <= kCrossingTimeMarginS + 1e-9;
 shift_s += std::max(1e-6, frame_dt_s)) {
 auto shifted = trajectory;
 for (auto& pose : shifted)
 pose.t_s += shift_s;
 candidate_gap = std::min(candidate_gap,
 timed_gap(shifted, relevant_predicted_frames, frame_dt_s,
 safety.ego_radius_m));
 }
 }
 const double candidate_required_gap = safety.clearance_m +
 std::max(0.0, safety.tracking_error_m);
 if (!trajectory.empty()) {
 if (out.prediction_evidence_available) {
 const double horizon_s = static_cast<double>(predicted_frames.size() - 1) *
 std::max(1e-6, frame_dt_s);
 any_covered = any_covered || trajectory.back().t_s <= horizon_s + 1e-9;
 } else {
 any_covered = any_covered || out.observed_approaches_clear;
 }
 if (!feasible_found && candidate_gap > best_gap) {
 best_gap = candidate_gap;
 out.transit_speed_mps = speed;
 out.execution_trajectory = trajectory;
 out.execution_continuous = continuous;
 out.execution_duration_s = trajectory.back().t_s;
 out.execution_min_gap_m = candidate_gap;
 out.execution_required_gap_m = candidate_required_gap;
 }
 }
 if (assessment.reason == DepartReason::kGoNow) {
 if (!feasible_found || candidate_gap > best_gap + 1e-12) {
 best_gap = candidate_gap;
 out.transit_speed_mps = speed;
 out.execution_trajectory = std::move(trajectory);
 out.execution_continuous = continuous;
 out.execution_duration_s = out.execution_trajectory.back().t_s;
 out.execution_min_gap_m = candidate_gap;
 out.execution_required_gap_m = candidate_required_gap;
 }
 feasible_found = true;
 out.departure = assessment;
 }
 if (!feasible_found &&
 (out.departure.reason == DepartReason::kInvalid ||
 (out.departure.reason == DepartReason::kHorizonShort &&
 assessment.reason != DepartReason::kHorizonShort))) {
 out.departure = assessment;
 }
 };

 for (const auto& candidate_path : crossing_paths) {
 for (double speed = preferred_speed;;
 speed = std::min(maximum_speed, speed + kSpeedSearchStepMps)) {
 // The conservative stop-turn-go interpretation always remains available.
 consider_trajectory(crossing_trajectory(
 candidate_path, ego_heading_rad, ego_speed_mps,
 speed, frame_dt_s, execution), speed, false);

 // A continuous interpretation is a separate, independently checked
 // candidate. It is admitted only when every corner has a fitting fillet and
 // the deployable map validates every resulting chord. This is a general
 // chassis/map contract and contains no crossing-actor or scenario truth.
 if (path_feasible && execution.continuous_curvature_tracking) {
 const double nominal_radius_m = speed /
 std::max(1e-9, execution.max_yaw_rate_radps);
 // Differential bases have no non-zero geometric minimum turn radius. A
 // smaller fillet is therefore a valid candidate when its curved portion is
 // slowed to the yaw-rate limit. Search a short deterministic set instead of
 // assuming the full-speed radius has the best dynamic clearance.
 for (const double radius_scale : {1.0, 0.75, 0.5, 0.25}) {
 const double radius_m = std::max(0.10,
 nominal_radius_m * radius_scale);
 const auto smooth = fillet_polyline(candidate_path, radius_m);
 if (smoothed_path_is_static_feasible(smooth, path_feasible)) {
 consider_trajectory(continuous_crossing_trajectory(
 smooth, ego_heading_rad, ego_speed_mps, speed, execution),
 speed, true);
 }
 }
 }
 if (speed >= maximum_speed - 1e-9)
 break;
 }
 }
 if (!any_covered && out.departure.reason != DepartReason::kInvalid)
 out.departure = {std::numeric_limits<double>::infinity(), DepartReason::kHorizonShort};
 return out;
}

CrossingOutput govern_crossing(CrossingState& state, const CrossingInput& input, const std::string& zone_id) {
 const auto output = [&](bool hold_active, bool committed, bool stop_valid) {
 CrossingOutput value;
 value.phase = state.phase;
 value.hold_active = hold_active;
 value.committed = committed;
 value.stop_point = state.stop_point;
 value.stop_direction = state.stop_direction;
 value.stop_valid = stop_valid;
 value.exit_point = state.exit_point;
 value.exit_valid = state.exit_valid;
 value.transit_speed_mps = state.transit_speed_mps;
 value.execution_trajectory = state.execution_trajectory;
 value.execution_continuous = state.execution_continuous;
 value.tracking_corridor_m = state.tracking_corridor_m;
 value.recovery_to_stop = state.recovery_to_stop;
 value.recovery_reverse = state.recovery_reverse;
 value.hold_position_acquired = state.hold_position_acquired;
 value.no_return_started = state.no_return_started;
 value.relocated_hold = state.relocated_hold;
 value.execution_initial_heading_rad = state.execution_initial_heading_rad;
 value.execution_alignment_required = state.execution_alignment_required;
 value.execution_alignment_complete = state.execution_alignment_complete;
 value.execution_elapsed_s = state.execution_elapsed_s;
 return value;
 };
 if (state.phase == CrossingPhase::Committed) {
 const bool alignment_was_complete = state.execution_alignment_complete;
 // The admitted trajectory clock starts after the separate stop-pose
 // alignment stage. Advancing it while the chassis is still rotating at the
 // hold pose consumes the traffic window without executing any of the proved
 // translation and eventually compares the stopped body with the exit pose.
 if (alignment_was_complete)
 state.execution_elapsed_s += std::max(0.0, input.dt_s);
 // Permission is re-evaluated until the body physically enters the conflict
 // envelope. Crossing the upstream stop line alone is not a no-return event:
 // the validated braking tail may still stop before traffic, after which a
 // changed forecast must be allowed to revoke the stale commitment.
 if (input.body_in_conflict || input.no_return_started)
 state.no_return_started = true;
 if (input.committed_contingency_escape)
 state.contingency_escape_started = true;
 if (input.body_cleared) {
 state.phase = CrossingPhase::Cleared;
 return output(false, false, false);
 }
 // A minimum-risk command may legitimately leave the originally admitted
 // tube when a fresh actor forecast makes both following that tube and
 // braking in place less safe. No-return forbids reversing through traffic
 // while the body remains in the approach envelope; it does not turn the old
 // timed schedule into permanent authority after the contingency has moved
 // the body to an upstream or lateral refuge. Re-enter through the ordinary
 // hold/admission contract from the measured state.
 if (state.no_return_started && state.contingency_escape_started &&
 !input.body_in_conflict && !input.no_return_started &&
 input.stopped_before_conflict && !input.committed_execution_blocked) {
 state.phase = CrossingPhase::Holding;
 state.clear_time_s = 0.0;
 state.clear_observation_samples = 0;
 state.last_clear_evidence_stamp_s =
 -std::numeric_limits<double>::infinity();
 state.stop_point = input.ego_position;
 const Vec2 refuge_to_exit{
 state.exit_point.x - input.ego_position.x,
 state.exit_point.y - input.ego_position.y,
 };
 const double refuge_direction_norm = std::hypot(
 refuge_to_exit.x, refuge_to_exit.y);
 if (refuge_direction_norm > 1e-9) {
 state.stop_direction = {
 refuge_to_exit.x / refuge_direction_norm,
 refuge_to_exit.y / refuge_direction_norm,
 };
 }
 state.stop_valid = true;
 state.execution_trajectory.clear();
 state.execution_continuous = false;
 state.tracking_corridor_m = 0.0;
 state.recovery_to_stop = false;
 state.recovery_reverse = false;
 state.hold_position_acquired = true;
 state.no_return_started = false;
 state.contingency_escape_started = false;
 state.relocated_hold = true;
 state.execution_alignment_required = false;
 state.execution_alignment_complete = true;
 state.execution_elapsed_s = 0.0;
 return output(true, false, true);
 }
 // A newly observed actor phase may invalidate the old schedule while a
 // different actuator-feasible continuation from the measured ego state is
 // still safe. Re-latch that complete timed proof instead of first driving
 // back to the historical stop pose. The local validator requested this
 // update by rejecting the old continuation, and it will independently
 // validate the replacement on the next control cycle.
 if (!state.no_return_started && input.committed_execution_blocked &&
 input.safe_to_commit && !input.execution_trajectory.empty() &&
 !input.execution_alignment_required) {
 state.execution_trajectory = input.execution_trajectory;
 state.execution_continuous = input.execution_continuous;
 state.tracking_corridor_m = std::max(0.0, input.tracking_corridor_m);
 state.transit_speed_mps = input.transit_speed_mps;
 state.execution_initial_heading_rad = input.execution_initial_heading_rad;
 state.execution_alignment_required = input.execution_alignment_required;
 state.execution_alignment_complete = !input.execution_alignment_required ||
 input.execution_alignment_complete;
 state.execution_elapsed_s = 0.0;
 return output(false, true, false);
 }
 if (!state.no_return_started &&
 input.committed_execution_blocked &&
 !input.body_in_conflict &&
 input.stopped_before_conflict) {
 state.phase = CrossingPhase::Holding;
 state.clear_time_s = 0.0;
 state.clear_observation_samples = 0;
 state.last_clear_evidence_stamp_s =
 -std::numeric_limits<double>::infinity();
 // Keep the original route-derived hold pose. It was proved safe over the
 // forecast before admission; replacing it with the arbitrary point where a
 // changed forecast happened to finish braking can strand the robot inside
 // the traffic approach. The local planner may retreat to this latched pose.
 state.stop_valid = true;
 state.execution_trajectory.clear();
 state.execution_continuous = false;
 state.tracking_corridor_m = 0.0;
 state.recovery_to_stop = true;
 state.recovery_reverse = input.stop_longitudinal_m > kStopLineToleranceM;
 state.hold_position_acquired = false;
 state.no_return_started = false;
 state.contingency_escape_started = false;
 state.relocated_hold = false;
 state.execution_alignment_required = false;
 state.execution_alignment_complete = true;
 state.execution_elapsed_s = 0.0;
 return output(true, false, true);
 }
 if (input.execution_alignment_complete)
 state.execution_alignment_complete = true;
 return output(false, true, false);
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
 state.execution_trajectory = input.execution_trajectory;
 state.execution_continuous = input.execution_continuous;
 state.tracking_corridor_m = std::max(0.0, input.tracking_corridor_m);
 state.recovery_to_stop = false;
 state.recovery_reverse = false;
 state.hold_position_acquired = false;
 state.no_return_started = input.body_in_conflict;
 state.contingency_escape_started = false;
 state.relocated_hold = false;
 state.execution_initial_heading_rad = input.execution_initial_heading_rad;
 state.execution_alignment_required = input.execution_alignment_required;
 state.execution_alignment_complete = !input.execution_alignment_required;
 state.execution_elapsed_s = 0.0;
 return output(false, true, false);
 }
 state.phase = CrossingPhase::Holding;
 if (input.position_settled)
 state.hold_position_acquired = true;
 else if (!input.position_ready)
 state.hold_position_acquired = false;
 // Retain reverse until the measured body is clearly upstream. A stateless
 // sign check at the plane made centimetre-scale caster/contact motion flip
 // the recovery heading every control cycle and prevented pose convergence.
 if (state.recovery_to_stop && state.recovery_reverse &&
 input.stop_longitudinal_m < -kStopLineToleranceM) {
 state.recovery_reverse = false;
 }
 if (state.recovery_to_stop && input.ready_to_commit)
 state.recovery_to_stop = false;
 if (!state.recovery_to_stop)
 state.recovery_reverse = false;
 state.no_return_started = false;
 if (input.ready_to_commit && input.safe_to_commit) {
 const bool distinct_evidence = !std::isfinite(input.evidence_stamp_s) ||
 !std::isfinite(state.last_clear_evidence_stamp_s) ||
 input.evidence_stamp_s > state.last_clear_evidence_stamp_s + 1e-9;
 if (distinct_evidence) {
 state.clear_time_s += std::max(0.0, input.dt_s);
 ++state.clear_observation_samples;
 state.last_clear_evidence_stamp_s = input.evidence_stamp_s;
 }
 if (state.clear_observation_samples >= kCrossingCommitConfirmSamples) {
   state.phase = CrossingPhase::Committed;
   state.clear_time_s = 0.0;
 state.clear_observation_samples = 0;
 state.execution_trajectory = input.execution_trajectory;
 state.execution_continuous = input.execution_continuous;
 state.tracking_corridor_m = std::max(0.0, input.tracking_corridor_m);
 state.recovery_to_stop = false;
 state.recovery_reverse = false;
 state.hold_position_acquired = false;
 state.contingency_escape_started = false;
 state.relocated_hold = false;
 state.execution_initial_heading_rad = input.execution_initial_heading_rad;
 state.execution_alignment_required = input.execution_alignment_required;
 state.execution_alignment_complete = !input.execution_alignment_required;
 state.execution_elapsed_s = 0.0;
 return output(false, true, false);
  }
 } else {
  state.clear_time_s = 0.0;
 state.clear_observation_samples = 0;
 state.last_clear_evidence_stamp_s = -std::numeric_limits<double>::infinity();
 }
 return output(true, false, state.stop_valid);
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
 const CrossingExecutionCaps& execution, double decision_dt_s,
 const std::vector<ObservedRay2D>& visibility_rays,
 double visibility_stamp_s, double evaluation_stamp_s,
 bool committed_execution_blocked,
 bool committed_contingency_escape,
 CrossingPathFeasibility path_feasible) {
 std::vector<ConflictZone> geometry;
 geometry.reserve(zones.size());
 for (const auto& zone : zones)
 geometry.push_back(zone.geometry);

 CrossingBehaviorDecision decision;
 decision.has_active_zone = !zones.empty();
 decision.assessment = evaluate_crossing(
 geometry, route, path_index, ego_position, ego_heading_rad, ego_speed_mps,
 predicted_frames, frame_dt_s, safety, execution,
 visibility_rays, visibility_stamp_s, evaluation_stamp_s,
 state_.phase == CrossingPhase::Committed || state_.relocated_hold,
 std::move(path_feasible));

 const auto has_control_authority = [&](const std::string& id) {
 const auto match = std::find_if(zones.begin(), zones.end(), [&](const auto& zone) {
 return zone.geometry.id == id;
 });
 return match != zones.end() && match->authority == CrossingAuthority::Control;
 };

 // A global replan is allowed to replace the geometric route, but it is not
 // an implicit cancellation of an already activated traffic behavior. Grid
 // search can transiently return an empty route, or a route whose first
 // segment starts beyond the semantic envelope, for one control cycle. If
 // that erased Holding, the following cycle would derive a different stop
 // tangent and rotate a stationary robot just as a safe window opened.
 //
 // Keep the latched contract while its controlled zone still exists in the
 // deployable map. With no fresh route assessment the gate is deliberately
 // closed: this fallback carries geometry and state continuity only, never
 // positive evidence for admission. Mission reset calls reset(), while a
 // removed/disabled zone follows the ordinary irrelevant path below.
 if (!decision.assessment.relevant &&
 (state_.phase == CrossingPhase::Holding ||
 state_.phase == CrossingPhase::Committed) &&
 has_control_authority(state_.zone_id)) {
 const auto active_zone = std::find_if(
 zones.begin(), zones.end(), [&](const auto& zone) {
 return zone.geometry.id == state_.zone_id;
 });
 const auto& geometry = active_zone->geometry;
 decision.assessment.relevant = true;
 decision.assessment.zone_id = state_.zone_id;
 decision.assessment.stop_point = state_.stop_point;
 decision.assessment.stop_direction = state_.stop_direction;
 decision.assessment.exit_point = state_.exit_point;
 decision.assessment.exit_direction = state_.exit_direction;
 decision.assessment.stop_distance_m = distance(ego_position, state_.stop_point);
 decision.assessment.transit_speed_mps = state_.transit_speed_mps;
 decision.assessment.body_in_conflict =
 distance_to_box(ego_position, geometry) <= safety.ego_radius_m;
 decision.assessment.no_return_started =
 distance_to_box(ego_position, geometry) <=
 safety.ego_radius_m + safety.clearance_m;
 if (state_.exit_valid) {
 const Vec2 exit_delta{
 ego_position.x - state_.exit_point.x,
 ego_position.y - state_.exit_point.y,
 };
 const double longitudinal =
 exit_delta.x * state_.exit_direction.x +
 exit_delta.y * state_.exit_direction.y;
 const double lateral =
 -exit_delta.x * state_.exit_direction.y +
 exit_delta.y * state_.exit_direction.x;
 const double corridor_half_width = safety.ego_radius_m +
 safety.clearance_m + safety.stop_position_tolerance_m;
 decision.assessment.body_cleared = longitudinal >= -0.10 &&
 std::abs(lateral) <= corridor_half_width;
 }
 }

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
 input.no_return_started = decision.assessment.no_return_started;
 input.body_cleared = decision.assessment.body_cleared;
 // Once Holding starts, admission and braking share the immutable stop pose
 // that was proven safe. Fresh predictions may change the release decision,
 // but they cannot move the virtual wall underneath a stopping vehicle.
 const Vec2 ready_stop = state_.stop_valid ? state_.stop_point : decision.assessment.stop_point;
 const Vec2 ready_direction = state_.stop_valid ? state_.stop_direction : decision.assessment.stop_direction;
 decision.readiness = assess_crossing_readiness(
 ego_position, ego_speed_mps, ego_heading_rad, ready_stop, ready_direction, safety);
 input.ready_to_commit = decision.readiness.ready();
 input.stop_longitudinal_m = decision.readiness.stop_longitudinal_m;
 input.position_ready = decision.readiness.position_ready;
 input.position_settled = decision.readiness.position_settled;
 input.safe_to_commit = decision.assessment.departure.reason == DepartReason::kGoNow &&
 decision.assessment.prediction_evidence_available &&
 decision.assessment.prediction_models_ready;
 decision.ready_to_commit = input.ready_to_commit;
 decision.safe_to_commit = input.safe_to_commit;
 input.stopped_before_conflict = !decision.assessment.body_in_conflict &&
 std::abs(ego_speed_mps) <= 0.10;
 input.committed_execution_blocked = committed_execution_blocked;
 input.committed_contingency_escape = committed_contingency_escape;
 input.ego_position = ego_position;
 input.stop_point = decision.assessment.stop_point;
 input.stop_direction = decision.assessment.stop_direction;
 input.stop_valid = true;
 input.exit_point = decision.assessment.exit_point;
 input.exit_direction = decision.assessment.exit_direction;
 input.exit_valid = true;
 input.transit_speed_mps = decision.assessment.transit_speed_mps;
 input.execution_trajectory = decision.assessment.execution_trajectory;
 input.execution_continuous = decision.assessment.execution_continuous;
 const auto& alignment_trajectory = state_.phase == CrossingPhase::Committed &&
 !committed_execution_blocked
 ? state_.execution_trajectory
 : input.execution_trajectory;
 input.execution_initial_heading_rad = initial_trajectory_heading(
 alignment_trajectory, ego_heading_rad);
 const double initial_heading_error = std::abs(crossing_wrap_angle(
 input.execution_initial_heading_rad - ego_heading_rad));
 input.execution_alignment_complete =
 initial_heading_error <= kCrossingInitialHeadingToleranceRad;
 input.execution_alignment_required = !input.execution_alignment_complete;
 if (std::isfinite(decision.assessment.execution_min_gap_m) &&
 std::isfinite(decision.assessment.execution_required_gap_m)) {
 // Admission already reserved tracking_error_m inside its required center-
 // line gap. The executor may consume that reserve plus any additional spare;
 // the resulting tube is exactly min_gap - physical clearance. Subtracting
 // the reserve a second time reduced valid execution to a few millimetres.
 input.tracking_corridor_m = std::max(
 0.0, decision.assessment.execution_min_gap_m -
 safety.clearance_m);
 }
 input.dt_s = decision_dt_s;
 input.evidence_stamp_s = visibility_stamp_s;
 const auto previous_phase = state_.phase;
 decision.control = govern_crossing(state_, input, decision.assessment.zone_id);
 if (state_.stop_valid) {
 // The state machine latches the virtual wall. Distance must be evaluated to
 // that same point even when a new prediction would choose an earlier stop;
 // mixing a fresh distance with the latched pose can produce a false zero.
 decision.assessment.stop_point = state_.stop_point;
 decision.assessment.stop_direction = state_.stop_direction;
 decision.assessment.stop_distance_m = state_.relocated_hold
 ? distance(ego_position, state_.stop_point)
 : route_distance_to_point(
 route, ego_position, state_.stop_point,
 safety.stop_position_tolerance_m);
 }
 if (decision.control.phase == CrossingPhase::Cleared)
 cleared_zone_id_ = state_.zone_id;
 decision.phase_changed = decision.control.phase != previous_phase;
 return decision;
}

} // namespace sim
