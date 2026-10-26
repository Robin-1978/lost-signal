#include "robot/autonomy/planning/dwa_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <vector>

#include "robot/autonomy/decision/arrival_confidence.hpp"

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
 std::size_t segment{0};
 double fraction{0.0};
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
 best.segment = i;
 best.fraction = projection;
    }
    accumulated_m += length_m;
  }
  return best;
}

PathProjection project_to_route(core::Vec2 point, const Route& route) {
 PathProjection best;
 if (route.points.empty()) return best;
 if (route.points.size() == 1) {
 best.distance_m = std::hypot(point.x - route.points.front().x_m,
 point.y - route.points.front().y_m);
 return best;
 }
 double accumulated_m = 0.0;
 for (std::size_t i = 0; i + 1 < route.points.size(); ++i) {
 const auto& a = route.points[i];
 const auto& b = route.points[i + 1];
 const double dx = b.x_m - a.x_m;
 const double dy = b.y_m - a.y_m;
 const double length_m = std::hypot(dx, dy);
 if (length_m < 1e-9) continue;
 const double fraction = std::clamp(
 ((point.x - a.x_m) * dx + (point.y - a.y_m) * dy) /
 (length_m * length_m),
 0.0, 1.0);
 const double distance_m = std::hypot(
 point.x - (a.x_m + fraction * dx),
 point.y - (a.y_m + fraction * dy));
 if (distance_m < best.distance_m) {
 best.distance_m = distance_m;
 best.along_m = accumulated_m + fraction * length_m;
 best.tangent_rad = std::atan2(dy, dx);
 best.segment = i;
 best.fraction = fraction;
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

struct RouteLocation {
 std::size_t segment{0};
 double fraction{0.0};
};

int segment_gear(const Route& route, std::size_t segment) {
 if (route.points.empty()) return 0;
 const std::size_t next = std::min(segment + 1, route.points.size() - 1);
 const int arriving_gear = route.points[next].gear;
 return arriving_gear != 0 ? arriving_gear : route.points[segment].gear;
}

PathProjection project_to_route_window(core::Vec2 point, const Route& route,
 RouteLocation cursor, double forward_search_m) {
 PathProjection best;
 if (route.points.size() < 2) return project_to_route(point, route);
 cursor.segment = std::min(cursor.segment, route.points.size() - 2);
 cursor.fraction = std::clamp(cursor.fraction, 0.0, 1.0);
 double forward_m = 0.0;
 for (std::size_t segment = cursor.segment;
 segment + 1 < route.points.size(); ++segment) {
 const auto& a = route.points[segment];
 const auto& b = route.points[segment + 1];
 const double dx = b.x_m - a.x_m;
 const double dy = b.y_m - a.y_m;
 const double length_m = std::hypot(dx, dy);
 if (length_m < 1e-9) continue;
 const double minimum_fraction =
 segment == cursor.segment ? cursor.fraction : 0.0;
 const double search_m = std::max(0.0, forward_search_m);
 if (forward_m > search_m + 1e-9) break;
 const double maximum_fraction = std::min(
 1.0, minimum_fraction +
 std::max(0.0, search_m - forward_m) / length_m);
 const double fraction = std::clamp(
 ((point.x - a.x_m) * dx + (point.y - a.y_m) * dy) /
 (length_m * length_m), minimum_fraction, maximum_fraction);
 const double distance_m = std::hypot(
 point.x - (a.x_m + fraction * dx),
 point.y - (a.y_m + fraction * dy));
 if (distance_m < best.distance_m) {
 best.distance_m = distance_m;
 best.along_m = forward_m +
 (fraction - minimum_fraction) * length_m;
 best.tangent_rad = std::atan2(dy, dx);
 best.segment = segment;
 best.fraction = fraction;
 }
 forward_m += (1.0 - minimum_fraction) * length_m;
 }
 return best;
}

// Route lookahead: closest route point, then advance lookahead_m along it.
// Preserve the planner's body heading and gear annotation at the selected
// segment so a Hybrid-A* reverse decision is not lost in local sampling.
RouteTarget route_goal(const Route& route, const PathProjection& projection,
 double lookahead_m) {
 if (route.points.size() == 1) {
 return {{route.points.front().x_m, route.points.front().y_m},
 route.points.front().yaw_rad, route.points.front().gear};
 }
 std::size_t segment = std::min(
 projection.segment, route.points.size() - 2);
 double fraction = std::clamp(projection.fraction, 0.0, 1.0);
 const int active_gear = segment_gear(route, segment);
 double remaining_m = std::max(0.0, lookahead_m);
 while (segment + 1 < route.points.size()) {
 const auto& a = route.points[segment];
 const auto& b = route.points[segment + 1];
 const double length_m = std::hypot(b.x_m - a.x_m, b.y_m - a.y_m);
 const double available_m = length_m * (1.0 - fraction);
 if (length_m > 1e-9 && remaining_m <= available_m) {
 const double target_fraction = fraction + remaining_m / length_m;
 const double yaw_delta = core::wrap_angle(b.yaw_rad - a.yaw_rad);
 return {{a.x_m + target_fraction * (b.x_m - a.x_m),
 a.y_m + target_fraction * (b.y_m - a.y_m)},
 core::wrap_angle(a.yaw_rad + target_fraction * yaw_delta),
 active_gear};
 }
 if (segment + 2 < route.points.size() &&
 segment_gear(route, segment + 1) != active_gear) {
 return {{b.x_m, b.y_m}, b.yaw_rad, active_gear};
 }
 remaining_m -= available_m;
 ++segment;
 fraction = 0.0;
 }
 const auto& last = route.points.back();
 return {{last.x_m, last.y_m}, last.yaw_rad, last.gear};
}

RouteLocation move_on_route(const Route& route, RouteLocation location,
 double distance_m) {
 if (route.points.size() < 2) return location;
 location.segment = std::min(location.segment, route.points.size() - 2);
 location.fraction = std::clamp(location.fraction, 0.0, 1.0);
 double remaining_m = std::abs(distance_m);
 if (distance_m >= 0.0) {
 while (location.segment + 1 < route.points.size()) {
 const auto& a = route.points[location.segment];
 const auto& b = route.points[location.segment + 1];
 const double length_m = std::hypot(b.x_m - a.x_m, b.y_m - a.y_m);
 const double available_m = length_m * (1.0 - location.fraction);
 if (length_m > 1e-9 && remaining_m <= available_m) {
 location.fraction += remaining_m / length_m;
 return location;
 }
 remaining_m -= available_m;
 if (location.segment + 2 >= route.points.size()) {
 location.fraction = 1.0;
 return location;
 }
 ++location.segment;
 location.fraction = 0.0;
 }
 return location;
 }
 while (true) {
 const auto& a = route.points[location.segment];
 const auto& b = route.points[location.segment + 1];
 const double length_m = std::hypot(b.x_m - a.x_m, b.y_m - a.y_m);
 const double available_m = length_m * location.fraction;
 if (length_m > 1e-9 && remaining_m <= available_m) {
 location.fraction -= remaining_m / length_m;
 return location;
 }
 remaining_m -= available_m;
 if (location.segment == 0) {
 location.fraction = 0.0;
 return location;
 }
 --location.segment;
 location.fraction = 1.0;
 }
}

core::Vec2 position_on_route(const Route& route, RouteLocation location) {
 const auto& a = route.points[location.segment];
 const auto& b = route.points[location.segment + 1];
 return {a.x_m + location.fraction * (b.x_m - a.x_m),
 a.y_m + location.fraction * (b.y_m - a.y_m)};
}

std::vector<core::Vec2> local_route_reference(
 const Route& route, const PathProjection& projection,
 double backward_m, double forward_m) {
 std::vector<core::Vec2> reference;
 if (route.points.empty()) return reference;
 if (route.points.size() == 1) {
 reference.push_back({route.points.front().x_m, route.points.front().y_m});
 return reference;
 }
 const RouteLocation current{projection.segment, projection.fraction};
 const RouteLocation first = move_on_route(
 route, current, -std::max(0.0, backward_m));
 const RouteLocation last = move_on_route(
 route, current, std::max(0.0, forward_m));
 reference.push_back(position_on_route(route, first));
 for (std::size_t point = first.segment + 1;
 point <= last.segment && point < route.points.size(); ++point) {
 reference.push_back({route.points[point].x_m, route.points[point].y_m});
 }
 const core::Vec2 last_position = position_on_route(route, last);
 if (std::hypot(reference.back().x - last_position.x,
 reference.back().y - last_position.y) > 1e-9) {
 reference.push_back(last_position);
 }
 return reference;
}

}  // namespace

DwaPlanner::DwaPlanner(DwaPlannerConfig config) : config_(config) {}

void DwaPlanner::reset() {
 reset_tracking();
 terminal_position_latched_ = false;
 terminal_identity_valid_ = false;
 terminal_identity_ = RoutePoint{};
}

void DwaPlanner::reset_tracking() {
 previous_w_ = 0.0;
 previous_lateral_ = 0.0;
 route_data_ = nullptr;
 route_size_ = 0;
 route_segment_ = 0;
 route_fraction_ = 0.0;
 route_pose_valid_ = false;
 cusp_braking_armed_ = false;
 cusp_segment_ = 0;
 terminal_mismatch_settle_count_ = 0;
}

LocalPlan DwaPlanner::plan(const EstimatedState& est,
                           const PerceptionResult& perception,
 const PredictionBatch& prediction,
                           const LocalMap& map, const Route& route,
                           const platform::ChassisLimits& limits) {
  LocalPlan out;
  if (!est.valid || !perception.scan_ok || !map.valid) return out;
 if (!route.valid || route.points.size() < 2) return out;
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
 lateral_samples = command_samples(
 lateral_window, std::max(1e-3, config_.lateral_sample_step_mps),
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
 const double route_tracking_tube_m =
 config_.route_progress_max_distance_m > 0.0
 ? config_.route_progress_max_distance_m
 : std::max(
 map.resolution_m,
 std::max(0.0, limits.footprint_radius_m) +
 std::max(0.0, config_.safety_margin_m) +
 std::max(0.0, config_.soft_inflation_radius_m) +
 std::max(0.0, map.resolution_m));

 const bool route_changed = route.points.data() != route_data_ ||
 route.points.size() != route_size_;
 if (route_changed) {
 route_data_ = route.points.data();
 route_size_ = route.points.size();
 route_segment_ = 0;
 route_fraction_ = 0.0;
 route_pose_valid_ = false;
 cusp_braking_armed_ = false;
 cusp_segment_ = 0;
 terminal_mismatch_settle_count_ = 0;
 }
 const double measured_displacement_m = route_pose_valid_
 ? std::hypot(est.x_m - route_pose_x_m_, est.y_m - route_pose_y_m_)
 : config_.route_progress_search_m;
 const double progress_search_m = std::min(
 std::max(0.0, config_.route_progress_search_m),
 std::max(0.0, config_.route_progress_distance_scale) *
 measured_displacement_m);
 PathProjection route_projection = project_to_route_window(
 {est.x_m, est.y_m}, route,
 {route_segment_, route_fraction_}, progress_search_m);
 if (!std::isfinite(route_projection.distance_m) ||
 route_projection.distance_m > config_.route_reacquire_distance_m) {
 route_projection = project_to_route({est.x_m, est.y_m}, route);
 }
 if (route_changed || route_projection.distance_m <=
 route_tracking_tube_m) {
 route_segment_ = route_projection.segment;
 route_fraction_ = route_projection.fraction;
 } else {
 route_projection.segment = route_segment_;
 route_projection.fraction = route_fraction_;
 const auto& a = route.points[route_segment_];
 const auto& b = route.points[route_segment_ + 1];
 const double x = a.x_m + route_fraction_ * (b.x_m - a.x_m);
 const double y = a.y_m + route_fraction_ * (b.y_m - a.y_m);
 route_projection.distance_m = std::hypot(est.x_m - x, est.y_m - y);
 route_projection.tangent_rad = std::atan2(b.y_m - a.y_m, b.x_m - a.x_m);
 }
 route_pose_valid_ = true;
 route_pose_x_m_ = est.x_m;
 route_pose_y_m_ = est.y_m;
 const bool has_next_segment = route_segment_ + 2 < route.points.size();
 const bool gear_change = has_next_segment &&
 segment_gear(route, route_segment_) !=
 segment_gear(route, route_segment_ + 1);
 const auto& segment_end = route.points[route_segment_ + 1];
 const double endpoint_distance_m = std::hypot(
 est.x_m - segment_end.x_m, est.y_m - segment_end.y_m);
 if (!gear_change || cusp_segment_ != route_segment_) {
 cusp_braking_armed_ = false;
 cusp_segment_ = route_segment_;
 }
 const bool braking_was_armed = cusp_braking_armed_;
 const double measured_speed_mps = std::abs(est.forward_mps);
 const double response_s = std::max(0.0, config_.response_delay_s);
 const double stopping_distance_m = decel > 0.0
 ? measured_speed_mps * response_s +
 measured_speed_mps * measured_speed_mps / (2.0 * decel)
 : std::numeric_limits<double>::infinity();
 const double cusp_capture_m = std::max(0.0, limits.footprint_radius_m);
 if (gear_change &&
 endpoint_distance_m <= cusp_capture_m + stopping_distance_m) {
 cusp_braking_armed_ = true;
 }
 // A physical car stops with tracking and actuator error around a cusp; it
 // does not land on an exact lattice fraction. The first decision inside the
 // braking envelope keeps the current gear and requests braking. Only a
 // later decision may shift, once measured speed has settled and the base
 // reference is inside its chassis-derived capture region.
 const bool settled_for_gear_change =
 std::abs(est.forward_mps) <= decel * dt + 1e-9;
 const bool cusp_reached = gear_change && braking_was_armed &&
 cusp_braking_armed_ && settled_for_gear_change &&
 endpoint_distance_m <= cusp_capture_m;
 out.gear_change_pending = gear_change;
 out.gear_change_braking_armed = cusp_braking_armed_;
 out.gear_change_endpoint_distance_m = endpoint_distance_m;
 const bool ordinary_segment_complete = !gear_change &&
 route_fraction_ >= 1.0 - 1e-9 &&
 route_projection.distance_m <= route_tracking_tube_m;
 if (has_next_segment &&
 (cusp_reached || ordinary_segment_complete)) {
 ++route_segment_;
 route_fraction_ = 0.0;
 route_projection.segment = route_segment_;
 route_projection.fraction = route_fraction_;
 cusp_braking_armed_ = false;
 cusp_segment_ = route_segment_;
 }
 const RouteTarget target =
 route_goal(route, route_projection, config_.lookahead_m);
 out.route_segment = route_segment_;
 out.requested_gear = target.gear;
 out.route_error_m = route_projection.distance_m;
 const core::Vec2 goal = target.position;
 // Position comes first at the terminal point. Switching to final yaw at the
 // lookahead distance made a differential base align early and stop beside
 // the goal with a persistent lateral error. Use the small arrival region as
 // the switch: approach the point by bearing, then settle the requested yaw.
 const double goal_distance = std::hypot(goal.x - est.x_m, goal.y - est.y_m);
 const double goal_bearing = std::atan2(goal.y - est.y_m, goal.x - est.x_m);
 const bool car_like = limits.max_curvature_inv_m > 0.0;
 const bool route_tracking_valid = route_projection.distance_m <=
 route_tracking_tube_m;
 const auto& terminal = route.points.back();
 const bool terminal_changed = !terminal_identity_valid_ ||
 terminal.x_m != terminal_identity_.x_m ||
 terminal.y_m != terminal_identity_.y_m ||
 terminal.yaw_rad != terminal_identity_.yaw_rad ||
 terminal.gear != terminal_identity_.gear;
 if (terminal_changed) {
 terminal_identity_valid_ = true;
 terminal_identity_ = terminal;
 terminal_position_latched_ = false;
 }
 const double terminal_distance_m = std::hypot(
 terminal.x_m - est.x_m, terminal.y_m - est.y_m);
 const double terminal_heading_error_rad = std::abs(core::wrap_angle(
 terminal.yaw_rad - est.yaw_rad));
 const double terminal_position_tolerance_m = confidence_bounded_tolerance(
 config_.terminal_position_tolerance_m,
 position_confidence_radius(est.position_sigma_m,
 config_.terminal_confidence_probability));
 const double terminal_heading_tolerance_rad = confidence_bounded_tolerance(
 config_.terminal_heading_tolerance_rad,
 heading_confidence_radius(est.heading_sigma_rad,
 config_.terminal_confidence_probability));
 const double terminal_position_reset_tolerance_m =
 std::max(0.0, config_.terminal_position_tolerance_m);
 if (terminal_position_latched_) {
 if (terminal_distance_m > terminal_position_reset_tolerance_m) {
 terminal_position_latched_ = false;
 }
 } else if (terminal_distance_m <= terminal_position_tolerance_m) {
 terminal_position_latched_ = true;
 }
 // The latch selects terminal-heading control through small estimate motion,
 // but it must never suppress translation after the current confidence
 // region leaves the arrival envelope. GoalArrivalPolicy evaluates that same
 // confidence-bounded position contract; keeping motion gated by the stale
 // latch can otherwise deadlock just outside a provable arrival.
 const bool terminal_heading_mode = terminal_position_latched_;
 const bool terminal_position_reached =
 terminal_distance_m <= terminal_position_tolerance_m;
 const auto& tracked_a = route.points[route_segment_];
 const auto& tracked_b = route.points[route_segment_ + 1];
 const double tracked_remaining_m = (1.0 - route_fraction_) * std::hypot(
 tracked_b.x_m - tracked_a.x_m, tracked_b.y_m - tracked_a.y_m);
 const bool route_terminal_consumed =
 route_segment_ + 2 >= route.points.size() &&
 tracked_remaining_m <=
 std::max(0.0, config_.terminal_position_tolerance_m);
 const bool stopped_terminal_mismatch = car_like &&
 terminal_distance_m <=
 std::max(0.0, config_.terminal_position_tolerance_m) &&
 goal_distance <=
 std::max(0.0, config_.terminal_position_tolerance_m) &&
 terminal_heading_error_rad >
 terminal_heading_tolerance_rad &&
 std::abs(est.forward_mps) <= decel * dt + 1e-9;
 terminal_mismatch_settle_count_ = stopped_terminal_mismatch
 ? terminal_mismatch_settle_count_ + 1 : 0;
 out.route_replan_requested = stopped_terminal_mismatch &&
 (route_terminal_consumed || terminal_mismatch_settle_count_ >=
 std::max(1, config_.terminal_replan_settle_cycles));
 double car_like_tracking_heading = target.yaw_rad;
 if (car_like && std::isfinite(target.yaw_rad)) {
 // A Hybrid-A* point carries the body heading that made its bicycle
 // primitive feasible. Track that heading and add bounded cross-track
 // feedback in the direction of travel. A bearing directly to the final
 // point is singular as its distance approaches zero: centimetres of
 // lateral error can otherwise rotate the requested heading by 90 degrees
 // and make an Ackermann base orbit a correct terminal pose.
 const double travel_yaw = core::wrap_angle(
 target.yaw_rad + (target.gear < 0 ? core::kPi : 0.0));
 const double lateral_error_m =
 -(est.x_m - goal.x) * std::sin(travel_yaw) +
 (est.y_m - goal.y) * std::cos(travel_yaw);
 const double correction_distance_m = std::max(
 std::max(1e-6, config_.lookahead_m),
 std::max(0.0, limits.footprint_radius_m));
 car_like_tracking_heading = core::wrap_angle(
 target.yaw_rad + std::atan2(-lateral_error_m,
 correction_distance_m));
 }
 const double heading_target = car_like
 ? car_like_tracking_heading
 : !terminal_heading_mode
 ? goal_bearing
 : std::isfinite(target.yaw_rad)
 ? target.yaw_rad
 : goal_bearing;
  const double goal_directed_yaw_rate = core::clamp(
      core::wrap_angle(heading_target - est.yaw_rad) /
          std::max(dt, predict_T),
      -w_max, w_max);
 double tracking_speed_limit_mps = v_max;
 if (car_like && route_tracking_valid) {
 // Keep lateral acceleration inside the declared chassis envelope. The
 // chord formula is the constant-curvature connector between the current
 // body pose and the route lookahead pose; reverse uses signed travel.
 if (goal_distance > 1e-6 && config_.lateral_accel_mps2 > 0.0) {
 const double signed_distance = target.gear < 0
 ? -goal_distance : goal_distance;
 const double target_curvature = 2.0 * std::sin(
 0.5 * core::wrap_angle(heading_target - est.yaw_rad)) /
 signed_distance;
 if (std::abs(target_curvature) > 1e-9) {
 tracking_speed_limit_mps = std::min(
 tracking_speed_limit_mps,
 std::sqrt(config_.lateral_accel_mps2 /
 std::abs(target_curvature)));
 }
 }
 // Cross-track reserve is a stopping-distance budget. Account for actuator
 // response delay, then solve d = v*t_response + v^2/(2*a) for v. This
 // slows before the route tube is exhausted instead of tuning a map-specific
 // corner speed.
 if (decel > 0.0) {
 const double remaining_tube_m = std::max(
 0.0, route_tracking_tube_m -
 route_projection.distance_m);
 const double response_s = std::max(0.0, config_.response_delay_s);
 const double delay_speed_mps =
 -decel * response_s + std::sqrt(
 decel * decel * response_s * response_s +
 2.0 * decel * remaining_tube_m);
 tracking_speed_limit_mps = std::min(
 tracking_speed_limit_mps, std::max(0.0, delay_speed_mps));
 }
 // Once a cusp has armed braking, apply the same response-delay stopping
 // equation to the remaining distance. This converges to zero at the cusp
 // and still respects the one-cycle reachable braking floor below.
 if (gear_change && cusp_braking_armed_ && decel > 0.0) {
 const double cusp_speed_mps =
 -decel * response_s + std::sqrt(
 decel * decel * response_s * response_s +
 2.0 * decel * endpoint_distance_m);
 tracking_speed_limit_mps = std::min(
 tracking_speed_limit_mps, std::max(0.0, cusp_speed_mps));
 }
 }

 const auto velocity_samples = command_samples(
 velocity_window, std::max(1e-3, config_.velocity_sample_step_mps),
 {est.forward_mps, 0.0, v_max, -v_max});

  const double ego_radius =
      limits.footprint_radius_m + config_.safety_margin_m;
 // Two-stage local costmap: occupied observations first receive the full
 // chassis envelope as lethal inflation; clearance outside that boundary
 // supplies the graded cost used below.
 const LocalCostmap costmap = make_local_costmap(
 map, std::max(0.0, ego_radius),
 std::max(0.0, config_.soft_inflation_radius_m));

 // Only the current route neighborhood can affect a finite rollout. Find
 // progress once on the complete route, then score every candidate against
 // a window derived from the chassis reach and full braking tail. This
 // keeps local-planning work bounded as global missions grow to kilometres.
 const double forward_braking_m = decel > 0.0
 ? v_max * v_max / (2.0 * decel)
 : 0.0;
 const double lateral_max = std::max(0.0, limits.max_lateral_mps);
 const double lateral_braking_m = config_.lateral_accel_mps2 > 0.0
 ? lateral_max * lateral_max /
 (2.0 * config_.lateral_accel_mps2)
 : 0.0;
 const double reachable_m = std::hypot(
 v_max * predict_T + forward_braking_m,
 lateral_max * predict_T + lateral_braking_m);
 const double route_window_m =
 std::max(std::max(0.0, config_.lookahead_m), reachable_m) +
 std::max(0.0, map.resolution_m);
 const std::vector<core::Vec2> reference = local_route_reference(
 route, route_projection, route_window_m, route_window_m);
 const PathProjection start_projection =
 project_to_path({est.x_m, est.y_m}, reference);

 std::vector<LocalCandidate> proposals;
 const auto commands_resolvable_motion = [&](double forward,
 double angular, double lateral) {
 // A command is motion when the chassis can execute a non-zero twist. Map
 // resolution controls collision sampling; it is not an actuator deadband.
 // Conflating the two labelled a slow Ackermann launch (for example
 // 0.04 m/s on a 0.10 m grid) as stationary, so every first command caused
 // another global replan before the base could accelerate. The platform
 // layer owns any real actuator deadband and the controller/validator still
 // check every selected command independently.
 return std::abs(forward) > 1e-9 || std::abs(angular) > 1e-9 ||
 std::abs(lateral) > 1e-9;
 };

  for (double velocity : velocity_samples) {
 // Once a base that can rotate in place enters the terminal position
 // region, spend its reachable longitudinal authority on braking and use
 // yaw alone to settle the pose. Allowing another translating sample here
 // makes a differential or holonomic base orbit the goal: the heading
 // target switches to terminal yaw, while the speed critic keeps rewarding
 // forward motion. A car-like base is excluded because it may need a
 // forward/reverse pose manoeuvre instead.
 const double terminal_braking_floor_mps = std::max(
 0.0, std::abs(est.forward_mps) - decel * dt);
 if (!car_like && terminal_position_reached &&
 std::abs(velocity) > terminal_braking_floor_mps + 1e-9) {
 continue;
 }
 const double one_cycle_braking_floor = std::max(
 0.0, std::abs(est.forward_mps) - decel * dt);
 if (car_like && std::abs(velocity) >
 std::max(tracking_speed_limit_mps, one_cycle_braking_floor) + 1e-9) {
 continue;
 }
 if (target.gear < 0 && velocity > 1e-9) continue;
 if (target.gear > 0 && velocity < -1e-9) continue;
 auto angular_window = next_window(est.yaw_rate_radps, -w_max, w_max,
 config_.yaw_accel_radps2 * dt);
 // For a car-like base, yaw rate is not an independent command axis:
 // w = v * curvature. Intersect the reachable yaw window with that exact
 // speed-dependent interval before sampling it. Sampling the full body-yaw
 // range and rejecting afterwards can leave only zero (or only one steering
 // sign) when |v|*kappa is narrower than the yaw lattice. That stranded slow
 // terminal manoeuvres even though both steering directions were physically
 // reachable. Differential bases use their wheel-space check below.
 if (car_like) {
 const double curvature_yaw_limit =
 std::abs(velocity) * limits.max_curvature_inv_m;
 angular_window.minimum = std::max(
 angular_window.minimum, -curvature_yaw_limit);
 angular_window.maximum = std::min(
 angular_window.maximum, curvature_yaw_limit);
 }
 const auto angular_samples = command_samples(
 angular_window, std::max(1e-3, config_.yaw_rate_sample_step_radps),
 {est.yaw_rate_radps, 0.0, goal_directed_yaw_rate});
    for (double angular : angular_samples) {
 // Differential commands are reachable in wheel space, not merely in two
 // independent body-twist intervals. Simultaneous linear and yaw changes
 // can satisfy both scalar windows while asking one tread to jump beyond
 // its motor acceleration or speed envelope.
 if (limits.differential_track_m > 0.0 &&
 limits.max_lateral_mps <= 0.0 &&
 limits.max_curvature_inv_m <= 0.0) {
 const double half_track = 0.5 * limits.differential_track_m;
 const double measured_left =
 est.forward_mps - est.yaw_rate_radps * half_track;
 const double measured_right =
 est.forward_mps + est.yaw_rate_radps * half_track;
 const double desired_left = velocity - angular * half_track;
 const double desired_right = velocity + angular * half_track;
 const double wheel_accel =
 accel + config_.yaw_accel_radps2 * half_track;
 const double wheel_delta = wheel_accel * dt;
 if (std::abs(desired_left - measured_left) > wheel_delta + 1e-9 ||
 std::abs(desired_right - measured_right) > wheel_delta + 1e-9) {
 continue;
 }
 if (limits.max_wheel_speed_mps > 0.0 &&
 (std::abs(desired_left) > limits.max_wheel_speed_mps + 1e-9 ||
 std::abs(desired_right) > limits.max_wheel_speed_mps + 1e-9)) {
 continue;
 }
 }
 // The car-like interval was intersected before sampling above; retain this
 // check as a numerical guard around the closed interval endpoints.
 if (car_like && std::abs(angular) >
 std::abs(velocity) * limits.max_curvature_inv_m + 1e-9) continue;
      for (double lateral : lateral_samples) {
      double x = est.x_m, y = est.y_m, heading = est.yaw_rad;
      double rollout_v = est.forward_mps, rollout_w = est.yaw_rate_radps;
      std::vector<core::Vec2> samples{{x, y}};
 double dynamic_free_time = predict_T;
 bool static_collision = false;
 bool dynamic_overlap = false;
 double worst_static_cost = costmap.cost_at(x, y);
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
 if (!costmap.lethal.segment_is_traversable(
 previous_x, previous_y, x, y) ||
 (gate_ && !gate_(previous_x, previous_y, x, y, heading))) {
 static_collision = true;
          break;
        }
 worst_static_cost = std::max(
 worst_static_cost, static_cast<double>(costmap.cost_at(x, y)));
 // Dynamic hypotheses are deliberately a score here, not an early
 // generation veto. The validator must see every statically executable
 // proposal so it can compare a moving escape with the independently
 // predicted risk of braking and holding. Erasing all overlapping
 // rollouts here is what produced a false one-cycle "no plan" in open
 // space while an actor approached a stopped robot.
 if (minimum_predicted_clearance(
 prediction, {previous_x, previous_y}, {x, y}, time,
 sample_time, ego_radius).clearance_m < 0.0) {
 dynamic_overlap = true;
 dynamic_free_time = std::min(dynamic_free_time, time);
 }
      }
 // Classical DWA admissibility: a collision-free finite rollout is not
 // executable unless its complete braking tail also fits. Continue from the
 // terminal actuator state, decelerating every commanded degree of freedom
 // to zero under the same limits. This is a cheap generator-side hard
 // constraint; the raw-evidence validator still performs its independent
 // check after candidate generation.
 double stop_v = rollout_v;
 double stop_w = rollout_w;
 double stop_lateral = lateral;
 double stop_time = predict_T;
 for (int stop_step = 0; !static_collision && stop_step < 1000 &&
 (std::abs(stop_v) > 1e-6 || std::abs(stop_w) > 1e-6 ||
 std::abs(stop_lateral) > 1e-6); ++stop_step) {
 const double next_v = velocity_step_toward(
 stop_v, 0.0, decel, decel, dt);
 const double next_w = velocity_step_toward(
 stop_w, 0.0, config_.yaw_accel_radps2,
 config_.yaw_accel_radps2, dt);
 const double next_lateral = velocity_step_toward(
 stop_lateral, 0.0, config_.lateral_accel_mps2,
 config_.lateral_accel_mps2, dt);
 const double average_v = 0.5 * (stop_v + next_v);
 const double average_w = 0.5 * (stop_w + next_w);
 const double average_lateral = 0.5 * (stop_lateral + next_lateral);
 const double middle_heading = heading + 0.5 * average_w * dt;
 const double previous_x = x;
 const double previous_y = y;
 x += (average_v * std::cos(middle_heading) -
 average_lateral * std::sin(middle_heading)) * dt;
 y += (average_v * std::sin(middle_heading) +
 average_lateral * std::cos(middle_heading)) * dt;
 heading += average_w * dt;
 const double next_time = stop_time + dt;
 samples.push_back({x, y});
 if (!costmap.lethal.segment_is_traversable(
 previous_x, previous_y, x, y) ||
 (gate_ && !gate_(previous_x, previous_y, x, y, heading))) {
 static_collision = true;
 break;
 }
 worst_static_cost = std::max(
 worst_static_cost, static_cast<double>(costmap.cost_at(x, y)));
 if (minimum_predicted_clearance(
 prediction, {previous_x, previous_y}, {x, y}, stop_time,
 next_time, ego_radius).clearance_m < 0.0) {
 dynamic_overlap = true;
 dynamic_free_time = std::min(dynamic_free_time, stop_time);
 }
 stop_v = next_v;
 stop_w = next_w;
 stop_lateral = next_lateral;
 stop_time = next_time;
 }
 // A car-like base may have to drive past the final position and reverse
 // through a cusp to attain its requested pose. Euclidean distance to the
 // lookahead point is not a valid progress measure for that maneuver: it
 // rewards holding as soon as the point moves behind the rear axle. Project
 // the complete stopping rollout onto the forward route window instead.
 // The window is stateful and direction changes are already gated at the
 // physical cusp, so a self-overlapping return leg cannot be skipped.
 const PathProjection directed_end_projection = project_to_route_window(
 {x, y}, route, {route_segment_, route_fraction_}, route_window_m);
 const double directed_progress_m =
 std::isfinite(directed_end_projection.distance_m)
 ? directed_end_projection.along_m
 : 0.0;
 const double goal_score = car_like && route_tracking_valid
 ? directed_progress_m
 : -std::hypot(goal.x - x, goal.y - y);
      const double heading_score =
          std::cos(core::wrap_angle(heading_target - heading));
 // Clearance is a bounded preference across the complete rollout and its
 // braking tail. Looking only at the endpoint lets a long lookahead chord
 // cut through a corner's soft band and finish in open space with no cost.
 // The hard veto stays with the inflated costmap and independent validator.
      const double normalized_clearance =
 1.0 - worst_static_cost;
      const double clearance_score =
 dynamic_overlap ? dynamic_free_time
 : predict_T + 1.0 + normalized_clearance;
      double distance_sum_m = 0.0;
      for (const auto& s : samples)
        distance_sum_m += project_to_path(s, reference).distance_m;
      const auto end_projection = project_to_path({x, y}, reference);
 const double path_heading_target = car_like
 ? heading_target
 : terminal_heading_mode
 ? heading_target
 : end_projection.tangent_rad;
      const double path_score =
          -distance_sum_m / static_cast<double>(samples.size()) +
 0.35 * std::cos(core::wrap_angle(path_heading_target - heading));
 const double progress_score = car_like && route_tracking_valid
 ? directed_progress_m
 : end_projection.along_m - start_projection.along_m;
      const bool angular_reversal = angular * previous_w_ < -0.04;
      const double oscillation_penalty = angular_reversal ? 0.5 : 0.0;
      const double angular_effort_penalty = 0.025 * std::abs(angular);
      const double score =
          config_.weights.w_goal * (goal_score + 0.7 * heading_score) +
          config_.weights.w_clear * clearance_score +
          config_.weights.w_path * path_score +
          config_.weights.w_progress * progress_score +
 config_.weights.w_speed *
 (target.gear < 0 ? -velocity : velocity) -
          config_.weights.w_oscillation * oscillation_penalty -
          angular_effort_penalty;
 if (!static_collision) {
 LocalCandidate candidate;
 candidate.forward_mps = velocity;
 candidate.yaw_rate_radps = angular;
 candidate.lateral_mps = lateral;
 candidate.cost = -score;
 candidate.feasible = true;
 proposals.push_back(candidate);
 if (commands_resolvable_motion(candidate.forward_mps,
 candidate.yaw_rate_radps, candidate.lateral_mps)) {
 ++out.moving_candidate_count;
 }
      }
      }  // lateral
    }
  }
 if (proposals.empty()) return out;
 const bool route_motion_required =
 !terminal_position_reached ||
 terminal_heading_error_rad >
 terminal_heading_tolerance_rad;
 // Safety feasibility is already a hard generation constraint and every
 // proposal still passes the independent validator. Within that admissible
 // set, an active route is a lexicographic progress objective: a stationary
 // local optimum must not outrank all feasible motion merely because critic
 // weights balance at a reverse departure or tight corner. Holding remains
 // in the ranked family, so the validator can select it after rejecting
 // moving proposals. An armed gear-change cusp is the one deliberate stop.
 const bool prioritize_route_motion =
 route_motion_required && !(gear_change && cusp_braking_armed_ &&
 endpoint_distance_m <= cusp_capture_m);
 const double minimum_sample_speed_mps = std::min(
 v_max, std::max(1e-3, config_.velocity_sample_step_mps));
 const double minimum_sample_stop_m = decel > 0.0
 ? minimum_sample_speed_mps * response_s +
 minimum_sample_speed_mps * minimum_sample_speed_mps / (2.0 * decel)
 : std::numeric_limits<double>::infinity();
 const bool route_lattice_exhausted =
 route_tracking_tube_m - route_projection.distance_m <
 minimum_sample_stop_m;
 if (car_like && prioritize_route_motion &&
 out.moving_candidate_count == 0 &&
 (route_lattice_exhausted ||
 std::abs(est.forward_mps) <= decel * dt + 1e-9)) {
 // A quantized dynamic window can contain only hold either at the edge of
 // the tracking tube or after a car-like base has physically stopped at an
 // intermediate pose it cannot leave. Waiting cannot improve either state;
 // request a new kinematic route from the current estimate. Dynamic actors
 // do not cause this branch because their overlap remains a ranked cost and
 // the independent validator owns the live veto.
 out.route_replan_requested = true;
 }
 std::stable_sort(proposals.begin(), proposals.end(),
 [prioritize_route_motion, &commands_resolvable_motion](const LocalCandidate& a,
 const LocalCandidate& b) {
 if (prioritize_route_motion) {
 const bool a_moves = commands_resolvable_motion(
 a.forward_mps, a.yaw_rate_radps, a.lateral_mps);
 const bool b_moves = commands_resolvable_motion(
 b.forward_mps, b.yaw_rate_radps, b.lateral_mps);
 if (a_moves != b_moves) return a_moves;
 }
 return a.cost < b.cost;
 });
 previous_w_ = proposals.front().yaw_rate_radps;
 previous_lateral_ = proposals.front().lateral_mps;
  out.has_plan = true;
 out.best = proposals.front();
 out.candidates = std::move(proposals);
  return out;
}

}  // namespace lost_signal::robot::autonomy
