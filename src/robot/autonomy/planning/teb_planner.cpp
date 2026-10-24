#include "robot/autonomy/planning/teb_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "robot/autonomy/planning/prediction.hpp"

namespace lost_signal::robot::autonomy {
namespace {

struct RouteProjection {
 std::size_t segment = 0;
 double fraction = 0.0;
 double distance_m = std::numeric_limits<double>::infinity();
};

RouteProjection project_route(const Route& route, core::Vec2 point) {
 RouteProjection best;
 if (route.points.size() < 2) return best;
 for (std::size_t i = 0; i + 1 < route.points.size(); ++i) {
 const core::Vec2 a{route.points[i].x_m, route.points[i].y_m};
 const core::Vec2 b{route.points[i + 1].x_m, route.points[i + 1].y_m};
 const core::Vec2 d{b.x - a.x, b.y - a.y};
 const double length2 = d.x * d.x + d.y * d.y;
 if (length2 < 1e-12) continue;
 const double u = std::clamp(
 ((point.x - a.x) * d.x + (point.y - a.y) * d.y) / length2,
 0.0, 1.0);
 const double distance = std::hypot(
 point.x - (a.x + u * d.x), point.y - (a.y + u * d.y));
 if (distance < best.distance_m) best = {i, u, distance};
 }
 return best;
}

core::Vec2 route_point_at(const Route& route, RouteProjection projection,
 double advance_m, int* gear) {
 if (route.points.empty()) return {};
 if (route.points.size() == 1) {
 if (gear) *gear = route.points.front().gear;
 return {route.points.front().x_m, route.points.front().y_m};
 }
 std::size_t segment = std::min(
 projection.segment, route.points.size() - 2);
 double remaining = std::max(0.0, advance_m);
 double fraction = std::clamp(projection.fraction, 0.0, 1.0);
 while (segment + 1 < route.points.size()) {
 const auto& a = route.points[segment];
 const auto& b = route.points[segment + 1];
 const double length = std::hypot(b.x_m - a.x_m, b.y_m - a.y_m);
 const double available = length * (1.0 - fraction);
 if (length > 1e-9 && remaining <= available) {
 const double u = fraction + remaining / length;
 if (gear) *gear = a.gear;
 return {a.x_m + u * (b.x_m - a.x_m),
 a.y_m + u * (b.y_m - a.y_m)};
 }
 remaining -= available;
 ++segment;
 fraction = 0.0;
 }
 if (gear) *gear = route.points.back().gear;
 return {route.points.back().x_m, route.points.back().y_m};
}

double point_dynamic_clearance(const PredictionBatch& prediction,
 core::Vec2 point, double time_s, double ego_radius_m,
 core::Vec2* push_direction) {
 double best = std::numeric_limits<double>::infinity();
 core::Vec2 best_push;
 for (const auto& hypothesis : prediction.hypotheses) {
 const auto state = predicted_state_at(hypothesis, prediction, time_s);
 if (!state) continue;
 const core::Vec2 delta{point.x - state->x_m, point.y - state->y_m};
 const double distance = std::hypot(delta.x, delta.y);
 const double clearance = distance - ego_radius_m - state->occupied_radius_m();
 if (clearance < best) {
 best = clearance;
 if (distance > 1e-9) {
 best_push = {delta.x / distance, delta.y / distance};
 } else {
 best_push = {1.0, 0.0};
 }
 }
 }
 if (push_direction != nullptr) *push_direction = best_push;
 return best;
}

double clamp_reachable(double target, double current, double positive_rate,
 double negative_rate, double dt) {
 const double lo = current - std::max(0.0, negative_rate) * dt;
 const double hi = current + std::max(0.0, positive_rate) * dt;
 return std::clamp(target, lo, hi);
}

} // namespace

TebPlanner::TebPlanner(TebPlannerConfig config) : config_(config) {}

void TebPlanner::reset() {
 last_band_.clear();
 previous_lateral_mps_ = 0.0;
}

LocalPlan TebPlanner::plan(const EstimatedState& est,
 const PerceptionResult& perception, const PredictionBatch& prediction,
 const LocalMap& map, const Route& route,
 const platform::ChassisLimits& limits) {
 LocalPlan out;
 last_band_.clear();
 if (!est.valid || !perception.scan_ok || !map.valid ||
 !route.valid || route.points.empty() ||
 !(limits.max_forward_mps > 0.0) || !(limits.max_yaw_rate_radps > 0.0)) {
 return out;
 }
 const int count = std::max(3, config_.samples);
 const double horizon = std::max(config_.control_dt_s, config_.horizon_s);
 // Optimize only the distance reachable by the command this decision may
 // publish. Using the chassis top speed from rest made the band demand a
 // fully observed multi-metre corridor even though acceleration limited the
 // first command to a crawl; DWA correctly reasons over that same reachable
 // window. The band expands naturally as measured speed rises.
 const double reachable_speed = std::min(
 limits.max_forward_mps,
 std::abs(est.forward_mps) +
 std::max(0.0, config_.accel_mps2) * config_.control_dt_s);
 const double cruise = std::max(0.05, reachable_speed);
 const double band_length = cruise * horizon;
 const RouteProjection projection = project_route(
 route, {est.x_m, est.y_m});
 std::vector<core::Vec2> reference(static_cast<std::size_t>(count));
 std::vector<int> gears(static_cast<std::size_t>(count), 0);
 reference.front() = {est.x_m, est.y_m};
 for (int i = 1; i < count; ++i) {
 const double distance = band_length * static_cast<double>(i) /
 static_cast<double>(count - 1);
 reference[static_cast<std::size_t>(i)] = route_point_at(
 route, projection, distance, &gears[static_cast<std::size_t>(i)]);
 }
 last_band_.resize(static_cast<std::size_t>(count));
 last_band_.front() = {reference.front(), 0.0};
 for (int i = 1; i < count; ++i) {
 const double distance = std::hypot(
 reference[static_cast<std::size_t>(i)].x -
 reference[static_cast<std::size_t>(i - 1)].x,
 reference[static_cast<std::size_t>(i)].y -
 reference[static_cast<std::size_t>(i - 1)].y);
 const double dt = std::clamp(
 distance / cruise, config_.min_band_dt_s, config_.max_band_dt_s);
 last_band_[static_cast<std::size_t>(i)] = {
 reference[static_cast<std::size_t>(i)],
 last_band_[static_cast<std::size_t>(i - 1)].time_s + dt};
 }
 const double required_clearance =
 std::max(0.0, limits.footprint_radius_m) +
 std::max(0.0, config_.extra_clearance_m);
 const double epsilon = std::max(0.02, map.resolution_m);
 for (int iteration = 0; iteration < std::max(0, config_.iterations);
 ++iteration) {
 for (int i = 1; i + 1 < count; ++i) {
 auto& vertex = last_band_[static_cast<std::size_t>(i)];
 const auto& previous = last_band_[static_cast<std::size_t>(i - 1)];
 const auto& next = last_band_[static_cast<std::size_t>(i + 1)];
 const core::Vec2 smooth{
 0.5 * (previous.position.x + next.position.x) - vertex.position.x,
 0.5 * (previous.position.y + next.position.y) - vertex.position.y};
 const core::Vec2 anchor{
 reference[static_cast<std::size_t>(i)].x - vertex.position.x,
 reference[static_cast<std::size_t>(i)].y - vertex.position.y};
 core::Vec2 delta{
 config_.smooth_weight * smooth.x + config_.route_weight * anchor.x,
 config_.smooth_weight * smooth.y + config_.route_weight * anchor.y};
 const double static_clearance = map.clearance(
 vertex.position.x, vertex.position.y);
 if (static_clearance < required_clearance) {
 const double gx = map.clearance(vertex.position.x + epsilon,
 vertex.position.y) -
 map.clearance(vertex.position.x - epsilon, vertex.position.y);
 const double gy = map.clearance(vertex.position.x,
 vertex.position.y + epsilon) -
 map.clearance(vertex.position.x, vertex.position.y - epsilon);
 const double gradient = std::hypot(gx, gy);
 if (gradient > 1e-9) {
 const double gain = config_.static_weight *
 (required_clearance - static_clearance) / gradient;
 delta.x += gain * gx;
 delta.y += gain * gy;
 }
 }
 core::Vec2 dynamic_push;
 const double dynamic_clearance = point_dynamic_clearance(
 prediction, vertex.position, vertex.time_s,
 limits.footprint_radius_m, &dynamic_push);
 if (dynamic_clearance < config_.extra_clearance_m) {
 const double gain = config_.dynamic_weight *
 (config_.extra_clearance_m - dynamic_clearance);
 delta.x += gain * dynamic_push.x;
 delta.y += gain * dynamic_push.y;
 // Time is elastic too: choose the nearby phase with more clearance while
 // preserving ordered, bounded intervals.
 const double shift = std::min(0.10, config_.max_band_dt_s * 0.5);
 const double earlier = point_dynamic_clearance(
 prediction, vertex.position, vertex.time_s - shift,
 limits.footprint_radius_m, nullptr);
 const double later = point_dynamic_clearance(
 prediction, vertex.position, vertex.time_s + shift,
 limits.footprint_radius_m, nullptr);
 const double proposed = vertex.time_s + (later > earlier ? shift : -shift);
 vertex.time_s = std::clamp(
 proposed, previous.time_s + config_.min_band_dt_s,
 next.time_s - config_.min_band_dt_s);
 }
 const double magnitude = std::hypot(delta.x, delta.y);
 if (magnitude > config_.spatial_step_cap_m && magnitude > 1e-12) {
 const double scale = config_.spatial_step_cap_m / magnitude;
 delta.x *= scale;
 delta.y *= scale;
 }
 const core::Vec2 proposed{
 vertex.position.x + delta.x, vertex.position.y + delta.y};
 if (map.is_free(proposed.x, proposed.y)) vertex.position = proposed;
 }
 }
 // Every optimized segment must remain continuously admissible. The dynamic
 // query minimizes relative motion over the whole time interval.
 for (std::size_t i = 1; i < last_band_.size(); ++i) {
 const auto& a = last_band_[i - 1];
 const auto& b = last_band_[i];
 const double heading = std::atan2(
 b.position.y - a.position.y, b.position.x - a.position.x);
 if (!map.segment_is_traversable(
 a.position.x, a.position.y, b.position.x, b.position.y) ||
 map.clearance(b.position.x, b.position.y) < required_clearance ||
 (gate_ && !gate_(a.position.x, a.position.y,
 b.position.x, b.position.y, heading)) ||
 minimum_predicted_clearance(
 prediction, a.position, b.position, a.time_s, b.time_s,
 limits.footprint_radius_m + config_.extra_clearance_m).clearance_m < 0.0) {
 last_band_.clear();
 return out;
 }
 }
 if (last_band_.size() < 2) return out;
 const auto& first = last_band_[1];
 const double first_dt = std::max(config_.control_dt_s, first.time_s);
 const core::Vec2 world_velocity{
 (first.position.x - est.x_m) / first_dt,
 (first.position.y - est.y_m) / first_dt};
 const double c = std::cos(est.yaw_rad), s = std::sin(est.yaw_rad);
 double forward = c * world_velocity.x + s * world_velocity.y;
 double lateral = -s * world_velocity.x + c * world_velocity.y;
 int gear = gears.size() > 1 ? gears[1] : 0;
 const double travel_speed = std::hypot(world_velocity.x, world_velocity.y);
 const double travel_heading = travel_speed > 1e-9
 ? std::atan2(world_velocity.y, world_velocity.x)
 : est.yaw_rad;
 if (gear == 0 && std::abs(core::wrap_angle(
 travel_heading - est.yaw_rad)) > 0.5 * core::kPi) {
 gear = -1;
 }
 if (gear < 0) {
 forward = -std::hypot(world_velocity.x, world_velocity.y);
 } else if (limits.max_lateral_mps <= 0.0) {
 forward = std::hypot(world_velocity.x, world_velocity.y);
 }
 forward = std::clamp(forward,
 -limits.max_forward_mps, limits.max_forward_mps);
 forward = clamp_reachable(
 forward, est.forward_mps, config_.accel_mps2,
 config_.decel_mps2, config_.control_dt_s);
 const auto& terminal = route.points.back();
 const bool at_terminal_position = std::hypot(
 terminal.x_m - est.x_m, terminal.y_m - est.y_m) <=
 std::max(0.0, config_.terminal_position_tolerance_m);
 const double heading_target = at_terminal_position &&
 std::isfinite(terminal.yaw_rad)
 ? terminal.yaw_rad
 : travel_heading + (gear < 0 ? core::kPi : 0.0);
 double yaw = core::clamp(
 core::wrap_angle(heading_target - est.yaw_rad) / first_dt,
 -limits.max_yaw_rate_radps, limits.max_yaw_rate_radps);
 yaw = clamp_reachable(
 yaw, est.yaw_rate_radps, config_.yaw_accel_radps2,
 config_.yaw_accel_radps2, config_.control_dt_s);
 if (limits.max_curvature_inv_m > 0.0) {
 const double max_yaw = std::abs(forward) * limits.max_curvature_inv_m;
 yaw = std::clamp(yaw, -max_yaw, max_yaw);
 }
 if (limits.max_lateral_mps > 0.0) {
 lateral = std::clamp(lateral,
 -limits.max_lateral_mps, limits.max_lateral_mps);
 lateral = clamp_reachable(
 lateral, previous_lateral_mps_, config_.lateral_accel_mps2,
 config_.lateral_accel_mps2, config_.control_dt_s);
 } else {
 lateral = 0.0;
 }
 LocalCandidate candidate;
 candidate.forward_mps = forward;
 candidate.lateral_mps = lateral;
 candidate.yaw_rate_radps = yaw;
 candidate.cost = 0.0;
 candidate.feasible = true;
 previous_lateral_mps_ = lateral;
 out.has_plan = true;
 out.best = candidate;
 return out;
}

} // namespace lost_signal::robot::autonomy
