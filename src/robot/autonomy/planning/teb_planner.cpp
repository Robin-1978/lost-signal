#include "robot/autonomy/planning/teb_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "robot/autonomy/decision/arrival_confidence.hpp"
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

Route offset_route(const Route& route, double lateral_offset_m,
 double transition_m) {
 if (std::abs(lateral_offset_m) < 1e-12 || route.points.size() < 2)
 return route;
 Route shifted = route;
 double progress_m = 0.0;
 for (std::size_t i = 0; i < shifted.points.size(); ++i) {
 if (i > 0) {
 progress_m += std::hypot(
 route.points[i].x_m - route.points[i - 1].x_m,
 route.points[i].y_m - route.points[i - 1].y_m);
 }
 const std::size_t before = i > 0 ? i - 1 : i;
 const std::size_t after = i + 1 < route.points.size() ? i + 1 : i;
 const double tangent = std::atan2(
 route.points[after].y_m - route.points[before].y_m,
 route.points[after].x_m - route.points[before].x_m);
 const double ramp = transition_m > 1e-9
 ? std::clamp(progress_m / transition_m, 0.0, 1.0)
 : 1.0;
 shifted.points[i].x_m +=
 -std::sin(tangent) * lateral_offset_m * ramp;
 shifted.points[i].y_m +=
 std::cos(tangent) * lateral_offset_m * ramp;
 }
 return shifted;
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
 if (config_.topology_enabled && route.valid && route.points.size() >= 2) {
 LocalPlan combined;
 std::vector<TebBandVertex> selected_band;
 double selected_cost = std::numeric_limits<double>::infinity();
 const double lateral_offset =
 std::max(0.0, config_.topology_lateral_clearance_scale) *
 (std::max(0.0, limits.footprint_radius_m) +
 std::max(0.0, config_.extra_clearance_m));
 const std::array<double, 3> offsets{0.0, lateral_offset, -lateral_offset};
 for (const double offset : offsets) {
 TebPlannerConfig branch_config = config_;
 branch_config.topology_enabled = false;
 TebPlanner branch(branch_config);
 branch.previous_lateral_mps_ = previous_lateral_mps_;
 if (std::abs(offset) < 1e-12) branch.last_band_ = last_band_;
 branch.gate_ = gate_;
 const Route seeded_route = offset_route(
 route, offset, std::max(0.0, config_.topology_transition_m));
 LocalPlan proposal = branch.plan(
 est, perception, prediction, map, seeded_route, limits);
 if (!combined.local_costmap.valid() && proposal.local_costmap.valid()) {
 combined.local_costmap = proposal.local_costmap;
 }
 if (!proposal.has_plan) continue;
 for (auto candidate : proposal.candidates) {
 const bool duplicate = std::any_of(
 combined.candidates.begin(), combined.candidates.end(),
 [&](const LocalCandidate& existing) {
 return std::abs(existing.forward_mps - candidate.forward_mps) < 1e-9 &&
 std::abs(existing.yaw_rate_radps - candidate.yaw_rate_radps) < 1e-9 &&
 std::abs(existing.lateral_mps - candidate.lateral_mps) < 1e-9;
 });
 if (!duplicate) combined.candidates.push_back(candidate);
 }
 if (proposal.best.cost < selected_cost) {
 selected_cost = proposal.best.cost;
 selected_band = branch.last_band_;
 }
 }
 if (combined.candidates.empty()) {
 last_band_.clear();
 return combined;
 }
 std::stable_sort(combined.candidates.begin(), combined.candidates.end(),
 [](const LocalCandidate& a, const LocalCandidate& b) {
 return a.cost < b.cost;
 });
 combined.has_plan = true;
 combined.best = combined.candidates.front();
 last_band_ = std::move(selected_band);
 previous_lateral_mps_ = combined.best.lateral_mps;
 return combined;
 }
 LocalPlan out;
 const std::vector<TebBandVertex> warm_band = last_band_;
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
 const LocalCostmap costmap = make_local_costmap(
 map, required_clearance,
 std::max(map.resolution_m, config_.soft_inflation_radius_m));
 out.local_costmap = make_local_costmap_diagnostic(costmap);
 if (warm_band.size() == last_band_.size() && !warm_band.empty() &&
 std::hypot(warm_band.front().position.x - est.x_m,
 warm_band.front().position.y - est.y_m) <=
 std::max(0.0, config_.warm_start_max_error_m)) {
 const double blend = std::clamp(config_.warm_start_blend, 0.0, 1.0);
 for (int i = 1; i + 1 < count; ++i) {
 const std::size_t index = static_cast<std::size_t>(i);
 const core::Vec2 seeded{
 last_band_[index].position.x * (1.0 - blend) +
 warm_band[index].position.x * blend,
 last_band_[index].position.y * (1.0 - blend) +
 warm_band[index].position.y * blend};
 if (costmap.lethal.is_free(seeded.x, seeded.y)) {
 last_band_[index].position = seeded;
 }
 }
 }
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
 const double static_cost = costmap.cost_at(
 vertex.position.x, vertex.position.y);
 if (static_cost > 0.0) {
 // Negative cost gradient points away from the lethal boundary.
 const double gx = costmap.cost_at(vertex.position.x - epsilon,
 vertex.position.y) -
 costmap.cost_at(vertex.position.x + epsilon, vertex.position.y);
 const double gy = costmap.cost_at(vertex.position.x,
 vertex.position.y - epsilon) -
 costmap.cost_at(vertex.position.x, vertex.position.y + epsilon);
 const double gradient = std::hypot(gx, gy);
 if (gradient > 1e-9) {
 const double gain = config_.static_weight *
 static_cost / gradient;
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
 if (costmap.lethal.is_free(proposed.x, proposed.y)) {
 vertex.position = proposed;
 }
 }
 }
 // Static geometry and terrain are generation-time hard constraints. Keep
 // dynamic overlap as a planner cost so the common validator can compare the
 // complete command family with stopping in place.
 double band_dynamic_clearance = std::numeric_limits<double>::infinity();
 for (std::size_t i = 1; i < last_band_.size(); ++i) {
 const auto& a = last_band_[i - 1];
 const auto& b = last_band_[i];
 const double heading = std::atan2(
 b.position.y - a.position.y, b.position.x - a.position.x);
 if (!costmap.lethal.segment_is_traversable(
 a.position.x, a.position.y, b.position.x, b.position.y) ||
 (gate_ && !gate_(a.position.x, a.position.y,
 b.position.x, b.position.y, heading))) {
 last_band_.clear();
 return out;
 }
 band_dynamic_clearance = std::min(
 band_dynamic_clearance,
 minimum_predicted_clearance(
 prediction, a.position, b.position, a.time_s, b.time_s,
 limits.footprint_radius_m + config_.extra_clearance_m).clearance_m);
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
 const double terminal_position_tolerance_m = confidence_bounded_tolerance(
 config_.terminal_position_tolerance_m,
 position_confidence_radius(est.position_sigma_m,
 config_.terminal_confidence_probability));
 const bool at_terminal_position = std::hypot(
 terminal.x_m - est.x_m, terminal.y_m - est.y_m) <=
 terminal_position_tolerance_m;
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

 // TEB's geometric band is chassis agnostic, but its first command is not:
 // a differential base can only realise a (v, w) pair through two wheel
 // speeds. Project the command through the same measured wheel window used by
 // DWA and the decision recovery path before exposing it to the validator.
 // This keeps the validator, controller and plant on one executable command,
 // even when a warm band asks for a sharp simultaneous turn and launch.
 const auto project_differential_command = [&](double& body_forward,
                                                double& body_yaw) {
   if (!(limits.differential_track_m > 0.0) ||
       limits.max_lateral_mps > 0.0 || limits.max_curvature_inv_m > 0.0) {
     return;
   }
   const double half_track = 0.5 * limits.differential_track_m;
   const double measured_left =
       est.forward_mps - est.yaw_rate_radps * half_track;
   const double measured_right =
       est.forward_mps + est.yaw_rate_radps * half_track;
   const double wheel_rate = std::max(0.0, config_.accel_mps2) +
                             std::max(0.0, config_.yaw_accel_radps2) *
                                 half_track;
   const double wheel_delta = wheel_rate * config_.control_dt_s;
   double left = std::clamp(body_forward - body_yaw * half_track,
                            measured_left - wheel_delta,
                            measured_left + wheel_delta);
   double right = std::clamp(body_forward + body_yaw * half_track,
                             measured_right - wheel_delta,
                             measured_right + wheel_delta);
   if (limits.max_wheel_speed_mps > 0.0) {
     left = std::clamp(left, -limits.max_wheel_speed_mps,
                       limits.max_wheel_speed_mps);
     right = std::clamp(right, -limits.max_wheel_speed_mps,
                        limits.max_wheel_speed_mps);
   }
   body_forward = 0.5 * (left + right);
   body_yaw = (right - left) / limits.differential_track_m;
 };
 project_differential_command(forward, yaw);
 LocalCandidate candidate;
 candidate.forward_mps = forward;
 candidate.lateral_mps = lateral;
 candidate.yaw_rate_radps = yaw;
 candidate.cost = -band_dynamic_clearance;
 candidate.feasible = true;
 out.has_plan = true;
 out.best = candidate;
 const auto append_unique = [&](LocalCandidate proposal) {
 const bool duplicate = std::any_of(
 out.candidates.begin(), out.candidates.end(),
 [&](const LocalCandidate& existing) {
 return std::abs(existing.forward_mps - proposal.forward_mps) < 1e-9 &&
 std::abs(existing.yaw_rate_radps - proposal.yaw_rate_radps) < 1e-9 &&
 std::abs(existing.lateral_mps - proposal.lateral_mps) < 1e-9;
 });
 if (!duplicate) out.candidates.push_back(proposal);
 };
 for (const double raw_scale : config_.candidate_speed_scales) {
 const double scale = std::clamp(raw_scale, 0.0, 1.0);
 LocalCandidate proposal = candidate;
 proposal.forward_mps = clamp_reachable(
 candidate.forward_mps * scale, est.forward_mps,
 config_.accel_mps2, config_.decel_mps2, config_.control_dt_s);
 proposal.yaw_rate_radps = clamp_reachable(
 candidate.yaw_rate_radps * scale, est.yaw_rate_radps,
 config_.yaw_accel_radps2, config_.yaw_accel_radps2,
 config_.control_dt_s);
 proposal.lateral_mps = clamp_reachable(
 candidate.lateral_mps * scale, previous_lateral_mps_,
 config_.lateral_accel_mps2, config_.lateral_accel_mps2,
 config_.control_dt_s);
 project_differential_command(proposal.forward_mps,
                              proposal.yaw_rate_radps);
 proposal.cost = candidate.cost + (1.0 - scale);
 append_unique(proposal);
 }
 if (!out.candidates.empty()) out.best = out.candidates.front();
 previous_lateral_mps_ = out.best.lateral_mps;
 return out;
}

} // namespace lost_signal::robot::autonomy
