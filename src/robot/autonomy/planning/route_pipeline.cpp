#include "robot/autonomy/planning/route_pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lost_signal::robot::autonomy {

namespace {

double distance_to_route(const EstimatedState& start, const Route& route) {
 if (!start.valid || !std::isfinite(start.x_m) ||
 !std::isfinite(start.y_m) || !route.valid || route.points.empty()) {
 return std::numeric_limits<double>::infinity();
 }
 double best = std::numeric_limits<double>::infinity();
 if (route.points.size() == 1) {
 return std::hypot(
 start.x_m - route.points.front().x_m,
 start.y_m - route.points.front().y_m);
 }
 for (std::size_t index = 0; index + 1 < route.points.size(); ++index) {
 const auto& a = route.points[index];
 const auto& b = route.points[index + 1];
 const double dx = b.x_m - a.x_m;
 const double dy = b.y_m - a.y_m;
 const double length2 = dx * dx + dy * dy;
 const double fraction = length2 > 1e-12
 ? std::clamp(((start.x_m - a.x_m) * dx +
 (start.y_m - a.y_m) * dy) / length2, 0.0, 1.0)
 : 0.0;
 best = std::min(best, std::hypot(
 start.x_m - (a.x_m + fraction * dx),
 start.y_m - (a.y_m + fraction * dy)));
 }
 return best;
}

} // namespace

bool RoutePlanningPipeline::same_request(
 const RoutePoint& goal, const iface::MapSnapshot& map) const {
 return has_request_ && requested_goal_.x_m == goal.x_m &&
 requested_goal_.y_m == goal.y_m &&
 requested_goal_.yaw_rad == goal.yaw_rad &&
 requested_goal_.gear == goal.gear && requested_map_id_ == map.map_id &&
 requested_map_revision_ == map.revision;
}

bool RoutePlanningPipeline::failed_start_changed(
 const EstimatedState& start, const iface::MapSnapshot& map) const {
 const bool start_valid = start.valid && std::isfinite(start.x_m) &&
 std::isfinite(start.y_m);
 if (start_valid != failed_start_valid_) return true;
 if (!start_valid) return false;
 if (map.has_raster()) {
 const auto cell = [&](double value, double origin) {
 return static_cast<std::int64_t>(
 std::floor((value - origin) / map.resolution_m));
 };
 return cell(start.x_m, map.origin_x_m) !=
 cell(failed_start_x_m_, map.origin_x_m) ||
 cell(start.y_m, map.origin_y_m) !=
 cell(failed_start_y_m_, map.origin_y_m);
 }
 return start.x_m != failed_start_x_m_ || start.y_m != failed_start_y_m_;
}

bool RoutePlanningPipeline::advance(
 IGlobalPlanner& planner, const EstimatedState& start,
 const RoutePoint& goal, bool has_goal, const iface::MapSnapshot& map,
 bool force_replan, Route& active_route,
 double maximum_route_deviation_m) {
 if (!has_goal) {
 if (pending_) planner.cancel_plan();
 const bool changed = pending_ || failed_ || active_route.valid;
 reset();
 active_route = Route{};
 return changed;
 }

 // The active route was validated from an earlier estimated start. Once the
 // current ROBOT estimate leaves a complete chassis-derived route tube, its
 // kinematic prefix is no longer a valid recovery instruction. Replan from
 // the estimate instead of asking a local planner to chase a point behind a
 // nonholonomic base. Infinity keeps the compatibility API opt-out explicit.
 const bool route_deviated = active_route.valid &&
 std::isfinite(maximum_route_deviation_m) &&
 maximum_route_deviation_m >= 0.0 &&
 distance_to_route(start, active_route) > maximum_route_deviation_m;
 force_replan = force_replan || route_deviated;
 const bool request_changed = !same_request(goal, map);
 if (pending_ && request_changed) {
 planner.cancel_plan();
 pending_ = false;
 }

 const bool retry_failed = failed_ &&
 (request_changed || route_deviated || failed_start_changed(start, map));
 const bool begin = !pending_ &&
 (request_changed || route_deviated ||
 (force_replan && !failed_) ||
 (!active_route.valid && !failed_) || retry_failed);
 bool route_state_changed = request_changed || route_deviated;
 if (begin) {
 requested_goal_ = goal;
 requested_map_id_ = map.map_id;
 requested_map_revision_ = map.revision;
 has_request_ = true;
 failed_ = false;
 // A changed goal/map or a complete route-tube departure invalidates the
 // old route immediately. A same-request local replan keeps the last
 // validated route until a replacement succeeds: global planning is an
 // atomic refresh, and Validator/Safety still gate every fallback command.
 if (request_changed || route_deviated) active_route = Route{};

 if (!planner.supports_incremental_plan()) {
 Route replacement = planner.plan(start, goal, map);
 if (!replacement.valid) {
 failed_ = true;
 failed_start_valid_ = start.valid && std::isfinite(start.x_m) &&
 std::isfinite(start.y_m);
 failed_start_x_m_ = start.x_m;
 failed_start_y_m_ = start.y_m;
 } else {
 active_route = std::move(replacement);
 }
 return route_state_changed || !active_route.valid;
 }
 planner.begin_plan(start, goal, map);
 pending_ = true;
 }

 if (!pending_) return route_state_changed;
 GlobalPlanProgress progress =
 planner.continue_plan(planner.incremental_plan_budget());
 if (progress.status == GlobalPlanStatus::kPending) {
 return route_state_changed || !active_route.valid;
 }

 pending_ = false;
 if (progress.status == GlobalPlanStatus::kSucceeded &&
 progress.route.valid) {
 active_route = std::move(progress.route);
 failed_ = false;
 return true;
 }

 failed_ = true;
 failed_start_valid_ = start.valid && std::isfinite(start.x_m) &&
 std::isfinite(start.y_m);
 failed_start_x_m_ = start.x_m;
 failed_start_y_m_ = start.y_m;
 return route_state_changed || !active_route.valid;
}

bool RoutePlanningPipeline::refresh(IGlobalPlanner& planner,
                                    const EstimatedState& start,
                                    const RoutePoint& goal, bool has_goal,
                                    const iface::MapSnapshot& map,
                                    bool force_replan,
                                    Route& active_route) {
  if (!has_goal) return false;
  if (!force_replan && active_route.valid) return false;

  // A planner may provide a fail-closed invalid route while localization is
  // still starting. Do not retain a previous route across that refresh.
  active_route = planner.plan(start, goal, map);
  return true;
}

}  // namespace lost_signal::robot::autonomy
