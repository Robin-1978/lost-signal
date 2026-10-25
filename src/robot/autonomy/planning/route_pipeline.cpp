#include "robot/autonomy/planning/route_pipeline.hpp"

#include <cmath>

namespace lost_signal::robot::autonomy {

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
 bool force_replan, Route& active_route) {
 if (!has_goal) {
 if (pending_) planner.cancel_plan();
 const bool changed = pending_ || failed_ || active_route.valid;
 reset();
 active_route = Route{};
 return changed;
 }

 const bool request_changed = !same_request(goal, map);
 if (pending_ && (request_changed || force_replan)) {
 planner.cancel_plan();
 pending_ = false;
 }

 const bool retry_failed = failed_ &&
 (request_changed || force_replan || failed_start_changed(start, map));
 const bool begin = !pending_ &&
 (request_changed || force_replan || (!active_route.valid && !failed_) ||
 retry_failed);
 if (begin) {
 requested_goal_ = goal;
 requested_map_id_ = map.map_id;
 requested_map_revision_ = map.revision;
 has_request_ = true;
 failed_ = false;
 active_route = Route{};

 if (!planner.supports_incremental_plan()) {
 active_route = planner.plan(start, goal, map);
 if (!active_route.valid) {
 failed_ = true;
 failed_start_valid_ = start.valid && std::isfinite(start.x_m) &&
 std::isfinite(start.y_m);
 failed_start_x_m_ = start.x_m;
 failed_start_y_m_ = start.y_m;
 }
 return true;
 }
 planner.begin_plan(start, goal, map);
 pending_ = true;
 }

 if (!pending_) return begin;
 GlobalPlanProgress progress =
 planner.continue_plan(planner.incremental_plan_budget());
 if (progress.status == GlobalPlanStatus::kPending) return true;

 pending_ = false;
 if (progress.status == GlobalPlanStatus::kSucceeded &&
 progress.route.valid) {
 active_route = std::move(progress.route);
 failed_ = false;
 return true;
 }

 active_route = Route{};
 failed_ = true;
 failed_start_valid_ = start.valid && std::isfinite(start.x_m) &&
 std::isfinite(start.y_m);
 failed_start_x_m_ = start.x_m;
 failed_start_y_m_ = start.y_m;
 return true;
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
