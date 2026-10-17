#include "autonomy/behavior/crossing_behavior.hpp"

#include <algorithm>
#include <cmath>

namespace sim {
namespace {

constexpr double kStopLineToleranceM = 0.10;

double distance(Vec2 a, Vec2 b) {
 return std::hypot(a.x - b.x, a.y - b.y);
}

bool inside(Vec2 point, const ConflictZone& zone) {
 return point.x >= zone.min.x && point.x <= zone.max.x && point.y >= zone.min.y && point.y <= zone.max.y;
}

double distance_to_box(Vec2 point, const ConflictZone& zone) {
 const double dx = std::max({zone.min.x - point.x, 0.0, point.x - zone.max.x});
 const double dy = std::max({zone.min.y - point.y, 0.0, point.y - zone.max.y});
 return std::hypot(dx, dy);
}

} // namespace

CrossingEvaluation evaluate_crossing(const std::vector<ConflictZone>& zones,
                                     const std::vector<RoutePoint>& route, std::size_t path_index,
                                     Vec2 ego_position, double cruise_mps, double brake_decel_mps2,
                                     const std::vector<std::vector<DynamicObjectState>>& predicted_frames,
                                     double frame_dt_s, double ego_radius_m, double margin_m) {
 CrossingEvaluation out;
 if (route.empty() || zones.empty())
  return out;
 const std::size_t start = std::min(path_index, route.size() - 1);
 double largest_actor_radius = 0.0;
 for (const auto& frame : predicted_frames) {
  for (const auto& actor : frame)
   largest_actor_radius = std::max(largest_actor_radius, actor.radius);
 }
 const double clear_distance = ego_radius_m + margin_m + largest_actor_radius;

 const ConflictZone* selected_zone = nullptr;
 std::size_t selected_entry = route.size();
 std::size_t selected_exit = route.size();
 std::size_t selected_distance = route.size();
 for (const auto& zone : zones) {
  const bool body_inside_zone = inside(ego_position, zone);
  std::size_t scan = 0;
  while (scan < route.size()) {
   while (scan < route.size() && !inside(route[scan].position, zone))
    ++scan;
   if (scan == route.size())
    break;
   const std::size_t entry = scan;
   while (scan + 1 < route.size() && inside(route[scan + 1].position, zone))
    ++scan;
   std::size_t exit = scan + 1 < route.size() ? scan + 1 : scan;
   // The maneuver ends at a safe refuge, not at the mathematical box edge.
   // assess_departure holds its endpoint for the rest of the horizon, so the
   // endpoint must clear the zone by both footprints and the safety margin.
   while (exit + 1 < route.size() && distance_to_box(route[exit].position, zone) < clear_distance)
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

 const double braking_distance = cruise_mps * cruise_mps / (2.0 * std::max(0.1, brake_decel_mps2));
 std::size_t stop = selected_entry;
 double walked = 0.0;
 // Derive the stop line from the canonical route, not the controller's
 // moving lookahead index. Its world position therefore remains fixed while
 // the robot approaches and waits.
 // govern_crossing accepts the body as settled up to the stop-line
 // tolerance beyond the nominal point. Reserve the same tolerance here so
 // that the accepted pose still satisfies the requested clearance.
 while (stop > 0 && walked < clear_distance + kStopLineToleranceM) {
  walked += distance(route[stop].position, route[stop - 1].position);
  --stop;
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
 out.inside_zone = inside(ego_position, *selected_zone);
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
 const double longitudinal = (ego_position.x - route[stop].position.x) * stop_direction.x +
                             (ego_position.y - route[stop].position.y) * stop_direction.y;
 // Keep route-level behavior dormant during unrelated upstream maneuvers.
 // Activation begins one short preview beyond the already conservative
 // braking stop distance, or immediately once route/body progress reaches
 // the conflict segment.
 const double activation_distance = braking_distance + 0.5;
 if (!inside(ego_position, *selected_zone) && start < selected_entry &&
     distance(ego_position, route[stop].position) > activation_distance)
  return {};
 out.stop_reached = longitudinal >= -kStopLineToleranceM;
 out.zone_id = selected_zone->id;
 out.entry_index = selected_entry;
 out.exit_index = selected_exit;
 out.stop_index = stop;
 out.stop_point = route[stop].position;
 out.stop_direction = stop_direction;
 out.stop_distance_m = std::max(0.0, -longitudinal);
 out.departure =
   assess_departure(crossing_path, 0, cruise_mps, predicted_frames, frame_dt_s, ego_radius_m, margin_m);
 return out;
}

CrossingOutput govern_crossing(CrossingState& state, const CrossingInput& input, const std::string& zone_id) {
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
  state.stop_valid = true;
 }
 if (state.phase == CrossingPhase::Committed || input.inside_zone) {
  state.phase = CrossingPhase::Committed;
  state.clear_time_s = 0.0;
  return {state.phase, false, true, state.stop_point, false};
 }
 state.phase = CrossingPhase::Holding;
 if (input.ready_to_commit && input.safe_to_commit) {
  state.clear_time_s += std::max(0.0, input.dt_s);
  if (state.clear_time_s + 1e-12 >= kCrossingCommitConfirmS) {
   state.phase = CrossingPhase::Committed;
   state.clear_time_s = 0.0;
   return {state.phase, false, true, state.stop_point, false};
  }
 } else {
  state.clear_time_s = 0.0;
 }
 return {state.phase, true, false, state.stop_point, state.stop_valid};
}

} // namespace sim
