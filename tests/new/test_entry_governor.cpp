#include "robot/autonomy/behavior/entry_governor.hpp"
#include "robot/autonomy/behavior/crossing_behavior.hpp"

#include <cmath>
#include <iostream>

#define CHECK(condition)                                                                                     \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n";                           \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

// Governor contract: one-sided brake. Occupied engages now, sustained Clear
// releases, silence releases, and no input combination ever commands motion.
int main() {
 const sim::CrossingExecutionCaps crossing_caps{1.0, 2.0, 2.0, 2.0, 2.0, 4.0};
 sim::GovernorState state;
 sim::GovernorInput idle;
 // Silent start: no hold, and silence never engages one.
 for (int i = 0; i < 100; ++i)
  CHECK(!govern_entry(state, idle).hold_active);
 // Occupied engages on the first fresh frame.
 sim::GovernorInput occ;
 occ.occupied_fresh = true;
 occ.stop_point = {3.0, 4.0};
 occ.stop_direction = {0.0, 1.0};
 occ.stop_valid = true;
 const auto held = govern_entry(state, occ);
 CHECK(held.hold_active && held.stop_valid);
 CHECK(held.stop_point.x == 3.0 && held.stop_point.y == 4.0);
 CHECK(held.stop_direction.x == 0.0 && held.stop_direction.y == 1.0);
 // Uncertain gaps keep both the hold and its virtual wall (never invent a
 // release and never erase the only safe stopping point).
 for (int i = 0; i < 10; ++i) {
  const auto gap = govern_entry(state, idle);
  CHECK(gap.hold_active);
  CHECK(gap.stop_valid);
  CHECK(gap.stop_point.x == 3.0 && gap.stop_point.y == 4.0);
 }
 // A single Clear does not release (needs 25 sustained).
 sim::GovernorInput clear;
 clear.clear_fresh = true;
 CHECK(govern_entry(state, clear).hold_active);
 for (int i = 0; i < 23; ++i)
  CHECK(govern_entry(state, clear).hold_active);
 CHECK(!govern_entry(state, clear).hold_active);
 CHECK(!state.stop_valid);
 // Re-engage, then prolonged silence releases (tracks gone: DWA owns it).
 CHECK(govern_entry(state, occ).hold_active);
 for (int i = 0; i < 14; ++i)
  CHECK(govern_entry(state, idle).hold_active);
 CHECK(!govern_entry(state, idle).hold_active);
 // Conflict geometry: crossing rays meet mid-way; parallel falls back.
 const auto cross = sim::governor_conflict_center({0, 0}, {1, 0}, {4, -4}, {0, 1});
 CHECK(cross.x > 3.0 && cross.x < 5.0 && cross.y > -1.0 && cross.y < 1.0);
 const auto parallel = sim::governor_conflict_center({0, 0}, {1, 0}, {0, 2}, {1, 0});
 CHECK(parallel.x == 0.0 && parallel.y == 1.0);

 // Generic dynamic interaction does not require a mapped crossing. A target
 // that crosses between prediction frames must still intersect the ego's
 // same-time swept route, while empty predictions remain unknown rather than
 // becoming a clear verdict.
 sim::EntryWindowRequest online_conflict;
 online_conflict.route = {{0.0, 0.0}, {2.0, 0.0}, {4.0, 0.0}};
 online_conflict.cruise_speed_mps = 1.0;
 online_conflict.frame_dt_s = 1.0;
 online_conflict.ego_radius_m = 0.2;
 online_conflict.clearance_m = 0.1;
 online_conflict.braking_deceleration_mps2 = 1.0;
 online_conflict.predicted_frames = {
 {{{0.5, -1.0}, {0.0, 2.0}, 0.2}},
 {{{0.5, 1.0}, {0.0, 2.0}, 0.2}},
 {{{0.5, 3.0}, {0.0, 2.0}, 0.2}},
 };
 const auto online_occupied = sim::assess_dynamic_entry_window(online_conflict);
 CHECK(online_occupied.prediction_evidence_available);
 CHECK(online_occupied.conflict);
 CHECK(online_occupied.governor.occupied_fresh);
 CHECK(online_occupied.governor.stop_valid);
 CHECK(online_occupied.safe_hold_available);
 CHECK(std::isfinite(online_occupied.stop_distance_m));
 CHECK(online_occupied.first_conflict_s > 0.0 &&
 online_occupied.first_conflict_s < 1.0);
 CHECK(online_occupied.governor.stop_direction.x == 1.0);
 CHECK(online_occupied.governor.stop_direction.y == 0.0);

 // Mapped semantics and online inference are intersected, never selected as
 // alternatives. The earlier online wall wins; an invalid held wall means
 // immediate stop and outranks either geometric wall.
 sim::GovernorOutput mapped_hold{true, {3.0, 0.0}, {1.0, 0.0}, true};
 sim::GovernorOutput online_hold{true, {1.0, 0.0}, {1.0, 0.0}, true};
 const std::vector<sim::Vec2> merge_route{{0.0, 0.0}, {4.0, 0.0}};
 const auto merged = sim::merge_entry_constraints(
 mapped_hold, online_hold, merge_route);
 CHECK(merged.hold_active && merged.stop_valid);
 CHECK(merged.stop_point.x == 1.0);
 sim::GovernorOutput immediate_hold;
 immediate_hold.hold_active = true;
 const auto immediate = sim::merge_entry_constraints(
 mapped_hold, immediate_hold, merge_route);
 CHECK(immediate.hold_active && !immediate.stop_valid);

 auto online_clear_request = online_conflict;
 for (auto& frame : online_clear_request.predicted_frames)
 frame.front().p.y += 10.0;
 const auto online_clear = sim::assess_dynamic_entry_window(
 online_clear_request);
 CHECK(online_clear.prediction_evidence_available);
 CHECK(!online_clear.conflict);
 CHECK(online_clear.governor.clear_fresh);
 CHECK(!online_clear.governor.stop_valid);

 auto online_unknown_request = online_conflict;
 online_unknown_request.predicted_frames = {{}, {}, {}};
 const auto online_unknown = sim::assess_dynamic_entry_window(
 online_unknown_request);
 CHECK(!online_unknown.prediction_evidence_available);
 CHECK(!online_unknown.governor.occupied_fresh);
 CHECK(!online_unknown.governor.clear_fresh);

 // Waiting is an action with a time occupancy, not a default-safe fallback.
 // If the target sweeps every upstream point including the current pose, the
 // conflict remains fresh but no virtual wall may command the robot to wait
 // where it will be struck.
 auto unsafe_hold_request = online_conflict;
 unsafe_hold_request.route = {{0.0, 0.0}, {2.0, 0.0}};
 unsafe_hold_request.predicted_frames = {
 {{{-0.5, 0.0}, {1.0, 0.0}, 0.3}},
 {{{0.5, 0.0}, {1.0, 0.0}, 0.3}},
 {{{1.5, 0.0}, {1.0, 0.0}, 0.3}},
 };
 const auto unsafe_hold = sim::assess_dynamic_entry_window(
 unsafe_hold_request);
 CHECK(unsafe_hold.conflict);
 CHECK(unsafe_hold.governor.occupied_fresh);
 CHECK(!unsafe_hold.safe_hold_available);
 CHECK(!unsafe_hold.governor.stop_valid);

 sim::GovernorState unsafe_hold_state;
 CHECK(govern_entry(unsafe_hold_state, occ).stop_valid);
 const auto invalidated_wall = govern_entry(
 unsafe_hold_state, unsafe_hold.governor);
 CHECK(invalidated_wall.hold_active);
 CHECK(!invalidated_wall.stop_valid);

 // Conflict geometry is a pure route contract. The nearest route encounter
 // wins regardless of map-file order, and its braking stop line stays fixed
 // as the controller lookahead advances.
 std::vector<sim::RoutePoint> route;
 for (int x = 0; x <= 12; ++x) {
  route.push_back({{static_cast<double>(x), 0.0},
                   0.0,
                   x == 0 ? sim::TravelDirection::Unspecified : sim::TravelDirection::Forward});
 }
 const std::vector<sim::ConflictZone> zones{
 {"far", {9.5, -1.0}, {10.5, 1.0}, {}},
 {"near", {4.0, -1.0}, {6.0, 1.0}, {}},
 };
 const std::vector<std::vector<sim::DynamicObjectState>> open_frames(151, {{{50.0, 50.0}, {0.0, 0.0}, 0.4}});
 const auto approach =
 sim::evaluate_crossing(zones, route, 0, {2.0, 0.0}, 0.0, 0.0, open_frames, 0.1,
 {0.35, 0.25, 0.0}, crossing_caps);
 CHECK(approach.relevant && approach.zone_id == "near");
CHECK(approach.entry_index == 4 && approach.exit_index == 7);
CHECK(approach.stop_index == 3);
CHECK(std::abs(approach.stop_point.x - 3.3) < 1e-6);
CHECK(!approach.execution_trajectory.empty());
CHECK(std::abs(approach.execution_trajectory.front().p.x -
 approach.stop_point.x) < 1e-12);
CHECK(std::abs(approach.execution_trajectory.front().p.y -
 approach.stop_point.y) < 1e-12);
 // A far-away unrelated track is not evidence that this crossing is clear.
 CHECK(!approach.prediction_evidence_available);
 CHECK(approach.departure.reason != sim::DepartReason::kGoNow);

 // Once a controlled crossing has activated, its hold pose is a behavior
 // contract. A global replan may change raster vertices or briefly publish
 // no usable route, but neither event is an implicit permission to discard
 // the safe stop and derive a new tangent on the next cycle.
 const std::vector<sim::CrossingZoneSpec> replan_controlled_zones{
 {zones[1], sim::CrossingAuthority::Control},
 };
 sim::CrossingBehaviorSystem replanned_behavior;
 const auto original_hold = replanned_behavior.assess(
 replan_controlled_zones, route, 0, {2.0, 0.0}, 0.0, 0.0,
 open_frames, 0.1, {0.35, 0.25, 0.0}, crossing_caps, 0.05);
 CHECK(original_hold.control.hold_active);
 CHECK(original_hold.control.stop_valid);
 const auto latched_stop = original_hold.control.stop_point;
 const auto latched_direction = original_hold.control.stop_direction;

 std::vector<sim::RoutePoint> diagonal_replan{
 {{2.0, -2.0}, 0.0, sim::TravelDirection::Unspecified},
 {{4.5, -0.5}, 0.0, sim::TravelDirection::Forward},
 {{6.5, 0.5}, 0.0, sim::TravelDirection::Forward},
 {{9.0, 2.0}, 0.0, sim::TravelDirection::Forward},
 };
 const auto changed_route_hold = replanned_behavior.assess(
 replan_controlled_zones, diagonal_replan, 0, {2.0, 0.0}, 0.0, 0.0,
 open_frames, 0.1, {0.35, 0.25, 0.0}, crossing_caps, 0.05);
 CHECK(changed_route_hold.control.hold_active);
 CHECK(std::abs(changed_route_hold.control.stop_point.x - latched_stop.x) < 1e-12);
 CHECK(std::abs(changed_route_hold.control.stop_point.y - latched_stop.y) < 1e-12);
 CHECK(std::abs(changed_route_hold.control.stop_direction.x - latched_direction.x) < 1e-12);
 CHECK(std::abs(changed_route_hold.control.stop_direction.y - latched_direction.y) < 1e-12);

 const auto missing_route_hold = replanned_behavior.assess(
 replan_controlled_zones, {}, 0, {2.0, 0.0}, 0.0, 0.0,
 open_frames, 0.1, {0.35, 0.25, 0.0}, crossing_caps, 0.05);
 CHECK(missing_route_hold.assessment.relevant);
 CHECK(missing_route_hold.control_authority);
 CHECK(missing_route_hold.control.hold_active);
 CHECK(!missing_route_hold.safe_to_commit);
 CHECK(std::abs(missing_route_hold.control.stop_point.x - latched_stop.x) < 1e-12);
 CHECK(std::abs(missing_route_hold.control.stop_point.y - latched_stop.y) < 1e-12);

 // Removing the semantic zone is an explicit cancellation boundary.
 const auto removed_zone = replanned_behavior.assess(
 {}, {}, 0, {2.0, 0.0}, 0.0, 0.0,
 open_frames, 0.1, {0.35, 0.25, 0.0}, crossing_caps, 0.05);
 CHECK(!removed_zone.control.hold_active);
 CHECK(removed_zone.control.phase == sim::CrossingPhase::Inactive);

 // A future conflict is discovered from map semantics, but it must not
 // constrain ordinary navigation before the forecast-plus-braking preview.
 // Once the robot enters that generic route-distance window, the same zone
 // becomes relevant without any level-specific trigger point.
 std::vector<sim::RoutePoint> long_route;
 for (int x = 0; x <= 30; ++x)
 long_route.push_back({{static_cast<double>(x), 0.0}, 0.0,
 x == 0 ? sim::TravelDirection::Unspecified : sim::TravelDirection::Forward});
 const std::vector<sim::ConflictZone> distant_zone{
 {"distant", {19.5, -1.0}, {20.5, 1.0}, {}},
 };
 const std::vector<std::vector<sim::DynamicObjectState>> short_forecast(
 31, {{{50.0, 50.0}, {0.0, 0.0}, 0.4}});
 auto bounded_crossing_caps = crossing_caps;
 bounded_crossing_caps.activation_horizon_s = 3.0;
 const auto distant_inactive = sim::evaluate_crossing(
 distant_zone, long_route, 0, {0.0, 0.0}, 0.0, 0.0,
 short_forecast, 0.1, {0.35, 0.25, 0.0}, bounded_crossing_caps);
 CHECK(!distant_inactive.relevant);
 const auto distant_active = sim::evaluate_crossing(
 distant_zone, long_route, 16, {16.0, 0.0}, 0.0, 0.0,
 short_forecast, 0.1, {0.35, 0.25, 0.0}, bounded_crossing_caps);
 CHECK(distant_active.relevant);

 // Conflict relevance is a footprint/Minkowski-envelope query, not a
 // center-line intersection query. The semantic region already describes
 // the mapped shared space; ego footprint and policy clearance expand it at
 // runtime without inventing an unknown participant footprint in the map.
 std::vector<sim::RoutePoint> near_miss_route;
 for (int x = 0; x <= 10; ++x)
 near_miss_route.push_back({{static_cast<double>(x), 0.65}, 0.0,
 x == 0 ? sim::TravelDirection::Unspecified : sim::TravelDirection::Forward});
 const std::vector<sim::ConflictZone> compact_zone{
 {"compact", {4.9, -0.1}, {5.1, 0.1}, {}},
 };
 const auto near_miss = sim::evaluate_crossing(
 compact_zone, near_miss_route, 0, {1.0, 0.65}, 0.0, 0.0,
 open_frames, 0.1, {0.35, 0.25, 0.0}, crossing_caps);
 CHECK(near_miss.relevant);
 CHECK(near_miss.zone_id == "compact");

 // A finite conflict box alone is not a safe hold oracle. The stop pose must
 // also clear the authored incoming traffic corridor by the ego footprint,
 // policy clearance, controller tolerance and corridor half-width.
 sim::ConflictZone approached_zone{
 "approached", {16.9, 6.1}, {17.35, 6.5},
 {{{{16.9, 6.3}, {13.5, 6.3}}, 0.2}}};
 std::vector<sim::RoutePoint> diagonal_route;
 for (int step = 0; step <= 8; ++step) {
 const double offset = 0.5 * step;
 diagonal_route.push_back({{13.5 + offset, 2.5 + offset},
 sim::kPi / 4.0,
 step == 0 ? sim::TravelDirection::Unspecified : sim::TravelDirection::Forward});
 }
 const auto approach_reserved = sim::evaluate_crossing(
 {approached_zone}, diagonal_route, 0, diagonal_route.front().position,
 sim::kPi / 4.0, 0.0, open_frames, 0.1,
 {0.38, 0.75, 0.0}, crossing_caps);
 CHECK(approach_reserved.relevant);
 const double required_approach_distance_m =
 0.38 + 0.75 + 0.10 + 0.20;
 CHECK(6.3 - approach_reserved.stop_point.y + 1e-6 >=
 required_approach_distance_m);

 // The approach width is infrastructure geometry, while live participant
 // occupancy is time-indexed evidence. Changing the prediction length must
 // never relocate the route-derived stop line. The complete stationary wait
 // is assessed independently and is unsafe here because the wide actor can
 // block traversal without relocating the otherwise valid infrastructure
 // pose.
 std::vector<std::vector<sim::DynamicObjectState>> wide_actor_frames(
 151, {{{17.0, 6.3}, {0.0, 0.0}, 0.45, 0.10, true}});
 const auto dynamic_geometry_reference = sim::evaluate_crossing(
 {approached_zone}, diagonal_route, 0, diagonal_route.front().position,
 sim::kPi / 4.0, 0.0, open_frames, 0.1,
 {0.38, 0.25, 0.0, 0.35, 0.10, 0.12}, crossing_caps);
 const auto dynamically_reserved = sim::evaluate_crossing(
 {approached_zone}, diagonal_route, 0, diagonal_route.front().position,
 sim::kPi / 4.0, 0.0, wide_actor_frames, 0.1,
 {0.38, 0.25, 0.0, 0.35, 0.10, 0.12}, crossing_caps);
 CHECK(dynamically_reserved.relevant);
 CHECK(std::abs(dynamically_reserved.stop_point.x -
 dynamic_geometry_reference.stop_point.x) < 1e-9);
 CHECK(std::abs(dynamically_reserved.stop_point.y -
 dynamic_geometry_reference.stop_point.y) < 1e-9);
 CHECK(dynamically_reserved.departure.reason !=
 sim::DepartReason::kGoNow);

 const std::vector<std::vector<sim::DynamicObjectState>> short_wide_forecast(
 wide_actor_frames.begin(), wide_actor_frames.begin() + 13);
 const auto short_horizon_stop = sim::evaluate_crossing(
 {approached_zone}, diagonal_route, 0, diagonal_route.front().position,
 sim::kPi / 4.0, 0.0, short_wide_forecast, 0.1,
 {0.38, 0.25, 0.0, 0.35, 0.10, 0.12}, crossing_caps);
 CHECK(std::abs(short_horizon_stop.stop_point.x -
 dynamically_reserved.stop_point.x) < 1e-9);
 CHECK(std::abs(short_horizon_stop.stop_point.y -
 dynamically_reserved.stop_point.y) < 1e-9);

 // A cross-traffic participant can still overlap the zone's conservative
 // spatial envelope while moving decisively outwards. Admission is governed
 // by the same-time swept trajectory; the intent gate must not require that
 // duplicate envelope to clear before the ego trajectory even starts.
 std::vector<std::vector<sim::DynamicObjectState>> outward_frames;
 for (int frame = 0; frame <= 150; ++frame) {
 const double time_s = 0.1 * frame;
 outward_frames.push_back({{{5.0, 1.1 + time_s}, {0.0, 1.0}, 0.2, 0.0, true}});
 }
 const auto outward_crossing = sim::evaluate_crossing(
 zones, route, 0, {2.0, 0.0}, 0.0, 0.0, outward_frames, 0.1,
 {0.35, 0.25, 0.0}, crossing_caps);
 CHECK(outward_crossing.departure.reason == sim::DepartReason::kGoNow);
 CHECK(outward_crossing.prediction_models_ready);
 CHECK(outward_crossing.relevant_threats_moving_away);
 auto immature_outward_frames = outward_frames;
 for (auto& frame : immature_outward_frames)
 frame.front().prediction_ready = false;
 const auto immature_outward_crossing = sim::evaluate_crossing(
 zones, route, 0, {2.0, 0.0}, 0.0, 0.0,
 immature_outward_frames, 0.1, {0.35, 0.25, 0.0}, crossing_caps);
 CHECK(immature_outward_crossing.departure.reason == sim::DepartReason::kGoNow);
 CHECK(!immature_outward_crossing.prediction_models_ready);
 CHECK(!immature_outward_crossing.relevant_threats_moving_away);
 const auto tracked_outward_crossing = sim::evaluate_crossing(
 zones, route, 0, {2.0, 0.0}, 0.0, 0.0, outward_frames, 0.1,
 {0.35, 0.25, 0.0, 0.35, 0.20}, crossing_caps);
 CHECK(std::abs(tracked_outward_crossing.execution_required_gap_m - 0.45) < 1e-9);
 const auto advanced =
 sim::evaluate_crossing(zones, route, 3, {1.2, 0.4}, 0.0, 0.0, open_frames, 0.1,
 {0.35, 0.25, 0.0}, crossing_caps);
 CHECK(advanced.zone_id == "near" && advanced.stop_index == approach.stop_index);
 CHECK(advanced.stop_point.x == approach.stop_point.x && !advanced.stop_reached);
 const auto lookahead_far_ahead =
 sim::evaluate_crossing(zones, route, 12, {2.0, 0.0}, 0.0, 0.0, open_frames, 0.1,
 {0.35, 0.25, 0.0}, crossing_caps);
 CHECK(lookahead_far_ahead.relevant && lookahead_far_ahead.zone_id == "near");
 CHECK(lookahead_far_ahead.stop_index == approach.stop_index);
 const auto at_stop =
 sim::evaluate_crossing(zones, route, 3,
 {approach.stop_point.x + 0.05, approach.stop_point.y + 0.05},
 0.0, 0.0, open_frames, 0.1,
 {0.35, 0.25, 0.0}, crossing_caps);
 CHECK(at_stop.stop_reached);
 CHECK(approach.stop_direction.x == 1.0 && approach.stop_direction.y == 0.0);
CHECK(std::abs(approach.stop_distance_m - 1.3) < 1e-6);

 // A diagonal segment that starts inside the relevance envelope and ends at
 // the first physically clear vertex must use that endpoint as the refuge.
 // Skipping one more raster vertex made a small crossing exceed an otherwise
 // sufficient prediction horizon and caused a permanent HorizonShort hold.
 const sim::ConflictZone diagonal_exit_zone{
 "diagonal-exit", {25.3, 17.8}, {25.7, 18.0}, {}};
 const std::vector<sim::RoutePoint> diagonal_exit_route{
 {{24.25, 19.25}, 0.0, sim::TravelDirection::Unspecified},
 {{24.75, 18.75}, 0.0, sim::TravelDirection::Forward},
 {{25.25, 18.25}, 0.0, sim::TravelDirection::Forward},
 {{25.75, 17.75}, 0.0, sim::TravelDirection::Forward},
 {{26.25, 17.25}, 0.0, sim::TravelDirection::Forward},
 {{26.75, 16.75}, 0.0, sim::TravelDirection::Forward},
 };
 auto diagonal_exit_caps = crossing_caps;
 diagonal_exit_caps.preferred_speed_mps = 0.7;
 diagonal_exit_caps.max_speed_mps = 1.0;
 const auto diagonal_exit_probe = sim::evaluate_crossing(
 {diagonal_exit_zone}, diagonal_exit_route, 0,
 diagonal_exit_route.front().position, -sim::kPi / 4.0, 0.0,
 open_frames, 0.1, {0.40, 0.032, 0.03, 0.35, 0.0},
 diagonal_exit_caps);
 const auto diagonal_exit = sim::evaluate_crossing(
 {diagonal_exit_zone}, diagonal_exit_route, 0,
 diagonal_exit_probe.stop_point, -sim::kPi / 4.0, 0.0,
 open_frames, 0.1, {0.40, 0.032, 0.03, 0.35, 0.0},
 diagonal_exit_caps);
 CHECK(diagonal_exit.relevant);
 CHECK(diagonal_exit.exit_index == 4);
 CHECK(diagonal_exit.exit_point.x == 26.25 && diagonal_exit.exit_point.y == 17.25);
 CHECK(diagonal_exit.execution_duration_s < 3.5);
 const std::vector<std::vector<sim::DynamicObjectState>> blocked_frames(151, {{{5.0, 0.0}, {0.0, 0.0}, 0.4}});
 const auto blocked_crossing =
 sim::evaluate_crossing(zones, route, 0, {0.0, 0.0}, 0.0, 0.0, blocked_frames, 0.1,
 {0.35, 0.25, 0.0}, crossing_caps);
 CHECK(blocked_crossing.departure.reason != sim::DepartReason::kGoNow);

 // Prediction silence is an unknown observation state, not a clear road.
 // The fixed-step predictor still returns timestamped empty frames when no
 // mature track exists; neither the route assessor nor the stateful behavior
 // may convert those frames into crossing permission.
 const std::vector<std::vector<sim::DynamicObjectState>> unknown_frames(151);
 const auto unknown_crossing =
 sim::evaluate_crossing(zones, route, 0, {2.0, 0.0}, 0.0, 0.0, unknown_frames, 0.1,
 {0.35, 0.25, 0.0}, crossing_caps);
 CHECK(!unknown_crossing.prediction_evidence_available);
 CHECK(unknown_crossing.departure.reason == sim::DepartReason::kInvalid);
 sim::CrossingBehaviorSystem unknown_behavior;
 const std::vector<sim::CrossingZoneSpec> controlled_zones{
 {{"near", {4.0, -1.0}, {6.0, 1.0}, {}}, sim::CrossingAuthority::Control},
 };
 for (int sample = 0; sample < 20; ++sample) {
 const auto decision = unknown_behavior.assess(
 controlled_zones, route, 0, {2.0, 0.0}, 0.0, 0.0,
 unknown_frames, 0.1, {0.35, 0.25, 0.20, 0.35}, crossing_caps, 0.05);
 CHECK(decision.control.hold_active);
 CHECK(!decision.control.committed);
 CHECK(!decision.safe_to_commit);
 }

 // Positive free-space evidence is a separate admission source. A physical
 // return before the traffic approach leaves it occluded, while dense rays
 // that extend through the complete authored approach prove it clear. The
 // same LiDAR frame cannot be counted twice by a faster decision loop.
 sim::ConflictZone observed_zone;
 observed_zone.id = "observed";
 observed_zone.min = {4.0, -0.5};
 observed_zone.max = {5.0, 0.5};
 sim::CrossingObservationApproach observed_approach;
 observed_approach.centerline = {{4.5, 0.5}, {4.5, 5.5}};
 observed_approach.half_width_m = 0.15;
 observed_zone.observation_approaches.push_back(observed_approach);

 // A tracked participant already on the mapped approach remains relevant
 // while moving away even when its constant-velocity forecast never touches
 // the conflict box. This is positive target evidence, unlike an unrelated
 // track elsewhere in the map, and the complete same-time trajectory is
 // still required to clear before admission.
 std::vector<std::vector<sim::DynamicObjectState>> departing_approach_frames;
 for (int frame = 0; frame <= 150; ++frame) {
 const double time_s = 0.1 * frame;
 departing_approach_frames.push_back({{
 {4.5, 3.0 + time_s}, {0.0, 1.0}, 0.25, 0.0, true,
 }});
 }
 const auto departing_approach = sim::evaluate_crossing(
 {observed_zone}, route, 0, {2.0, 0.0}, 0.0, 0.0,
 departing_approach_frames, 0.1, {0.35, 0.25, 0.0}, crossing_caps);
 CHECK(departing_approach.prediction_evidence_available);
 CHECK(departing_approach.relevant_threats_moving_away);
 CHECK(departing_approach.departure.reason == sim::DepartReason::kGoNow);

 std::vector<sim::ObservedRay2D> occluded_rays;
 for (int ray = 0; ray < 720; ++ray) {
 const double angle = -sim::kPi + 2.0 * sim::kPi *
 static_cast<double>(ray) / 720.0;
 occluded_rays.push_back({{2.0, 0.0},
 {2.0 + std::cos(angle), std::sin(angle)}, false, true,
 2.0 * sim::kPi / 720.0});
 }
 const auto occluded = sim::evaluate_crossing(
 {observed_zone}, route, 0, {2.0, 0.0}, 0.0, 0.0,
 unknown_frames, 0.1, {0.35, 0.25, 0.0}, crossing_caps,
 occluded_rays, 1.0);
 CHECK(occluded.free_space_evidence_available);
 CHECK(!occluded.observed_approaches_clear);
 CHECK(occluded.departure.reason != sim::DepartReason::kGoNow);

 // The presence of an unrelated track must not suppress the occlusion-aware
 // free-space gate or turn that remote object into crossing-clear evidence.
 const auto unrelated_track_occluded = sim::evaluate_crossing(
 {observed_zone}, route, 0, {2.0, 0.0}, 0.0, 0.0,
 open_frames, 0.1, {0.35, 0.25, 0.0}, crossing_caps,
 occluded_rays, 1.0);
 CHECK(!unrelated_track_occluded.prediction_evidence_available);
 CHECK(unrelated_track_occluded.free_space_evidence_available);
 CHECK(!unrelated_track_occluded.observed_approaches_clear);
 CHECK(unrelated_track_occluded.departure.reason != sim::DepartReason::kGoNow);

 std::vector<sim::ObservedRay2D> clear_rays;
 for (int ray = 0; ray < 1440; ++ray) {
 const double angle = -sim::kPi + 2.0 * sim::kPi *
 static_cast<double>(ray) / 1440.0;
 clear_rays.push_back({{2.0, 0.0},
 {2.0 + 20.0 * std::cos(angle), 20.0 * std::sin(angle)},
 false, false, 2.0 * sim::kPi / 1440.0});
 }
 const auto observed_clear = sim::evaluate_crossing(
 {observed_zone}, route, 0, {2.0, 0.0}, 0.0, 0.0,
 unknown_frames, 0.1, {0.35, 0.25, 0.0}, crossing_caps,
 clear_rays, 2.0);
 CHECK(observed_clear.free_space_evidence_available);
 CHECK(!observed_clear.observed_approaches_clear);
 CHECK(observed_clear.observation_coverage == 1.0);
 CHECK(observed_clear.departure.reason != sim::DepartReason::kGoNow);

 // A neighboring no-return ray is only angular sampling support. The direct
 // ray stops at the actor, so the point behind that physical return remains
 // occluded and geometric coverage still cannot grant admission.
 const sim::Vec2 actor_centre{4.5, 2.5};
 std::vector<sim::ObservedRay2D> grazing_actor_rays;
 for (int ray = 0; ray < 1440; ++ray) {
 const double angle = -sim::kPi + 2.0 * sim::kPi *
 static_cast<double>(ray) / 1440.0;
 const sim::Vec2 endpoint{
 2.0 + 20.0 * std::cos(angle), 20.0 * std::sin(angle)};
 const sim::Vec2 direction{std::cos(angle), std::sin(angle)};
 const sim::Vec2 to_actor{actor_centre.x - 2.0, actor_centre.y};
 const double along = to_actor.x * direction.x + to_actor.y * direction.y;
 const double cross = std::abs(
 to_actor.x * direction.y - to_actor.y * direction.x);
 if (along > 0.0 && cross < 0.08)
 continue;
 grazing_actor_rays.push_back({{2.0, 0.0}, endpoint, false, false,
 2.0 * sim::kPi / 1440.0});
 }
 const double actor_angle = std::atan2(actor_centre.y, actor_centre.x - 2.0);
 const double actor_range = std::hypot(actor_centre.x - 2.0, actor_centre.y);
 grazing_actor_rays.push_back({
 {2.0, 0.0},
 {2.0 + (actor_range - 0.05) * std::cos(actor_angle),
 (actor_range - 0.05) * std::sin(actor_angle)},
 true, true, 2.0 * sim::kPi / 1440.0});
 const auto occupied_approach_result = sim::evaluate_crossing(
 {observed_zone}, route, 0, {2.0, 0.0}, 0.0, 0.0,
 unknown_frames, 0.1, {0.35, 0.25, 0.0}, crossing_caps,
 grazing_actor_rays, 2.0);
 CHECK(occupied_approach_result.free_space_evidence_available);
 CHECK(!occupied_approach_result.observed_approaches_clear);
 CHECK(occupied_approach_result.departure.reason != sim::DepartReason::kGoNow);

 // A no-return ray is free only as far as the sensor range. Three metres do
 // not cover this complete approach from the current stop pose, so a short
 // LiDAR cannot turn the unseen tail into a clear-road assertion.
 std::vector<sim::ObservedRay2D> short_rays;
 for (int ray = 0; ray < 1440; ++ray) {
 const double angle = -sim::kPi + 2.0 * sim::kPi *
 static_cast<double>(ray) / 1440.0;
 short_rays.push_back({{2.0, 0.0},
 {2.0 + 3.0 * std::cos(angle), 3.0 * std::sin(angle)},
 false, false, 2.0 * sim::kPi / 1440.0});
 }
 const auto short_range = sim::evaluate_crossing(
 {observed_zone}, route, 0, {2.0, 0.0}, 0.0, 0.0,
 unknown_frames, 0.1, {0.35, 0.25, 0.0}, crossing_caps,
 short_rays, 2.0, 2.0);
 CHECK(short_range.free_space_evidence_available);
 CHECK(!short_range.observed_approaches_clear);
 CHECK(short_range.observation_coverage < 1.0);
 CHECK(short_range.departure.reason != sim::DepartReason::kGoNow);

 // Geometric coverage remains useful diagnostic evidence, but neither a
 // fresh nor an old scan can invent a physical speed bound for an unseen
 // real-world participant.
 const auto aged_scan = sim::evaluate_crossing(
 {observed_zone}, route, 0, {2.0, 0.0}, 0.0, 0.0,
 unknown_frames, 0.1, {0.35, 0.25, 0.0}, crossing_caps,
 clear_rays, 2.0, 9.0);
 CHECK(aged_scan.free_space_evidence_available);
 CHECK(aged_scan.observation_coverage == 1.0);
 CHECK(!aged_scan.observed_approaches_clear);
 CHECK(aged_scan.departure.reason != sim::DepartReason::kGoNow);

 sim::CrossingBehaviorSystem observed_behavior;
 const std::vector<sim::CrossingZoneSpec> observed_controlled{
 {observed_zone, sim::CrossingAuthority::Control},
 };
 const auto observed_stop = observed_clear.stop_point;
 auto one_scan = observed_behavior.assess(
 observed_controlled, route, 0, observed_stop, 0.0, 0.0,
 unknown_frames, 0.1, {0.35, 0.25, 0.0, 0.35}, crossing_caps,
 0.05, clear_rays, 2.0);
 CHECK(one_scan.control.hold_active && !one_scan.control.committed);
 one_scan = observed_behavior.assess(
 observed_controlled, route, 0, observed_stop, 0.0, 0.0,
 unknown_frames, 0.1, {0.35, 0.25, 0.0, 0.35}, crossing_caps,
 0.05, clear_rays, 2.0);
 CHECK(one_scan.control.hold_active && !one_scan.control.committed);
 const auto second_scan = observed_behavior.assess(
 observed_controlled, route, 0, observed_stop, 0.0, 0.0,
 unknown_frames, 0.1, {0.35, 0.25, 0.0, 0.35}, crossing_caps,
 0.05, clear_rays, 2.1);
 CHECK(second_scan.control.hold_active);
 CHECK(!second_scan.control.committed);

 const auto cleared =
 sim::evaluate_crossing(zones, route, 8, {8.0, 0.0}, 0.0, 0.0, open_frames, 0.1,
 {0.35, 0.25, 0.0}, crossing_caps);
 CHECK(cleared.relevant && cleared.zone_id == "far");
 const auto index_ahead_but_body_inside =
 sim::evaluate_crossing(zones, route, 8, {5.0, 0.0}, 0.0, 0.0, open_frames, 0.1,
 {0.35, 0.25, 0.0}, crossing_caps);
 CHECK(index_ahead_but_body_inside.relevant);
 CHECK(index_ahead_but_body_inside.zone_id == "near" && index_ahead_but_body_inside.body_in_conflict);

 // The readiness gate is behavior logic, independent of mission content.
 // Position is one-sided: arriving slightly before the line is allowed by
 // the authored tolerance, but speed and body heading remain mandatory.
 const sim::CrossingSafetyEnvelope readiness_safety{0.35, 0.25, 0.20, 0.35};
 const auto ready = sim::assess_crossing_readiness(
 {1.75, 2.0}, 0.05, 0.0, {2.0, 2.0}, {2.0, 0.0}, readiness_safety);
 CHECK(ready.position_ready && ready.speed_ready && ready.heading_ready && ready.ready());
 CHECK(!ready.position_settled);
 CHECK(std::abs(ready.stop_longitudinal_m + 0.25) < 1e-12);
 const auto moving = sim::assess_crossing_readiness(
 {1.75, 2.0}, 0.11, 0.0, {2.0, 2.0}, {1.0, 0.0}, readiness_safety);
 CHECK(moving.position_ready && !moving.speed_ready && !moving.ready());
 const auto misaligned = sim::assess_crossing_readiness(
 {1.75, 2.0}, 0.0, 0.36, {2.0, 2.0}, {1.0, 0.0}, readiness_safety);
 CHECK(!misaligned.heading_ready && !misaligned.ready());
 const auto tightly_settled = sim::assess_crossing_readiness(
 {1.95, 2.0}, 0.0, 0.0, {2.0, 2.0}, {1.0, 0.0}, readiness_safety);
 CHECK(tightly_settled.position_settled && tightly_settled.position_ready);

 const auto forward_pose = sim::crossing_pose_approach(
 {0.0, 0.0}, 0.0, {1.0, 0.0}, {1.0, 0.0}, false, 0.0, 0.05);
 CHECK(!forward_pose.reverse && std::abs(forward_pose.body_heading_rad) < 1e-12);
 const auto overshot_pose = sim::crossing_pose_approach(
 {1.06, 0.0}, 0.0, {1.0, 0.0}, {1.0, 0.0}, false, 0.0, 0.05);
 CHECK(overshot_pose.reverse && std::abs(overshot_pose.body_heading_rad) < 1e-12);
 const auto forced_reverse_pose = sim::crossing_pose_approach(
 {0.0, 1.0}, sim::kPi * 0.5, {0.0, 0.0}, {1.0, 0.0}, true, 0.12, 0.05);
 CHECK(forced_reverse_pose.reverse);
 CHECK(std::abs(forced_reverse_pose.body_heading_rad) < 1e-12);
 const auto offset_axle_acquired = sim::crossing_pose_approach(
 {0.0, 0.0}, sim::kPi * 0.5, {-0.12, 0.12}, {1.0, 0.0}, false,
 0.12, 0.01);
 CHECK(offset_axle_acquired.control_position_acquired);
 CHECK(std::abs(offset_axle_acquired.body_heading_rad) < 1e-12);

 // Final stop-heading alignment starts only inside an inner acquisition
 // radius, then remains active through measured motion inside the outer
 // behavior tolerance. Crossing the outer bound returns to pose approach.
 sim::CrossingState pose_latch;
 sim::CrossingInput pose_input;
 pose_input.relevant = true;
 pose_input.position_ready = true;
 pose_input.position_settled = true;
 auto pose_output = sim::govern_crossing(pose_latch, pose_input);
 CHECK(pose_output.hold_position_acquired);
 pose_input.position_settled = false;
 pose_output = sim::govern_crossing(pose_latch, pose_input);
 CHECK(pose_output.hold_position_acquired);
 pose_input.position_ready = false;
 pose_output = sim::govern_crossing(pose_latch, pose_input);
 CHECK(!pose_output.hold_position_acquired);

 // A semantic crossing holds at a stable stop point, requires a sustained
 // safe window, then keeps its commitment until the route clears the zone.
 sim::CrossingState crossing;
 sim::CrossingInput blocked;
 blocked.relevant = true;
 blocked.stop_point = {5.0, 2.0};
 blocked.stop_valid = true;
 blocked.exit_point = {8.0, 2.0};
 blocked.exit_valid = true;
 blocked.transit_speed_mps = 0.7;
 blocked.dt_s = 0.05;
 auto crossing_out = sim::govern_crossing(crossing, blocked);
 CHECK(crossing_out.hold_active && crossing_out.stop_valid);
 CHECK(crossing_out.stop_point.x == 5.0 && crossing_out.stop_point.y == 2.0);
 sim::CrossingInput moved_stop = blocked;
 moved_stop.stop_point = {6.0, 2.0};
 crossing_out = sim::govern_crossing(crossing, moved_stop);
 CHECK(crossing_out.stop_point.x == 5.0 && crossing_out.stop_point.y == 2.0);
 sim::CrossingInput crossing_clear = blocked;
 crossing_clear.safe_to_commit = true;
 // A clear prediction before the body settles at the stop line cannot grant
 // entry and cannot accumulate confirmation time.
 for (int sample = 0; sample < 10; ++sample)
  CHECK(sim::govern_crossing(crossing, crossing_clear).hold_active);
 crossing_clear.ready_to_commit = true;
 const int confirming_samples = sim::kCrossingCommitConfirmSamples;
 for (int sample = 0; sample + 1 < confirming_samples; ++sample) {
  crossing_out = sim::govern_crossing(crossing, crossing_clear);
  CHECK(crossing_out.hold_active && !crossing_out.committed);
 }
 crossing_out = sim::govern_crossing(crossing, crossing_clear);
 CHECK(!crossing_out.hold_active && crossing_out.committed);
 CHECK(std::abs(crossing_out.execution_elapsed_s) < 1e-12);
 CHECK(crossing_out.exit_valid && crossing_out.exit_point.x == 8.0);
 CHECK(std::abs(crossing_out.transit_speed_mps - 0.7) < 1e-12);
 // Initial alignment is a one-shot part of the committed execution. Once
 // the local tracker reports it complete, later heading error at a planned
 // stop-turn-go corner cannot re-arm the initial-alignment latch.
 crossing.execution_alignment_required = true;
 crossing.execution_alignment_complete = false;
 sim::CrossingInput alignment_complete = blocked;
 alignment_complete.execution_alignment_complete = true;
 crossing_out = sim::govern_crossing(crossing, alignment_complete);
 CHECK(crossing_out.committed && crossing_out.execution_alignment_complete);
// Alignment is outside the admitted translation clock. The first completed
// report arms that clock; only the following cycle may consume it.
CHECK(std::abs(crossing_out.execution_elapsed_s) < 1e-12);
 sim::CrossingInput later_heading_error = blocked;
 later_heading_error.execution_alignment_complete = false;
 crossing_out = sim::govern_crossing(crossing, later_heading_error);
 CHECK(crossing_out.committed && crossing_out.execution_alignment_complete);
CHECK(std::abs(crossing_out.execution_elapsed_s - blocked.dt_s) < 1e-12);

 // A planned zero-speed turn is not evidence that admission became unsafe.
 // Revocation requires independent local-validator rejection as well as a
 // physical stop before conflict entry.
 sim::CrossingInput planned_turn = blocked;
 planned_turn.stopped_before_conflict = true;
 crossing_out = sim::govern_crossing(crossing, planned_turn);
 CHECK(crossing_out.committed && !crossing_out.hold_active);
 // If the old timed proof is rejected but a fresh continuation from the
 // measured pose is safe, latch that replacement without replaying the old
 // stop-to-current prefix or reversing to the historical stop pose.
 sim::CrossingInput replanned = blocked;
 replanned.committed_execution_blocked = true;
 replanned.safe_to_commit = true;
 replanned.transit_speed_mps = 0.8;
 replanned.execution_trajectory = {
 {0.0, {6.0, 2.0}}, {1.0, {7.0, 2.0}}, {2.0, {8.0, 2.0}},
 };
 replanned.execution_continuous = true;
 replanned.tracking_corridor_m = 0.2;
 replanned.execution_initial_heading_rad = 0.0;
 replanned.execution_alignment_complete = true;
 crossing_out = sim::govern_crossing(crossing, replanned);
 CHECK(crossing_out.committed && !crossing_out.recovery_to_stop);
 CHECK(crossing_out.execution_trajectory.size() == 3);
 CHECK(crossing_out.execution_trajectory.front().p.x == 6.0);
 CHECK(crossing_out.execution_continuous);
 CHECK(std::abs(crossing_out.execution_elapsed_s) < 1e-12);
 CHECK(std::abs(crossing_out.transit_speed_mps - 0.8) < 1e-12);
 // A moving replacement that first requires a new in-place alignment is not
 // a continuous continuation of the admitted maneuver. Keep the current
 // commitment until the local layer stops safely, then re-enter the normal
 // hold/confirmation gate instead of turning in the approach envelope.
 const auto replacement_trajectory = crossing_out.execution_trajectory;
 sim::CrossingInput discontinuous_replan = replanned;
 discontinuous_replan.execution_trajectory = {
 {0.0, {6.1, 2.0}}, {1.0, {6.1, 3.0}}, {2.0, {8.0, 3.0}},
 };
 discontinuous_replan.execution_alignment_required = true;
 discontinuous_replan.execution_alignment_complete = false;
 crossing_out = sim::govern_crossing(crossing, discontinuous_replan);
 CHECK(crossing_out.committed && !crossing_out.recovery_to_stop);
CHECK(crossing_out.execution_trajectory.size() == replacement_trajectory.size());
CHECK(crossing_out.execution_trajectory.front().p.x ==
 replacement_trajectory.front().p.x);
 sim::CrossingInput blocked_but_moving = blocked;
 blocked_but_moving.committed_execution_blocked = true;
 crossing_out = sim::govern_crossing(crossing, blocked_but_moving);
 CHECK(crossing_out.committed && !crossing_out.hold_active);
 // If a changed forecast safely stops the body before physical entry, the
 // commitment is revoked into a named reverse recovery. Reaching the
 // original route-derived hold pose clears that recovery before a new
 // admission can accumulate.
 sim::CrossingInput revoked = blocked;
 revoked.stopped_before_conflict = true;
 revoked.committed_execution_blocked = true;
 revoked.stop_longitudinal_m = 0.11;
 crossing_out = sim::govern_crossing(crossing, revoked);
 CHECK(crossing_out.hold_active && !crossing_out.committed);
 CHECK(std::abs(crossing_out.execution_elapsed_s) < 1e-12);
 CHECK(crossing_out.recovery_to_stop);
 CHECK(crossing_out.recovery_reverse);
 // Motion around the switching plane remains in the latched gear. Only
 // crossing the opposite hysteresis boundary changes recovery direction.
 sim::CrossingInput recovery_noise = blocked;
 recovery_noise.stop_longitudinal_m = 0.09;
 crossing_out = sim::govern_crossing(crossing, recovery_noise);
 CHECK(crossing_out.recovery_reverse);
 sim::CrossingInput recovered_upstream = blocked;
 recovered_upstream.stop_longitudinal_m = -0.11;
 crossing_out = sim::govern_crossing(crossing, recovered_upstream);
 CHECK(crossing_out.recovery_to_stop && !crossing_out.recovery_reverse);
 sim::CrossingInput recovered = blocked;
 recovered.ready_to_commit = true;
 crossing_out = sim::govern_crossing(crossing, recovered);
 CHECK(crossing_out.hold_active && !crossing_out.recovery_to_stop);
 for (int sample = 0; sample < confirming_samples; ++sample)
 crossing_out = sim::govern_crossing(crossing, crossing_clear);
 CHECK(crossing_out.committed);
 CHECK(std::abs(crossing_out.execution_elapsed_s) < 1e-12);
 // Crossing the no-return line is latched for the lifetime of this
 // commitment. A later local block must not send the body reversing through
 // cross traffic toward the hold pose.
 sim::CrossingInput entered = blocked;
entered.no_return_started = true;
 crossing_out = sim::govern_crossing(crossing, entered);
 CHECK(crossing_out.committed && crossing_out.no_return_started);
 sim::CrossingInput blocked_after_entry = blocked;
 blocked_after_entry.committed_execution_blocked = true;
 blocked_after_entry.stopped_before_conflict = true;
 crossing_out = sim::govern_crossing(crossing, blocked_after_entry);
 CHECK(crossing_out.committed && !crossing_out.hold_active);
 CHECK(crossing_out.no_return_started && !crossing_out.recovery_to_stop);

 // A named contingency remains committed while it is still carrying the
 // body out of the approach envelope. Once the body reaches a lateral or
 // upstream refuge, the obsolete timed crossing schedule is revoked and the
 // ordinary stop/admission contract takes over again. A normal downstream
 // traversal uses body_cleared instead and never enters this branch.
 auto contingency_crossing = crossing;
 sim::CrossingInput escaping = blocked;
 escaping.no_return_started = true;
 escaping.committed_contingency_escape = true;
 escaping.ego_position = {7.0, -2.0};
 auto contingency_out = sim::govern_crossing(contingency_crossing, escaping);
 CHECK(contingency_out.committed && contingency_out.no_return_started);
 sim::CrossingInput reached_refuge = blocked;
 reached_refuge.no_return_started = false;
 reached_refuge.stopped_before_conflict = true;
 reached_refuge.ego_position = {7.0, -2.0};
 reached_refuge.stop_longitudinal_m = 0.5;
 contingency_out = sim::govern_crossing(contingency_crossing, reached_refuge);
 CHECK(contingency_out.hold_active && !contingency_out.committed);
CHECK(!contingency_out.recovery_to_stop && !contingency_out.recovery_reverse);
 CHECK(contingency_out.relocated_hold);
CHECK(contingency_out.stop_point.x == 7.0 && contingency_out.stop_point.y == -2.0);

 // Admission time includes physical yaw acceleration, rather than assuming
 // an instantaneous jump to max yaw rate. A slower angular actuator must
 // therefore require a longer predicted crossing for the same route.
 const auto fast_yaw = sim::evaluate_crossing(
 zones, route, 0, {2.0, 0.0}, sim::kPi / 2.0, 0.0, open_frames, 0.1,
 {0.35, 0.25, 0.0}, {1.0, 2.0, 2.0, 2.0, 2.0, 8.0});
 const auto slow_yaw = sim::evaluate_crossing(
 zones, route, 0, {2.0, 0.0}, sim::kPi / 2.0, 0.0, open_frames, 0.1,
 {0.35, 0.25, 0.0}, {1.0, 2.0, 2.0, 2.0, 2.0, 0.5});
 CHECK(!fast_yaw.execution_trajectory.empty());
 CHECK(!slow_yaw.execution_trajectory.empty());
 CHECK(slow_yaw.execution_duration_s > fast_yaw.execution_duration_s + 1.0);

 // A sharp route corner is represented as stop-turn-go. The timed path
 // reaches the corner, holds the same position for the finite yaw maneuver,
 // and only then starts the next leg. This makes the behavior proof
 // executable by a differential base and exposes the zero-speed boundary.
 const std::vector<sim::RoutePoint> corner_route{
 {{0.0, 0.0}, 0.0, sim::TravelDirection::Unspecified},
 {{2.0, 0.0}, 0.0, sim::TravelDirection::Forward},
 {{4.0, 0.0}, 0.0, sim::TravelDirection::Forward},
 {{5.0, 0.0}, 0.0, sim::TravelDirection::Forward},
 {{5.0, 1.0}, sim::kPi / 2.0, sim::TravelDirection::Forward},
 {{5.0, 3.0}, sim::kPi / 2.0, sim::TravelDirection::Forward},
 };
 const std::vector<sim::ConflictZone> corner_zone{
 {"corner", {4.8, 1.8}, {5.2, 2.2}, {}},
 };
 const auto corner_crossing = sim::evaluate_crossing(
 corner_zone, corner_route, 0, {2.0, 0.0}, 0.0, 0.0,
 open_frames, 0.1, {0.35, 0.25, 0.0}, crossing_caps);
 CHECK(!corner_crossing.execution_trajectory.empty());
 bool held_at_corner = false;
 for (std::size_t index = 1;
 index < corner_crossing.execution_trajectory.size(); ++index) {
 const auto& previous = corner_crossing.execution_trajectory[index - 1];
 const auto& current = corner_crossing.execution_trajectory[index];
 if (std::hypot(previous.p.x - 5.0, previous.p.y) < 1e-9 &&
 std::hypot(current.p.x - 5.0, current.p.y) < 1e-9 &&
 current.t_s > previous.t_s + 0.1) {
 held_at_corner = true;
 break;
 }
 }
 CHECK(held_at_corner);

 // Raster corners may be removed only through the deployable map's
 // independent line-feasibility contract. A rejected chord must reproduce
 // the unpruned timed proof, while an accepted chord removes unnecessary
 // stop-turn-go occupancy from an otherwise open crossing.
 const std::vector<sim::RoutePoint> raster_route{
 {{0.0, 0.0}, 0.0, sim::TravelDirection::Unspecified},
 {{2.0, 0.0}, 0.0, sim::TravelDirection::Forward},
 {{3.0, 0.0}, 0.0, sim::TravelDirection::Forward},
 {{4.0, 0.4}, 0.0, sim::TravelDirection::Forward},
 {{4.5, -0.4}, 0.0, sim::TravelDirection::Forward},
 {{5.0, 0.4}, 0.0, sim::TravelDirection::Forward},
 {{5.5, -0.4}, 0.0, sim::TravelDirection::Forward},
 {{6.0, 0.4}, 0.0, sim::TravelDirection::Forward},
 {{7.0, 0.0}, 0.0, sim::TravelDirection::Forward},
 };
 const std::vector<sim::ConflictZone> raster_zone{
 {"raster", {4.9, -0.1}, {5.1, 0.1},
 {{{{4.9, 0.0}, {3.0, 0.0}}, 0.1}}},
 };
 const auto raster_probe = sim::evaluate_crossing(
 raster_zone, raster_route, 0, {3.0, 0.0}, 0.0, 0.0,
 open_frames, 0.1, {0.35, 0.25, 0.0}, crossing_caps);
 CHECK(raster_probe.relevant);
 const auto rejected_shortcut = sim::evaluate_crossing(
 raster_zone, raster_route, raster_probe.stop_index,
 raster_probe.stop_point, 0.0, 0.0,
 open_frames, 0.1, {0.35, 0.25, 0.0}, crossing_caps,
 {}, -std::numeric_limits<double>::infinity(),
 -std::numeric_limits<double>::infinity(), false,
 [](sim::Vec2 from, sim::Vec2 to) {
 return std::hypot(to.x - from.x, to.y - from.y) <= 1.2;
 });
 const auto unpruned_shortcut = sim::evaluate_crossing(
 raster_zone, raster_route, raster_probe.stop_index,
 raster_probe.stop_point, 0.0, 0.0,
 open_frames, 0.1, {0.35, 0.25, 0.0}, crossing_caps);
 const auto accepted_shortcut = sim::evaluate_crossing(
 raster_zone, raster_route, raster_probe.stop_index,
 raster_probe.stop_point, 0.0, 0.0,
 open_frames, 0.1, {0.35, 0.25, 0.0}, crossing_caps,
 {}, -std::numeric_limits<double>::infinity(),
 -std::numeric_limits<double>::infinity(), false,
 [](sim::Vec2, sim::Vec2) { return true; });
 CHECK(!rejected_shortcut.execution_trajectory.empty());
 CHECK(!unpruned_shortcut.execution_trajectory.empty());
 CHECK(!accepted_shortcut.execution_trajectory.empty());
 CHECK(std::abs(rejected_shortcut.execution_duration_s -
 unpruned_shortcut.execution_duration_s) < 1e-12);
 CHECK(accepted_shortcut.execution_duration_s + 0.5 <
 rejected_shortcut.execution_duration_s);
 const auto blocked_canonical = sim::evaluate_crossing(
 raster_zone, raster_route, raster_probe.stop_index,
 raster_probe.stop_point, 0.0, 0.0,
 open_frames, 0.1, {0.35, 0.25, 0.0}, crossing_caps,
 {}, -std::numeric_limits<double>::infinity(),
 -std::numeric_limits<double>::infinity(), false,
 [](sim::Vec2, sim::Vec2) { return false; });
 CHECK(blocked_canonical.execution_trajectory.empty());
 CHECK(blocked_canonical.departure.reason != sim::DepartReason::kGoNow);

 // A conflict exit is a control-handoff boundary, not a docking pose. The
 // final samples therefore retain substantial forward progress; a future
 // stop request is checked independently by the local validator's physical
 // braking tail.
 CHECK(approach.execution_trajectory.size() >= 2);
 const auto& final_pose = approach.execution_trajectory.back();
 const auto& prior_pose = approach.execution_trajectory[
 approach.execution_trajectory.size() - 2];
 const double final_sample_speed = std::hypot(
 final_pose.p.x - prior_pose.p.x, final_pose.p.y - prior_pose.p.y) /
 (final_pose.t_s - prior_pose.t_s);
 CHECK(final_sample_speed > 0.5 * approach.transit_speed_mps);

 sim::CrossingInput done = blocked;
 done.relevant = false;
 done.body_cleared = true;
 crossing_out = sim::govern_crossing(crossing, done);
 CHECK(crossing_out.phase == sim::CrossingPhase::Cleared);
 CHECK(!crossing_out.hold_active && !crossing_out.committed);
 crossing_out = sim::govern_crossing(crossing, done);
 CHECK(crossing_out.phase == sim::CrossingPhase::Inactive);
 std::cout << "entry governor green\n";
 return 0;
}
