#include "robot/autonomy/behavior/entry_governor.hpp"
#include "robot/autonomy/behavior/crossing_behavior.hpp"

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
 const sim::CrossingExecutionCaps crossing_caps{1.0, 2.0, 2.0, 2.0, 2.0};
 sim::GovernorState state;
 sim::GovernorInput idle;
 // Silent start: no hold, and silence never engages one.
 for (int i = 0; i < 100; ++i)
  CHECK(!govern_entry(state, idle).hold_active);
 // Occupied engages on the first fresh frame.
 sim::GovernorInput occ;
 occ.occupied_fresh = true;
 occ.stop_point = {3.0, 4.0};
 occ.stop_valid = true;
 const auto held = govern_entry(state, occ);
 CHECK(held.hold_active && held.stop_valid);
 CHECK(held.stop_point.x == 3.0 && held.stop_point.y == 4.0);
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
   {"far", {9.5, -1.0}, {10.5, 1.0}},
   {"near", {4.0, -1.0}, {6.0, 1.0}},
 };
 const std::vector<std::vector<sim::DynamicObjectState>> open_frames(151, {{{50.0, 50.0}, {0.0, 0.0}, 0.4}});
 const auto approach =
 sim::evaluate_crossing(zones, route, 0, {2.0, 0.0}, 0.0, 0.0, open_frames, 0.1,
 {0.35, 0.25, 0.0}, crossing_caps);
 CHECK(approach.relevant && approach.zone_id == "near");
CHECK(approach.entry_index == 4 && approach.exit_index == 7);
CHECK(approach.stop_index == 3);
CHECK(std::abs(approach.stop_point.x - 3.3) < 1e-6);
 CHECK(approach.departure.reason == sim::DepartReason::kGoNow);

 // Conflict relevance is a footprint/Minkowski-envelope query, not a
 // center-line intersection query. This route skirts the semantic box but
 // still overlaps a participant once both bodies and policy clearance are
 // included, so behavior must derive a stop instead of leaving the case to a
 // late lidar-only emergency response.
 std::vector<sim::RoutePoint> near_miss_route;
 for (int x = 0; x <= 10; ++x)
 near_miss_route.push_back({{static_cast<double>(x), 0.9}, 0.0,
 x == 0 ? sim::TravelDirection::Unspecified : sim::TravelDirection::Forward});
 const std::vector<sim::ConflictZone> compact_zone{
 {"compact", {4.9, -0.1}, {5.1, 0.1}, 0.4},
 };
 const auto near_miss = sim::evaluate_crossing(
 compact_zone, near_miss_route, 0, {1.0, 0.9}, 0.0, 0.0,
 open_frames, 0.1, {0.35, 0.25, 0.0}, crossing_caps);
 CHECK(near_miss.relevant);
 CHECK(near_miss.zone_id == "compact");

 // A cross-traffic participant can still overlap the zone's conservative
 // spatial envelope while moving decisively outwards. Admission is governed
 // by the same-time swept trajectory; the intent gate must not require that
 // duplicate envelope to clear before the ego trajectory even starts.
 std::vector<std::vector<sim::DynamicObjectState>> outward_frames;
 for (int frame = 0; frame <= 150; ++frame) {
 const double time_s = 0.1 * frame;
 outward_frames.push_back({{{5.0, 1.1 + time_s}, {0.0, 1.0}, 0.2}});
 }
 const auto outward_crossing = sim::evaluate_crossing(
 zones, route, 0, {2.0, 0.0}, 0.0, 0.0, outward_frames, 0.1,
 {0.35, 0.25, 0.0}, crossing_caps);
 CHECK(outward_crossing.departure.reason == sim::DepartReason::kGoNow);
 CHECK(outward_crossing.relevant_threats_moving_away);
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
 {{"near", {4.0, -1.0}, {6.0, 1.0}}, sim::CrossingAuthority::Control},
 };
 for (int sample = 0; sample < 20; ++sample) {
 const auto decision = unknown_behavior.assess(
 controlled_zones, route, 0, {2.0, 0.0}, 0.0, 0.0,
 unknown_frames, 0.1, {0.35, 0.25, 0.20, 0.35}, crossing_caps, 0.05);
 CHECK(decision.control.hold_active);
 CHECK(!decision.control.committed);
 CHECK(!decision.safe_to_commit);
 }
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
 CHECK(std::abs(ready.stop_longitudinal_m + 0.25) < 1e-12);
 const auto moving = sim::assess_crossing_readiness(
 {1.75, 2.0}, 0.11, 0.0, {2.0, 2.0}, {1.0, 0.0}, readiness_safety);
 CHECK(moving.position_ready && !moving.speed_ready && !moving.ready());
 const auto misaligned = sim::assess_crossing_readiness(
 {1.75, 2.0}, 0.0, 0.36, {2.0, 2.0}, {1.0, 0.0}, readiness_safety);
 CHECK(!misaligned.heading_ready && !misaligned.ready());

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
 CHECK(crossing_out.exit_valid && crossing_out.exit_point.x == 8.0);
 CHECK(std::abs(crossing_out.transit_speed_mps - 0.7) < 1e-12);
 // A changed prediction cannot cancel an in-progress crossing.
 crossing_out = sim::govern_crossing(crossing, blocked);
 CHECK(crossing_out.committed && !crossing_out.hold_active);
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
