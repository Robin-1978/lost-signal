#include "autonomy/behavior/entry_governor.hpp"
#include "autonomy/behavior/crossing_behavior.hpp"

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
   sim::evaluate_crossing(zones, route, 0, {2.0, 0.0}, 1.0, 1.0, open_frames, 0.1, 0.35, 0.25);
 CHECK(approach.relevant && approach.zone_id == "near");
 CHECK(approach.entry_index == 4 && approach.exit_index == 7);
 CHECK(approach.stop_index == 2 && approach.stop_point.x == 2.0);
 CHECK(approach.departure.reason == sim::DepartReason::kGoNow);
 const auto advanced =
   sim::evaluate_crossing(zones, route, 3, {1.2, 0.4}, 1.0, 1.0, open_frames, 0.1, 0.35, 0.25);
 CHECK(advanced.zone_id == "near" && advanced.stop_index == approach.stop_index);
 CHECK(advanced.stop_point.x == approach.stop_point.x && !advanced.stop_reached);
 const auto at_stop =
   sim::evaluate_crossing(zones, route, 3, {2.1, 0.4}, 1.0, 1.0, open_frames, 0.1, 0.35, 0.25);
 CHECK(at_stop.stop_reached);
 CHECK(approach.stop_direction.x == 1.0 && approach.stop_direction.y == 0.0);
 CHECK(std::abs(approach.stop_distance_m) < 1e-12);
 const std::vector<std::vector<sim::DynamicObjectState>> blocked_frames(151, {{{5.0, 0.0}, {0.0, 0.0}, 0.4}});
 const auto blocked_crossing =
   sim::evaluate_crossing(zones, route, 0, {0.0, 0.0}, 1.0, 1.0, blocked_frames, 0.1, 0.35, 0.25);
 CHECK(blocked_crossing.departure.reason != sim::DepartReason::kGoNow);
 const auto cleared =
   sim::evaluate_crossing(zones, route, 8, {8.0, 0.0}, 1.0, 1.0, open_frames, 0.1, 0.35, 0.25);
 CHECK(cleared.relevant && cleared.zone_id == "far");
 const auto index_ahead_but_body_inside =
   sim::evaluate_crossing(zones, route, 8, {5.0, 0.0}, 1.0, 1.0, open_frames, 0.1, 0.35, 0.25);
 CHECK(index_ahead_but_body_inside.relevant);
 CHECK(index_ahead_but_body_inside.zone_id == "near" && index_ahead_but_body_inside.inside_zone);

 // A semantic crossing holds at a stable stop point, requires a sustained
 // safe window, then keeps its commitment until the route clears the zone.
 sim::CrossingState crossing;
 sim::CrossingInput blocked;
 blocked.relevant = true;
 blocked.stop_point = {5.0, 2.0};
 blocked.stop_valid = true;
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
 for (int sample = 0; sample < 3; ++sample) {
  crossing_out = sim::govern_crossing(crossing, crossing_clear);
  CHECK(crossing_out.hold_active && !crossing_out.committed);
 }
 crossing_out = sim::govern_crossing(crossing, crossing_clear);
 CHECK(!crossing_out.hold_active && crossing_out.committed);
 // A changed prediction cannot cancel an in-progress crossing.
 crossing_out = sim::govern_crossing(crossing, blocked);
 CHECK(crossing_out.committed && !crossing_out.hold_active);
 sim::CrossingInput done = blocked;
 done.relevant = false;
 crossing_out = sim::govern_crossing(crossing, done);
 CHECK(crossing_out.phase == sim::CrossingPhase::Inactive);
 CHECK(!crossing_out.hold_active && !crossing_out.committed);
 std::cout << "entry governor green\n";
 return 0;
}
