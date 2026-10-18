#include "sim/game/mission/systems/dynamic_traffic_system.hpp"

#include <cmath>
#include <iostream>

#define CHECK(condition)                                                                                     \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n";                           \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

int main() {
 const sim::CrossingExecutionCaps crossing_caps{1.0, 2.0, 2.0, 2.0, 2.0};
 std::vector<sim::RoutePoint> route;
 for (int x = 0; x <= 8; ++x) {
  route.push_back({{static_cast<double>(x), 0.0},
                   0.0,
                   x == 0 ? sim::TravelDirection::Unspecified : sim::TravelDirection::Forward});
 }
 std::vector<mission::ConflictZoneDefinition> zones{
   {"crossing", {4.0, -1.0}, {5.0, 1.0}, "shadow", {"lesson"}},
 };
 const std::vector<std::vector<sim::DynamicObjectState>> open_frames(121, {{{50.0, 50.0}, {0.0, 0.0}, 0.4}});

 const auto off_route_ready = sim::assess_crossing_readiness(
 {4.0, 5.0}, 0.0, 0.0, {4.0, 0.0}, {1.0, 0.0}, {0.35, 0.25, 0.0});
 CHECK(!off_route_ready.position_ready);
 CHECK(std::abs(off_route_ready.stop_lateral_m - 5.0) < 1e-12);

 mission::DynamicTrafficSystem adapter;
 sim::CrossingBehaviorSystem system;
 auto active_zones = adapter.active_zones(zones, "lesson");
 CHECK(active_zones.size() == 1);
 CHECK(active_zones.front().authority == sim::CrossingAuthority::Observe);
 CHECK(active_zones.front().geometry.participant_radius_m == 0.0);
 const auto shadow =
 system.assess(active_zones, route, 0, {2.0, 0.0}, 0.0, 0.0, open_frames, 0.1,
 {0.35, 0.25, 0.0}, crossing_caps, 0.1);
 CHECK(shadow.has_active_zone);
 CHECK(shadow.assessment.relevant && shadow.assessment.zone_id == "crossing");
 CHECK(!shadow.control_authority && !shadow.control.hold_active);

 // Continuous route geometry is authoritative. Neither endpoint is inside
 // this narrow box, but the segment crosses it and must still produce a stop
 // and exit contract.
 const std::vector<sim::RoutePoint> sparse_route{
  {{0.0, 0.0}, 0.0, sim::TravelDirection::Unspecified},
  {{3.9, 0.0}, 0.0, sim::TravelDirection::Forward},
  {{5.1, 0.0}, 0.0, sim::TravelDirection::Forward},
  {{8.0, 0.0}, 0.0, sim::TravelDirection::Forward},
 };
 const auto sparse = system.assess(active_zones, sparse_route, 0, {0.0, 0.0}, 0.0, 0.0,
 open_frames, 0.1, {0.35, 0.25, 0.0}, crossing_caps, 0.1);
 CHECK(sparse.assessment.relevant);
 CHECK(sparse.assessment.zone_id == "crossing");
 CHECK(sparse.assessment.entry_index == 2);
 CHECK(sparse.assessment.exit_index >= sparse.assessment.entry_index);

 zones.front().authority = "control";
 active_zones = adapter.active_zones(zones, "lesson");
 CHECK(active_zones.front().authority == sim::CrossingAuthority::Control);
 system.reset();
 const auto controlled = system.assess(active_zones, route, 0, shadow.assessment.stop_point, 0.0, 0.0,
 open_frames, 0.1, {0.35, 0.25, 0.0}, crossing_caps, 0.1);
 CHECK(controlled.control_authority);
 CHECK(controlled.control.hold_active && !controlled.control.committed);
 // Position and zero speed are insufficient: commitment also requires the
 // chassis to face the route tangent at the stop pose.
 for (int sample = 0; sample < 10; ++sample) {
 const auto wrong_heading = system.assess(
 active_zones, route, 0, shadow.assessment.stop_point, 0.0,
 sim::kPi, open_frames, 0.1, {0.35, 0.25, 0.0, 0.35},
 crossing_caps, 0.1);
 CHECK(wrong_heading.control.hold_active);
 CHECK(!wrong_heading.control.committed);
 }
 sim::CrossingBehaviorDecision committed;
 const int confirmation_samples = static_cast<int>(std::ceil(sim::kCrossingCommitConfirmS / 0.1));
 for (int sample = 0; sample < confirmation_samples; ++sample) {
 committed = system.assess(active_zones, route, 0, shadow.assessment.stop_point, 0.0, 0.0,
 open_frames, 0.1, {0.35, 0.25, 0.0}, crossing_caps, 0.1);
 }
 CHECK(committed.control.committed && !committed.control.hold_active);
 CHECK(committed.control.exit_valid);
 CHECK(committed.control.transit_speed_mps == 1.0);

 // Controller lookahead may already be at the route end while the body is
 // still approaching. Physical progress keeps the commitment relevant.
 const auto lookahead_ahead = system.assess(active_zones, route, route.size() - 1,
 shadow.assessment.stop_point, 0.0, 0.0, open_frames, 0.1,
 {0.35, 0.25, 0.0}, crossing_caps, 0.1);
 CHECK(lookahead_ahead.assessment.relevant);
 CHECK(lookahead_ahead.control.committed);

 // Route projection can briefly lose the narrow authored box after a
 // lookahead/replan. The latched pass line, not evaluator relevance, owns an
 // in-progress crossing until the physical body passes the stored exit.
 const std::vector<sim::RoutePoint> route_without_zone{
 {{9.0, 0.0}, 0.0, sim::TravelDirection::Forward},
 {{10.0, 0.0}, 0.0, sim::TravelDirection::Forward},
 };
 const auto projection_gap = system.assess(active_zones, route_without_zone, 0,
 {5.5, 0.0}, 0.5, 0.0, open_frames, 0.1,
 {0.35, 0.25, 0.0}, crossing_caps, 0.1);
 CHECK(!projection_gap.assessment.relevant);
 CHECK(projection_gap.control_authority);
 CHECK(projection_gap.control.committed);
 const auto passed_exit = system.assess(active_zones, route_without_zone, 0,
 {7.0, 0.0}, 0.5, 0.0, open_frames, 0.1,
 {0.35, 0.25, 0.0}, crossing_caps, 0.1);
 CHECK(passed_exit.control_authority);
 CHECK(passed_exit.control.phase == sim::CrossingPhase::Cleared);
 CHECK(!passed_exit.control.committed && !passed_exit.control.hold_active);
 const auto same_zone_again = system.assess(active_zones, route, 0,
 shadow.assessment.stop_point, 0.0, 0.0, open_frames, 0.1,
 {0.35, 0.25, 0.0}, crossing_caps, 0.1);
 CHECK(same_zone_again.assessment.relevant);
 CHECK(same_zone_again.control.phase == sim::CrossingPhase::Cleared);
 CHECK(!same_zone_again.control.hold_active && !same_zone_again.control.committed);

 system.reset();
 const auto inactive_zones = adapter.active_zones(zones, "other");
 const auto inactive =
 system.assess(inactive_zones, route, 0, {2.0, 0.0}, 0.0, 0.0, open_frames, 0.1,
 {0.35, 0.25, 0.0}, crossing_caps, 0.1);
 CHECK(!inactive.has_active_zone && !inactive.assessment.relevant);
 std::cout << "dynamic traffic system green\n";
 return 0;
}
