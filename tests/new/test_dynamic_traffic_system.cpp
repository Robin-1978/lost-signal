#include "mission/systems/dynamic_traffic_system.hpp"

#include <iostream>

#define CHECK(condition)                                                                                     \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n";                           \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

int main() {
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

 mission::DynamicTrafficSystem system;
 const auto shadow =
   system.assess(zones, "lesson", route, 0, {2.0, 0.0}, 0.0, 1.0, 1.0, open_frames, 0.1, 0.35, 0.25, 0.1);
 CHECK(shadow.has_active_zone);
 CHECK(shadow.assessment.relevant && shadow.assessment.zone_id == "crossing");
 CHECK(!shadow.control_authority && !shadow.control.hold_active);

 zones.front().authority = "control";
 system.reset();
 const auto controlled = system.assess(zones, "lesson", route, 0, shadow.assessment.stop_point, 0.0, 1.0, 1.0,
                                       open_frames, 0.1, 0.35, 0.25, 0.1);
 CHECK(controlled.control_authority);
 CHECK(controlled.control.hold_active && !controlled.control.committed);
 const auto committed = system.assess(zones, "lesson", route, 0, shadow.assessment.stop_point, 0.0, 1.0, 1.0,
                                      open_frames, 0.1, 0.35, 0.25, 0.1);
 CHECK(committed.control.committed && !committed.control.hold_active);

 system.reset();
 const auto inactive =
   system.assess(zones, "other", route, 0, {2.0, 0.0}, 0.0, 1.0, 1.0, open_frames, 0.1, 0.35, 0.25, 0.1);
 CHECK(!inactive.has_active_zone && !inactive.assessment.relevant);
 std::cout << "dynamic traffic system green\n";
 return 0;
}
