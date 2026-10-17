#include "sim/game/mission/systems/field_dispatch_system.hpp"

#include <iostream>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

int main() {
 mission::MapDefinition map;
 map.size_m = {20.0, 14.0};
 map.markers.emplace("a", mission::Pose{2.0, 2.0, 0.0});
 map.markers.emplace("b", mission::Pose{10.0, 2.0, 0.0});
 map.markers.emplace("c", mission::Pose{16.0, 10.0, 0.0});

 mission::FieldDispatchSystem system;
 system.reset(42);
 CHECK(system.update(1.0, 1.0, map, {2.0, 2.0}, 0.0).empty());
 const auto opened = system.update(2.0, 1.0, map, {2.0, 2.0}, 0.0);
 CHECK(opened.size() == 1);
 CHECK(opened.front().kind == mission::FieldDispatchTransitionKind::Opened);
 CHECK(system.state().kind == "survey");
 CHECK(system.state().marker_id == "b");
 const auto first_target = system.state().target;
 const auto completed = system.update(2.8, 0.8, map, first_target, 0.0);
 CHECK(completed.size() == 1);
 CHECK(completed.front().kind == mission::FieldDispatchTransitionKind::Completed);
 CHECK(system.state().completed_count == 1);

 const auto delivery_opened = system.update(8.8, 6.0, map, first_target, 0.0);
 CHECK(delivery_opened.size() == 1);
 CHECK(system.state().kind == "delivery");
 CHECK(system.state().stage == 0);
 CHECK(system.state().marker_id == "c");
 const auto pickup = system.state().target;
 const auto picked_up = system.update(9.6, 0.8, map, pickup, 0.0);
 CHECK(picked_up.size() == 1);
 CHECK(picked_up.front().kind == mission::FieldDispatchTransitionKind::PickedUp);
 CHECK(system.state().stage == 1);
 CHECK(system.state().marker_id == "b");
 CHECK(system.state().target.x != pickup.x || system.state().target.y != pickup.y);
 const auto dropoff = system.state().target;
 const auto delivered = system.update(10.4, 0.8, map, dropoff, 0.0);
 CHECK(delivered.size() == 1);
 CHECK(delivered.front().kind == mission::FieldDispatchTransitionKind::Completed);
 CHECK(system.state().completed_count == 2);
 return 0;
}
