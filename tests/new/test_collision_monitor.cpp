#include "mission/systems/collision_monitor.hpp"

#include <cmath>
#include <iostream>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

int main() {
 sim::GridMap occupancy(100, 100, 0.1);
 occupancy.addBox(4.0, 4.0, 5.0, 5.0);
 mission::CollisionMonitor monitor({10.0, 10.0}, 0.5,
 {{{4.0, 4.0}, {5.0, 5.0}}});

 mission::CollisionObservation free;
 free.position = {2.0, 2.0};
 free.dt_s = 0.02;
 free.occupancy = &occupancy;
 const auto clear = monitor.assess(free);
 CHECK(!clear.static_collision);
 CHECK(std::abs(clear.static_clearance_m - 1.5) < 1e-12);

 // Continuous planner geometry expands authored boxes by the circular robot
 // footprint. A segment below the expanded box is free; touching or crossing
 // it is not. World boundaries obey the same footprint contract.
 CHECK(monitor.static_motion_free({2.0, 3.4}, {6.0, 3.4}));
 CHECK(!monitor.static_motion_free({2.0, 3.5}, {6.0, 3.5}));
 CHECK(!monitor.static_motion_free({3.0, 4.5}, {6.0, 4.5}));
 CHECK(!monitor.static_motion_free({0.49, 2.0}, {2.0, 2.0}));
 CHECK(monitor.static_motion_free({0.51, 2.0}, {2.0, 2.0}));

 auto domain_hit = free;
 domain_hit.position = {4.5, 4.5};
 CHECK(monitor.assess(domain_hit).static_collision);

 monitor.reset();
 auto rigid = free;
 rigid.backend = mission::CollisionBackend::RigidBody;
 rigid.rigid_static_contact = true;
 for (int sample = 0; sample < 12; ++sample)
 CHECK(!monitor.assess(rigid).static_collision);
 CHECK(monitor.assess(rigid).static_collision); // 13 * 20 ms >= 250 ms

 monitor.reset();
 auto actors = free;
 actors.actors = {
 {{{2.3, 2.0}, {}, 0.2}, true, true},
 {{{2.4, 2.0}, {}, 0.2}, false, false},
 };
 actors.backend = mission::CollisionBackend::RigidBody;
 const auto social_only = monitor.assess(actors);
 CHECK(social_only.social_contacts == 1);
 CHECK(!social_only.dynamic_collision);
 CHECK(std::abs(social_only.dynamic_clearance_m + 0.4) < 1e-12);
 actors.actors[1].rigid_contact = true;
 CHECK(monitor.assess(actors).dynamic_collision);
 return 0;
}
