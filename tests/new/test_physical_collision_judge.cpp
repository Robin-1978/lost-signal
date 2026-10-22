#include "sim/game/scenario/physical_collision_judge.hpp"

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
 mission::PhysicalCollisionJudge judge(
 {10.0, 10.0}, 0.5, {{{4.0, 4.0}, {5.0, 5.0}, 0.0}});
 sim::platform::RigidVehicleState state;
 state.planar = {2.0, 2.0, 0.0, 0.0, 0.0};
 const std::vector<mission::PhysicalActorTruth> actors{
 {"person", {2.3, 2.0}, 0.2, true},
 {"forklift", {4.0, 2.0}, 0.6, false},
 };
 const auto clear = judge.assess({2.0, 2.0}, state, actors);
 CHECK(!clear.static_collision);
 CHECK(!clear.dynamic_collision);
 CHECK(clear.social_contacts == 0);
 CHECK(std::abs(clear.static_clearance_m - 1.5) < 1e-12);

 state.actor_collisions = {"person"};
 const auto social = judge.assess({2.0, 2.0}, state, actors);
 CHECK(!social.dynamic_collision);
 CHECK(social.social_contacts == 1);

 state.actor_collisions = {"forklift"};
 const auto dynamic = judge.assess({2.0, 2.0}, state, actors);
 CHECK(dynamic.dynamic_collision);
 CHECK(!dynamic.static_collision);

 state.static_collision = true;
 CHECK(judge.assess({2.0, 2.0}, state, actors).static_collision);
 return 0;
}
