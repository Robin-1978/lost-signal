#include "algo/behavior_registry.hpp"

#include <cmath>
#include <iostream>

#define CHECK(condition)                                                                                     \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n";                           \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

namespace {
bool throws_unknown(const std::string& type) {
 try {
  sim::ActorBehaviorParams params;
  sim::BehaviorRegistry::evaluate(type, params, 0.0);
 } catch (const std::runtime_error&) {
  return true;
 }
 return false;
}
} // namespace

// Behavior registry contract: every builtin evaluates its documented motion,
// unknown types throw loudly (never fallback), registration is explicit.
int main() {
 sim::BehaviorRegistry::register_builtin_behaviors();
 const auto types = sim::BehaviorRegistry::builtin_types();
 CHECK(types.size() == 4);

 // Patrol ping-pongs a<->b at constant speed.
 sim::ActorBehaviorParams pingpong;
 pingpong.points = {{0.0, 0.0}, {4.0, 0.0}};
 pingpong.speed_mps = 2.0;
 pingpong.radius_m = 0.45;
 auto at = sim::BehaviorRegistry::evaluate("patrol", pingpong, 1.0);
 CHECK(std::abs(at.p.x - 2.0) < 1e-9 && at.p.y == 0.0);
 CHECK(std::abs(at.v.x - 2.0) < 1e-9 && at.v.y == 0.0);
 at = sim::BehaviorRegistry::evaluate("patrol", pingpong, 3.0);
 CHECK(std::abs(at.p.x - 2.0) < 1e-9);
 CHECK(std::abs(at.v.x + 2.0) < 1e-9);
 CHECK(std::abs(at.radius - 0.45) < 1e-12);

 // Wander loops the waypoints and returns.
 sim::ActorBehaviorParams loop;
 loop.points = {{0.0, 0.0}, {4.0, 0.0}, {4.0, 3.0}, {0.0, 3.0}};
 loop.speed_mps = 1.0;
 loop.radius_m = 0.25;
 const auto start = sim::BehaviorRegistry::evaluate("wander", loop, 0.0);
 CHECK(start.p.x == 0.0 && start.p.y == 0.0);
 const auto lap = sim::BehaviorRegistry::evaluate("wander", loop, 14.0);
 CHECK(std::abs(lap.p.x - 0.0) < 1e-9 && std::abs(lap.p.y - 0.0) < 1e-9);

 // Dwell waits at waypoints with waits[i] seconds.
 sim::ActorBehaviorParams dwell = loop;
 dwell.waits = {2.0, 0.0, 0.0, 0.0};
 at = sim::BehaviorRegistry::evaluate("dwell", dwell, 1.0);
 CHECK(at.p.x == 0.0 && at.p.y == 0.0);
 CHECK(at.v.x == 0.0 && at.v.y == 0.0);

 // Seeded-wander: same loop, phase-shifted by seed, bit-identical replays.
 sim::ActorBehaviorParams seeded = loop;
 seeded.seed = sim::hash_actor_id("big_ped_lane");
 const auto s0 = sim::BehaviorRegistry::evaluate("seeded-wander", seeded, 0.0);
 const auto s0_again = sim::BehaviorRegistry::evaluate("seeded-wander", seeded, 0.0);
 CHECK(s0.p.x == s0_again.p.x && s0.p.y == s0_again.p.y);
 CHECK(s0.radius == 0.25);
 // Distinct ids desynchronize (rotation and/or direction differ: same start
 // point with opposite directions still separates one step later).
 sim::ActorBehaviorParams seeded2 = loop;
 seeded2.seed = sim::hash_actor_id("big_ped_market");
 const auto t0 = sim::BehaviorRegistry::evaluate("seeded-wander", seeded2, 0.0);
 const auto t1 = sim::BehaviorRegistry::evaluate("seeded-wander", seeded2, 3.5);
 const auto s1 = sim::BehaviorRegistry::evaluate("seeded-wander", seeded, 3.5);
 CHECK(seeded.seed != seeded2.seed);
 const bool apart_now = (t0.p.x != s0.p.x || t0.p.y != s0.p.y);
 const bool apart_later = (t1.p.x != s1.p.x || t1.p.y != s1.p.y);
 CHECK(apart_now || apart_later);
 // Rotation is exact: seed 1 on a 4-loop starts at points[1], forward.
 sim::ActorBehaviorParams rot = loop;
 rot.seed = 1;
 const auto r0 = sim::BehaviorRegistry::evaluate("seeded-wander", rot, 0.0);
 CHECK(r0.p.x == 4.0 && r0.p.y == 0.0);
 // Seed 33 (bit5 set): same offset 1, reversed — starts at points[1] too
 // (loop entry shared) but leaves toward points[0].
 sim::ActorBehaviorParams rev = loop;
 rev.seed = 33;
 const auto v0 = sim::BehaviorRegistry::evaluate("seeded-wander", rev, 0.0);
 const auto v1 = sim::BehaviorRegistry::evaluate("seeded-wander", rev, 1.0);
 CHECK(v0.p.x == 4.0 && v0.p.y == 0.0);
 CHECK(std::abs(v1.p.x - 4.0) + std::abs(v1.p.y - 0.0) > 0.5);
 CHECK(v1.v.x < 0.0); // reversed leg heads -x, forward would head +y.
 // Degenerate input matches wander_state exactly.
 sim::ActorBehaviorParams single;
 single.points = {{1.0, 2.0}};
 single.seed = 99;
 const auto d0 = sim::BehaviorRegistry::evaluate("seeded-wander", single, 3.0);
 CHECK(d0.p.x == 1.0 && d0.p.y == 2.0 && d0.v.x == 0.0 && d0.v.y == 0.0);

 // Unknown types fail loudly (content typos die at construction, not on road).
 CHECK(throws_unknown("teleport"));
 CHECK(throws_unknown("Patrol"));
 std::cout << "behavior registry green\n";
 return 0;
}
