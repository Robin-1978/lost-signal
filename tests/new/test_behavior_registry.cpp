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
 CHECK(types.size() == 3);

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

 // Unknown types fail loudly (content typos die at construction, not on road).
 CHECK(throws_unknown("teleport"));
 CHECK(throws_unknown("Patrol"));
 std::cout << "behavior registry green\n";
 return 0;
}
