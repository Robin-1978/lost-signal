#include "algo/behavior_registry.hpp"

#include <cstdint>
#include <stdexcept>
#include <string>

namespace sim {
namespace {

DynState evaluate_patrol(const ActorBehaviorParams& params, double time_s) {
 Vec2 a{0, 0};
 Vec2 b{0, 0};
 if (!params.points.empty()) a = params.points.front();
 if (params.points.size() > 1) b = params.points[1];
 return patrol_state(Patrol{a, b, params.speed_mps, params.radius_m}, time_s);
}

DynState evaluate_wander(const ActorBehaviorParams& params, double time_s) {
 Wander wander;
 wander.points = params.points;
 wander.speed = params.speed_mps;
 wander.radius = params.radius_m;
 const auto state = wander_state(wander, time_s);
 return DynState{state.p, state.v, state.radius};
}

DynState evaluate_seeded_wander(const ActorBehaviorParams& params, double time_s) {
 SeededWander wander;
 wander.points = params.points;
 wander.seed = params.seed;
 wander.speed = params.speed_mps;
 wander.radius = params.radius_m;
 const auto state = seeded_wander_state(wander, time_s);
 return DynState{state.p, state.v, state.radius};
}

DynState evaluate_dwell(const ActorBehaviorParams& params, double time_s) {
 Dwell dwell;
 dwell.points = params.points;
 dwell.waits = params.waits;
 dwell.speed = params.speed_mps;
 dwell.radius = params.radius_m;
 const auto state = dwell_state(dwell, time_s);
 return DynState{state.p, state.v, state.radius};
}

} // namespace

void BehaviorRegistry::register_evaluator(const std::string& type, BehaviorEvaluator evaluator) {
 registry()[type] = std::move(evaluator);
}

DynState BehaviorRegistry::evaluate(const std::string& type, const ActorBehaviorParams& params,
                                    double time_s) {
 const auto found = registry().find(type);
 if (found == registry().end()) {
  throw std::runtime_error("unknown actor behavior '" + type + "'");
 }
 return found->second(params, time_s);
}

std::vector<std::string> BehaviorRegistry::builtin_types() {
 return {"dwell", "patrol", "seeded-wander", "wander"};
}

void BehaviorRegistry::register_builtin_behaviors() {
 register_evaluator("patrol", evaluate_patrol);
 register_evaluator("wander", evaluate_wander);
 register_evaluator("dwell", evaluate_dwell);
 register_evaluator("seeded-wander", evaluate_seeded_wander);
}

std::map<std::string, BehaviorEvaluator>& BehaviorRegistry::registry() {
 static std::map<std::string, BehaviorEvaluator> instance;
 return instance;
}

} // namespace sim
