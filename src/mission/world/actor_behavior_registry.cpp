#include "mission/world/actor_behavior_registry.hpp"

#include <cstdint>
#include <stdexcept>
#include <string>

namespace mission {
namespace {

sim::DynamicObjectState evaluate_patrol(const ActorBehaviorParams& params, double time_s) {
 sim::Vec2 a{0, 0};
 sim::Vec2 b{0, 0};
 if (!params.points.empty()) a = params.points.front();
 if (params.points.size() > 1) b = params.points[1];
 return patrol_state(Patrol{a, b, params.speed_mps, params.radius_m}, time_s);
}

sim::DynamicObjectState evaluate_wander(const ActorBehaviorParams& params, double time_s) {
 Wander wander;
 wander.points = params.points;
 wander.speed = params.speed_mps;
 wander.radius = params.radius_m;
 const auto state = wander_state(wander, time_s);
 return sim::DynamicObjectState{state.p, state.v, state.radius};
}

sim::DynamicObjectState evaluate_seeded_wander(const ActorBehaviorParams& params, double time_s) {
 SeededWander wander;
 wander.points = params.points;
 wander.seed = params.seed;
 wander.speed = params.speed_mps;
 wander.radius = params.radius_m;
 const auto state = seeded_wander_state(wander, time_s);
 return sim::DynamicObjectState{state.p, state.v, state.radius};
}

sim::DynamicObjectState evaluate_dwell(const ActorBehaviorParams& params, double time_s) {
 Dwell dwell;
 dwell.points = params.points;
 dwell.waits = params.waits;
 dwell.speed = params.speed_mps;
 dwell.radius = params.radius_m;
 const auto state = dwell_state(dwell, time_s);
 return sim::DynamicObjectState{state.p, state.v, state.radius};
}

} // namespace

void ActorBehaviorRegistry::register_evaluator(const std::string& type, ActorBehaviorEvaluator evaluator) {
 registry()[type] = std::move(evaluator);
}

sim::DynamicObjectState ActorBehaviorRegistry::evaluate(const std::string& type, const ActorBehaviorParams& params,
                                    double time_s) {
 const auto found = registry().find(type);
 if (found == registry().end()) {
  throw std::runtime_error("unknown actor behavior '" + type + "'");
 }
 return found->second(params, time_s);
}

std::vector<std::string> ActorBehaviorRegistry::builtin_types() {
 return {"dwell", "patrol", "seeded-wander", "wander"};
}

void ActorBehaviorRegistry::register_builtin_behaviors() {
 register_evaluator("patrol", evaluate_patrol);
 register_evaluator("wander", evaluate_wander);
 register_evaluator("dwell", evaluate_dwell);
 register_evaluator("seeded-wander", evaluate_seeded_wander);
}

std::map<std::string, ActorBehaviorEvaluator>& ActorBehaviorRegistry::registry() {
 static std::map<std::string, ActorBehaviorEvaluator> instance;
 return instance;
}

} // namespace mission
