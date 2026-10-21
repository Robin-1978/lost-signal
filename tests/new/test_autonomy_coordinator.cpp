#include "robot/autonomy/pipeline/autonomy_coordinator.hpp"

#include <cassert>
#include <memory>

namespace {

class SubgoalPolicy final : public sim::IHighLevelPolicy {
 public:
 std::string name() const override { return "test-subgoal"; }
 void reset(std::uint64_t) override { ++resets; }
 sim::StrategyAction decide(const sim::StrategyObservation&) override {
 sim::StrategyAction action;
 action.mode = sim::StrategyMode::LocalSubgoal;
 action.has_subgoal = true;
 action.subgoal = {1.0, 0.0};
 return action;
 }
 int resets{0};
};

} // namespace

int main() {
 auto policy = std::make_shared<SubgoalPolicy>();
 sim::autonomy::AutonomyCoordinator coordinator(policy);
 coordinator.reset(42);
 assert(policy->resets == 1);

 sim::autonomy::AutonomyCoordinatorInput input;
 input.observation.ego.position = {0.0, 0.0};
 input.observation.goal = {2.0, 0.0};
 input.safety.fallback_target = {2.0, 0.0};
 input.safety.map_size = {10.0, 10.0};
 input.safety.subgoal_is_safe = [](sim::Vec2) { return true; };
 const auto output = coordinator.update(input);
 assert(output.policy_name == "test-subgoal");
 assert(output.decision.applied.has_subgoal);
 assert(output.decision.applied.subgoal.x == 1.0);

 coordinator.configure_global_planner(::autonomy::default_global_planner());
 assert(coordinator.global_planning().planner().id() == "astar-grid");
 coordinator.set_localization_config({});
 coordinator.estimator().reset(0.0, 0.0, 0.0, 7);
 assert(coordinator.estimator().state().timestamp_s == 0.0);
 return 0;
}
