#include "autonomy/behavior/high_level_policy.hpp"
#include "mission/component_registry.hpp"
#include "mission/content_catalog.hpp"
#include "scenario/scenario_session.hpp"

#include <iostream>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

namespace {

class HoldPolicy final : public sim::IHighLevelPolicy {
 public:
 std::string name() const override { return "test-hold"; }
 void reset(std::uint64_t seed) override { (void)seed; }
 sim::StrategyAction decide(const sim::StrategyObservation& observation) override {
 CHECK_STAMP = observation.stamp_s;
 sim::StrategyAction action;
 action.mode = sim::StrategyMode::Hold;
 return action;
 }
 double CHECK_STAMP{-1.0};
};

} // namespace

int main() {
 sim::StrategyObservation observation;
 observation.ego.position = {2.0, 3.0};
 observation.route_target = {4.0, 3.0};
 sim::StrategySafetyContext context;
 context.fallback_target = observation.route_target;
 context.map_size = {20.0, 10.0};
 context.subgoal_is_safe = [](sim::Vec2 point) { return point.x < 8.0; };

 sim::StrategyAction unsafe;
 unsafe.mode = sim::StrategyMode::LocalSubgoal;
 unsafe.has_subgoal = true;
 unsafe.subgoal = {12.0, 3.0};
 unsafe.speed_scale = 1.4;
 const auto rejected = sim::apply_strategy_safety(unsafe, observation, context);
 CHECK(rejected.overridden);
 CHECK(rejected.applied.mode == sim::StrategyMode::FollowRoute);
 CHECK(!rejected.applied.has_subgoal);
 CHECK(rejected.applied.speed_scale == 1.0);

 observation.rollover_risk = 0.91;
observation.traction_slip_observed = true;
 sim::StrategyAction fast;
 const auto capped = sim::apply_strategy_safety(fast, observation, context);
 CHECK(capped.applied.speed_scale == 0.25);
 CHECK(capped.safety_reasons.size() == 1);
 CHECK(capped.safety_reasons.front() == "rollover-speed-cap");

 observation.rollover_risk = 0.0;
 context.traction_relief_enabled = false;
 const auto terrain_relief_disabled = sim::apply_strategy_safety(fast, observation, context);
 CHECK(terrain_relief_disabled.applied.speed_scale == 1.0);
 context.traction_relief_enabled = true;
 const auto terrain_relief = sim::apply_strategy_safety(fast, observation, context);
 CHECK(terrain_relief.applied.speed_scale == 0.55);

 observation.visible_clearance_m = 0.1;
 const auto stopped = sim::apply_strategy_safety(fast, observation, context);
 CHECK(stopped.applied.mode == sim::StrategyMode::Hold);
 CHECK(stopped.applied.speed_scale == 0.0);

 auto baseline = sim::make_route_baseline_policy();
 baseline->reset(42);
 CHECK(baseline->name() == "route-baseline");
 CHECK(baseline->decide(observation).mode == sim::StrategyMode::FollowRoute);

 const auto catalog = mission::ContentCatalog::load(LOST_SIGNAL_TEST_ASSETS);
 const auto& presentation = catalog.presentation_for_level("boot");
 auto hold = std::make_shared<HoldPolicy>();
 mission::ScenarioSession mission(catalog, "boot", presentation.recommended_build,
 std::nullopt, std::vector<std::string>{}, mission::ComponentRegistry::builtins(),
 std::shared_ptr<const autonomy::IGlobalPlanner>{}, mission::PhysicsMode::Bullet, hold);
 const auto start = mission.snapshot().robot;
 mission.start();
 for (int step = 0; step < 30; ++step)
 mission.step();
 CHECK(mission.strategy_policy_name() == "test-hold");
 CHECK(mission.snapshot().strategy.applied.mode == sim::StrategyMode::Hold);
 CHECK(mission.snapshot().strategy.applied.speed_scale == 0.0);
 CHECK(std::abs(mission.snapshot().robot.x - start.x) < 0.02);
 CHECK(std::abs(mission.snapshot().robot.y - start.y) < 0.02);
 CHECK(hold->CHECK_STAMP >= 0.0);
 return 0;
}
