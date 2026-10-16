#pragma once

#include "autonomy/mapping/grid_map.hpp"
#include "sim/runtime_state.hpp"

#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace sim {

enum class StrategyMode { FollowRoute, Hold, LocalSubgoal, Explore, ReturnSafe };

struct StrategyObservation {
 double stamp_s{0.0};
 EstimatedState ego;
 Vec2 route_target;
 Vec2 goal;
 double visible_clearance_m{999.0};
 double rollover_risk{0.0};
 bool traction_saturated{false};
 int goals_reached{0};
};

struct StrategyAction {
 StrategyMode mode{StrategyMode::FollowRoute};
 double speed_scale{1.0};
 bool has_subgoal{false};
 Vec2 subgoal;
 double hold_s{0.0};
 double sensor_focus_rad{0.0};
};

struct StrategyDecision {
 StrategyAction requested;
 StrategyAction applied;
 bool overridden{false};
 std::vector<std::string> safety_reasons;
};

// Stable upper-policy boundary. A learned policy can implement this interface
// without gaining access to Bullet bodies, wheel torque, actor truth or future
// paths. The observation is timestamped estimated state plus task facts.
class IHighLevelPolicy {
 public:
 virtual ~IHighLevelPolicy() = default;
 virtual std::string name() const = 0;
 virtual void reset(std::uint64_t seed) = 0;
 virtual StrategyAction decide(const StrategyObservation& observation) = 0;
};

class RouteBaselinePolicy final : public IHighLevelPolicy {
 public:
 std::string name() const override { return "route-baseline"; }
 void reset(std::uint64_t seed) override { (void)seed; }
 StrategyAction decide(const StrategyObservation& observation) override {
 (void)observation;
 return {};
 }
};

struct StrategySafetyContext {
 Vec2 fallback_target;
 Vec2 map_size;
 double max_subgoal_distance_m{10.0};
 std::function<bool(Vec2)> subgoal_is_safe;
};

StrategyDecision apply_strategy_safety(const StrategyAction& requested,
 const StrategyObservation& observation, const StrategySafetyContext& context);

std::shared_ptr<IHighLevelPolicy> make_route_baseline_policy();

} // namespace sim
