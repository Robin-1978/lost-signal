#include "high_level_policy.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace sim {
namespace {

bool finite(Vec2 value) {
 return std::isfinite(value.x) && std::isfinite(value.y);
}

void override_reason(StrategyDecision& decision, std::string reason) {
 decision.overridden = true;
 decision.safety_reasons.push_back(std::move(reason));
}

} // namespace

StrategyDecision apply_strategy_safety(const StrategyAction& requested,
 const StrategyObservation& observation, const StrategySafetyContext& context) {
 StrategyDecision decision;
 decision.requested = requested;
 decision.applied = requested;

 if (!std::isfinite(decision.applied.speed_scale)) {
 decision.applied.speed_scale = 0.0;
 override_reason(decision, "non-finite-speed");
 }
 const double clipped_speed = std::clamp(decision.applied.speed_scale, 0.0, 1.0);
 if (std::abs(clipped_speed - decision.applied.speed_scale) > 1e-12) {
 decision.applied.speed_scale = clipped_speed;
 override_reason(decision, "speed-clipped");
 }
 if (!std::isfinite(decision.applied.hold_s)) {
 decision.applied.hold_s = 0.0;
 override_reason(decision, "non-finite-hold");
 }
 decision.applied.hold_s = std::clamp(decision.applied.hold_s, 0.0, 8.0);

 if (decision.applied.mode == StrategyMode::Hold) {
 decision.applied.speed_scale = 0.0;
 }

 if (decision.applied.has_subgoal || decision.applied.mode == StrategyMode::LocalSubgoal) {
 const Vec2 subgoal = decision.applied.subgoal;
 const bool inside = finite(subgoal) && subgoal.x >= 0.0 && subgoal.y >= 0.0 &&
 subgoal.x < context.map_size.x && subgoal.y < context.map_size.y;
 const double distance = finite(subgoal)
 ? std::hypot(subgoal.x - observation.ego.position.x,
 subgoal.y - observation.ego.position.y)
 : 1e30;
 const bool safe = inside && distance <= context.max_subgoal_distance_m &&
 (!context.subgoal_is_safe || context.subgoal_is_safe(subgoal));
 if (!safe) {
 decision.applied.mode = StrategyMode::FollowRoute;
 decision.applied.has_subgoal = false;
 decision.applied.subgoal = context.fallback_target;
 override_reason(decision, "unsafe-subgoal");
 }
 }

 if (observation.rollover_risk >= 0.85 && decision.applied.speed_scale > 0.25) {
 decision.applied.speed_scale = 0.25;
 override_reason(decision, "rollover-speed-cap");
 }
 if (observation.traction_saturated && decision.applied.speed_scale > 0.55) {
 decision.applied.speed_scale = 0.55;
 override_reason(decision, "traction-speed-cap");
 }
 if (observation.visible_clearance_m < 0.20) {
 decision.applied.mode = StrategyMode::Hold;
 decision.applied.speed_scale = 0.0;
 override_reason(decision, "clearance-hold");
 }
 return decision;
}

std::shared_ptr<IHighLevelPolicy> make_route_baseline_policy() {
 return std::make_shared<RouteBaselinePolicy>();
}

} // namespace sim
