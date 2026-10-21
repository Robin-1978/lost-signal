#pragma once

#include "robot/autonomy/behavior/high_level_policy.hpp"

#include <cstdint>
#include <memory>
#include <string>

namespace sim::autonomy {

// The coordinator is the deployable owner of the upper autonomy policy
// boundary.  A composition root (simulation, replay, or a native robot
// process) supplies only estimated state and provider-neutral safety facts.
// It never receives a level, actor, physics body, or simulator truth object.
struct AutonomyCoordinatorInput {
 StrategyObservation observation;
 StrategySafetyContext safety;
};

struct AutonomyCoordinatorOutput {
 StrategyDecision decision;
 std::string policy_name;
};

class AutonomyCoordinator {
 public:
 explicit AutonomyCoordinator(std::shared_ptr<IHighLevelPolicy> policy = {});

 void reset(std::uint64_t seed);
 AutonomyCoordinatorOutput update(const AutonomyCoordinatorInput& input);

 const std::string& policy_name() const { return policy_name_; }

 private:
 std::shared_ptr<IHighLevelPolicy> policy_;
 std::string policy_name_;
};

} // namespace sim::autonomy
