#include "robot/autonomy/pipeline/autonomy_coordinator.hpp"

#include <stdexcept>

namespace sim::autonomy {

AutonomyCoordinator::AutonomyCoordinator(std::shared_ptr<IHighLevelPolicy> policy)
 : policy_(policy ? std::move(policy) : make_route_baseline_policy()) {
 if (!policy_)
 throw std::invalid_argument("autonomy coordinator requires a policy");
 policy_name_ = policy_->name();
}

void AutonomyCoordinator::reset(std::uint64_t seed) {
 policy_->reset(seed);
 policy_name_ = policy_->name();
}

AutonomyCoordinatorOutput AutonomyCoordinator::update(
 const AutonomyCoordinatorInput& input) {
 // Policy selection and policy safety are one ROBOT-side transaction.  The
 // caller owns only observation assembly and cannot accidentally publish a
 // raw custom-policy action that bypasses the common safety shield.
 const auto requested = policy_->decide(input.observation);
 AutonomyCoordinatorOutput output;
 output.decision = apply_strategy_safety(requested, input.observation, input.safety);
 output.policy_name = policy_name_;
 return output;
}

} // namespace sim::autonomy
