#include "robot/autonomy/pipeline/autonomy_coordinator.hpp"

#include <stdexcept>
#include <utility>

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

void AutonomyCoordinator::configure_route_tracking(double kp, double ki,
 double kd, double output_limit, double integral_limit) {
 route_tracking_ = std::make_unique<RouteTrackingSystem>(
 kp, ki, kd, output_limit, integral_limit);
}

RouteTrackingSystem& AutonomyCoordinator::route_tracking() {
 if (!route_tracking_)
 throw std::logic_error("route tracking has not been configured");
 return *route_tracking_;
}

const RouteTrackingSystem& AutonomyCoordinator::route_tracking() const {
 if (!route_tracking_)
 throw std::logic_error("route tracking has not been configured");
 return *route_tracking_;
}

bool AutonomyCoordinator::update_controlled_stop(
 bool carrying_stop, double peak_wheel_speed_mps,
 const ControlledStopLatch::Config& config) {
 return controlled_stop_latch_.update(carrying_stop, peak_wheel_speed_mps, config);
}

void AutonomyCoordinator::reset_controlled_stop() {
 controlled_stop_latch_.reset();
}

void AutonomyCoordinator::configure_global_planner(
 std::shared_ptr<const ::autonomy::IGlobalPlanner> planner) {
 if (!planner)
 throw std::invalid_argument("autonomy coordinator requires a global planner");
 global_planning_ = std::make_unique<::autonomy::GlobalPlanningSystem>(std::move(planner));
}

::autonomy::GlobalPlanningSystem& AutonomyCoordinator::global_planning() {
 if (!global_planning_)
 throw std::logic_error("global planning has not been configured");
 return *global_planning_;
}

const ::autonomy::GlobalPlanningSystem& AutonomyCoordinator::global_planning() const {
 if (!global_planning_)
 throw std::logic_error("global planning has not been configured");
 return *global_planning_;
}

} // namespace sim::autonomy
