#pragma once

#include "robot/autonomy/behavior/high_level_policy.hpp"
#include "robot/autonomy/control/route_tracking_system.hpp"

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

 // Route tracking is stateful actuator-facing autonomy. It is configured by
 // the composition root from the selected chassis profile, but its state is
 // owned by ROBOT so SIM cannot become the long-term owner of PID progress.
 void configure_route_tracking(double kp, double ki, double kd,
 double output_limit, double integral_limit);
 bool has_route_tracking() const { return route_tracking_ != nullptr; }
 RouteTrackingSystem& route_tracking();
 const RouteTrackingSystem& route_tracking() const;

 const std::string& policy_name() const { return policy_name_; }

 private:
 std::shared_ptr<IHighLevelPolicy> policy_;
 std::string policy_name_;
 std::unique_ptr<RouteTrackingSystem> route_tracking_;
};

} // namespace sim::autonomy
