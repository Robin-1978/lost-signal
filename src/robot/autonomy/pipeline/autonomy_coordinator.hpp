#pragma once

#include "robot/autonomy/behavior/high_level_policy.hpp"
#include "robot/autonomy/control/route_tracking_system.hpp"
#include "robot/autonomy/control/controlled_stop_latch.hpp"
#include "robot/autonomy/localization/estimation_system.hpp"
#include "robot/autonomy/localization/localization_supervisor.hpp"
#include "robot/autonomy/planning/global/global_planner.hpp"
#include "robot/autonomy/pipeline/autonomy_pipeline.hpp"

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

 bool update_controlled_stop(bool carrying_stop, double peak_wheel_speed_mps,
 const ControlledStopLatch::Config& config);
 void reset_controlled_stop();

 void set_estimator_motion_policy(const MotionPolicy& policy) {
 estimator_.set_motion_policy(policy);
 }
 EstimationSystem& estimator() { return estimator_; }
 const EstimationSystem& estimator() const { return estimator_; }

 void set_localization_config(const autonomy::LocalizationSupervisorConfig& config) {
 localization_supervisor_.set_config(config);
 }
 autonomy::LocalizationSupervisor& localization_supervisor() { return localization_supervisor_; }
 const autonomy::LocalizationSupervisor& localization_supervisor() const {
 return localization_supervisor_;
 }

 void configure_global_planner(std::shared_ptr<const ::autonomy::IGlobalPlanner> planner);
 ::autonomy::GlobalPlanningSystem& global_planning();
 const ::autonomy::GlobalPlanningSystem& global_planning() const;

 void configure_pipeline(const GridMap& static_map, int width, int height,
 double resolution_m, std::uint32_t seed,
 ObservedObstacleLayerConfig obstacle_config = {},
 const runtime::IMapProvider* map_provider = nullptr);
 bool has_pipeline() const { return pipeline_ != nullptr; }
 AutonomyPipeline& pipeline();
 const AutonomyPipeline& pipeline() const;
 void reset_pipeline(std::uint32_t seed);

 const std::string& policy_name() const { return policy_name_; }

 private:
 std::shared_ptr<IHighLevelPolicy> policy_;
 std::string policy_name_;
 std::unique_ptr<RouteTrackingSystem> route_tracking_;
 ControlledStopLatch controlled_stop_latch_;
 EstimationSystem estimator_;
 autonomy::LocalizationSupervisor localization_supervisor_;
 std::unique_ptr<::autonomy::GlobalPlanningSystem> global_planning_;
 std::unique_ptr<AutonomyPipeline> pipeline_;
};

} // namespace sim::autonomy
