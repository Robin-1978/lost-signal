#pragma once

#include <memory>

#include "robot/autonomy/control/controller.hpp"
#include "robot/autonomy/estimation/estimator.hpp"
#include "robot/autonomy/mapping/mapper.hpp"
#include "robot/autonomy/perception/perception.hpp"
#include "robot/autonomy/planning/global_planner.hpp"
#include "robot/autonomy/planning/local_planner.hpp"
#include "robot/autonomy/validation/validator.hpp"
#include "robot/iface/command.hpp"
#include "robot/iface/map.hpp"
#include "robot/iface/observation.hpp"
#include "robot/lifecycle/lifecycle.hpp"
#include "robot/platform/description.hpp"

namespace lost_signal::robot::autonomy {

// ROBOT-owned autonomy transaction. Owns the algorithm module stack and runs
// one deterministic pipeline per update():
//
//   estimator (predict/correct) -> perception -> mapper ->
//   local planner -> validator -> controller
//
// The global planner is event-driven (goal/map change) and lives outside the
// per-tick path; its Route feeds the local planner once a goal exists.
// SIM must never call these modules directly; it goes through
// RobotApplication. A lifecycle module: ingest/update run only while Active.
class AutonomyCoordinator : public lifecycle::ILifecycleModule {
 public:
  explicit AutonomyCoordinator(
      platform::ChassisLimits limits = {.max_forward_mps = 2.0,
                                        .max_yaw_rate_radps = 2.0,
                                        .footprint_radius_m = 0.4});

  const char* name() const override { return "autonomy"; }
  core::Status configure() override;
  core::Status activate() override;
  core::Status deactivate() override;
  core::Status cleanup() override;
  lifecycle::ModuleState state() const override { return state_; }

  void ingest_observation(const iface::ObservationFrame& obs);
  iface::MotionCommand update(iface::MapSnapshot map, core::MonotonicNs now_ns);
  bool has_observation() const { return has_observation_; }
  const iface::ObservationFrame& last_observation() const { return last_obs_; }
  const EstimatedState& estimate() const;
  const PerceptionResult& perception() const { return perception_; }
  LocalMap local_map() const;

 private:
  lifecycle::ModuleState state_ = lifecycle::ModuleState::kUnconfigured;
  platform::ChassisLimits limits_;
  std::unique_ptr<IEstimator> estimator_;
  std::unique_ptr<IPerception> perception_module_;
  std::unique_ptr<IMapper> mapper_;
  std::unique_ptr<ILocalPlanner> local_planner_;
  std::unique_ptr<IValidator> validator_;
  std::unique_ptr<IController> controller_;
  bool has_observation_ = false;
  iface::ObservationFrame last_obs_{};
  PerceptionResult perception_{};
};

}  // namespace lost_signal::robot::autonomy
