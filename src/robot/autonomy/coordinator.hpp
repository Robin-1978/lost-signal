#pragma once

#include <memory>
#include <string>

#include "robot/autonomy/behavior/behavior.hpp"
#include "robot/autonomy/control/controller.hpp"
#include "robot/autonomy/control/stop_latch.hpp"
#include "robot/autonomy/diagnostics/decision_evidence.hpp"
#include "robot/autonomy/planning/astar_planner.hpp"
#include "robot/autonomy/planning/dwa_planner.hpp"
#include "robot/autonomy/safety/safety.hpp"
#include "robot/autonomy/estimation/estimator.hpp"
#include "robot/autonomy/localization/scan_matcher.hpp"
#include "robot/autonomy/localization/supervisor.hpp"
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
//   behavior (dynamic decision) -> local planner -> validator ->
//   controller -> SAFETY (independent final gate, latch on critical)
//
// The behavior layer turns tracks + estimate into ONE constraint
// (hold / speed cap) for the local planner. It selects no trajectory and
// emits no command. Task-level policy (which goal, return-to-charge) stays
// in RobotExecutive, one timescale above.
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
                                        .footprint_radius_m = 0.4},
      core::MonotonicNs decision_period_ns = 100'000'000,  // 10 Hz default
      LidarPerceptionConfig perception_config = {});

  const char* name() const override { return "autonomy"; }
  core::Status configure() override;
  core::Status activate() override;
  core::Status deactivate() override;
  core::Status cleanup() override;
  lifecycle::ModuleState state() const override { return state_; }

  void ingest_observation(const iface::ObservationFrame& obs);
  iface::MotionCommand update(iface::MapSnapshot map,
                              const iface::ChannelHealth& channel,
                              core::MonotonicNs now_ns);
  // Cartesian goal override (map frame). Refreshes the active route through
  // the global planner and wakes the decision branch. Place-based tasking
  // still enters via the executive; this is the deliberate operator path.
  void set_goal(const RoutePoint& goal);
  const Route& active_route() const { return active_route_; }
  // Deployment seeding: known start pose (dock, operator initialpose).
  // Snaps the estimator and marks the supervisor localized so the normal
  // cadence path takes over; every later correction is still gated.
  void seed_pose(double x_m, double y_m, double yaw_rad);
  // Event wakeup: force the decision branch on the next update, without
  // waiting for the decision period (goal arrival, operator override).
  void request_replan() { replan_requested_ = true; }
  void clear_safety() { safety_->clear_safety(); }
  void clear_stop_latch() { latch_ = ControlledStopLatch{}; }
  bool has_observation() const { return has_observation_; }
  const iface::ObservationFrame& last_observation() const { return last_obs_; }
  const EstimatedState& estimate() const;
  const PerceptionResult& perception() const { return perception_; }
  const BehaviorDecision& last_decision() const { return decision_; }
  SafetyVerdict last_safety() const { return safety_->last_verdict(); }
  bool localized() const { return supervisor_.localized(); }
  const std::string& last_localization_action() const {
    return last_localization_action_;
  }
  diagnostics::DecisionEvidence last_evidence() const {
    return evidence_.current() != nullptr ? *evidence_.current()
                                          : diagnostics::DecisionEvidence{};
  }
  core::MonotonicNs last_decision_ns() const { return last_decision_ns_; }
  LocalMap local_map() const;

 private:
  lifecycle::ModuleState state_ = lifecycle::ModuleState::kUnconfigured;
  platform::ChassisLimits limits_;
  void run_localization(const iface::MapSnapshot& map,
                        core::MonotonicNs now_ns);
  std::unique_ptr<IEstimator> estimator_;
  std::unique_ptr<IPerception> perception_module_;
  std::unique_ptr<IMapper> mapper_;
  ScanMatcher scan_matcher_;
  LocalizationSupervisor supervisor_;
  std::unique_ptr<IBehaviorPolicy> behavior_;
  std::unique_ptr<IGlobalPlanner> global_planner_;
  std::unique_ptr<ILocalPlanner> local_planner_;
  std::unique_ptr<IValidator> validator_;
  std::unique_ptr<IController> controller_;
  std::unique_ptr<ISafetyMonitor> safety_;
  bool has_observation_ = false;
  iface::ObservationFrame last_obs_{};
  PerceptionResult perception_{};
  BehaviorDecision decision_{};
  RoutePoint goal_{};
  bool has_goal_ = false;
  Route active_route_{};
  core::MonotonicNs decision_period_ns_;
  core::MonotonicNs last_decision_ns_ = core::kInvalidTime;
  std::uint64_t last_map_revision_ = 0;
  bool replan_requested_ = false;
  iface::MotionCommand last_cmd_ = iface::MotionCommand::SafeStop(0);
  std::string last_localization_action_ = "init";
  std::string last_constraint_ = "none";
  std::string last_reason_ = "init";
  // Controlled-stop latch: withholds motion until measured tread settles.
  // Prevents a braking decision on a grade from chattering into a
  // brake/accelerate limit cycle. Independent of planner yaw intent.
  ControlledStopLatch latch_;
  bool last_published_stop_ = false;
  diagnostics::DecisionEvidenceSystem evidence_;
};

}  // namespace lost_signal::robot::autonomy
