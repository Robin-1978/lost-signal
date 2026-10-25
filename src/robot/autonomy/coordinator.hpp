#pragma once

#include <memory>
#include <string>

#include "robot/autonomy/behavior/behavior.hpp"
#include "robot/autonomy/control/controller.hpp"
#include "robot/autonomy/control/grip_adapter.hpp"
#include "robot/autonomy/control/stop_latch.hpp"
#include "robot/autonomy/decision/goal_arrival.hpp"
#include "robot/autonomy/decision/autonomy_pipeline.hpp"
#include "robot/autonomy/diagnostics/decision_evidence.hpp"
#include "robot/autonomy/diagnostics/trajectory_tracking.hpp"
#include "robot/autonomy/mapping/traversal.hpp"
#include "robot/autonomy/safety/safety.hpp"
#include "robot/autonomy/estimation/estimator.hpp"
#include "robot/autonomy/estimation/observation_pipeline.hpp"
#include "robot/autonomy/mapping/mapper.hpp"
#include "robot/autonomy/perception/perception.hpp"
#include "robot/autonomy/planning/global_planner.hpp"
#include "robot/autonomy/planning/local_planner.hpp"
#include "robot/autonomy/validation/validator.hpp"
#include "robot/iface/command.hpp"
#include "robot/iface/map.hpp"
#include "robot/iface/observation.hpp"
#include "robot/core/transform.hpp"
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
      LidarPerceptionConfig perception_config = {},
 GripAdapterConfig grip_config = {},
 double wheel_base_m = 0.56,
 GoalArrivalConfig arrival_config = {},
 std::string local_planner_kind = "dwa",
 platform::NavigationSafetyPolicy navigation_policy = {},
 platform::MotionDynamicsPolicy motion_policy = {});

  const char* name() const override { return "autonomy"; }
  core::Status configure() override;
  core::Status activate() override;
  core::Status deactivate() override;
  core::Status cleanup() override;
  lifecycle::ModuleState state() const override { return state_; }

  void ingest_observation(const iface::ObservationFrame& obs);
 iface::MotionCommand update(const iface::MapSnapshot& map,
                              const iface::ChannelHealth& channel,
                              core::MonotonicNs now_ns);
  // TF publisher (REP-105 semantics). Static edges come from the
  // description (set once before ticking; replaces the whole tree, so
  // dynamic edges reset). Every update() then refreshes two dynamic
  // edges from the pre/post-correction estimate: odom->base_link is the
  // continuous prediction pose, map->odom is the per-tick correction
  // delta (identity when nothing corrected — the offline drift signal).
  // This class is the only odom publisher: the estimator keeps a single
  // fused pose and never splits odom from map itself.
  //
  // Single extrinsic source: set_static_frames also resolves base<-lidar
  // and fans its full transform out to every consumer (perception, mapper,
  // scan matcher, global search, SLAM matcher, validator, safety).
  // Algorithm configs carry thresholds only; unknown mount = identity.
  void set_static_frames(const core::FrameTransformTree& tree);
  // Declared plant loadout (support/COM/ride/mass, description never
  // truth): fans support width + COM into the validator envelope and
  // latches the ride height for the survey context. Called once with the
  // machine file's plant, like set_static_frames.
  void set_plant_geometry(const platform::PlantGeometry& plant);
  const core::FrameTransformTree& tf() const { return tf_; }
  // Cartesian goal override (map frame). Refreshes the active route through
  // the global planner and wakes the decision branch. Place-based tasking
  // still enters via the executive; this is the deliberate operator path.
  // Decision cadence (SIM config / deployment tuning). Non-positive is
 // ignored so a bad config can never disable the control loop.
 void set_decision_period(core::MonotonicNs period_ns) {
 if (period_ns <= 0) return;
 decision_period_ns_ = period_ns;
 const double period_s = static_cast<double>(period_ns) * 1e-9;
 trajectory_tracking_limits_.forward_speed_mps =
 trajectory_tracking_limits_.position_m / period_s;
 trajectory_tracking_limits_.yaw_rate_radps =
 trajectory_tracking_limits_.heading_rad / period_s;
 }

 void set_goal(const RoutePoint& goal);
  const Route& active_route() const { return active_route_; }
  // Withdraws the active route (executive cancel path). Drops the goal and
  // invalidates the route so update() holds instead of driving a dead
  // goal. Only the executive decides lifetime; this just executes it.
  void clear_goal();
 // Result of the autonomy-owned arrival policy. The executive owns goal
 // lifetime; the coordinator only exposes the pipeline result.
 bool goal_arrived() const { return goal_arrival_decision_.arrived; }
 const GoalArrivalDecision& goal_arrival_decision() const {
 return goal_arrival_decision_;
 }
  // Deployment seeding: known start pose (dock, operator initialpose).
  // Snaps the estimator and marks the supervisor localized so the normal
  // cadence path takes over; every later correction is still gated.
 void seed_pose(double x_m, double y_m, double yaw_rad,
 double position_sigma_m = 0.10, double heading_sigma_rad = 0.05);
  // Startup mode passthrough (see supervisor::StartupMode): mapless
  // missions disable the global branch; SLAM still establishes the frame.
  void set_startup_mode(StartupMode mode) {
 observations_.set_startup_mode(mode);
  }
  // Event wakeup: force the decision branch on the next update, without
  // waiting for the decision period (goal arrival, operator override).
  void request_replan() { replan_requested_ = true; }
  void clear_safety() { safety_->clear_safety(); }
  void clear_stop_latch() { latch_ = ControlledStopLatch{}; }
  // Online grip factor in [min_grip, 1.0]: 1.0 is nominal (typical grip
  // assumed), lower is slip-tightened. Read-only probe for tests/logs.
  double grip_factor() const { return grip_.factor(); }
  core::FrameTransformTree tf_;
 bool has_observation() const { return observations_.has_observation(); }
 const iface::ObservationFrame& last_observation() const {
 return observations_.latest();
 }
  const EstimatedState& estimate() const;
 const EstimatedState& odometry_estimate() const {
 return estimator_->odometry_estimate();
 }
  SlipObservation last_slip() const { return estimator_->last_slip(); }
 const PerceptionResult& perception() const {
 return observations_.perception();
 }
  const BehaviorDecision& last_decision() const { return decision_; }
  SafetyVerdict last_safety() const { return safety_->last_verdict(); }
 bool localized() const { return observations_.localized(); }
  const std::string& last_localization_action() const {
 return observations_.last_localization_action();
  }
 std::size_t slam_keyframes() const {
 return observations_.slam_keyframes();
 }
  std::size_t slam_loops_closed() const {
 return observations_.slam_loops_closed();
  }
 std::size_t slam_tracks() const { return observations_.slam_tracks(); }
  const SlamTrackResult& last_track_result() const {
 return observations_.last_track_result();
  }
  const SlamLoopResult& last_slam_result() const {
 return observations_.last_slam_result();
  }
  diagnostics::DecisionEvidence last_evidence() const {
    return evidence_.current() != nullptr ? *evidence_.current()
                                          : diagnostics::DecisionEvidence{};
  }
 diagnostics::TrajectoryTrackingReport last_trajectory_tracking() const {
 return trajectory_tracking_report_;
 }
  core::MonotonicNs last_decision_ns() const { return last_decision_ns_; }
  LocalMap local_map() const;

 private:
  lifecycle::ModuleState state_ = lifecycle::ModuleState::kUnconfigured;
  platform::ChassisLimits limits_;
  // Speed-envelope governor (control layer): owns the grip factor and the
  // limit-shaping policy. The coordinator only wires observations into it
  // and consumes the tightened envelope; no adaptation state lives here.
  GripAdapter grip_;
 GoalArrivalPolicy goal_arrival_;
 GoalArrivalDecision goal_arrival_decision_;
  // Traversal belief (mapping layer): roughness from IMU motion evidence
  // over the surveyed elevation prior. Bound on map revision, observed
  // every tick; the global planner prices it, the validator gates on the
  // surveyed grade. Never SIM truth (materials stay unpublished).
  TraversalModel traversal_;
  // Lidar mount height above the body origin (TF fan-out; the validator's
  // surveyed sensor height rides on it).
  double lidar_mount_z_m_ = 0.35;
  // Body origin above ground (declared ride = wheel radius; the survey
  // context's absolute sensor height rides on it, like the mount).
  double ride_height_m_ = 0.10;
 // ROBOT-side sensor pipeline owns normalization, one-shot async views,
 // estimator/perception dispatch, localization and adaptation cadence.
 ObservationPipeline observations_;
 DecisionCyclePipeline decision_pipeline_;
  std::unique_ptr<IEstimator> estimator_;
  std::unique_ptr<IPerception> perception_module_;
  std::unique_ptr<IMapper> mapper_;
  std::unique_ptr<IBehaviorPolicy> behavior_;
  std::unique_ptr<IGlobalPlanner> global_planner_;
  std::unique_ptr<ILocalPlanner> local_planner_;
  std::unique_ptr<IValidator> validator_;
  std::unique_ptr<IController> controller_;
  std::unique_ptr<ISafetyMonitor> safety_;
  BehaviorDecision decision_{};
  RoutePoint goal_{};
  bool has_goal_ = false;
  Route active_route_{};
  // Mapper snapshot owning the live elevation overlay for the validator
  // (coordinator wires, holds no policy; refreshed per decision branch).
  LocalMap live_map_{};
  core::MonotonicNs decision_period_ns_;
  core::MonotonicNs last_decision_ns_ = core::kInvalidTime;
  std::uint64_t last_map_revision_ = 0;
  bool replan_requested_ = false;
 bool local_plan_reset_requested_ = false;
  iface::MotionCommand last_cmd_ = iface::MotionCommand::SafeStop(0);
  std::string last_constraint_ = "none";
  std::string last_reason_ = "init";
 double last_prediction_horizon_s_ = 0.0;
 std::size_t last_prediction_hypotheses_ = 0;
 double last_validated_static_clearance_m_ =
 std::numeric_limits<double>::infinity();
 double last_validated_dynamic_clearance_m_ =
 std::numeric_limits<double>::infinity();
 std::size_t last_route_point_count_ = 0;
 std::size_t last_route_segment_ = 0;
 std::size_t last_moving_candidate_count_ = 0;
 int last_requested_gear_ = 0;
 double last_route_error_m_ = std::numeric_limits<double>::infinity();
 bool last_gear_change_pending_ = false;
 bool last_gear_change_braking_armed_ = false;
 double last_gear_change_endpoint_distance_m_ =
 std::numeric_limits<double>::infinity();
  // Controlled-stop latch: withholds motion until measured tread settles.
  // Prevents a braking decision on a grade from chattering into a
  // brake/accelerate limit cycle. Independent of planner yaw intent.
  ControlledStopLatch latch_;
  bool last_published_stop_ = false;
  diagnostics::DecisionEvidenceSystem evidence_;
 diagnostics::TrajectoryTrackingSystem trajectory_tracking_;
 diagnostics::TrajectoryTrackingReport trajectory_tracking_report_;
 diagnostics::TrajectoryTrackingLimits trajectory_tracking_limits_;
};

}  // namespace lost_signal::robot::autonomy
