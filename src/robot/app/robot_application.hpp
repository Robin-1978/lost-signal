#pragma once

#include <memory>
#include <string>
#include <vector>

#include "robot/app/robot.hpp"
#include "robot/autonomy/coordinator.hpp"
#include "robot/autonomy/diagnostics/decision_evidence.hpp"
#include "robot/autonomy/diagnostics/trajectory_tracking.hpp"
#include "robot/autonomy/diagnostics/vehicle_measurement.hpp"
#include "robot/core/status.hpp"
#include "robot/executive/executive.hpp"
#include "robot/iface/channel.hpp"
#include "robot/iface/command.hpp"
#include "robot/iface/endpoint.hpp"
#include "robot/iface/map.hpp"
#include "robot/iface/motion.hpp"
#include "robot/iface/observation.hpp"
#include "robot/iface/task.hpp"
#include "robot/lifecycle/bringup.hpp"
#include "robot/platform/chassis.hpp"
#include "robot/platform/description.hpp"

namespace lost_signal::robot::app {

// Composition root for one robot process. Owns lifecycle and ordering of
// Executive + AutonomyCoordinator + channel + map view + description.
//
// Bringup (Nav2-style, ordered):
//   0. register endpoints via register_provider() (may precede configure);
//   1. configure(): brings every module Unconfigured -> Inactive, in order.
//      Any failure blocks the whole bringup.
//   2. activate(): brings every module Inactive -> Active, in order, with
//      reverse-order rollback on the first failure.
//   3. tick() runs only while Active; otherwise it returns safe stop without
//      touching algorithm state.
//   4. shutdown(): deactivate + cleanup, reverse order.
//
// Provider model (Sim or Real, same seam):
//   1. startup: register every endpoint via register_provider() with a stable
//      endpoint_id, role, backend, and frame_id;
//   2. run: providers push ObservationFrames via publish_observation()
//      (validated, sequenced, queued in RuntimeChannel);
//   3. tick(): drains the channel, runs the coordinator transaction, and
//      publishes one validated MotionCommand back into the channel;
//   4. providers pull the command via take_command() (expiry enforced).
//
// SIM, replay, UI, and native drivers are adapters around this class. None
// may reach AutonomyCoordinator or RobotExecutive directly, and none may
// read simulator ground truth into this path.
class RobotApplication : public IRobot {
 public:
  explicit RobotApplication(
      std::unique_ptr<iface::IMapProvider> maps,
      // Default bring-up is the canonical hardware contract (mast up:
      // lidar rides 0.35 m over the deck). Single source in
      // platform::default_robot_description — no literal copy here.
      platform::RobotDescription desc =
          platform::default_robot_description());

  // Ordered bringup. Must succeed before tick() processes anything.
  core::Status configure() override;
  core::Status activate() override;
  void shutdown() override;
  lifecycle::ModuleState bringup_state() const override {
    return bringup_.state();
  }
  void set_goal(const autonomy::RoutePoint& goal) override {
    coordinator_.set_goal(goal);
  }
 void set_initial_pose(double x_m, double y_m, double yaw_rad,
 double position_sigma_m, double heading_sigma_rad) override {
 coordinator_.seed_pose(x_m, y_m, yaw_rad, position_sigma_m,
 heading_sigma_rad);
 }
  // Startup mode passthrough (mapless missions skip global retries).
  void set_startup_mode(autonomy::StartupMode mode) {
    coordinator_.set_startup_mode(mode);
  }
 // Operator/manual motion remains inside the ROBOT safety pipeline. It is a
 // temporary command source for calibration and teaching labs, not a second
 // planner or a truth shortcut.
 void set_operator_command(double forward_mps, double yaw_rate_radps) {
  coordinator_.set_operator_command(forward_mps, yaw_rate_radps);
 }
 void clear_operator_command() { coordinator_.clear_operator_command(); }

  // Task / map intent (typed, no coordinates from SIM truth).
  // Async: returns a handle immediately; poll goal_status, cancel_goal.
  // Deployment/SIM decision cadence. Must be set before the first tick.
 void set_decision_period(core::MonotonicNs period_ns) {
 coordinator_.set_decision_period(period_ns);
 }

 // Select the single outer motion-loop owner before configure(). Native
 // hardware profiles use this to prevent the host chassis PID from running
 // when an MCU owns joint or body control. SIM and legacy native duty paths
 // keep the default host owner.
 void set_chassis_control_owner(iface::ChassisControlOwner owner) {
 control_owner_ = owner;
 }

 iface::GoalHandle request_goal(const iface::TaskRequest& req) override;
  iface::GoalStatus goal_status(iface::GoalHandle handle) const override {
    return executive_.goal_status(handle);
  }
  bool cancel_goal(iface::GoalHandle handle) override {
    if (!executive_.cancel_goal(handle)) return false;
    // Successful cancel withdraws the coordinator's active route: the next
    // tick holds instead of driving a dead goal. Failed cancels (unknown
    // or terminal handle) leave the route untouched.
    coordinator_.clear_goal();
    return true;
  }
  core::Status request_map(const std::string& map_id) override;

  // Provider bring-up: claim (endpoint_id, role) ownership.
  bool register_provider(const iface::EndpointDescriptor& desc) override;

  // Provider -> robot: validated + queued, false when rejected.
  bool publish_observation(const iface::ObservationFrame& obs) override;
  // Robot -> plant: newest non-expired actuator setpoints, else safe zero.
  // The chassis layer translates the validated body command using the latest
  // joint feedback; loops never close on stale joints.
  iface::ActuatorCommand take_actuator(core::MonotonicNs now_ns) override;
 // Build the typed setpoint consumed by an MCU-owned chassis loop. The
 // capability is checked at the transport boundary; this method only
 // performs the chassis-specific kinematic conversion.
 iface::ChassisSetpoint take_chassis_setpoint(
 const iface::ChassisCapabilities& capabilities,
 const iface::ActuatorCommand& actuator,
 std::uint64_t sequence, core::MonotonicNs now_ns);
  // Latest decision evidence (read-only ops/logging).
  autonomy::diagnostics::DecisionEvidence last_evidence() const;
  // Robot -> provider: newest non-expired body command, else safe stop.
  // Retained for smart-drive placements and logging; the default plant
  // consumes take_actuator().
  iface::MotionCommand take_command(core::MonotonicNs now_ns) override;

  // One deterministic cycle. Always returns a valid command; safe stop
  // without processing while not Active.
  iface::MotionCommand tick(core::MonotonicNs now_ns) override;

  iface::GoalStatus current_goal() const override;
  iface::MapSnapshot active_map() const override;
  iface::ChannelHealth channel_health() const override;
  const platform::RobotDescription& description() const { return desc_; }
 iface::ActuatorMode actuator_mode() const {
 return chassis_ ? chassis_->actuator_mode() : iface::ActuatorMode::kDiffDuty;
 }

  // Read-only robot outputs for OFFLINE comparison only (SIM eval / logging).
  // Never fed back into tick() or any same-cycle decision. Carries the
  // ROBOT estimate so SIM can score truth-vs-estimate after the run; truth
  // must never flow the other way.
  struct Diagnostics {
    bool has_observation = false;
    core::MonotonicNs last_obs_stamp_ns = core::kInvalidTime;
    std::uint64_t last_obs_seq = 0;
    autonomy::EstimatedState estimate;
 // Continuous REP-105 odom-frame state. `estimate` is the map-frame pose
 // used for navigation and may jump when localization is corrected.
 autonomy::EstimatedState odometry_estimate;
 // Anonymous ROBOT-side dynamic tracks for diagnostics and visualization.
 // These are perception estimates, never SIM actor identities or truth.
 std::vector<autonomy::DynamicTrack> dynamic_tracks;
    autonomy::SafetyState safety = autonomy::SafetyState::kStopped;
    std::string safety_reason = "init";
    autonomy::diagnostics::DecisionEvidence evidence;
 autonomy::diagnostics::TrajectoryTrackingReport trajectory_tracking;
    bool braking_active = false;
    double braking_distance_m = 0.0;
    std::string slip_quality = "Invalid";
    double slip_confidence = 0.0;
    // Online grip factor in [min_grip, 1.0]: the ROBOT's own traction
    // belief (control-layer governor, measured slip evidence only).
    // Offline scoring reads it; nothing feeds it back into tick().
    double grip_factor = 1.0;
    bool localized = false;
    std::string localization_action = "init";
    // Offline SLAM session signals (mapless drift + loop-closure evidence).
    std::size_t slam_keyframes = 0;
    std::size_t slam_loops_closed = 0;
 std::uint64_t slam_loop_from_id = 0;
 std::uint64_t slam_loop_to_id = 0;
 double slam_loop_correction_m = 0.0;
 double slam_loop_correction_rad = 0.0;
 double slam_loop_forward_score = 0.0;
 double slam_loop_reverse_score = 0.0;
 double slam_loop_overlap = 0.0;
 double slam_loop_forward_margin = 0.0;
 double slam_loop_reverse_margin = 0.0;
 std::string slam_loop_diagnostic = "init";
    // Incremental tracking counters (scan-to-submap, bounded window).
    std::size_t slam_tracks = 0;
 double track_correction_m = 0.0;
 double track_correction_rad = 0.0;
    std::string track_diagnostic = "init";
    lifecycle::ModuleState bringup = lifecycle::ModuleState::kUnconfigured;
    iface::GoalStatus goal;
    iface::ChannelHealth health;
  };
  Diagnostics diagnostics() const;

 private:
  std::unique_ptr<iface::IMapProvider> maps_;
  platform::RobotDescription desc_;
  std::unique_ptr<platform::IChassis> chassis_;
  iface::EndpointRegistry registry_;
  iface::RuntimeChannel channel_;
  autonomy::AutonomyCoordinator coordinator_;
  executive::RobotExecutive executive_;
  autonomy::diagnostics::VehicleMeasurementSystem measurement_;
  lifecycle::BringupManager bringup_;
  bool bringup_wired_ = false;
 iface::ChassisControlOwner control_owner_ =
 iface::ChassisControlOwner::kHostMotionLoop;
 iface::MotionCommand last_body_command_{};
};

}  // namespace lost_signal::robot::app
