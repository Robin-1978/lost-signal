#pragma once

#include <memory>
#include <string>

#include "robot/app/robot.hpp"
#include "robot/autonomy/coordinator.hpp"
#include "robot/autonomy/diagnostics/decision_evidence.hpp"
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
      platform::RobotDescription desc = platform::RobotDescription{
          .robot_id = "ro1",
          .version = 1,
          .links = {{"base_link", "world"}, {"lidar_link", "base_link"}},
          .sensors = {{"ro1.lidar", "lidar_link"}},
          .limits = {.max_forward_mps = 2.0,
                     .max_yaw_rate_radps = 2.0,
                     .footprint_radius_m = 0.4}});

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

  // Task / map intent (typed, no coordinates from SIM truth).
  core::Status submit_task(const iface::TaskRequest& req) override;
  core::Status request_map(const std::string& map_id) override;

  // Provider bring-up: claim (endpoint_id, role) ownership.
  bool register_provider(const iface::EndpointDescriptor& desc) override;

  // Provider -> robot: validated + queued, false when rejected.
  bool publish_observation(const iface::ObservationFrame& obs) override;
  // Robot -> plant: newest non-expired actuator setpoints, else safe zero.
  // The chassis layer translates the validated body command using the latest
  // joint feedback; loops never close on stale joints.
  iface::ActuatorCommand take_actuator(core::MonotonicNs now_ns) override;
  // Latest decision evidence (read-only ops/logging).
  autonomy::diagnostics::DecisionEvidence last_evidence() const;
  // Robot -> provider: newest non-expired body command, else safe stop.
  // Retained for smart-drive placements and logging; the default plant
  // consumes take_actuator().
  iface::MotionCommand take_command(core::MonotonicNs now_ns) override;

  // One deterministic cycle. Always returns a valid command; safe stop
  // without processing while not Active.
  iface::MotionCommand tick(core::MonotonicNs now_ns) override;

  iface::TaskStatus task_status() const override;
  iface::MapSnapshot active_map() const override;
  iface::ChannelHealth channel_health() const override;
  const platform::RobotDescription& description() const { return desc_; }

  // Read-only robot outputs for OFFLINE comparison only (SIM eval / logging).
  // Never fed back into tick() or any same-cycle decision. Carries the
  // ROBOT estimate so SIM can score truth-vs-estimate after the run; truth
  // must never flow the other way.
  struct Diagnostics {
    bool has_observation = false;
    core::MonotonicNs last_obs_stamp_ns = core::kInvalidTime;
    std::uint64_t last_obs_seq = 0;
    autonomy::EstimatedState estimate;
    autonomy::SafetyState safety = autonomy::SafetyState::kStopped;
    std::string safety_reason = "init";
    autonomy::diagnostics::DecisionEvidence evidence;
    bool braking_active = false;
    double braking_distance_m = 0.0;
    bool localized = false;
    std::string localization_action = "init";
    lifecycle::ModuleState bringup = lifecycle::ModuleState::kUnconfigured;
    iface::TaskStatus task;
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
};

}  // namespace lost_signal::robot::app
