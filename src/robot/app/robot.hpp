#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>

#include "robot/autonomy/coordinator.hpp"
#include "robot/autonomy/diagnostics/decision_evidence.hpp"
#include "robot/core/status.hpp"
#include "robot/iface/channel.hpp"
#include "robot/iface/command.hpp"
#include "robot/iface/endpoint.hpp"
#include "robot/iface/map.hpp"
#include "robot/iface/observation.hpp"
#include "robot/iface/task.hpp"
#include "robot/lifecycle/lifecycle.hpp"
#include "robot/platform/description.hpp"

namespace lost_signal::robot::app {

// IRobot: deployable robot facade. One interface, two deployment families:
//   - DifferentialRobot / native bring-up: real drivers behind the same seam;
//   - SimRobot (src/sim/robot/): same facade backed by World + sim providers.
// Tests and tools program against IRobot*; they never branch on deployment.
class IRobot {
 public:
  virtual ~IRobot() = default;
  // Ordered bringup: configure -> activate must succeed before tick()
  // processes anything. A failed module blocks the whole robot.
  virtual core::Status configure() = 0;
  virtual core::Status activate() = 0;
  virtual void shutdown() = 0;
  virtual lifecycle::ModuleState bringup_state() const = 0;
  // Cartesian goal override (map frame). Feeds the coordinator's global
  // planner and wakes the decision branch. Place-based tasking still enters
  // via request_goal; this is the deliberate operator path.
  virtual void set_goal(const autonomy::RoutePoint& goal) = 0;
 // Operator/dock initial pose in the selected map frame. This is explicit
 // deployment input with uncertainty, never inferred from simulator truth.
 virtual void set_initial_pose(double x_m, double y_m, double yaw_rad,
 double position_sigma_m, double heading_sigma_rad) = 0;
  // Typed articulated-arm operator input. Implementations must apply the
  // same watchdog and fresh encoder gate as their native actuator path.
  virtual bool set_operator_arm_effort(
      std::uint8_t count, const std::array<double, 4>& effort) = 0;
  virtual void clear_operator_arm_effort() = 0;
  // Async goal request: resolves place_id through the map provider, and —
  // only once the executive accepts — arms the coordinator route, returning
  // a handle immediately. Unknown ids fail closed with an invalid handle
  // and arm nothing. Poll goal_status; cancel_goal withdraws an active goal.
  virtual iface::GoalHandle request_goal(const iface::TaskRequest& req) = 0;
  virtual iface::GoalStatus goal_status(iface::GoalHandle handle) const = 0;
  virtual bool cancel_goal(iface::GoalHandle handle) = 0;
  virtual core::Status request_map(const std::string& map_id) = 0;
  virtual bool register_provider(const iface::EndpointDescriptor& desc) = 0;
  virtual bool publish_observation(const iface::ObservationFrame& obs) = 0;
  virtual iface::MotionCommand take_command(core::MonotonicNs now_ns) = 0;
  virtual iface::ActuatorCommand take_actuator(core::MonotonicNs now_ns) = 0;
  virtual iface::MotionCommand tick(core::MonotonicNs now_ns) = 0;
  virtual iface::GoalStatus current_goal() const = 0;
  virtual iface::MapSnapshot active_map() const = 0;
  virtual iface::ChannelHealth channel_health() const = 0;
  virtual autonomy::diagnostics::DecisionEvidence last_evidence() const = 0;
};

}  // namespace lost_signal::robot::app
