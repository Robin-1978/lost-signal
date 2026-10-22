#pragma once

#include <cstdint>

#include "robot/core/status.hpp"
#include "robot/iface/task.hpp"
#include "robot/lifecycle/lifecycle.hpp"

namespace lost_signal::robot::executive {

// Task/resource authority. Owns lifecycle, never computes wheel commands.
// A lifecycle module: request_goal is rejected while not Active.
//
// Single active goal slot: a new request preempts the running goal (which
// reports kCanceled/"preempted"), keeping dispatch deterministic for one
// base. An empty place_id is a hold intent and completes immediately
// (kDone/"hold") without touching the active slot.
class RobotExecutive : public lifecycle::ILifecycleModule {
 public:
  const char* name() const override { return "executive"; }
  core::Status configure() override;
  core::Status activate() override;
  core::Status deactivate() override;
  core::Status cleanup() override;
  lifecycle::ModuleState state() const override { return state_; }

  // Non-blocking request. Returns an invalid handle when not Active or when
  // request_id is empty (fail-closed: no goal is armed).
  iface::GoalHandle request_goal(const std::string& place_id,
                                 const std::string& request_id);
  iface::GoalStatus goal_status(iface::GoalHandle handle) const;
  // Cancel an active goal. False when the handle is unknown or terminal.
  bool cancel_goal(iface::GoalHandle handle);
  // Arrival signal from the application tick (coordinator route completion).
  // Only an active goal in the matching slot can complete.
  void note_arrival(bool arrived);
  iface::GoalStatus current_goal() const { return current_; }
  bool has_active_goal() const {
    return current_.handle.valid() &&
           current_.state == iface::GoalState::kActive;
  }

 private:
  lifecycle::ModuleState state_ = lifecycle::ModuleState::kUnconfigured;
  iface::GoalStatus current_{};
  // Last terminal goal (hold ack, arrival, cancel, preempt): retained so a
  // holder can poll once after completion instead of racing the slot.
  iface::GoalStatus last_terminal_{};
  std::uint64_t next_id_ = 1;
};

}  // namespace lost_signal::robot::executive
