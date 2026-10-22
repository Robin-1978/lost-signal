#include "robot/executive/executive.hpp"

namespace lost_signal::robot::executive {

core::Status RobotExecutive::configure() {
  current_ = iface::GoalStatus{};
  last_terminal_ = iface::GoalStatus{};
  state_ = lifecycle::ModuleState::kInactive;
  return core::Status::Ok();
}

core::Status RobotExecutive::activate() {
  if (state_ != lifecycle::ModuleState::kInactive) {
    return core::Status::Fail("executive: activate requires Inactive");
  }
  state_ = lifecycle::ModuleState::kActive;
  return core::Status::Ok();
}

core::Status RobotExecutive::deactivate() {
  if (state_ == lifecycle::ModuleState::kActive) {
    state_ = lifecycle::ModuleState::kInactive;
  }
  return core::Status::Ok();
}

core::Status RobotExecutive::cleanup() {
  current_ = iface::GoalStatus{};
  last_terminal_ = iface::GoalStatus{};
  state_ = lifecycle::ModuleState::kUnconfigured;
  return core::Status::Ok();
}

iface::GoalHandle RobotExecutive::request_goal(const std::string& place_id,
                                               const std::string& request_id) {
  if (state_ != lifecycle::ModuleState::kActive || request_id.empty()) {
    return iface::GoalHandle{};  // fail-closed: no goal armed
  }
  if (place_id.empty()) {
    // Hold intent: acknowledged at once, active slot untouched.
    last_terminal_.handle.id = next_id_++;
    last_terminal_.state = iface::GoalState::kDone;
    last_terminal_.detail = "hold";
    last_terminal_.place_id.clear();
    return last_terminal_.handle;
  }
  if (has_active_goal()) {
    current_.state = iface::GoalState::kCanceled;
    current_.detail = "preempted";
    last_terminal_ = current_;
  }
  current_.handle.id = next_id_++;
  current_.state = iface::GoalState::kActive;
  current_.detail = "goal:" + place_id;
  current_.place_id = place_id;
  return current_.handle;
}

iface::GoalStatus RobotExecutive::goal_status(iface::GoalHandle handle) const {
  if (!handle.valid()) {
    return iface::GoalStatus{handle, iface::GoalState::kFailed,
                             "invalid handle", ""};
  }
  if (current_.handle == handle) return current_;
  if (last_terminal_.handle == handle) return last_terminal_;
  return iface::GoalStatus{handle, iface::GoalState::kFailed, "unknown goal",
                           ""};
}

bool RobotExecutive::cancel_goal(iface::GoalHandle handle) {
  if (!handle.valid() || !(current_.handle == handle)) return false;
  if (current_.state != iface::GoalState::kActive) return false;
  current_.state = iface::GoalState::kCanceled;
  current_.detail = "canceled";
  last_terminal_ = current_;
  return true;
}

void RobotExecutive::note_arrival(bool arrived) {
  if (arrived && has_active_goal()) {
    current_.state = iface::GoalState::kDone;
    current_.detail = "arrived";
    last_terminal_ = current_;
  }
}

}  // namespace lost_signal::robot::executive
