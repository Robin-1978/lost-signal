#include "robot/executive/executive.hpp"

namespace lost_signal::robot::executive {

core::Status RobotExecutive::configure() {
  status_ = iface::TaskStatus{};
  active_place_.clear();
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
  status_ = iface::TaskStatus{};
  active_place_.clear();
  state_ = lifecycle::ModuleState::kUnconfigured;
  return core::Status::Ok();
}

core::Status RobotExecutive::submit(const iface::TaskRequest& req) {
  if (state_ != lifecycle::ModuleState::kActive) {
    return core::Status::Fail("executive: submit requires Active");
  }
  if (req.request_id.empty()) return core::Status::Fail("empty request_id");
  // Empty place_id = hold position, still a valid intent.
  active_place_ = req.place_id;
  status_.state = req.place_id.empty() ? iface::TaskState::kIdle
                                       : iface::TaskState::kActive;
  status_.detail = req.place_id.empty() ? "hold" : "goal:" + req.place_id;
  return core::Status::Ok();
}

}  // namespace lost_signal::robot::executive
