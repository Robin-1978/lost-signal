#include "robot/executive/executive.hpp"

namespace lost_signal::robot::executive {

core::Status RobotExecutive::submit(const iface::TaskRequest& req) {
  if (req.request_id.empty()) return core::Status::Fail("empty request_id");
  // Empty place_id = hold position, still a valid intent.
  active_place_ = req.place_id;
  status_.state = req.place_id.empty() ? iface::TaskState::kIdle
                                       : iface::TaskState::kActive;
  status_.detail = req.place_id.empty() ? "hold" : "goal:" + req.place_id;
  return core::Status::Ok();
}

}  // namespace lost_signal::robot::executive
