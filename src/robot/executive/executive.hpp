#pragma once

#include "robot/core/status.hpp"
#include "robot/iface/task.hpp"

namespace lost_signal::robot::executive {

// Task/resource authority. Owns lifecycle, never computes wheel commands.
class RobotExecutive {
 public:
  core::Status submit(const iface::TaskRequest& req);
  iface::TaskStatus status() const { return status_; }

 private:
  iface::TaskStatus status_{};
  std::string active_place_;
};

}  // namespace lost_signal::robot::executive
