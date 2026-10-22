#include <cstdio>

#include "robot/app/robot_application.hpp"

int main() {
  // Native robot process: no sim headers, no world truth.
  lost_signal::robot::app::RobotApplication app(nullptr);
  // Ordered bringup: a failed module blocks the whole process.
  if (!app.configure()) {
    std::printf("robot: configure failed\n");
    return 1;
  }
  if (!app.activate()) {
    std::printf("robot: activate failed\n");
    return 1;
  }
  lost_signal::robot::iface::TaskRequest req;
  req.request_id = "bringup";
  req.place_id = "";
  if (!app.submit_task(req)) return 1;
  auto cmd = app.tick(0);
  std::printf("robot tick: safe_stop=%d\n", cmd.is_safe_stop() ? 1 : 0);
  return 0;
}
