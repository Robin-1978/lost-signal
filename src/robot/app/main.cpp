#include <cstdio>

#include "robot/app/robot_application.hpp"

int main() {
  // Native robot process: no sim headers, no world truth.
  lost_signal::robot::app::RobotApplication app(nullptr);
  lost_signal::robot::iface::TaskRequest req;
  req.request_id = "bringup";
  req.place_id = "";
  if (!app.submit_task(req)) return 1;
  auto cmd = app.tick(0);
  std::printf("robot tick: safe_stop=%d\n", cmd.is_safe_stop() ? 1 : 0);
  return 0;
}
