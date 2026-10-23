#include <cstdio>
#include <string>

#include "robot/app/robot_application.hpp"
#include "robot/platform/robot_config.hpp"

int main(int argc, char** argv) {
  using namespace lost_signal;
  // Native robot process: no sim headers, no world truth. Optional
  // hardware contract file (--robot); without it, code defaults bring up
  // the differential base.
  std::string robot_path;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--robot" && i + 1 < argc) robot_path = argv[++i];
  }
  std::string used, err;
  auto config =
      robot::platform::load_default_robot_config(robot_path, &used, &err);
  if (!robot_path.empty() && !config.has_value()) {
    std::fprintf(stderr, "bad --robot file: %s\n", err.c_str());
    return 1;
  }
  robot::platform::RobotDescription desc;
  if (config.has_value()) {
    std::fprintf(stderr, "robot config: %s\n", used.c_str());
    desc = robot::platform::description_from_config(*config);
  }
  robot::app::RobotApplication app(nullptr, std::move(desc));
  // Ordered bringup: a failed module blocks the whole process.
  if (!app.configure()) {
    std::printf("robot: configure failed\n");
    return 1;
  }
  if (!app.activate()) {
    std::printf("robot: activate failed\n");
    return 1;
  }
  robot::iface::TaskRequest req;
  req.request_id = "bringup";
  req.place_id = "";
  if (!app.request_goal(req).valid()) return 1;
  auto cmd = app.tick(0);
  std::printf("robot tick: safe_stop=%d\n", cmd.is_safe_stop() ? 1 : 0);
  return 0;
}
