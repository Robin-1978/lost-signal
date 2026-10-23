#include <cassert>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>

#include "robot/platform/robot_config.hpp"
#include "sim/eval/robot_config.hpp"
#include "sim/scenario/scenario_session.hpp"

int main() {
  using namespace lost_signal;
  const std::string dir =
      std::string(TEST_CONFIG_DIR);  // CMake-injected configs path

  // All four seeds load: kind preserved, factory plant numbers filled.
  for (const char* kind : {"differential", "ackermann", "swerve",
                           "mecanum"}) {
    robot::platform::RobotConfig cfg;
    std::string err;
    assert(robot::platform::load_robot_config(dir + "/robot/" + kind + ".json", cfg,
                                        &err));
    assert(cfg.profile.chassis_kind == kind);
    assert(cfg.profile.wheel_radius_m == 0.1);
    assert(cfg.limits.max_forward_mps == 2.0);
    assert(!cfg.sensors.empty() && !cfg.links.empty());
    assert(cfg.robot_id == "ro1");
    // Both sides from one config.
    const auto desc = robot::platform::description_from_config(cfg);
    assert(desc.chassis_kind == kind);
    assert(desc.limits.max_forward_mps == 2.0);
    assert(desc.sensors.size() == cfg.sensors.size());
    const auto motor = sim::eval::motor_from_config(cfg);
    assert(motor.wheel_radius_m == 0.1);
  }
  // Geometry follows kind: ackermann is longer/wider than diff.
  {
    robot::platform::RobotConfig diff, ack;
    std::string err;
    assert(robot::platform::load_robot_config(dir + "/robot/differential.json", diff,
                                        &err));
    assert(robot::platform::load_robot_config(dir + "/robot/ackermann.json", ack,
                                        &err));
    const auto md = sim::eval::motor_from_config(diff);
    const auto ma = sim::eval::motor_from_config(ack);
    assert(ma.track_width_m == 0.58 && md.track_width_m == 0.5);
    // Session wiring: SIM motor numbers land in the live world.
    sim::scenario::ScenarioSession session(
        robot::platform::description_from_config(ack),
        sim::eval::motor_from_config(ack));
    assert(session.setup_world().motor_params().track_width_m == 0.58);
  }
  // End-to-end: mounts from the config shift the live rays. A forward
  // lidar mount sees the box face sooner (same engine, same truth).
  {
    robot::platform::RobotConfig cfg;
    std::string err;
    assert(robot::platform::load_robot_config(dir + "/robot/differential.json", cfg,
                                        &err));
    robot::platform::SensorMount lidar;
    lidar.sensor_id = "ro1.lidar";
    lidar.link = "lidar_link";
    lidar.xyz_m[0] = 0.3;
    cfg.sensors = {lidar};
    sim::scenario::ScenarioSession session(
        robot::platform::description_from_config(cfg),
        sim::eval::motor_from_config(cfg));
    session.setup_world().set_static_boxes({{2.0, -0.5, 3.0, 0.5, 1.0}});
    session.setup_world().add_place({"dock", 6.0, 0.0, 0.0});
    robot::iface::TaskRequest req;
    req.request_id = "m1";
    req.place_id = "dock";
    assert(session.request_goal(req).valid());
    for (int i = 0; i < 10; ++i) session.step_once();
    assert(!session.episode().steps().empty());
    const auto& ranges =
        session.episode().steps().back().obs.ranges_m;
    assert(!ranges.empty());
    float nearest = ranges[0];
    for (float r : ranges) nearest = std::min(nearest, r);
    // Face at 2.0 seen from a 0.3-forward mount, before driving far.
    assert(nearest < 1.8f && nearest > 1.0f);
  }
  // Fail-closed cases leave output untouched.
  {
    const std::string tmp =
        (std::filesystem::temp_directory_path() / "test_robot_cfg.json")
            .string();
    auto write = [&](const std::string& body) {
      std::FILE* fp = std::fopen(tmp.c_str(), "wb");
      std::fputs(body.c_str(), fp);
      std::fclose(fp);
    };
    robot::platform::RobotConfig cfg;
    std::string err;
    write(R"({"kind": "differential", "drive": {"stall_force_N": -5}})");
    assert(!robot::platform::load_robot_config(tmp, cfg, &err));
    write(R"({"kind": "nope"})");
    assert(!robot::platform::load_robot_config(tmp, cfg, &err));
    write(R"({"kind": "differential", "chassis_kind": "ackermann"})");
    assert(!robot::platform::load_robot_config(tmp, cfg, &err));  // disagree
    write(R"({"chassis_kind": "differential"})");  // kind alias works
    assert(robot::platform::load_robot_config(tmp, cfg, &err));
    assert(cfg.profile.chassis_kind == "differential");
    write(R"({"kind": "differential", "limits": {"max_forward_mps": -1}})");
    assert(!robot::platform::load_robot_config(tmp, cfg, &err));
    write(R"({"kind": "differential", "sensors": [{"sensor_id": "x"}]})");
    assert(!robot::platform::load_robot_config(tmp, cfg, &err));  // no link/pose
    write(R"({bad json)");
    assert(!robot::platform::load_robot_config(tmp, cfg, &err));
    std::filesystem::remove(tmp);
  }

  std::printf("robot_config ok\n");
  return 0;
}
