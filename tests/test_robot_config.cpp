#include <cassert>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>

#include "sim/eval/robot_config.hpp"
#include "sim/scenario/scenario_session.hpp"

int main() {
  using namespace lost_signal;
  const std::string dir =
      std::string(TEST_CONFIG_DIR);  // CMake-injected configs path

  // All four seeds load: kind preserved, factory plant numbers filled.
  for (const char* kind : {"differential", "ackermann", "swerve",
                           "mecanum"}) {
    sim::eval::RobotConfig cfg;
    std::string err;
    assert(sim::eval::load_robot_config(dir + "/" + kind + ".json", cfg,
                                        &err));
    assert(cfg.profile.chassis_kind == kind);
    assert(cfg.profile.wheel_radius_m == 0.1);
    assert(cfg.limits.max_forward_mps == 2.0);
    assert(!cfg.sensors.empty() && !cfg.links.empty());
    assert(cfg.robot_id == "ro1");
    // Both sides from one config.
    const auto desc = sim::eval::description_from_config(cfg);
    assert(desc.chassis_kind == kind);
    assert(desc.limits.max_forward_mps == 2.0);
    assert(desc.sensors.size() == cfg.sensors.size());
    const auto motor = sim::eval::motor_from_config(cfg);
    assert(motor.wheel_radius_m == 0.1);
  }
  // Geometry follows kind: ackermann is longer/wider than diff.
  {
    sim::eval::RobotConfig diff, ack;
    std::string err;
    assert(sim::eval::load_robot_config(dir + "/differential.json", diff,
                                        &err));
    assert(sim::eval::load_robot_config(dir + "/ackermann.json", ack,
                                        &err));
    const auto md = sim::eval::motor_from_config(diff);
    const auto ma = sim::eval::motor_from_config(ack);
    assert(ma.track_width_m == 0.58 && md.track_width_m == 0.5);
    // Session wiring: SIM motor numbers land in the live world.
    sim::scenario::ScenarioSession session(
        sim::eval::description_from_config(ack),
        sim::eval::motor_from_config(ack));
    assert(session.setup_world().motor_params().track_width_m == 0.58);
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
    sim::eval::RobotConfig cfg;
    std::string err;
    write(R"({"kind": "differential", "drive": {"stall_force_N": -5}})");
    assert(!sim::eval::load_robot_config(tmp, cfg, &err));
    write(R"({"kind": "nope"})");
    assert(!sim::eval::load_robot_config(tmp, cfg, &err));
    write(R"({"kind": "differential", "chassis_kind": "ackermann"})");
    assert(!sim::eval::load_robot_config(tmp, cfg, &err));  // disagree
    write(R"({"chassis_kind": "differential"})");  // kind alias works
    assert(sim::eval::load_robot_config(tmp, cfg, &err));
    assert(cfg.profile.chassis_kind == "differential");
    write(R"({"kind": "differential", "limits": {"max_forward_mps": -1}})");
    assert(!sim::eval::load_robot_config(tmp, cfg, &err));
    write(R"({"kind": "differential", "sensors": [{"sensor_id": "x"}]})");
    assert(!sim::eval::load_robot_config(tmp, cfg, &err));  // no link/pose
    write(R"({bad json)");
    assert(!sim::eval::load_robot_config(tmp, cfg, &err));
    std::filesystem::remove(tmp);
  }

  std::printf("robot_config ok\n");
  return 0;
}
