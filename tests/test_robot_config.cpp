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
  // Chassis hard geometry: explicit in differential.json (fore/aft ball
  // pair, COM at the axle, unladen), defaults rule in tracked.json (no
  // supports block -> legacy pair from wheel_base).
  {
    robot::platform::RobotConfig diff, tracked;
    std::string err;
    assert(robot::platform::load_robot_config(dir + "/robot/differential.json", diff,
                                        &err));
    assert(diff.profile.body_half_height_m == 0.05);
    assert(diff.profile.body_half_width_m == 0.12);
    assert(diff.profile.drive_axle_x_m == 0.0);
    assert(diff.profile.tire_half_width_m == 0.04);
    assert(diff.profile.com_height_m == 0.1);
    assert(diff.profile.payload_mass_kg == 0.0);
    assert(diff.profile.payload_height_m == 0.0);
    assert(diff.profile.idler_trail_m == 0.02);
    assert(diff.profile.supports.size() == 2);
    assert(diff.profile.supports[0].kind == "ball");
    assert(std::abs(diff.profile.supports[0].x_m - 0.252) < 1e-12);
    assert(std::abs(diff.profile.supports[1].x_m + 0.252) < 1e-12);
    const auto md = sim::eval::motor_from_config(diff);
    assert(md.body_half_height_m == 0.05);
    assert(md.drive_axle_x_m == 0.0);
    assert(md.wheel_mass_kg == 0.2);
    assert(md.knuckle_mass_kg == 0.1);
    assert(md.tire_density_kgpm3 == 400.0);
    assert(md.com_height_m == 0.1);  // explicit COM carries through
    assert(md.payload_mass_kg == 0.0);
    assert(md.idler_trail_m == 0.02);
    assert(md.support_count == 2);
    assert(md.supports[0].x_m == diff.profile.supports[0].x_m);
    // Declared plant loadout reaches the autonomy description (never
    // truth): ride/support from geometry, COM unfolded (unladen), mass.
    const auto dd = robot::platform::description_from_config(diff);
    assert(dd.plant.wheel_radius_m == 0.1);
    assert(dd.plant.track_width_m == 0.5);
    assert(std::abs(dd.plant.com_height_m - 0.1) < 1e-12);
    assert(dd.plant.total_mass_kg == 24.0);
    assert(robot::platform::valid_robot_description(dd));
    assert(robot::platform::load_robot_config(dir + "/robot/tracked.json", tracked,
                                        &err));
    assert(tracked.profile.supports.empty());  // rule path, not explicit
    const auto mt = sim::eval::motor_from_config(tracked);
    // Tracks provide their own support surface. A synthetic caster pair would
    // add unmodelled contact and corrupt skid-steer yaw dynamics.
    assert(mt.support_count == -1);
    assert(mt.com_height_m == 0.1);  // omitted com follows the axle
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
 write(R"({"kind": "differential", "local_planner": "teb"})");
 assert(robot::platform::load_robot_config(tmp, cfg, &err));
 assert(cfg.local_planner_kind == "teb");
 assert(robot::platform::description_from_config(cfg).local_planner_kind ==
 "teb");
 write(R"({"kind": "differential", "local_planner": "magic"})");
 assert(!robot::platform::load_robot_config(tmp, cfg, &err));
    write(R"({"kind": "differential", "limits": {"max_forward_mps": -1}})");
    assert(!robot::platform::load_robot_config(tmp, cfg, &err));
    write(R"({"kind": "differential", "sensors": [{"sensor_id": "x"}]})");
    assert(!robot::platform::load_robot_config(tmp, cfg, &err));  // no link/pose
    // Chassis fail-closed: bad kind, bad deck (belly at/below floor),
    // non-array supports, oversized support list all refuse.
    write(R"({"kind": "differential",
              "chassis": {"supports": [{"kind": "nope", "x_m": 0.2}]}})");
    assert(!robot::platform::load_robot_config(tmp, cfg, &err));
    write(R"({"kind": "differential",
              "chassis": {"supports": [{"kind": "ball", "radius_m": -1}]}})");
    assert(!robot::platform::load_robot_config(tmp, cfg, &err));
    write(R"({"kind": "differential",
              "chassis": {"body_half_height_m": 0.5}})");
    assert(!robot::platform::load_robot_config(tmp, cfg, &err));
    write(R"({"kind": "differential",
              "chassis": {"supports": {"kind": "ball"}}})");
    assert(!robot::platform::load_robot_config(tmp, cfg, &err));
    write(R"({"kind": "differential", "chassis": "ball"})");
    assert(!robot::platform::load_robot_config(tmp, cfg, &err));
    write(R"({"kind": "differential",
              "chassis": {"wheel_mass_kg": -1}})");
    assert(!robot::platform::load_robot_config(tmp, cfg, &err));
    write(R"({"kind": "differential",
              "chassis": {"supports": [{"kind": "ball", "mass_kg": 0}]}})");
    assert(!robot::platform::load_robot_config(tmp, cfg, &err));
    // COM/payload/trail fail-closed: belly breach, negative COM, laden
    // without placement defaults refused... (payload height 0 = chassis
    // COM is legal ballast; negative mass/height and COM below the deck
    // are not).
    write(R"({"kind": "differential",
              "chassis": {"body_half_height_m": 0.09,
                          "com_height_m": 0.05}})");
    assert(!robot::platform::load_robot_config(tmp, cfg, &err));
    write(R"({"kind": "differential",
              "chassis": {"com_height_m": -1}})");
    assert(!robot::platform::load_robot_config(tmp, cfg, &err));
    write(R"({"kind": "differential",
              "chassis": {"payload_mass_kg": -2}})");
    assert(!robot::platform::load_robot_config(tmp, cfg, &err));
    write(R"({"kind": "differential",
              "chassis": {"payload_mass_kg": 5, "payload_height_m": 0.3}})");
    assert(robot::platform::load_robot_config(tmp, cfg, &err));
    assert(cfg.profile.payload_mass_kg == 5.0);
    assert(cfg.profile.payload_height_m == 0.3);
    // Laden fold (master-style weighted average, description-side):
    // (24*0.1 + 5*0.3)/29 reduces the envelope COM exactly.
    {
      const auto dd = robot::platform::description_from_config(cfg);
      assert(dd.plant.total_mass_kg == 29.0);
      assert(std::abs(dd.plant.com_height_m - 3.9 / 29.0) < 1e-12);
    }
    write(R"({bad json)");
    assert(!robot::platform::load_robot_config(tmp, cfg, &err));
    std::filesystem::remove(tmp);
  }

  std::printf("robot_config ok\n");
  return 0;
}
