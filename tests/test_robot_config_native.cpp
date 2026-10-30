// Robot-side config test: links robot_platform ONLY (no sim libraries,
// no sim headers) — this is the native-deployability proof. The same
// files feed SIM through sim/eval translation.
#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>
#include <string>

#include "robot/platform/chassis.hpp"
#include "robot/platform/description.hpp"
#include "robot/platform/robot_config.hpp"

int main() {
  using namespace lost_signal::robot;
  const std::string dir = std::string(TEST_CONFIG_DIR) + "/robot";

  for (const char* kind : {"differential", "ackermann", "swerve",
                           "mecanum"}) {
    platform::RobotConfig cfg;
    std::string err;
    assert(platform::load_robot_config(dir + "/" + kind + ".json", cfg,
                                       &err));
    assert(cfg.profile.chassis_kind == kind);
    const platform::RobotDescription desc =
        platform::description_from_config(cfg);
    assert(desc.chassis_kind == kind);
    assert(desc.limits.max_forward_mps == 2.0);
 assert(desc.motion.acceleration_mps2 == 2.0);
 assert(desc.motion.brake_deceleration_mps2 == 2.0);
 const bool four_brakes = std::string(kind) == "swerve" ||
 std::string(kind) == "mecanum";
 assert(cfg.brake_max_torque_Nm == (four_brakes ? 1.2 : 2.4));
 assert(cfg.brake_apply_time_s == 0.12);
 assert(cfg.brake_release_time_s == 0.08);
 if (std::string(kind) == "swerve" || std::string(kind) == "mecanum") {
 assert(desc.limits.max_lateral_mps > 0.0);
 }
    assert(!desc.sensors.empty() && !desc.links.empty());
    // Real mast heights: lidar rides 0.35 m above base 0.25 m forward,
    // IMU low on the deck — the true 3D sensor pose raycasts from there.
    bool saw_lidar = false, saw_imu = false;
    for (const auto& s : cfg.sensors) {
      if (s.link == "lidar_link") {
        saw_lidar = true;
        assert(s.xyz_m[0] == 0.25 && s.xyz_m[2] == 0.35);
      }
      if (s.link == "imu_link") saw_imu = true;
    }
    assert(saw_lidar && saw_imu);
  }
  // Ackermann geometry differs from diff (factory numbers, not code).
  {
    platform::RobotConfig diff, ack;
    std::string err;
    assert(platform::load_robot_config(dir + "/differential.json", diff,
                                       &err));
    assert(platform::load_robot_config(dir + "/ackermann.json", ack,
                                       &err));
    assert(diff.profile.track_width_m == 0.5);
    assert(ack.profile.track_width_m == 0.58);
    assert(ack.profile.wheel_base_m == 0.62);
  }
  // Single source: factory mast == seed files == app default. A bare
  // file (no links/sensors sections) falls back to the factory instead
  // of silently parking the lidar at the deck.
  {
    const auto factory = platform::default_robot_description();
    assert(factory.links.size() == 3 && factory.sensors.size() == 2);
    platform::RobotConfig bare;
    std::string err;
    // Bare kind via temp file: sections absent -> factory mast.
    {
      const char* body = R"({"kind": "differential"})";
      std::FILE* fp = std::fopen("/tmp/test_bare_kind.json", "wb");
      assert(fp != nullptr);
      std::fputs(body, fp);
      std::fclose(fp);
      assert(platform::load_robot_config("/tmp/test_bare_kind.json", bare,
                                         &err));
      assert(bare.sensors.size() == factory.sensors.size());
      assert(bare.links.size() == factory.links.size());
      assert(bare.sensors[0].xyz_m[0] == 0.25);
      assert(bare.sensors[0].xyz_m[2] == 0.35);
      std::remove("/tmp/test_bare_kind.json");
    }
    // Seeds carry the same numbers explicitly.
    platform::RobotConfig diff;
    assert(platform::load_robot_config(
        dir + "/differential.json", diff, &err));
    assert(diff.sensors.size() == factory.sensors.size());
    for (std::size_t i = 0; i < factory.sensors.size(); ++i) {
      assert(diff.sensors[i].sensor_id == factory.sensors[i].sensor_id);
      assert(diff.sensors[i].link == factory.sensors[i].link);
      for (int k = 0; k < 3; ++k) {
        assert(diff.sensors[i].xyz_m[k] == factory.sensors[i].xyz_m[k]);
      }
    }
  }
  // Declared geometry must reach the chassis factory unchanged. This is
 // the ROBOT-side hardware contract; no SIM state is involved.
 {
 platform::RobotDescription d = platform::default_robot_description();
 d.plant.wheel_radius_m = 0.23;
 d.plant.track_width_m = 0.91;
 d.plant.wheel_base_m = 1.37;
 d.limits.max_forward_mps = 2.0;
 d.limits.max_yaw_rate_radps = 1.5;
 d.limits.footprint_radius_m = 0.5;
 for (const char* kind : {"differential", "ackermann", "swerve", "mecanum"}) {
 d.chassis_kind = kind;
 auto chassis = platform::make_chassis(d);
 assert(chassis != nullptr);
 if (auto* diff =
 dynamic_cast<platform::DifferentialChassis*>(chassis.get())) {
 assert(diff->wheel_radius_m() == 0.23);
 assert(diff->track_width_m() == 0.91);
 } else if (auto* ack =
 dynamic_cast<platform::AckermannChassis*>(chassis.get())) {
 assert(ack->wheel_radius_m() == 0.23);
 assert(ack->track_width_m() == 0.91);
 assert(ack->wheel_base_m() == 1.37);
 } else if (auto* hol =
 dynamic_cast<platform::HolonomicChassis*>(chassis.get())) {
 assert(hol->wheel_radius_m() == 0.23);
 assert(hol->track_width_m() == 0.91);
 assert(hol->wheel_base_m() == 1.37);
 } else {
 assert(false);
 }
 }
 d.chassis_kind = "tracked";
 auto tracked = platform::make_chassis(d);
 assert(tracked != nullptr);
 auto* td = dynamic_cast<platform::DifferentialChassis*>(tracked.get());
 assert(td != nullptr && td->wheel_radius_m() == 0.23 &&
 td->track_width_m() == 0.91);
 // Bad declared geometry refuses the machine instead of falling back.
 d.chassis_kind = "differential";
 d.plant.wheel_radius_m = 0.0;
 assert(platform::make_chassis(d) == nullptr);
 }
 // Direct hardware-package descriptions fail closed on non-finite limits,
 // geometry and mounts even when they bypass the JSON loader.
 {
  platform::RobotDescription bad = platform::default_robot_description();
  bad.limits.max_forward_mps = std::numeric_limits<double>::quiet_NaN();
  assert(!platform::valid_robot_description(bad));
  bad = platform::default_robot_description();
  bad.plant.wheel_base_m = std::numeric_limits<double>::infinity();
  assert(!platform::valid_robot_description(bad));
  bad = platform::default_robot_description();
  bad.links[1].xyz_m[2] = std::numeric_limits<double>::quiet_NaN();
  assert(!platform::valid_robot_description(bad));
 }
 // Fail-closed: unknown kind, disagreement, bad numbers, bad JSON.
  {
    platform::RobotConfig cfg;
    std::string err;
    assert(!platform::load_robot_config(dir + "/no-such.json", cfg, &err));
 const char* zero_lag =
 R"({"kind":"differential","brake":{"max_torque_Nm":5.0,"apply_time_s":0.0,"release_time_s":0.08}})";
 std::FILE* fp = std::fopen("/tmp/test_zero_brake_lag.json", "wb");
 assert(fp != nullptr);
 std::fputs(zero_lag, fp);
 std::fclose(fp);
 assert(!platform::load_robot_config(
 "/tmp/test_zero_brake_lag.json", cfg, &err));
 assert(err.find("positive") != std::string::npos);
 std::remove("/tmp/test_zero_brake_lag.json");
  }

  std::printf("robot_config_native ok\n");
  return 0;
}
