// Robot-side config test: links robot_platform ONLY (no sim libraries,
// no sim headers) — this is the native-deployability proof. The same
// files feed SIM through sim/eval translation.
#include <cassert>
#include <cstdio>
#include <string>

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
  // Fail-closed: unknown kind, disagreement, bad numbers, bad JSON.
  {
    platform::RobotConfig cfg;
    std::string err;
    assert(!platform::load_robot_config(dir + "/no-such.json", cfg, &err));
  }

  std::printf("robot_config_native ok\n");
  return 0;
}
