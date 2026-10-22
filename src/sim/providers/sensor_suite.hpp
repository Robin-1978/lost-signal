#pragma once

#include "robot/app/robot.hpp"
#include "robot/iface/envelope.hpp"
#include "sim/providers/lidar2d.hpp"
#include "sim/providers/sim_imu.hpp"
#include "sim/providers/sim_motion.hpp"
#include "sim/world/world.hpp"

namespace lost_signal::sim::providers {

// SimSensorSuite: the single composer of sim sensing. Owns the motion
// endpoint + lidar + imu providers, claims all three descriptors on the app,
// and packs their typed messages into ONE envelope via the canonical
// robot-owned pack helpers. Harnesses (ScenarioSession, SimRobot) never
// hand-pack and never touch WorldTruth beyond stepping the world.
class SimSensorSuite {
 public:
  explicit SimSensorSuite(world::World* world);

  void register_all(robot::app::IRobot& app);
  bool poll_envelope(robot::iface::ObservationFrame& out);
  void consume(const robot::iface::MotionCommand& cmd);

  SimMotionEndpoint& motion() { return motion_; }
  Sim2DLidarProvider& lidar() { return lidar_; }
  SimImuProvider& imu() { return imu_; }

 private:
  world::World* world_;
  SimMotionEndpoint motion_;
  Sim2DLidarProvider lidar_;
  SimImuProvider imu_;
};

}  // namespace lost_signal::sim::providers
