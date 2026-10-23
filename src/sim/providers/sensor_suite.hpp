#pragma once

#include <cstdint>
#include <deque>

#include "robot/app/robot.hpp"
#include "robot/iface/envelope.hpp"
#include "robot/platform/description.hpp"
#include "sim/providers/lidar2d.hpp"
#include "sim/providers/lidar3d.hpp"
#include "sim/providers/sim_imu.hpp"
#include "sim/providers/sim_motion.hpp"
#include "sim/world/world.hpp"

namespace lost_signal::sim::providers {

// Deterministic delivery shaping (S7 sensor realism, SIM-side only).
// Seeded, replay-deterministic, default off (all zeros = passthrough).
struct DeliveryConfig {
  std::uint64_t seed = 0;
  // Whole-tick hold-back: the k-th poll delivers the sample from k polls
  // ago (acquisition stamps preserved, never rewritten).
  int latency_steps = 0;
  // Per-sample loss probability in [0, 1]: dropped envelopes never
  // deliver. The poll reports no data that tick (existing path).
  double drop_probability = 0.0;
};

// Bounded IMU realism lives on the device model (SimImuProvider); the
// suite only forwards the switch. Delivery shaping stays here: it is
// envelope-level orchestration, not a device model.

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
  void consume(const robot::iface::ActuatorCommand& cmd);
  // Sensor realism switches (S7). Invalid configs are ignored (previous
  // settings kept, never half-applied). Default off = exact truth.
  void set_delivery(DeliveryConfig config);
  void set_imu_realism(ImuRealismConfig config);
  // Sensor mount extrinsics in the base frame (xyz meters, RPY rad).
  // Rejected (non-finite) input keeps the previous mount. Default zeros.
  void set_lidar_mount(const double xyz_m[3], const double rpy_rad[3]) {
    lidar_.set_mount(xyz_m, rpy_rad);
    lidar3d_.set_mount(xyz_m, rpy_rad);
  }
  void set_imu_mount(const double xyz_m[3], const double rpy_rad[3]) {
    imu_.set_mount(xyz_m, rpy_rad);
  }
  // Scan source: single horizontal ring (default, legacy) or the
  // multi-line composite (ground-filtered min-per-azimuth from the 16-line
  // cloud — sees low plates the horizontal ring flies over). Pre-run only.
  void set_composite_scan(bool on) { composite_scan_ = on; }
  bool composite_scan() const { return composite_scan_; }
  // Bulk mounts from the robot description: matched by link name against
  // each provider's frame_id (lidar_link, imu_link). Unmatched sensors
  // are ignored; motion needs no mount (joint-space + base twist).
  void set_mounts(
      const std::vector<robot::platform::SensorMount>& sensors);

  SimMotionEndpoint& motion() { return motion_; }
  Sim2DLidarProvider& lidar() { return lidar_; }
  Sim3DLidarProvider& lidar3d() { return lidar3d_; }
  SimImuProvider& imu() { return imu_; }

 private:
  world::World* world_;
  SimMotionEndpoint motion_;
  Sim2DLidarProvider lidar_;
  Sim3DLidarProvider lidar3d_;
  SimImuProvider imu_;
  bool composite_scan_ = false;
  std::uint64_t composite_seq_ = 0;
  DeliveryConfig delivery_;
  std::deque<robot::iface::ObservationFrame> delivery_queue_;
  std::uint64_t delivery_rng_ = 0;
};

}  // namespace lost_signal::sim::providers
