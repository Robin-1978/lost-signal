#pragma once

#include <cstdint>
#include <deque>

#include "robot/app/robot.hpp"
#include "robot/iface/envelope.hpp"
#include "sim/providers/lidar2d.hpp"
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

// Bounded IMU realism (SIM-side only). Bias is constant per run, noise is
// white per poll, both seeded. Defaults are exact truth; the estimator
// already tolerates the bounded error (gyro-led blend + slip gate).
struct ImuRealismConfig {
  std::uint64_t seed = 0;
  double gyro_noise_radps = 0.0;   // white, per axis
  double gyro_bias_radps = 0.0;    // constant z-bias for the run
  double accel_noise_mps2 = 0.0;   // white, per axis
};

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

  SimMotionEndpoint& motion() { return motion_; }
  Sim2DLidarProvider& lidar() { return lidar_; }
  SimImuProvider& imu() { return imu_; }

 private:
  world::World* world_;
  SimMotionEndpoint motion_;
  Sim2DLidarProvider lidar_;
  SimImuProvider imu_;
  DeliveryConfig delivery_;
  ImuRealismConfig imu_realism_;
  std::deque<robot::iface::ObservationFrame> delivery_queue_;
  std::uint64_t delivery_rng_ = 0;
  std::uint64_t imu_rng_ = 0;
};

}  // namespace lost_signal::sim::providers
