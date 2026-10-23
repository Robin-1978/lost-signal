#pragma once

#include <cstdint>

#include "robot/core/transform.hpp"
#include "robot/iface/message.hpp"
#include "robot/iface/provider.hpp"

// SimImu: measured yaw rate + planar specific force from WorldTruth,
// expressed in the sensor frame through the mount extrinsics (orientation
// rotates body rates into the device; the yaw-acceleration lever arm at the
// mount offset is exact planar rigid-body kinematics). Bias is constant per run, noise is white per poll, both seeded;
// defaults are exact truth. The estimator never sees truth directly,
// only this device model.
namespace lost_signal::sim::world {
class World;
}  // namespace lost_signal::sim::world

namespace lost_signal::sim::providers {

struct ImuRealismConfig {
  std::uint64_t seed = 0;
  double gyro_noise_radps = 0.0;   // white, per axis
  double gyro_bias_radps = 0.0;    // constant z-bias for the run
  double accel_noise_mps2 = 0.0;   // white, per axis
};

class SimImuProvider
    : public robot::iface::ISensorProvider<robot::iface::ImuMessage> {
 public:
  explicit SimImuProvider(world::World* world);

  const robot::iface::EndpointDescriptor& descriptor() const override;
  bool is_ready() const override { return world_ != nullptr; }
  bool poll(robot::iface::ImuMessage& out) override;
  // Device-model tuning (S7). Invalid configs are ignored (previous
  // settings kept, never half-applied).
  void set_realism(ImuRealismConfig config);
  // Sensor mount extrinsics in the base frame (xyz meters, RPY rad,
  // REP-103). Non-finite input is rejected, previous mount kept. Default
  // zeros = IMU at the base origin, aligned with base axes.
  void set_mount(const double xyz_m[3], const double rpy_rad[3]);

 private:
  world::World* world_ = nullptr;
  robot::iface::EndpointDescriptor desc_;
  std::uint64_t seq_ = 0;
  ImuRealismConfig realism_;
  std::uint64_t rng_state_ = 0;
  double mount_xyz_m_[3] = {0.0, 0.0, 0.0};
  double mount_rpy_rad_[3] = {0.0, 0.0, 0.0};
  // Previous yaw rate + stamp for the lever-arm angular acceleration.
  // First poll (or repeated stamp) contributes no angular acceleration.
  double prev_yaw_rate_radps_ = 0.0;
  double prev_stamp_s_ = -1.0;
};

}  // namespace lost_signal::sim::providers
