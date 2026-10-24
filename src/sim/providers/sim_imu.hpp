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
  double gyro_noise_radps = 0.0;  // white, per axis
  // Exact calibration trim, per axis (x, y, z): harness knob for
  // bias-robustness probes, adds without scaling. Zero by default.
  double gyro_trim_radps[3] = {0.0, 0.0, 0.0};
  // Device imperfection (seeded, never truth): per-run bias drawn once per
  // axis from N(0, stddev) at set_realism, then a random walk per poll
  // (sigma * sqrt(dt)). Both zero by default = exact.
  double gyro_bias_stddev_radps = 0.0;
  double gyro_bias_walk_radps_sqrt_s = 0.0;
  double accel_noise_mps2 = 0.0;  // white, per axis
};

class SimImuProvider
    : public robot::iface::ISensorProvider<robot::iface::ImuMessage> {
 public:
  explicit SimImuProvider(world::World* world);

  const robot::iface::EndpointDescriptor& descriptor() const override;
  bool is_ready() const override { return world_ != nullptr; }
  bool poll(robot::iface::ImuMessage& out) override;
  // Device-model tuning (S7). Invalid configs are ignored (previous
  // settings kept, never half-applied). Setting realism redraws the
  // per-run gyro bias (seeded determinism: same seed, same bias).
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
  // Independent seeded streams (draw order never couples gyro/accel/bias).
  std::uint64_t gyro_state_ = 0;
  std::uint64_t accel_state_ = 0;
  std::uint64_t bias_state_ = 0;
  // Per-run drawn bias, per axis (x, y, z) + walk on top. Redrawn at
  // set_realism; trim (config) adds separately as the exact harness knob.
  double bias_draw_radps_[3] = {0.0, 0.0, 0.0};
  double mount_xyz_m_[3] = {0.0, 0.0, 0.0};
  double mount_rpy_rad_[3] = {0.0, 0.0, 0.0};
  // Previous world velocity + attitude + stamp for differentiated
  // quantities: body-origin linear acceleration, angular acceleration, and
  // roll/pitch rates from the attitude delta (truth carries no roll/pitch
  // rate channel; yaw rides the first-class truth rate). Height must be
  // differentiated twice before it is acceleration; the intermediate
  // vertical velocity is retained explicitly. First/repeated stamps
  // contribute no derivatives.
  double prev_yaw_rate_radps_ = 0.0;
  double prev_world_vx_mps_ = 0.0;
  double prev_world_vy_mps_ = 0.0;
  double prev_world_vz_mps_ = 0.0;
  double prev_body_z_m_ = 0.0;
  bool has_prev_vertical_velocity_ = false;
  double prev_stamp_s_ = -1.0;
  bool has_prev_attitude_ = false;
  robot::core::Quaternion prev_body_q_{0.0, 0.0, 0.0, 1.0};
  double prev_omega_x_ = 0.0, prev_omega_y_ = 0.0;
};

}  // namespace lost_signal::sim::providers
