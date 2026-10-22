#pragma once

#include <cstdint>

#include "robot/iface/message.hpp"
#include "robot/iface/provider.hpp"

// SimImu: measured yaw rate + planar specific force from WorldTruth.
// Bias is constant per run, noise is white per poll, both seeded;
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

 private:
  world::World* world_ = nullptr;
  robot::iface::EndpointDescriptor desc_;
  std::uint64_t seq_ = 0;
  ImuRealismConfig realism_;
  std::uint64_t rng_state_ = 0;
};

}  // namespace lost_signal::sim::providers
