#pragma once

#include "robot/iface/message.hpp"
#include "robot/iface/provider.hpp"

namespace lost_signal::sim::providers {

// SimImuProvider: synthetic measured yaw rate + planar acceleration.
// Deterministic zero-motion v1; noise/bias models extend this class.
class SimImuProvider
    : public robot::iface::ISensorProvider<robot::iface::ImuMessage> {
 public:
  SimImuProvider();

  const robot::iface::EndpointDescriptor& descriptor() const override;
  bool is_ready() const override { return true; }
  bool poll(robot::iface::ImuMessage& out) override;

  void set_step(std::uint64_t step, robot::core::MonotonicNs stamp_ns);

 private:
  robot::iface::EndpointDescriptor desc_;
  std::uint64_t seq_ = 0;
  robot::core::MonotonicNs stamp_ns_ = 0;
};

}  // namespace lost_signal::sim::providers
