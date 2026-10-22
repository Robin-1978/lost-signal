#pragma once

#include <cstdint>

#include "robot/iface/message.hpp"
#include "robot/iface/provider.hpp"

namespace lost_signal::sim::providers {

// Sim16Lidar: synthetic 16-ring cloud. One call = one full rotation.
class Sim16LidarProvider
    : public robot::iface::ISensorProvider<robot::iface::Lidar3DCloud> {
 public:
  Sim16LidarProvider(const char* endpoint_id, int azimuth_steps);

  const robot::iface::EndpointDescriptor& descriptor() const override;
  bool is_ready() const override { return true; }
  bool poll(robot::iface::Lidar3DCloud& out) override;

  void set_step(std::uint64_t step, robot::core::MonotonicNs stamp_ns);

 private:
  robot::iface::EndpointDescriptor desc_;
  int azimuth_steps_;
  std::uint64_t seq_ = 0;
  std::uint64_t step_ = 0;
  robot::core::MonotonicNs stamp_ns_ = 0;
};

}  // namespace lost_signal::sim::providers
