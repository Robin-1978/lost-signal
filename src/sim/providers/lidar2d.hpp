#pragma once

#include <cstdint>

#include "robot/iface/message.hpp"
#include "robot/iface/provider.hpp"

namespace lost_signal::sim::providers {

// Sim2DLidar: synthetic planar scan from WorldTruth (SIM-owned generation).
// ROBOT only sees the typed Lidar2DScan through ISensorProvider.
class Sim2DLidarProvider
    : public robot::iface::ISensorProvider<robot::iface::Lidar2DScan> {
 public:
  Sim2DLidarProvider(const char* endpoint_id, int beam_count,
                     float max_range_m);

  const robot::iface::EndpointDescriptor& descriptor() const override;
  bool is_ready() const override { return true; }
  bool poll(robot::iface::Lidar2DScan& out) override;

  // Test/demo hook: advance the synthetic world step driving the scan.
  void set_step(std::uint64_t step, robot::core::MonotonicNs stamp_ns);

 private:
  robot::iface::EndpointDescriptor desc_;
  int beam_count_;
  float max_range_m_;
  std::uint64_t seq_ = 0;
  std::uint64_t step_ = 0;
  robot::core::MonotonicNs stamp_ns_ = 0;
};

}  // namespace lost_signal::sim::providers
