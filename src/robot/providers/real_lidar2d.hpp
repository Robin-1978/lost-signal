#pragma once

#include <cstddef>
#include <vector>

#include "robot/iface/message.hpp"
#include "robot/iface/provider.hpp"

namespace lost_signal::robot::providers {

// Real2DLidar: native/replay seam for a 2D lidar without a hardware SDK in
// this repo. Constructed from a recorded frame batch (replay) or fed live by
// a native driver adapter outside the deployable lib. Same ISensorProvider
// contract as Sim2DLidar, only the backend flag differs.
class Real2DLidarProvider
    : public iface::ISensorProvider<iface::Lidar2DScan> {
 public:
  Real2DLidarProvider(const char* endpoint_id, std::vector<float> ranges_m,
                      float angle_increment_rad);

  const iface::EndpointDescriptor& descriptor() const override;
  bool is_ready() const override { return !frames_fed_ || cursor_ < batch_; }
  bool poll(iface::Lidar2DScan& out) override;

  // Replay hook: queue one recorded rotation for the next poll.
  void feed(core::MonotonicNs stamp_ns);

 private:
  iface::EndpointDescriptor desc_;
  std::vector<float> ranges_m_;
  float angle_increment_rad_;
  std::uint64_t seq_ = 0;
  std::size_t batch_ = 0;
  std::size_t cursor_ = 0;
  bool frames_fed_ = false;
  core::MonotonicNs next_stamp_ns_ = core::kInvalidTime;
};

}  // namespace lost_signal::robot::providers
