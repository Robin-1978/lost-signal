#include "sim/providers/lidar2d.hpp"

#include <cmath>

namespace lost_signal::sim::providers {

Sim2DLidarProvider::Sim2DLidarProvider(const char* endpoint_id, int beam_count,
                                       float max_range_m)
    : beam_count_(beam_count), max_range_m_(max_range_m) {
  desc_.endpoint_id = endpoint_id;
  desc_.role = robot::iface::EndpointRole::kLidar;
  desc_.backend = robot::iface::ProviderBackend::kSim;
  desc_.frame_id = "lidar_link";
}

const robot::iface::EndpointDescriptor& Sim2DLidarProvider::descriptor()
    const {
  return desc_;
}

void Sim2DLidarProvider::set_step(std::uint64_t step,
                                  robot::core::MonotonicNs stamp_ns) {
  step_ = step;
  stamp_ns_ = stamp_ns;
}

bool Sim2DLidarProvider::poll(robot::iface::Lidar2DScan& out) {
  seq_ += 1;
  out.seq = seq_;
  out.stamp = stamp_ns_;
  out.src = desc_.endpoint_id;
  out.frm = robot::core::Frame::kLidarLink;
  // Deterministic wall at max_range with a synthetic notch drifting by step,
  // so tracking/estimation tests see motion without any truth leak.
  out.ranges_m.assign(beam_count_, max_range_m_);
  int notch = static_cast<int>(step_ % beam_count_);
  out.ranges_m[notch] = max_range_m_ * 0.5f;
  out.angle_min_rad = -static_cast<float>(M_PI);
  out.angle_increment_rad =
      static_cast<float>(2.0 * M_PI / beam_count_);
  return out.valid();
}

}  // namespace lost_signal::sim::providers
