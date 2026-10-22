#include "sim/providers/lidar16.hpp"

#include <cmath>

namespace lost_signal::sim::providers {

Sim16LidarProvider::Sim16LidarProvider(const char* endpoint_id,
                                       int azimuth_steps)
    : azimuth_steps_(azimuth_steps) {
  desc_.endpoint_id = endpoint_id;
  desc_.role = robot::iface::EndpointRole::kLidar;
  desc_.backend = robot::iface::ProviderBackend::kSim;
  desc_.frame_id = "lidar_link";
}

const robot::iface::EndpointDescriptor& Sim16LidarProvider::descriptor()
    const {
  return desc_;
}

void Sim16LidarProvider::set_step(std::uint64_t step,
                                  robot::core::MonotonicNs stamp_ns) {
  step_ = step;
  stamp_ns_ = stamp_ns;
}

bool Sim16LidarProvider::poll(robot::iface::Lidar3DCloud& out) {
  seq_ += 1;
  out.seq = seq_;
  out.stamp = stamp_ns_;
  out.src = desc_.endpoint_id;
  out.frm = robot::core::Frame::kLidarLink;
  out.ring_count = 16;
  out.scan_period_s = 0.1f;
  out.points.clear();
  out.points.reserve(16 * azimuth_steps_);
  for (int ring = 0; ring < 16; ++ring) {
    float z = -0.4f + ring * 0.05f;
    for (int i = 0; i < azimuth_steps_; ++i) {
      float az = static_cast<float>(2.0 * M_PI * i / azimuth_steps_);
      float r = 5.0f - 0.01f * (step_ % 50);
      robot::iface::Lidar3DPoint p;
      p.x_m = r * std::cos(az);
      p.y_m = r * std::sin(az);
      p.z_m = z;
      p.intensity = 0.8f;
      p.ring = static_cast<std::uint16_t>(ring);
      p.time_offset_s =
          -out.scan_period_s + out.scan_period_s * i / azimuth_steps_;
      out.points.push_back(p);
    }
  }
  return out.valid();
}

}  // namespace lost_signal::sim::providers
