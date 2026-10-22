#include "robot/providers/real_lidar2d.hpp"

namespace lost_signal::robot::providers {

Real2DLidarProvider::Real2DLidarProvider(const char* endpoint_id,
                                         std::vector<float> ranges_m,
                                         float angle_increment_rad)
    : ranges_m_(std::move(ranges_m)),
      angle_increment_rad_(angle_increment_rad) {
  desc_.endpoint_id = endpoint_id;
  desc_.role = iface::EndpointRole::kLidar;
  desc_.backend = iface::ProviderBackend::kNative;
  desc_.frame_id = "lidar_link";
}

const iface::EndpointDescriptor& Real2DLidarProvider::descriptor() const {
  return desc_;
}

void Real2DLidarProvider::feed(core::MonotonicNs stamp_ns) {
  batch_ += 1;
  frames_fed_ = true;
  next_stamp_ns_ = stamp_ns;
}

bool Real2DLidarProvider::poll(iface::Lidar2DScan& out) {
  if (cursor_ >= batch_) return false;  // replay exhausted (or never fed)
  seq_ += 1;
  out.seq = seq_;
  out.stamp = next_stamp_ns_;
  out.src = desc_.endpoint_id;
  out.frm = core::Frame::kLidarLink;
  out.ranges_m = ranges_m_;
  out.angle_min_rad = 0.0f;
  out.angle_increment_rad = angle_increment_rad_;
  cursor_ += 1;
  return out.valid();
}

}  // namespace lost_signal::robot::providers
