#include "robot/iface/message.hpp"

namespace lost_signal::robot::iface {

bool Lidar2DScan::valid() const {
  if (src.empty()) return false;
  if (stamp == core::kInvalidTime) return false;
  if (seq == 0) return false;
  if (ranges_m.empty()) return false;
  if (angle_increment_rad <= 0.0f) return false;
  // Intensities ride parallel when present; empty means unknown (legacy).
  if (!intensities.empty() && intensities.size() != ranges_m.size()) {
    return false;
  }
  // Time offsets ride parallel when present; empty means frozen (legacy).
  if (!time_offsets_s.empty() && time_offsets_s.size() != ranges_m.size()) {
    return false;
  }
  return true;
}

bool Lidar3DCloud::valid() const {
  if (src.empty()) return false;
  if (stamp == core::kInvalidTime) return false;
  if (seq == 0) return false;
  if (points.empty()) return false;
  if (ring_count == 0) return false;
  if (scan_period_s <= 0.0f) return false;
  return true;
}

bool ImuMessage::valid() const {
  if (src.empty()) return false;
  if (stamp == core::kInvalidTime) return false;
  if (seq == 0) return false;
  return true;
}

}  // namespace lost_signal::robot::iface
