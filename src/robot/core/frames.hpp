#pragma once

namespace lost_signal::robot::core {

// Frame convention: map -> odom -> base_link -> sensor link.
// Follows REP-105 naming. Only these frames cross the provider boundary.
enum class Frame {
  kUnknown = 0,
  kMap = 1,
  kOdom = 2,
  kBaseLink = 3,
  kLidarLink = 4,
  kImuLink = 5,
};

}  // namespace lost_signal::robot::core
