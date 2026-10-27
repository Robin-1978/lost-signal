#pragma once

#include "robot/transport/ring_buffer.hpp"

namespace lost_signal::robot::iface {

// Compatibility alias for early integrations. Transport buffering belongs to
// robot::transport; interface contracts do not own a receive queue.
using RingBuffer = robot::transport::RingBuffer;

} // namespace lost_signal::robot::iface
