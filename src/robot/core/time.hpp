#pragma once

#include <cstdint>

namespace lost_signal::robot::core {

// One monotonic clock for the whole robot process. Nanoseconds since an
// arbitrary robot-boot origin. Wall-clock time must never enter the decision
// path; convert once at a provider boundary.
using MonotonicNs = std::int64_t;

inline constexpr MonotonicNs kInvalidTime = -1;

}  // namespace lost_signal::robot::core
