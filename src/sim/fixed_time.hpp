#pragma once

#include <cstdint>

namespace lost_signal::sim {

// One canonical integer timebase for every simulation entry point. Physics,
// actor policies, contact state, and actuator lag advance only on this grid;
// playback rate changes how quickly these quanta are executed in wall time.
inline constexpr std::int64_t kNanosecondsPerSecond = 1'000'000'000;
inline constexpr std::int64_t kPhysicsQuantumNs = 1'000'000; // 1 ms
inline constexpr std::int64_t kScenarioTransitionNs = 20'000'000; // 20 ms

} // namespace lost_signal::sim
