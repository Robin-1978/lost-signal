#pragma once

namespace lost_signal::robot::iface {

// Wire/API contract versions. Bump when meaning changes, never silently
// reinterpret an old field.
inline constexpr int kApiVersion = 1;
inline constexpr int kMessageSchemaVersion = 1;

}  // namespace lost_signal::robot::iface
