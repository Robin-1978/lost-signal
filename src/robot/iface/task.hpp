#pragma once

#include <cstdint>
#include <string>

namespace lost_signal::robot::iface {

// Typed operator intent. Semantic place only: resolved through IMapProvider,
// never as a raw coordinate from SIM truth.
struct TaskRequest {
  std::string request_id;
  std::string place_id;  // empty = hold position (completes immediately)
};

// Async goal handle. request_goal returns immediately with a handle; the
// caller polls goal_status and may cancel_goal. Handles are scoped to the
// executive that issued them; id 0 is never valid.
struct GoalHandle {
  std::uint64_t id = 0;
  bool valid() const { return id != 0; }
  bool operator==(const GoalHandle& other) const { return id == other.id; }
};

enum class GoalState {
  kPending = 0,  // accepted, not yet dispatched (currently unused: single slot)
  kActive = 1,   // dispatched, en route
  kDone = 2,     // arrived (or hold acknowledged)
  kFailed = 3,   // unknown place, bringup violation, unknown handle query
  kCanceled = 4,  // operator cancel or preemption by a newer goal
};

struct GoalStatus {
  GoalHandle handle;
  GoalState state = GoalState::kFailed;
  std::string detail;    // stable code for logs/tests ("arrived", "preempted"...)
  std::string place_id;  // semantic target of this goal
};

inline const char* goal_state_name(GoalState state) {
  switch (state) {
    case GoalState::kPending:
      return "pending";
    case GoalState::kActive:
      return "active";
    case GoalState::kDone:
      return "done";
    case GoalState::kFailed:
      return "failed";
    case GoalState::kCanceled:
      return "canceled";
  }
  return "unknown";
}

}  // namespace lost_signal::robot::iface
