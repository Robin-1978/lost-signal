#pragma once

#include <string>

namespace lost_signal::robot::iface {

// Typed operator intent. Semantic place only: resolved through IMapProvider,
// never as a raw coordinate from SIM truth.
struct TaskRequest {
  std::string request_id;
  std::string place_id;  // empty = no goal (hold position)
};

enum class TaskState {
  kIdle = 0,
  kActive = 1,
  kDone = 2,
  kFailed = 3,
};

struct TaskStatus {
  TaskState state = TaskState::kIdle;
  std::string detail;
};

}  // namespace lost_signal::robot::iface
