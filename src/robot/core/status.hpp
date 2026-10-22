#pragma once

#include <string>

namespace lost_signal::robot::core {

// Minimal explicit status. No exceptions across the boundary.
struct Status {
  bool ok = true;
  std::string reason;

  static Status Ok() { return {true, {}}; }
  static Status Fail(std::string r) { return {false, std::move(r)}; }

  explicit operator bool() const { return ok; }
};

}  // namespace lost_signal::robot::core
