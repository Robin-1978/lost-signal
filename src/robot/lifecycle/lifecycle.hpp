#pragma once

#include <string>

#include "robot/core/status.hpp"

namespace lost_signal::robot::lifecycle {

// Per-module lifecycle, Nav2-style: configure once, then activate for the
// running phase. A module that fails to configure blocks the whole bringup;
// RobotApplication refuses to tick until every module is Active.
enum class ModuleState {
  kUnconfigured = 0,
  kInactive = 1,  // configured, not processing
  kActive = 2,    // processing
  kFailed = 3,    // terminal until cleanup + re-configure
};

inline const char* to_string(ModuleState s) {
  switch (s) {
    case ModuleState::kUnconfigured:
      return "unconfigured";
    case ModuleState::kInactive:
      return "inactive";
    case ModuleState::kActive:
      return "active";
    case ModuleState::kFailed:
      return "failed";
  }
  return "unknown";
}

class ILifecycleModule {
 public:
  virtual ~ILifecycleModule() = default;
  virtual const char* name() const = 0;
  virtual core::Status configure() = 0;
  virtual core::Status activate() = 0;
  virtual core::Status deactivate() = 0;
  virtual core::Status cleanup() = 0;
  virtual ModuleState state() const = 0;
};

}  // namespace lost_signal::robot::lifecycle
