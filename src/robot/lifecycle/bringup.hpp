#pragma once

#include <cstddef>
#include <string>

#include "robot/core/status.hpp"
#include "robot/lifecycle/lifecycle.hpp"

namespace lost_signal::robot::lifecycle {

// Ordered bringup over non-owning module pointers (owned by the composition
// root). configure_all() runs in registration order; activate_all() requires
// every module Inactive and rolls back (reverse-order deactivate) on the
// first failure. deactivate_all()/cleanup_all() run in reverse order.
class BringupManager {
 public:
  static constexpr std::size_t kMaxModules = 16;

  bool add_module(ILifecycleModule* module);
  std::size_t size() const { return count_; }

  core::Status configure_all();
  core::Status activate_all();
  void deactivate_all();
  void cleanup_all();

  // Aggregate: Failed if any module failed, else the minimum progress state.
  ModuleState state() const;
  const char* failed_module() const { return failed_; }

 private:
  ILifecycleModule* modules_[kMaxModules]{};
  std::size_t count_ = 0;
  const char* failed_ = "";
};

}  // namespace lost_signal::robot::lifecycle
