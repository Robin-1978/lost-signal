#include "robot/lifecycle/bringup.hpp"

namespace lost_signal::robot::lifecycle {

bool BringupManager::add_module(ILifecycleModule* module) {
  if (module == nullptr || count_ >= kMaxModules) return false;
  modules_[count_++] = module;
  return true;
}

core::Status BringupManager::configure_all() {
  for (std::size_t i = 0; i < count_; ++i) {
    core::Status s = modules_[i]->configure();
    if (!s) {
      failed_ = modules_[i]->name();
      return core::Status::Fail(std::string("configure failed: ") + failed_ +
                               ": " + s.reason);
    }
  }
  failed_ = "";
  return core::Status::Ok();
}

core::Status BringupManager::activate_all() {
  std::size_t activated = 0;
  for (std::size_t i = 0; i < count_; ++i) {
    core::Status s = modules_[i]->activate();
    if (!s) {
      failed_ = modules_[i]->name();
      for (std::size_t j = activated; j > 0; --j) {
        (void)modules_[j - 1]->deactivate();
      }
      return core::Status::Fail(std::string("activate failed: ") + failed_ +
                               ": " + s.reason);
    }
    activated += 1;
  }
  failed_ = "";
  return core::Status::Ok();
}

void BringupManager::deactivate_all() {
  for (std::size_t i = count_; i > 0; --i) {
    (void)modules_[i - 1]->deactivate();
  }
}

void BringupManager::cleanup_all() {
  for (std::size_t i = count_; i > 0; --i) {
    (void)modules_[i - 1]->cleanup();
  }
}

ModuleState BringupManager::state() const {
  if (count_ == 0) return ModuleState::kUnconfigured;
  bool any_failed = false;
  bool all_active = true;
  bool all_inactive = true;
  for (std::size_t i = 0; i < count_; ++i) {
    ModuleState s = modules_[i]->state();
    if (s == ModuleState::kFailed) any_failed = true;
    if (s != ModuleState::kActive) all_active = false;
    if (s != ModuleState::kInactive) all_inactive = false;
  }
  if (any_failed) return ModuleState::kFailed;
  if (all_active) return ModuleState::kActive;
  if (all_inactive) return ModuleState::kInactive;
  return ModuleState::kUnconfigured;  // mixed / transitional
}

}  // namespace lost_signal::robot::lifecycle
