#pragma once

#include "robot/core/status.hpp"
#include "robot/iface/task.hpp"
#include "robot/lifecycle/lifecycle.hpp"

namespace lost_signal::robot::executive {

// Task/resource authority. Owns lifecycle, never computes wheel commands.
// A lifecycle module: submit() is rejected until Active.
class RobotExecutive : public lifecycle::ILifecycleModule {
 public:
  const char* name() const override { return "executive"; }
  core::Status configure() override;
  core::Status activate() override;
  core::Status deactivate() override;
  core::Status cleanup() override;
  lifecycle::ModuleState state() const override { return state_; }

  core::Status submit(const iface::TaskRequest& req);
  iface::TaskStatus status() const { return status_; }

 private:
  lifecycle::ModuleState state_ = lifecycle::ModuleState::kUnconfigured;
  iface::TaskStatus status_{};
  std::string active_place_;
};

}  // namespace lost_signal::robot::executive
