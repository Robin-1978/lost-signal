#include <cassert>
#include <cstdio>
#include <string>

#include "robot/lifecycle/bringup.hpp"

namespace {

class Stub : public lost_signal::robot::lifecycle::ILifecycleModule {
 public:
  Stub(const char* n, bool fail_configure = false, bool fail_activate = false)
      : name_(n), fail_c_(fail_configure), fail_a_(fail_activate) {}

  const char* name() const override { return name_; }
  lost_signal::robot::core::Status configure() override {
    using ls = lost_signal::robot::lifecycle::ModuleState;
    using lost_signal::robot::core::Status;
    if (fail_c_) {
      state_ = ls::kFailed;
      return Status::Fail("stub configure fail");
    }
    state_ = ls::kInactive;
    log_ += "C";
    return Status::Ok();
  }
  lost_signal::robot::core::Status activate() override {
    using ls = lost_signal::robot::lifecycle::ModuleState;
    using lost_signal::robot::core::Status;
    if (state_ != ls::kInactive) return Status::Fail("not inactive");
    if (fail_a_) {
      state_ = ls::kFailed;
      return Status::Fail("stub activate fail");
    }
    state_ = ls::kActive;
    log_ += "A";
    return Status::Ok();
  }
  lost_signal::robot::core::Status deactivate() override {
    using ls = lost_signal::robot::lifecycle::ModuleState;
    using lost_signal::robot::core::Status;
    if (state_ == ls::kActive) {
      state_ = ls::kInactive;
      log_ += "D";
    }
    return Status::Ok();
  }
  lost_signal::robot::core::Status cleanup() override {
    using ls = lost_signal::robot::lifecycle::ModuleState;
    using lost_signal::robot::core::Status;
    state_ = ls::kUnconfigured;
    log_ += "K";
    return Status::Ok();
  }
  lost_signal::robot::lifecycle::ModuleState state() const override {
    return state_;
  }
  std::string log_;

 private:
  const char* name_;
  bool fail_c_;
  bool fail_a_;
  lost_signal::robot::lifecycle::ModuleState state_ =
      lost_signal::robot::lifecycle::ModuleState::kUnconfigured;
};

}  // namespace

int main() {
  using namespace lost_signal::robot::lifecycle;

  // Happy path: configure/activate in order, deactivate/cleanup reversed.
  {
    BringupManager m;
    Stub a("a"), b("b");
    assert(m.add_module(&a) && m.add_module(&b));
    assert(m.configure_all());
    assert(a.log_ == "C" && b.log_ == "C");
    assert(m.state() == ModuleState::kInactive);
    assert(m.activate_all());
    assert(a.log_ == "CA" && b.log_ == "CA");
    assert(m.state() == ModuleState::kActive);
    m.deactivate_all();
    assert(a.log_ == "CAD" && b.log_ == "CAD");  // b first, then a
    m.cleanup_all();
    assert(m.state() == ModuleState::kUnconfigured);
  }

  // Configure failure blocks bringup and names the module.
  {
    BringupManager m;
    Stub a("a"), bad("bad", true), c("c");
    m.add_module(&a);
    m.add_module(&bad);
    m.add_module(&c);
    assert(!m.configure_all());
    assert(std::string(m.failed_module()) == "bad");
    assert(c.log_.empty());  // never reached
    assert(m.state() == ModuleState::kFailed);
  }

  // Activate failure rolls back already-activated modules (reverse order).
  {
    BringupManager m;
    Stub a("a"), bad("bad", false, true), c("c");
    m.add_module(&a);
    m.add_module(&bad);
    m.add_module(&c);
    assert(m.configure_all());
    a.log_.clear();
    bad.log_.clear();
    c.log_.clear();
    assert(!m.activate_all());
    assert(std::string(m.failed_module()) == "bad");
    assert(a.log_ == "AD");  // activated then rolled back
    assert(c.log_.empty());
    assert(m.state() == ModuleState::kFailed);
  }

  std::printf("lifecycle ok\n");
  return 0;
}
