#include <array>
#include <cassert>
#include <cstdio>
#include <cstdint>
#include <deque>
#include <utility>
#include <vector>

#include "robot/app/native_runtime.hpp"

namespace {
using namespace lost_signal::robot;

constexpr core::MonotonicNs kControlPeriodNs = 20'000'000;

iface::MotionCapabilities hil_capabilities() {
  iface::MotionCapabilities capabilities;
  capabilities.control_owner = iface::MotionControlOwner::kHostMotionLoop;
  capabilities.chassis_mode = iface::ActuatorMode::kDiffDuty;
  capabilities.drive_count = 2;
  capabilities.control_period_ns = kControlPeriodNs;
  capabilities.feedback_period_ns = kControlPeriodNs;
  // The application command contract currently uses its shared 200 ms
  // watchdog; the board profile must accept that typed age and may still
  // choose a shorter expiry in a measured deployment profile.
  capabilities.maximum_command_age_ns = 300'000'000;
  capabilities.profile_id = "hil.native.contract";
  capabilities.profile_revision = 1;
  return capabilities;
}

iface::MotionFeedback feedback(core::MonotonicNs stamp_ns,
                               std::uint64_t sequence) {
  iface::MotionFeedback value;
  value.sequence = sequence;
  value.stamp_ns = stamp_ns;
  value.source = "hil.encoder";
  value.frame = core::Frame::kBaseLink;
  value.odometry_valid = true;
  value.forward_mps = 0.0;
  value.yaw_rate_radps = 0.0;
  value.drive_count = 2;
  return value;
}

// A deterministic board model for the native seam. It deliberately has no
// access to SIM objects: it accepts typed setpoints and produces only typed
// encoder feedback. `advance()` is a board-timer replay, so expiry/watchdog
// behavior is tested without sleeping on wall-clock time.
class ReplayBoard final : public iface::IChassisMcuTransport {
 public:
  ReplayBoard() : capabilities_(hil_capabilities()) {}

  const iface::ChassisCapabilities& capabilities() const override {
    return capabilities_;
  }

  bool send(const iface::ChassisSetpoint& setpoint) override {
    if (!iface::valid_chassis_setpoint(setpoint, capabilities_,
                                       setpoint.stamp_ns)) {
      return false;
    }
    sent_.push_back(setpoint);
    return true;
  }

  bool poll_feedback(iface::MotionFeedback& out) override {
    if (feedback_.empty()) return false;
    out = feedback_.front();
    feedback_.pop_front();
    return true;
  }

  void mechanical_safe_stop(core::MonotonicNs now_ns) noexcept override {
    ++mechanical_stops_;
    last_stop_ns_ = now_ns;
  }

  void queue_feedback(iface::MotionFeedback value) {
    feedback_.push_back(std::move(value));
  }

  // This is the board's local timer path. Once the most recently accepted
  // command has expired, the board stops even if the host never calls again.
  bool advance(core::MonotonicNs now_ns) {
    if (sent_.empty() || watchdog_stopped_) return watchdog_stopped_;
    if (sent_.back().expiry_ns <= now_ns) {
      watchdog_stopped_ = true;
      ++watchdog_stops_;
    }
    return watchdog_stopped_;
  }

  const std::vector<iface::ChassisSetpoint>& sent() const { return sent_; }
  std::size_t mechanical_stops() const { return mechanical_stops_; }
  std::size_t watchdog_stops() const { return watchdog_stops_; }
  bool watchdog_stopped() const { return watchdog_stopped_; }
  core::MonotonicNs last_stop_ns() const { return last_stop_ns_; }

 private:
  iface::ChassisCapabilities capabilities_;
  std::deque<iface::MotionFeedback> feedback_;
  std::vector<iface::ChassisSetpoint> sent_;
  std::size_t mechanical_stops_ = 0;
  std::size_t watchdog_stops_ = 0;
  bool watchdog_stopped_ = false;
  core::MonotonicNs last_stop_ns_ = core::kInvalidTime;
};

class ReplayAdapter final : public app::INativeAdapter {
 public:
  ReplayAdapter() {
    endpoints_.push_back({"hil.encoder", iface::EndpointRole::kEncoder,
                          iface::ProviderBackend::kNative, "base_link"});
  }

  const std::vector<iface::EndpointDescriptor>& endpoints() const override {
    return endpoints_;
  }

  bool poll_observation(iface::ObservationFrame&) override { return false; }

  bool write_actuator(const iface::ActuatorCommand&) override {
    // A typed chassis board owns the transport in this harness; this path
    // must never be selected by NativeRuntime.
    return false;
  }

  iface::IChassisMcuTransport* chassis_mcu() override { return &board; }

  void mechanical_safe_stop(iface::ActuatorMode, core::MonotonicNs now_ns) noexcept
      override {
    ++mechanical_stops;
    last_stop_ns = now_ns;
  }

  ReplayBoard board;
  std::size_t mechanical_stops = 0;
  core::MonotonicNs last_stop_ns = core::kInvalidTime;

 private:
  std::vector<iface::EndpointDescriptor> endpoints_;
};

struct Trace {
  std::vector<bool> cycle_results;
  std::vector<iface::MotionSetpointKind> setpoint_kinds;
  std::vector<std::array<double, 3>> body_values;
  std::size_t conformance_failures = 0;
  std::size_t runtime_stops = 0;
  std::size_t board_watchdog_stops = 0;
};

Trace run_nominal_replay() {
  app::RobotApplication application(nullptr);
  ReplayAdapter adapter;
  app::NativeRuntime runtime(application, adapter,
                             {.decision_period_ns = kControlPeriodNs});
  assert(runtime.start());

  const core::MonotonicNs start = app::native_monotonic_now_ns();
  Trace trace;
  // Fixed event script. The second frame is acquired 5 ms before release,
  // cycle three intentionally drops feedback, and cycle four recovers with
  // the next sequence. Receipt, not a guessed wall-clock sleep, drives the
  // liveness contract.
  constexpr std::array<core::MonotonicNs, 6> kAcquisitionDelay = {
      0, 5'000'000, 0, 10'000'000, 0, 5'000'000};
  constexpr std::array<bool, 6> kHasFeedback = {true, true, false,
                                                 true, true, true};
  std::uint64_t sequence = 1;
  for (std::size_t i = 0; i < kHasFeedback.size(); ++i) {
    const core::MonotonicNs now =
        start + static_cast<core::MonotonicNs>(i) * kControlPeriodNs;
    if (kHasFeedback[i]) {
      adapter.board.queue_feedback(
          feedback(now - kAcquisitionDelay[i], sequence++));
    }
    trace.cycle_results.push_back(runtime.cycle(now));
    if (!trace.cycle_results.back()) {
      std::fprintf(stderr, "nominal replay failed at cycle %zu\n", i);
      assert(false);
    }
    const auto& sent = adapter.board.sent().back();
    trace.setpoint_kinds.push_back(sent.kind);
    trace.body_values.push_back(
        {sent.forward_mps, sent.lateral_mps, sent.yaw_rate_radps});
    assert(!adapter.board.advance(now));
  }

  // Let the board timer expire the last command in simulated time. No host
  // cycle or wall-clock wait is involved in this assertion.
  const core::MonotonicNs expiry = adapter.board.sent().back().expiry_ns;
  assert(adapter.board.advance(expiry));
  assert(adapter.board.watchdog_stopped());
  trace.conformance_failures = runtime.stats().conformance_failures;
  trace.runtime_stops = adapter.mechanical_stops;
  trace.board_watchdog_stops = adapter.board.watchdog_stops();
  runtime.shutdown();
  return trace;
}

void assert_same_trace(const Trace& lhs, const Trace& rhs) {
  assert(lhs.cycle_results == rhs.cycle_results);
  assert(lhs.setpoint_kinds == rhs.setpoint_kinds);
  assert(lhs.body_values == rhs.body_values);
  assert(lhs.conformance_failures == rhs.conformance_failures);
  assert(lhs.board_watchdog_stops == rhs.board_watchdog_stops);
}

void test_feedback_timeout_and_mechanical_stop() {
  app::RobotApplication application(nullptr);
  ReplayAdapter adapter;
  app::NativeRuntime runtime(application, adapter,
                             {.decision_period_ns = kControlPeriodNs});
  assert(runtime.start());
  const core::MonotonicNs start = app::native_monotonic_now_ns();
  adapter.board.queue_feedback(feedback(start, 1));
  assert(runtime.cycle(start));

  // The board advertises a 20 ms feedback period; three periods are the
  // runtime liveness window. A 40 ms gap is therefore a deterministic fault.
  const core::MonotonicNs timeout = start + 4 * kControlPeriodNs;
  assert(!runtime.cycle(timeout));
  assert(runtime.stats().conformance_failures > 0);
  assert(adapter.board.mechanical_stops() >= 1);
  assert(adapter.mechanical_stops >= 1);
  runtime.shutdown();
}

void test_sequence_replay_and_delayed_feedback() {
  app::RobotApplication application(nullptr);
  ReplayAdapter adapter;
  app::NativeRuntime runtime(application, adapter,
                             {.decision_period_ns = kControlPeriodNs});
  assert(runtime.start());
  const core::MonotonicNs start = app::native_monotonic_now_ns();
  adapter.board.queue_feedback(feedback(start, 1));
  assert(runtime.cycle(start));

  // A delayed but fresh acquisition is valid when its own stamp is within the
  // provider freshness contract.
  adapter.board.queue_feedback(feedback(start + kControlPeriodNs - 10'000'000, 2));
  assert(runtime.cycle(start + kControlPeriodNs));

  // Replaying sequence 2 is a transport fault even though the payload is
  // finite and the acquisition stamp is fresh.
  adapter.board.queue_feedback(
      feedback(start + 2 * kControlPeriodNs, 2));
  assert(!runtime.cycle(start + 2 * kControlPeriodNs));
  assert(runtime.stats().conformance_failures > 0);
  assert(adapter.board.mechanical_stops() >= 1);
  runtime.shutdown();
}
}  // namespace

int main() {
  const Trace first = run_nominal_replay();
  const Trace second = run_nominal_replay();
  assert_same_trace(first, second);
  assert(first.cycle_results.size() == 6);
  assert(first.board_watchdog_stops == 1);
  test_feedback_timeout_and_mechanical_stop();
  test_sequence_replay_and_delayed_feedback();
  std::puts("native_hil ok");
  return 0;
}
