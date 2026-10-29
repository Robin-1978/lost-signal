#include "robot/app/native_runtime.hpp"

#include <algorithm>
#include <chrono>
#include <limits>

#include "robot/iface/envelope.hpp"

namespace lost_signal::robot::app {

namespace {
using SteadyClock = std::chrono::steady_clock;

core::MonotonicNs feedback_timeout_ns(
    const iface::MotionCapabilities& capabilities,
    core::MonotonicNs decision_period_ns) {
 // Three feedback periods carry the same liveness meaning as the provider
 // conformance monitor. The host release period prevents a faster board from
 // expiring between two decision releases; the board's command age remains
 // the hard upper bound from its typed profile.
 constexpr core::MonotonicNs kLivenessPeriods = 3;
 const core::MonotonicNs period = capabilities.feedback_period_ns;
 const core::MonotonicNs max_ns =
     std::numeric_limits<core::MonotonicNs>::max();
 const core::MonotonicNs three_periods =
     period > max_ns / kLivenessPeriods ? max_ns
                                        : period * kLivenessPeriods;
 return std::min(capabilities.maximum_command_age_ns,
                 std::max(decision_period_ns, three_periods));
}

bool registered_encoder_endpoint(const INativeAdapter& adapter,
                                 const iface::MotionFeedback& feedback) {
 const std::string measured_frame = iface::frame_name(feedback.frame);
 for (const auto& endpoint : adapter.endpoints()) {
  if (endpoint.endpoint_id == feedback.source &&
      endpoint.role == iface::EndpointRole::kEncoder &&
      endpoint.backend == iface::ProviderBackend::kNative &&
      (endpoint.frame_id.empty() || endpoint.frame_id == measured_frame)) {
   return true;
  }
 }
 return false;
}

SteadyClock::time_point steady_time_point(core::MonotonicNs stamp_ns) {
 return SteadyClock::time_point(std::chrono::duration_cast<
 SteadyClock::duration>(std::chrono::nanoseconds(stamp_ns)));
}
} // namespace

core::MonotonicNs native_monotonic_now_ns() {
 const auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(
 SteadyClock::now().time_since_epoch()).count();
 return nanoseconds < 0 ? 0 : static_cast<core::MonotonicNs>(nanoseconds);
}

NativeRuntime::NativeRuntime(RobotApplication& application,
 INativeAdapter& adapter, NativeRuntimeConfig config)
 : application_(application), adapter_(adapter), config_(config) {}

NativeRuntime::~NativeRuntime() { shutdown(); }

bool NativeRuntime::start() {
 if (active_.load() || config_.decision_period_ns <= 0) return false;
 chassis_mcu_feedback_required_ = false;
 chassis_mcu_feedback_seen_ = false;
 chassis_mcu_feedback_timeout_ns_ = core::kInvalidTime;
 chassis_mcu_last_feedback_ns_ = core::kInvalidTime;
 // The adapter owns the physical handshake: this is where a real transport
 // verifies the board profile, calibration artifact, units and ownership
 // against the deployable ROBOT description. Do it before registering any
 // endpoint or activating a controller so a mismatch can never emit motion.
 if (!adapter_.validate_startup_profile(application_.description())) {
  return false;
 }
 if (auto* mcu = adapter_.chassis_mcu(); mcu != nullptr) {
 if (!iface::valid_chassis_capabilities(mcu->capabilities())) {
   return false;
  }
 const auto& caps = mcu->capabilities();
 const auto expected_mode = application_.actuator_mode();
 if (caps.chassis_mode != expected_mode ||
 caps.drive_count != iface::actuator_drive_count(expected_mode) ||
 caps.steer_count != iface::actuator_steer_count(expected_mode)) {
 return false;
 }
 application_.set_chassis_control_owner(caps.control_owner);
 chassis_mcu_feedback_required_ = true;
 chassis_mcu_feedback_seen_ = false;
 chassis_mcu_feedback_timeout_ns_ =
     feedback_timeout_ns(caps, config_.decision_period_ns);
 chassis_mcu_last_feedback_ns_ = core::kInvalidTime;
 } else {
 application_.set_chassis_control_owner(
 iface::ChassisControlOwner::kHostMotionLoop);
 }
 for (const auto& endpoint : adapter_.endpoints()) {
 if (endpoint.backend != iface::ProviderBackend::kNative ||
 !application_.register_provider(endpoint)) {
 return false;
 }
 conformance_.expect(
     endpoint, adapter_.endpoint_nominal_rate_hz(endpoint.endpoint_id));
 }
 application_.set_decision_period(config_.decision_period_ns);
 if (!application_.configure() || !application_.activate()) {
 application_.shutdown();
 return false;
 }

 if (adapter_.chassis_mcu() == nullptr) {
  chassis_mcu_feedback_required_ = false;
  chassis_mcu_feedback_seen_ = false;
  chassis_mcu_feedback_timeout_ns_ = core::kInvalidTime;
  chassis_mcu_last_feedback_ns_ = core::kInvalidTime;
 }

 const core::MonotonicNs now_ns = native_monotonic_now_ns();
 actuator_mode_ = application_.take_actuator(now_ns).mode;
 {
 std::lock_guard<std::mutex> lock(state_mutex_);
 armed_expiry_ns_ = core::kInvalidTime;
 watchdog_fired_ = false;
 }
 watchdog_exit_.store(false);
 active_.store(true);
 watchdog_thread_ = std::thread(&NativeRuntime::watchdog_loop, this);
 return true;
}

bool NativeRuntime::cycle(core::MonotonicNs now_ns) {
 if (!active_.load() || now_ns < 0) return false;
 iface::ObservationFrame observation;
 if (auto* mcu = adapter_.chassis_mcu(); mcu != nullptr) {
 bool invalid_feedback = false;
 iface::MotionFeedback feedback;
 while (mcu->poll_feedback(feedback)) {
 {
 std::lock_guard<std::mutex> lock(stats_mutex_);
 ++stats_.observations_polled;
 }
 if (!feedback.valid()) {
  conformance_.reject(now_ns, feedback.source, "motion_feedback",
                      "typed MCU feedback failed structural validation");
  invalid_feedback = true;
  break;
 }
 const auto& caps = mcu->capabilities();
 if (!registered_encoder_endpoint(adapter_, feedback)) {
  conformance_.reject(now_ns, feedback.source, "motion_feedback_source",
                      "feedback source is not a registered native encoder "
                      "endpoint with the declared frame");
  invalid_feedback = true;
  break;
 }
 if (feedback.drive_count != caps.drive_count ||
     feedback.steer_count != caps.steer_count) {
  conformance_.reject(
      now_ns, feedback.source, "motion_feedback_layout",
      "feedback drive/steer joint count does not match MCU capabilities");
  invalid_feedback = true;
  break;
 }
 observation = iface::make_envelope(feedback);
 conformance_.observe(observation, now_ns);
 if (application_.publish_observation(observation)) {
 std::lock_guard<std::mutex> lock(stats_mutex_);
 ++stats_.observations_accepted;
 }
 chassis_mcu_feedback_seen_ = true;
 chassis_mcu_last_feedback_ns_ = now_ns;
 feedback = {};
 }
 if (invalid_feedback) {
  const auto report = conformance_.report(now_ns);
  {
   std::lock_guard<std::mutex> lock(stats_mutex_);
   stats_.conformance_failures = report.failures.size();
  }
  mechanical_stop(now_ns, false);
  return false;
 }
 if (chassis_mcu_feedback_required_ &&
     (!chassis_mcu_feedback_seen_ ||
      chassis_mcu_last_feedback_ns_ == core::kInvalidTime ||
      now_ns < chassis_mcu_last_feedback_ns_ ||
      now_ns - chassis_mcu_last_feedback_ns_ >
          chassis_mcu_feedback_timeout_ns_)) {
  conformance_.reject(
      now_ns, "chassis_mcu", "motion_feedback_timeout",
      chassis_mcu_feedback_seen_
          ? "no valid typed MCU feedback within the profile liveness window"
          : "typed MCU produced no feedback before the first command");
  const auto report = conformance_.report(now_ns);
  {
   std::lock_guard<std::mutex> lock(stats_mutex_);
   stats_.conformance_failures = report.failures.size();
  }
  mechanical_stop(now_ns, false);
  return false;
 }
 }
 while (adapter_.poll_observation(observation)) {
 {
 std::lock_guard<std::mutex> lock(stats_mutex_);
 ++stats_.observations_polled;
 }
 conformance_.observe(observation, now_ns);
 if (application_.publish_observation(observation)) {
 std::lock_guard<std::mutex> lock(stats_mutex_);
 ++stats_.observations_accepted;
 }
 observation = iface::ObservationFrame{};
 }

 conformance_.check_liveness(now_ns);
 const auto conformance_report = conformance_.report(now_ns);
 {
  std::lock_guard<std::mutex> lock(stats_mutex_);
  stats_.conformance_failures = conformance_report.failures.size();
 }
 if (!conformance_report.failures.empty()) {
  // A provider stream violation is a hardware fault, not an algorithmic
  // input to recover from. Stop before another ROBOT decision can consume a
  // malformed or stale measurement.
  mechanical_stop(now_ns, false);
  return false;
 }
 application_.tick(now_ns);
 const iface::ActuatorCommand command = application_.take_actuator(now_ns);
 const bool written = publish_output(command, now_ns);
 {
 std::lock_guard<std::mutex> lock(stats_mutex_);
 ++stats_.cycles;
 }
 return written;
}

int NativeRuntime::run(const std::atomic_bool& stop_requested) {
 if (!active_.load() && !start()) return 1;
 auto next = SteadyClock::now();
 while (!stop_requested.load() && active_.load()) {
 const core::MonotonicNs now_ns = native_monotonic_now_ns();
 if (!cycle(now_ns)) {
 shutdown();
 return 2;
 }
 next += std::chrono::nanoseconds(config_.decision_period_ns);
 const auto after = SteadyClock::now();
 if (next <= after) {
 // Skip missed wall-clock releases. ROBOT sees the current monotonic time;
 // running catch-up decisions without new measurements would manufacture
 // control cycles and starve the independent output watchdog.
 const auto late = after - next;
 const auto periods = late /
 std::chrono::nanoseconds(config_.decision_period_ns) + 1;
 next += periods * std::chrono::nanoseconds(config_.decision_period_ns);
 }
 std::this_thread::sleep_until(next);
 }
 shutdown();
 return 0;
}

bool NativeRuntime::publish_output(const iface::ActuatorCommand& command,
 core::MonotonicNs now_ns) {
 iface::ActuatorCommand output = command;
 if (!iface::valid_actuator_command(output, now_ns)) {
 output = iface::ActuatorCommand::SafeStop(now_ns, actuator_mode_);
 }
 actuator_mode_ = output.mode;
 core::MonotonicNs published_expiry_ns = output.expiry_ns;
 bool ok = false;
 {
 std::lock_guard<std::mutex> io_lock(io_mutex_);
  if (auto* mcu = adapter_.chassis_mcu(); mcu != nullptr) {
 const iface::ChassisSetpoint setpoint =
 application_.take_chassis_setpoint(mcu->capabilities(),
 output,
 ++chassis_sequence_, now_ns);
 published_expiry_ns = setpoint.expiry_ns;
   ok = iface::valid_chassis_setpoint(setpoint, mcu->capabilities(),
                                      now_ns) &&
        mcu->send(setpoint);
  } else {
   ok = adapter_.write_actuator(output);
  }
 if (!ok) {
  adapter_.mechanical_safe_stop(actuator_mode_, now_ns);
  if (auto* mcu = adapter_.chassis_mcu(); mcu != nullptr) {
   mcu->mechanical_safe_stop(now_ns);
  }
 }
 }
 {
 std::lock_guard<std::mutex> lock(stats_mutex_);
 if (ok) {
 ++stats_.actuator_writes;
 } else {
 ++stats_.actuator_write_failures;
 }
 }
 if (!ok) {
 std::lock_guard<std::mutex> lock(state_mutex_);
 armed_expiry_ns_ = core::kInvalidTime;
 watchdog_fired_ = true;
 return false;
 }
 {
 std::lock_guard<std::mutex> lock(state_mutex_);
 armed_expiry_ns_ = published_expiry_ns;
 watchdog_fired_ = false;
 }
 watchdog_cv_.notify_all();
 return true;
}

void NativeRuntime::watchdog_loop() {
 std::unique_lock<std::mutex> lock(state_mutex_);
 while (!watchdog_exit_.load()) {
 if (armed_expiry_ns_ == core::kInvalidTime || watchdog_fired_) {
 watchdog_cv_.wait(lock, [this] {
 return watchdog_exit_.load() ||
 (armed_expiry_ns_ != core::kInvalidTime && !watchdog_fired_);
 });
 continue;
 }
 const core::MonotonicNs expiry_ns = armed_expiry_ns_;
 if (native_monotonic_now_ns() <= expiry_ns) {
 watchdog_cv_.wait_until(lock, steady_time_point(expiry_ns), [this, expiry_ns] {
 return watchdog_exit_.load() || armed_expiry_ns_ != expiry_ns;
 });
 continue;
 }
 watchdog_fired_ = true;
 lock.unlock();
 mechanical_stop(native_monotonic_now_ns(), true);
 lock.lock();
 }
}

void NativeRuntime::mechanical_stop(core::MonotonicNs now_ns,
 bool watchdog) {
 {
 std::lock_guard<std::mutex> io_lock(io_mutex_);
  if (auto* mcu = adapter_.chassis_mcu(); mcu != nullptr) {
   mcu->mechanical_safe_stop(now_ns);
  }
 adapter_.mechanical_safe_stop(actuator_mode_, now_ns);
 }
 if (watchdog) {
 std::lock_guard<std::mutex> lock(stats_mutex_);
 ++stats_.watchdog_stops;
 }
}

void NativeRuntime::shutdown() {
 if (!active_.exchange(false)) return;
 watchdog_exit_.store(true);
 watchdog_cv_.notify_all();
 if (watchdog_thread_.joinable()) watchdog_thread_.join();
 mechanical_stop(native_monotonic_now_ns(), false);
 application_.shutdown();
 chassis_mcu_feedback_required_ = false;
 chassis_mcu_feedback_seen_ = false;
 chassis_mcu_feedback_timeout_ns_ = core::kInvalidTime;
 chassis_mcu_last_feedback_ns_ = core::kInvalidTime;
}

NativeRuntimeStats NativeRuntime::stats() const {
 std::lock_guard<std::mutex> lock(stats_mutex_);
 return stats_;
}

} // namespace lost_signal::robot::app
