#include "robot/app/native_runtime.hpp"

#include <algorithm>
#include <chrono>
#include <limits>

namespace lost_signal::robot::app {

namespace {
using SteadyClock = std::chrono::steady_clock;

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
 if (auto* mcu = adapter_.chassis_mcu(); mcu != nullptr) {
  if (!iface::valid_chassis_capabilities(mcu->capabilities()) ||
      mcu->capabilities().control_owner !=
          iface::ChassisControlOwner::kHostMotionLoop) {
   return false;
  }
 }
 for (const auto& endpoint : adapter_.endpoints()) {
 if (endpoint.backend != iface::ProviderBackend::kNative ||
 !application_.register_provider(endpoint)) {
 return false;
 }
 }
 application_.set_decision_period(config_.decision_period_ns);
 if (!application_.configure() || !application_.activate()) {
 application_.shutdown();
 return false;
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
 while (adapter_.poll_observation(observation)) {
 {
 std::lock_guard<std::mutex> lock(stats_mutex_);
 ++stats_.observations_polled;
 }
 if (application_.publish_observation(observation)) {
 std::lock_guard<std::mutex> lock(stats_mutex_);
 ++stats_.observations_accepted;
 }
 observation = iface::ObservationFrame{};
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
 bool ok = false;
 {
 std::lock_guard<std::mutex> io_lock(io_mutex_);
  if (auto* mcu = adapter_.chassis_mcu(); mcu != nullptr) {
   iface::ChassisSetpoint setpoint;
   setpoint.kind = iface::ChassisSetpointKind::kActuatorDuty;
   setpoint.chassis_mode = output.mode;
   setpoint.sequence = ++chassis_sequence_;
   setpoint.stamp_ns = output.stamp_ns;
   setpoint.expiry_ns = output.expiry_ns;
   setpoint.drive_duty = output.drive;
   setpoint.steer_duty = output.steer;
   setpoint.service_brake = output.service_brake;
   ok = iface::valid_chassis_setpoint(setpoint, mcu->capabilities(),
                                      now_ns) &&
        mcu->send(setpoint);
  } else {
   ok = adapter_.write_actuator(output);
  }
 if (!ok) adapter_.mechanical_safe_stop(actuator_mode_, now_ns);
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
 armed_expiry_ns_ = output.expiry_ns;
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
}

NativeRuntimeStats NativeRuntime::stats() const {
 std::lock_guard<std::mutex> lock(stats_mutex_);
 return stats_;
}

} // namespace lost_signal::robot::app
