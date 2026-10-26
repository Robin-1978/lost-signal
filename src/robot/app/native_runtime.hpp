#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <thread>
#include <vector>

#include "robot/app/robot_application.hpp"

namespace lost_signal::robot::app {

// The clock domain shared by native adapters, RobotApplication, commands,
// and the independent output watchdog. It is monotonic and has no wall-clock
// meaning; hardware drivers stamp acquisitions in this domain at ingress.
core::MonotonicNs native_monotonic_now_ns();

// Hardware boundary for a deployment-specific adapter. An adapter may own
// several asynchronously sampled endpoints; poll_observation() is
// non-blocking and drains one already acquired frame at a time. Only typed
// measurements cross into ROBOT. The interface contains no simulator state,
// actor identity, score, or truth pose.
class INativeAdapter {
 public:
 virtual ~INativeAdapter() = default;
 virtual const std::vector<iface::EndpointDescriptor>& endpoints() const = 0;
 virtual bool poll_observation(iface::ObservationFrame& out) = 0;
 virtual bool write_actuator(const iface::ActuatorCommand& command) = 0;

 // Independent mechanical stop path. Implementations must make this
 // idempotent and must not depend on the ordinary command queue succeeding.
 virtual void mechanical_safe_stop(iface::ActuatorMode mode,
 core::MonotonicNs now_ns) noexcept = 0;
};

struct NativeRuntimeConfig {
 core::MonotonicNs decision_period_ns = 20'000'000; // 50 Hz deployment loop
};

struct NativeRuntimeStats {
 std::uint64_t cycles = 0;
 std::uint64_t observations_polled = 0;
 std::uint64_t observations_accepted = 0;
 std::uint64_t actuator_writes = 0;
 std::uint64_t actuator_write_failures = 0;
 std::uint64_t watchdog_stops = 0;
};

// Provider-neutral native process loop. The decision thread drains all
// acquired observations, ticks the same RobotApplication used by SIM/replay,
// and writes one expiry-bounded ActuatorCommand. A separate watchdog thread
// observes that expiry and invokes the mechanical stop path even when the
// decision thread stalls.
class NativeRuntime {
 public:
 NativeRuntime(RobotApplication& application, INativeAdapter& adapter,
 NativeRuntimeConfig config = {});
 ~NativeRuntime();

 NativeRuntime(const NativeRuntime&) = delete;
 NativeRuntime& operator=(const NativeRuntime&) = delete;

 bool start();
 bool cycle(core::MonotonicNs now_ns);
 int run(const std::atomic_bool& stop_requested);
 void shutdown();

 bool active() const { return active_.load(); }
 NativeRuntimeStats stats() const;

 private:
 bool publish_output(const iface::ActuatorCommand& command,
 core::MonotonicNs now_ns);
 void watchdog_loop();
 void mechanical_stop(core::MonotonicNs now_ns, bool watchdog);

 RobotApplication& application_;
 INativeAdapter& adapter_;
 NativeRuntimeConfig config_;
 iface::ActuatorMode actuator_mode_ = iface::ActuatorMode::kDiffDuty;

 std::atomic_bool active_{false};
 std::atomic_bool watchdog_exit_{false};
 std::thread watchdog_thread_;
 mutable std::mutex state_mutex_;
 std::condition_variable watchdog_cv_;
 core::MonotonicNs armed_expiry_ns_ = core::kInvalidTime;
 bool watchdog_fired_ = false;
 mutable std::mutex io_mutex_;
 mutable std::mutex stats_mutex_;
 NativeRuntimeStats stats_;
};

} // namespace lost_signal::robot::app
