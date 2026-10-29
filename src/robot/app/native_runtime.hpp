#pragma once

#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "robot/app/robot_application.hpp"
#include "robot/iface/conformance.hpp"

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
 // Optional nominal acquisition rate used by the stream conformance monitor.
 // Zero means the adapter does not want a rate/silence gate for that endpoint.
 virtual double endpoint_nominal_rate_hz(
     const std::string& endpoint_id) const {
   (void)endpoint_id;
   return 0.0;
 }
 // Performs the deployment-specific profile/calibration handshake before
 // ROBOT lifecycle activation. The adapter may query a board over UART, CAN,
 // USB or another native link and compare ownership, units, joint layout,
 // robot-description revision, and calibration range. A default keeps
 // simple duty adapters source-compatible; production adapters should fail
 // closed until their measured profile is acknowledged.
 virtual bool validate_startup_profile(
     const platform::RobotDescription& description) const {
   (void)description;
   return true;
 }
 virtual bool poll_observation(iface::ObservationFrame& out) = 0;
 virtual bool write_actuator(const iface::ActuatorCommand& command) = 0;

 // Optional typed chassis-MCU transport. Returning nullptr keeps the
 // compatibility duty adapter path. A non-null transport must advertise a
 // validated ownership/profile; host duty, MCU joint, and MCU body setpoints
 // are selected by the same typed contract.
 virtual iface::IChassisMcuTransport* chassis_mcu() { return nullptr; }

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
 std::uint64_t conformance_failures = 0;
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
 std::uint64_t chassis_sequence_ = 0;
 iface::ProviderConformanceMonitor conformance_;
 bool chassis_mcu_feedback_required_ = false;
 bool chassis_mcu_feedback_seen_ = false;
 core::MonotonicNs chassis_mcu_feedback_timeout_ns_ =
     core::kInvalidTime;
 core::MonotonicNs chassis_mcu_last_feedback_ns_ = core::kInvalidTime;

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
