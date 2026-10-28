#include <cassert>
#include <chrono>
#include <cstdio>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

#include "robot/app/native_runtime.hpp"

namespace {
using namespace lost_signal::robot;

class FakeNativeAdapter : public app::INativeAdapter {
 public:
 FakeNativeAdapter() {
 endpoints_ = {
 {"native.lidar", iface::EndpointRole::kLidar,
 iface::ProviderBackend::kNative, "lidar_link"},
 {"native.encoder", iface::EndpointRole::kEncoder,
 iface::ProviderBackend::kNative, "base_link"},
 {"native.imu", iface::EndpointRole::kImu,
 iface::ProviderBackend::kNative, "imu_link"},
 };
 }

 const std::vector<iface::EndpointDescriptor>& endpoints() const override {
 return endpoints_;
 }
 bool validate_startup_profile(
     const platform::RobotDescription&) const override {
  return !reject_profile_;
 }
 bool poll_observation(iface::ObservationFrame& out) override {
 std::lock_guard<std::mutex> lock(mutex_);
 if (observations_.empty()) return false;
 out = std::move(observations_.front());
 observations_.pop_front();
 return true;
 }
 bool write_actuator(const iface::ActuatorCommand& command) override {
 std::lock_guard<std::mutex> lock(mutex_);
 writes_.push_back(command);
 return !fail_writes_;
 }
 void mechanical_safe_stop(iface::ActuatorMode mode,
 core::MonotonicNs now_ns) noexcept override {
 std::lock_guard<std::mutex> lock(mutex_);
 stops_.push_back(iface::ActuatorCommand::SafeStop(now_ns, mode));
 }

 void push(iface::ObservationFrame observation) {
 std::lock_guard<std::mutex> lock(mutex_);
 observations_.push_back(std::move(observation));
 }
 void fail_writes() {
 std::lock_guard<std::mutex> lock(mutex_);
 fail_writes_ = true;
 }
 void reject_profile() {
  std::lock_guard<std::mutex> lock(mutex_);
  reject_profile_ = true;
 }
 std::size_t writes() const {
 std::lock_guard<std::mutex> lock(mutex_);
 return writes_.size();
 }
 std::size_t stops() const {
 std::lock_guard<std::mutex> lock(mutex_);
 return stops_.size();
 }
 iface::ActuatorCommand last_write() const {
 std::lock_guard<std::mutex> lock(mutex_);
 return writes_.back();
 }

 private:
 std::vector<iface::EndpointDescriptor> endpoints_;
 mutable std::mutex mutex_;
 std::deque<iface::ObservationFrame> observations_;
 std::vector<iface::ActuatorCommand> writes_;
 std::vector<iface::ActuatorCommand> stops_;
 bool fail_writes_ = false;
 bool reject_profile_ = false;
};

class FakeChassisMcu final : public iface::IChassisMcuTransport {
 public:
 FakeChassisMcu() {
 caps_.control_owner = iface::ChassisControlOwner::kHostMotionLoop;
 caps_.chassis_mode = iface::ActuatorMode::kDiffDuty;
 caps_.drive_count = 2;
 caps_.control_period_ns = 1'000'000;
 caps_.feedback_period_ns = 1'000'000;
 caps_.maximum_command_age_ns = 200'000'000;
 caps_.profile_id = "native.test";
 caps_.profile_revision = 1;
 }
 const iface::ChassisCapabilities& capabilities() const override {
 return caps_;
 }
 bool send(const iface::ChassisSetpoint& setpoint) override {
 last = setpoint;
 ++sends;
 return true;
 }
 bool poll_feedback(iface::MotionFeedback&) override { return false; }
 void mechanical_safe_stop(core::MonotonicNs) noexcept override {
 ++stops;
 }

 iface::ChassisCapabilities caps_{};
 iface::ChassisSetpoint last{};
 int sends = 0;
 int stops = 0;
};

class FakeMcuAdapter final : public FakeNativeAdapter {
 public:
 iface::IChassisMcuTransport* chassis_mcu() override { return &mcu; }
 FakeChassisMcu mcu;
};

iface::ObservationFrame lidar(core::MonotonicNs now_ns) {
 iface::ObservationFrame out;
 out.sequence = 1;
 out.stamp_ns = now_ns;
 out.source = "native.lidar";
 out.frame = core::Frame::kLidarLink;
 out.scan_valid = true;
 out.ranges_m = {3.0f, 3.0f, 3.0f, 3.0f};
 out.scan_angle_increment_rad = 1.5707963267948966;
 return out;
}

iface::ObservationFrame encoder(core::MonotonicNs now_ns) {
 iface::ObservationFrame out;
 out.sequence = 1;
 out.stamp_ns = now_ns;
 out.source = "native.encoder";
 out.frame = core::Frame::kBaseLink;
 out.odometry_valid = true;
 out.drive_count = 2;
 return out;
}

iface::ObservationFrame imu(core::MonotonicNs now_ns) {
 iface::ObservationFrame out;
 out.sequence = 1;
 out.stamp_ns = now_ns;
 out.source = "native.imu";
 out.frame = core::Frame::kImuLink;
 out.imu_valid = true;
 out.imu_az_mps2 = 9.80665;
 return out;
}
} // namespace

int main() {
 using namespace lost_signal::robot;

 // A native adapter must be able to reject an unacknowledged board profile
 // or calibration artifact before any provider is registered or actuator
 // output is emitted. This proves the startup boundary fails closed.
 {
  app::RobotApplication rejected_application(nullptr);
  FakeNativeAdapter rejected_adapter;
  rejected_adapter.reject_profile();
  app::NativeRuntime rejected_runtime(
      rejected_application, rejected_adapter,
      {.decision_period_ns = 1'000'000});
  assert(!rejected_runtime.start());
  assert(!rejected_runtime.active());
  assert(rejected_adapter.writes() == 0);
 }

 {
  app::RobotApplication malformed_application(nullptr);
  FakeNativeAdapter malformed_adapter;
  app::NativeRuntime malformed_runtime(
      malformed_application, malformed_adapter,
      {.decision_period_ns = 1'000'000});
  assert(malformed_runtime.start());
  const core::MonotonicNs malformed_now = app::native_monotonic_now_ns();
  auto malformed = lidar(malformed_now);
  malformed.sequence = 0;
  malformed_adapter.push(std::move(malformed));
  assert(!malformed_runtime.cycle(malformed_now));
  assert(malformed_runtime.stats().conformance_failures > 0);
  assert(malformed_adapter.stops() >= 1);
  malformed_runtime.shutdown();
 }

 app::RobotApplication application(nullptr);
 FakeNativeAdapter adapter;
 app::NativeRuntime runtime(
 application, adapter, {.decision_period_ns = 1'000'000});
 assert(runtime.start());
 const core::MonotonicNs now_ns = app::native_monotonic_now_ns();
 adapter.push(lidar(now_ns));
 adapter.push(encoder(now_ns));
 adapter.push(imu(now_ns));
 assert(runtime.cycle(now_ns));
 const auto first = runtime.stats();
 assert(first.cycles == 1);
 assert(first.observations_polled == 3);
 assert(first.observations_accepted == 3);
 assert(first.actuator_writes == 1);
 assert(adapter.writes() == 1);
 assert(adapter.last_write().is_safe_stop());

 // The same deployment runtime can target a typed chassis transport. It
 // validates ownership during bring-up and sends an expiry-bounded typed
 // setpoint instead of a legacy actuator frame.
 app::RobotApplication mcu_application(nullptr);
 FakeMcuAdapter mcu_adapter;
 app::NativeRuntime mcu_runtime(
 mcu_application, mcu_adapter, {.decision_period_ns = 1'000'000});
 assert(mcu_runtime.start());
 const core::MonotonicNs mcu_now_ns = app::native_monotonic_now_ns();
 assert(mcu_runtime.cycle(mcu_now_ns));
 assert(mcu_adapter.mcu.sends == 1);
 assert(mcu_adapter.mcu.last.kind == iface::ChassisSetpointKind::kActuatorDuty);
 assert(mcu_adapter.mcu.last.sequence == 1);
 assert(mcu_adapter.mcu.last.stamp_ns == mcu_now_ns);
 mcu_runtime.shutdown();

 // Stop enforcement is independent of the decision thread. Once its
 // published command expires, the watchdog invokes the adapter's separate
 // mechanical path even though no further cycle() call occurs.
 std::this_thread::sleep_for(std::chrono::milliseconds(230));
 assert(adapter.stops() >= 1);
 assert(runtime.stats().watchdog_stops >= 1);

 // An ordinary write failure immediately requests the same mechanical stop
 // and fails the cycle so the process cannot keep running without output.
 adapter.fail_writes();
 const core::MonotonicNs later_ns = app::native_monotonic_now_ns();
 assert(!runtime.cycle(later_ns));
 assert(runtime.stats().actuator_write_failures == 1);
 const std::size_t stops_before_shutdown = adapter.stops();
 runtime.shutdown();
 assert(adapter.stops() == stops_before_shutdown + 1);
 assert(!runtime.active());

 std::printf("native_runtime ok\n");
 return 0;
}
