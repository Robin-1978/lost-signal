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

class FakeNativeAdapter final : public app::INativeAdapter {
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
