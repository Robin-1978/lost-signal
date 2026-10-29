#include <algorithm>
#include <atomic>
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <mutex>
#include <thread>
#include <utility>
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
 return !fail_sends;
 }
 bool poll_feedback(iface::MotionFeedback& out) override {
  if (feedback.empty()) return false;
  out = std::move(feedback.front());
  feedback.pop_front();
  return true;
 }
 void mechanical_safe_stop(core::MonotonicNs) noexcept override {
 ++stops;
 }

 void push_feedback(iface::MotionFeedback value) {
  feedback.push_back(std::move(value));
 }

 iface::ChassisCapabilities caps_{};
 iface::ChassisSetpoint last{};
 std::deque<iface::MotionFeedback> feedback;
 int sends = 0;
 int stops = 0;
 bool fail_sends = false;
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

iface::MotionFeedback mcu_feedback(core::MonotonicNs now_ns,
                                   std::uint64_t sequence = 1) {
 iface::MotionFeedback out;
 out.sequence = sequence;
 out.stamp_ns = now_ns;
 out.source = "native.encoder";
 out.frame = core::Frame::kBaseLink;
 out.odometry_valid = true;
 out.drive_count = 2;
 out.forward_mps = 0.0;
 out.yaw_rate_radps = 0.0;
 return out;
}

// A blocked join() has to fail this test promptly and say why, rather than
// waiting for the suite-level timeout. A healthy run reports progress every
// few hundred microseconds; the monitor aborts after a whole idle `limit`.
class IdleDeadman {
 public:
  explicit IdleDeadman(std::chrono::milliseconds limit)
      : monitor_([this, limit] { watch(limit); }) {}
  ~IdleDeadman() {
   {
    std::lock_guard<std::mutex> lock(mutex_);
    done_ = true;
   }
   wake_.notify_all();
   monitor_.join();
  }
  IdleDeadman(const IdleDeadman&) = delete;
  IdleDeadman& operator=(const IdleDeadman&) = delete;

  void progress() { progress_.fetch_add(1, std::memory_order_relaxed); }

 private:
  void watch(std::chrono::milliseconds limit) {
   std::unique_lock<std::mutex> lock(mutex_);
   std::uint64_t seen = progress_.load(std::memory_order_relaxed);
   while (!wake_.wait_for(lock, limit, [this] { return done_; })) {
    const std::uint64_t now = progress_.load(std::memory_order_relaxed);
    if (now == seen) {
     std::fprintf(stderr,
                  "test_native_runtime: no progress for %lld ms; a "
                  "NativeRuntime call is blocked (lost watchdog wakeup?)\n",
                  static_cast<long long>(limit.count()));
     std::abort();
    }
    seen = now;
   }
  }

  std::mutex mutex_;
  std::condition_variable wake_;
  bool done_ = false;
  std::atomic<std::uint64_t> progress_{0};
  // Declared last so the monitor starts after every member it reads exists.
  std::thread monitor_;
};
} // namespace

int main() {
 using namespace lost_signal::robot;
 IdleDeadman deadman(std::chrono::seconds(5));

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
 mcu_adapter.mcu.push_feedback(mcu_feedback(mcu_now_ns));
 assert(mcu_runtime.cycle(mcu_now_ns));
 assert(mcu_adapter.mcu.sends == 1);
 assert(mcu_adapter.mcu.last.kind == iface::ChassisSetpointKind::kActuatorDuty);
 assert(mcu_adapter.mcu.last.sequence == 1);
 assert(mcu_adapter.mcu.last.stamp_ns == mcu_now_ns);
 mcu_runtime.shutdown();

 // A typed chassis cannot continue on a silent encoder stream. The timeout
 // derives from the board's declared feedback period and command age rather
 // than a scenario-specific constant.
 {
  app::RobotApplication timeout_application(nullptr);
  FakeMcuAdapter timeout_adapter;
  app::NativeRuntime timeout_runtime(
      timeout_application, timeout_adapter, {.decision_period_ns = 1'000'000});
  assert(timeout_runtime.start());
  const core::MonotonicNs first_ns = app::native_monotonic_now_ns();
  timeout_adapter.mcu.push_feedback(mcu_feedback(first_ns));
  assert(timeout_runtime.cycle(first_ns));
  const core::MonotonicNs liveness_ns =
      std::max(timeout_adapter.mcu.caps_.feedback_period_ns * 3,
               core::MonotonicNs{1'000'000});
  assert(!timeout_runtime.cycle(first_ns + liveness_ns + 1));
  assert(timeout_runtime.stats().conformance_failures > 0);
  assert(timeout_adapter.mcu.stops >= 1);
  timeout_runtime.shutdown();
 }

 // Replayed feedback is a transport fault, even when its values are finite.
 {
  app::RobotApplication ordering_application(nullptr);
  FakeMcuAdapter ordering_adapter;
  app::NativeRuntime ordering_runtime(
      ordering_application, ordering_adapter, {.decision_period_ns = 1'000'000});
  assert(ordering_runtime.start());
  const core::MonotonicNs first_ns = app::native_monotonic_now_ns();
  ordering_adapter.mcu.push_feedback(mcu_feedback(first_ns, 1));
  assert(ordering_runtime.cycle(first_ns));
  ordering_adapter.mcu.push_feedback(mcu_feedback(first_ns + 1'000'000, 1));
  assert(!ordering_runtime.cycle(first_ns + 1'000'000));
  assert(ordering_runtime.stats().conformance_failures > 0);
  assert(ordering_adapter.mcu.stops >= 1);
  ordering_runtime.shutdown();
 }

 // Joint layout is part of the board capability contract; a partial encoder
 // packet must never be interpreted as a valid differential chassis sample.
 {
  app::RobotApplication layout_application(nullptr);
  FakeMcuAdapter layout_adapter;
  app::NativeRuntime layout_runtime(
      layout_application, layout_adapter, {.decision_period_ns = 1'000'000});
  assert(layout_runtime.start());
  const core::MonotonicNs now = app::native_monotonic_now_ns();
  auto malformed = mcu_feedback(now);
  malformed.drive_count = 1;
  layout_adapter.mcu.push_feedback(std::move(malformed));
  assert(!layout_runtime.cycle(now));
  assert(layout_runtime.stats().conformance_failures > 0);
  assert(layout_adapter.mcu.stops >= 1);
  layout_runtime.shutdown();
 }

 // The typed source identity is part of the hardware boundary. A packet from
 // another endpoint is rejected before it can become ROBOT odometry.
 {
  app::RobotApplication source_application(nullptr);
  FakeMcuAdapter source_adapter;
  app::NativeRuntime source_runtime(
      source_application, source_adapter, {.decision_period_ns = 1'000'000});
  assert(source_runtime.start());
  const core::MonotonicNs now = app::native_monotonic_now_ns();
  auto wrong_source = mcu_feedback(now);
  wrong_source.source = "native.imu";
  source_adapter.mcu.push_feedback(std::move(wrong_source));
  assert(!source_runtime.cycle(now));
  assert(source_runtime.stats().conformance_failures > 0);
  assert(source_adapter.mcu.stops >= 1);
  source_runtime.shutdown();
 }

 // A feedback frame can be structurally valid and still be too old for the
 // ROBOT clock. Provider freshness rejects it before a decision is made.
 {
  app::RobotApplication stale_application(nullptr);
  FakeMcuAdapter stale_adapter;
  app::NativeRuntime stale_runtime(
      stale_application, stale_adapter, {.decision_period_ns = 1'000'000});
  assert(stale_runtime.start());
  const core::MonotonicNs now = app::native_monotonic_now_ns();
  const core::MonotonicNs stale_age =
      iface::ProviderConformanceConfig{}.maximum_sample_age_ns + 1;
  stale_adapter.mcu.push_feedback(mcu_feedback(now - stale_age));
  assert(!stale_runtime.cycle(now));
  assert(stale_runtime.stats().conformance_failures > 0);
  assert(stale_adapter.mcu.stops >= 1);
  stale_runtime.shutdown();
 }

 // A transport send failure invokes both the typed MCU stop and the adapter's
 // supervisory stop path before the failed cycle returns.
 {
  app::RobotApplication send_application(nullptr);
  FakeMcuAdapter send_adapter;
  app::NativeRuntime send_runtime(
      send_application, send_adapter, {.decision_period_ns = 1'000'000});
  assert(send_runtime.start());
  const core::MonotonicNs now = app::native_monotonic_now_ns();
  send_adapter.mcu.push_feedback(mcu_feedback(now));
  send_adapter.mcu.fail_sends = true;
  assert(!send_runtime.cycle(now));
  assert(send_runtime.stats().actuator_write_failures == 1);
  assert(send_adapter.mcu.stops >= 1);
  assert(send_adapter.stops() >= 1);
  send_runtime.shutdown();
 }

 // Native ownership selects exactly one outer loop. The same runtime seam
 // must carry joint-velocity setpoints when the board owns the joint loop and
 // body-twist setpoints when it owns the body loop; host PID is not run in
 // either case.
 {
  app::RobotApplication joint_application(nullptr);
  FakeMcuAdapter joint_adapter;
  joint_adapter.mcu.caps_.control_owner =
      iface::ChassisControlOwner::kMcuJointLoop;
  app::NativeRuntime joint_runtime(
      joint_application, joint_adapter, {.decision_period_ns = 1'000'000});
  assert(joint_runtime.start());
  joint_application.set_operator_command(0.1, 0.0);
  const core::MonotonicNs now = app::native_monotonic_now_ns();
  joint_adapter.mcu.push_feedback(mcu_feedback(now));
  joint_adapter.push(lidar(now));
  joint_adapter.push(imu(now));
  assert(joint_runtime.cycle(now));
  assert(joint_adapter.mcu.last.kind == iface::ChassisSetpointKind::kJointVelocity);
  assert(iface::valid_chassis_setpoint(joint_adapter.mcu.last,
                                       joint_adapter.mcu.caps_, now));
  joint_runtime.shutdown();

  app::RobotApplication body_application(nullptr);
  FakeMcuAdapter body_adapter;
  body_adapter.mcu.caps_.control_owner =
      iface::ChassisControlOwner::kMcuBodyLoop;
  app::NativeRuntime body_runtime(
      body_application, body_adapter, {.decision_period_ns = 1'000'000});
  assert(body_runtime.start());
  body_application.set_operator_command(0.1, 0.0);
  const core::MonotonicNs body_now = app::native_monotonic_now_ns();
  body_adapter.mcu.push_feedback(mcu_feedback(body_now));
  body_adapter.push(lidar(body_now));
  body_adapter.push(imu(body_now));
  assert(body_runtime.cycle(body_now));
  assert(body_adapter.mcu.last.kind == iface::ChassisSetpointKind::kBodyTwist);
  assert(iface::valid_chassis_setpoint(body_adapter.mcu.last,
                                       body_adapter.mcu.caps_, body_now));
  body_runtime.shutdown();
 }

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

 // The watchdog thread evaluates its wait predicate under the runtime's state
 // mutex, so shutdown() has to publish the exit request under that mutex as
 // well. Otherwise the notification can fall between the predicate check and
 // the wait: the wakeup is lost and join() never returns. The thread reaches
 // its first wait shortly after start() returns, so close the runtime after a
 // pseudo-random delay of up to 16 microseconds to sweep across that window.
 {
  constexpr int kCycles = 3000;
  constexpr std::uint64_t kMaxCloseDelayNs = 16'000;
  std::uint64_t rng = 0x9E3779B97F4A7C15ull;
  for (int cycle_index = 0; cycle_index < kCycles; ++cycle_index) {
   rng ^= rng << 13;
   rng ^= rng >> 7;
   rng ^= rng << 17;
   app::RobotApplication race_application(nullptr);
   FakeNativeAdapter race_adapter;
   app::NativeRuntime race_runtime(
       race_application, race_adapter, {.decision_period_ns = 1'000'000});
   assert(race_runtime.start());
   const auto close_at = std::chrono::steady_clock::now() +
       std::chrono::nanoseconds(rng % (kMaxCloseDelayNs + 1));
   while (std::chrono::steady_clock::now() < close_at) {
   }
   race_runtime.shutdown();
   assert(!race_runtime.active());
   deadman.progress();
  }
 }

 std::printf("native_runtime ok\n");
 return 0;
}
