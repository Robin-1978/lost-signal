#include <atomic>
#include <charconv>
#include <csignal>
#include <cstdio>
#include <string>
#include <system_error>
#include <vector>

#include "robot/app/native_runtime.hpp"
#include "robot/platform/robot_config.hpp"

namespace {
std::atomic_bool stop_requested{false};
static_assert(std::atomic_bool::is_always_lock_free,
 "signal stop flag must be lock-free");

void request_stop(int) { stop_requested.store(true); }

// Safe deployment placeholder used when this SDK-neutral repository is not
// linked with a hardware adapter. It can exercise lifecycle and watchdog
// behavior, but accepts only mechanical safe-stop commands.
class SafeHoldAdapter final
 : public lost_signal::robot::app::INativeAdapter {
 public:
 const std::vector<lost_signal::robot::iface::EndpointDescriptor>& endpoints()
 const override {
 return endpoints_;
 }
 bool poll_observation(
 lost_signal::robot::iface::ObservationFrame&) override {
 return false;
 }
 bool write_actuator(
 const lost_signal::robot::iface::ActuatorCommand& command) override {
 return command.is_safe_stop();
 }
 void mechanical_safe_stop(lost_signal::robot::iface::ActuatorMode,
 lost_signal::robot::core::MonotonicNs) noexcept override {}

 private:
 std::vector<lost_signal::robot::iface::EndpointDescriptor> endpoints_;
};

bool parse_positive_ms(const char* text,
 lost_signal::robot::core::MonotonicNs& period_ns) {
 long long milliseconds = 0;
 const std::string value = text == nullptr ? "" : text;
 const auto parsed = std::from_chars(
 value.data(), value.data() + value.size(), milliseconds);
 if (parsed.ec != std::errc{} || parsed.ptr != value.data() + value.size() ||
 milliseconds <= 0 || milliseconds > 60'000) {
 return false;
 }
 period_ns = static_cast<lost_signal::robot::core::MonotonicNs>(
 milliseconds) * 1'000'000;
 return true;
}
} // namespace

int main(int argc, char** argv) {
 using namespace lost_signal;
 std::string robot_path;
 robot::core::MonotonicNs decision_period_ns = 20'000'000;
 for (int i = 1; i < argc; ++i) {
 const std::string argument = argv[i];
 if (argument == "--robot" && i + 1 < argc) {
 robot_path = argv[++i];
 } else if (argument == "--decision-period-ms" && i + 1 < argc) {
 if (!parse_positive_ms(argv[++i], decision_period_ns)) {
 std::fprintf(stderr, "bad --decision-period-ms (expected 1..60000)\n");
 return 1;
 }
 } else if (argument == "--help") {
 std::printf(
 "usage: lost_signal_robot [--robot FILE] "
 "[--decision-period-ms N]\n");
 return 0;
 } else {
 std::fprintf(stderr, "unknown or incomplete argument: %s\n", argument.c_str());
 return 1;
 }
 }

 std::string used, error;
 auto config =
 robot::platform::load_default_robot_config(robot_path, &used, &error);
 if (!robot_path.empty() && !config.has_value()) {
 std::fprintf(stderr, "bad --robot file: %s\n", error.c_str());
 return 1;
 }
 robot::platform::RobotDescription description;
 if (config.has_value()) {
 std::fprintf(stderr, "robot config: %s\n", used.c_str());
 description = robot::platform::description_from_config(*config);
 }

 robot::app::RobotApplication application(nullptr, std::move(description));
 SafeHoldAdapter adapter;
 robot::app::NativeRuntime runtime(
 application, adapter, {.decision_period_ns = decision_period_ns});
 if (!runtime.start()) {
 std::fprintf(stderr, "robot: native runtime bringup failed\n");
 return 1;
 }

 std::signal(SIGINT, request_stop);
 std::signal(SIGTERM, request_stop);
 std::fprintf(stderr,
 "robot: active in safe-hold mode; link an INativeAdapter for hardware I/O\n");
 const int result = runtime.run(stop_requested);
 const auto stats = runtime.stats();
 std::fprintf(stderr,
 "robot: stopped cycles=%llu observations=%llu/%llu writes=%llu "
 "write_failures=%llu watchdog_stops=%llu\n",
 static_cast<unsigned long long>(stats.cycles),
 static_cast<unsigned long long>(stats.observations_accepted),
 static_cast<unsigned long long>(stats.observations_polled),
 static_cast<unsigned long long>(stats.actuator_writes),
 static_cast<unsigned long long>(stats.actuator_write_failures),
 static_cast<unsigned long long>(stats.watchdog_stops));
 return result;
}
