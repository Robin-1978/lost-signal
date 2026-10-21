#include "robot/runtime/provider_conformance.hpp"

#include <cmath>
#include <iostream>
#include <string>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ \
 << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

namespace {

using sim::runtime::EndpointDescriptor;
using sim::runtime::EndpointKind;
using sim::runtime::EndpointRole;
using sim::runtime::ProviderBackend;
using sim::runtime::ProviderConformanceConfig;
using sim::runtime::ProviderConformanceMonitor;
using sim::runtime::ProviderConformanceReport;
using sim::runtime::TimePoint;

EndpointDescriptor lidar_endpoint() {
 EndpointDescriptor descriptor;
 descriptor.endpoint_id = "native.lidar.endpoint";
 descriptor.provider_id = "native.lidar.driver";
 descriptor.source_id = "native.lidar";
 descriptor.frame_id = "lidar_link";
 descriptor.clock_id = "monotonic";
 descriptor.role = EndpointRole::ObservationSource;
 descriptor.kind = EndpointKind::Lidar;
 descriptor.backend = ProviderBackend::Native;
 descriptor.nominal_rate_hz = 10.0;
 return descriptor;
}

sim::MeasurementHeader header(double stamp, std::uint64_t sequence,
 const std::string& frame_id, const std::string& source_id) {
 return {stamp, sequence, frame_id, 0.0, sim::kMeasurementSchemaVersion, source_id};
}

sim::ObservationFrame make_frame(std::uint64_t index, double dt_s = 0.1,
 const std::string& frame_id = "lidar_link",
 const std::string& source_id = "native.lidar") {
 sim::ObservationFrame frame;
 const double stamp = 5.0 + dt_s * static_cast<double>(index);
 frame.time_s = stamp;
 sim::Scan scan;
 scan.n_rays = 8;
 scan.max_range = 8.0;
 scan.angle_min_rad = -sim::kPi;
 scan.angle_increment_rad = sim::kPi / 4.0;
 for (int ray = 0; ray < 8; ++ray) {
 scan.ranges.push_back(3.0);
 scan.hit.push_back(1);
 scan.intensities.push_back(0.5);
 }
 scan.header = header(stamp, index, frame_id, source_id);
 frame.lidar = scan;

 sim::EncoderObservation encoders;
 encoders.linear_velocity_mps = 0.25;
 encoders.angular_velocity_radps = 0.02;
 encoders.header = header(stamp, index, "base_link", "native.encoders");
 frame.encoders = encoders;

 sim::ImuObservation imu;
 imu.yaw_rate_radps = 0.01;
 imu.header = header(stamp, index, "imu_link", "native.imu");
 frame.imu = imu;

 sim::PowerObservation power;
 power.voltage_v = 48.0;
 power.current_a = 3.5;
 power.state_of_charge = 0.8;
 power.remaining_energy_wh = 300.0;
 power.header = header(stamp, index, "battery_link", "native.power");
 frame.power = power;
 return frame;
}

TimePoint arrival(const sim::ObservationFrame& frame, double latency_s = 0.02) {
 return TimePoint::from_seconds(frame.time_s + latency_s);
}

bool has_rule(const ProviderConformanceReport& report, const std::string& rule) {
 for (const auto& failure : report.failures) {
 if (failure.rule == rule)
 return true;
 }
 return false;
}

void print_failures(const char* label, const ProviderConformanceReport& report) {
 for (const auto& failure : report.failures)
 std::cerr << label << " failure rule=" << failure.rule
 << " source=" << failure.source_id << " detail=" << failure.detail << "\n";
}

// A conforming 10 Hz adapter that bundles LiDAR, encoders, IMU and power on one
// observation clock must pass with the measured rate reported back.
int good_stream() {
 ProviderConformanceMonitor monitor;
 monitor.expect(lidar_endpoint());
 for (std::uint64_t index = 0; index < 20; ++index) {
 const auto frame = make_frame(index);
 monitor.observe(frame, arrival(frame));
 }
 const auto report = monitor.report(TimePoint::from_seconds(7.02));
 print_failures("good", report);
 CHECK(report.passed);
 CHECK(report.frames == 20);
 CHECK(report.measurements == 80);
 CHECK(report.failures.empty());
 const auto rate = report.observed_rate_hz.find("native.lidar");
 CHECK(rate != report.observed_rate_hz.end());
 CHECK(std::abs(rate->second - 10.0) < 0.5);
 return 0;
}

int duplicate_sequence() {
 ProviderConformanceMonitor monitor;
 monitor.expect(lidar_endpoint());
 for (std::uint64_t index = 0; index < 6; ++index)
 monitor.observe(make_frame(index), arrival(make_frame(index)));
 const auto repeated = make_frame(5);
 monitor.observe(repeated, arrival(repeated));
 const auto report = monitor.report(arrival(repeated));
 print_failures("sequence", report);
 CHECK(!report.passed);
 CHECK(has_rule(report, "sequence"));
 return 0;
}

int reordered_timestamp() {
 ProviderConformanceMonitor monitor;
 monitor.expect(lidar_endpoint());
 for (std::uint64_t index = 0; index < 6; ++index)
 monitor.observe(make_frame(index), arrival(make_frame(index)));
 const auto late = make_frame(2);
 monitor.observe(late, arrival(make_frame(6)));
 const auto report = monitor.report(arrival(make_frame(6)));
 print_failures("ordering", report);
 CHECK(!report.passed);
 CHECK(has_rule(report, "ordering"));
 return 0;
}

int stale_sample() {
 ProviderConformanceMonitor monitor;
 monitor.expect(lidar_endpoint());
 const auto frame = make_frame(0);
 monitor.observe(frame, TimePoint::from_seconds(frame.time_s + 1.0));
 const auto report = monitor.report(TimePoint::from_seconds(frame.time_s + 1.0));
 print_failures("stale", report);
 CHECK(!report.passed);
 CHECK(has_rule(report, "freshness"));
 return 0;
}

int wrong_identity() {
 ProviderConformanceMonitor monitor;
 monitor.expect(lidar_endpoint());
 const auto frame = make_frame(0, 0.1, "rear_lidar", "native.lidar");
 monitor.observe(frame, arrival(frame));
 const auto report = monitor.report(arrival(frame));
 print_failures("identity", report);
 CHECK(!report.passed);
 CHECK(has_rule(report, "identity"));
 return 0;
}

int silent_source() {
 ProviderConformanceMonitor monitor;
 monitor.expect(lidar_endpoint());
 for (std::uint64_t index = 0; index < 3; ++index)
 monitor.observe(make_frame(index), arrival(make_frame(index)));
 const auto last = arrival(make_frame(2));
 monitor.check_liveness(TimePoint::from_seconds(last.seconds() + 5.0));
 const auto report = monitor.report(TimePoint::from_seconds(last.seconds() + 5.0));
 print_failures("silence", report);
 CHECK(!report.passed);
 CHECK(has_rule(report, "silence"));
 return 0;
}

int absolute_pose_requires_declaration() {
 const auto frame = make_frame(0);
 ProviderConformanceMonitor strict;
 strict.expect(lidar_endpoint());
 auto with_pose = frame;
 with_pose.absolute_pose = sim::PoseObservation{};
 with_pose.absolute_pose->position = {1.0, 2.0};
 with_pose.absolute_pose->header =
 header(frame.time_s, 0, "map", "native.truth-like-provider");
 strict.observe(with_pose, arrival(frame));
 CHECK(has_rule(strict.report(arrival(frame)), "absolute_pose"));

 ProviderConformanceConfig allowed;
 allowed.allow_absolute_pose = true;
 ProviderConformanceMonitor tolerant(allowed);
 tolerant.expect(lidar_endpoint());
 tolerant.observe(with_pose, arrival(frame));
 const auto report = tolerant.report(arrival(frame));
 print_failures("absolute_pose", report);
 CHECK(report.passed);
 return 0;
}

int rate_mismatch() {
 ProviderConformanceMonitor monitor;
 monitor.expect(lidar_endpoint());
 for (std::uint64_t index = 0; index < 12; ++index) {
 const auto frame = make_frame(index, 0.5);
 monitor.observe(frame, arrival(frame));
 }
 const auto last = make_frame(11, 0.5);
 const auto report = monitor.report(arrival(last));
 print_failures("rate", report);
 CHECK(!report.passed);
 CHECK(has_rule(report, "rate"));
 return 0;
}

} // namespace

int main() {
 if (good_stream() != 0) return 1;
 if (duplicate_sequence() != 0) return 1;
 if (reordered_timestamp() != 0) return 1;
 if (stale_sample() != 0) return 1;
 if (wrong_identity() != 0) return 1;
 if (silent_source() != 0) return 1;
 if (absolute_pose_requires_declaration() != 0) return 1;
 if (rate_mismatch() != 0) return 1;
 std::cout << "provider_conformance ok\n";
 return 0;
}
