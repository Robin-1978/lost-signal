#include "robot/runtime/provider_conformance.hpp"

#include "robot/runtime/runtime_channel.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace sim::runtime {
namespace {

struct Measurement {
 EndpointKind kind;
 const MeasurementHeader* header;
};

constexpr double kMaximumRepresentableStampS =
 static_cast<double>(std::numeric_limits<std::int64_t>::max()) / 1e9;

} // namespace

const char* endpoint_kind_name(EndpointKind kind) {
 switch (kind) {
 case EndpointKind::Lidar: return "lidar";
 case EndpointKind::Imu: return "imu";
 case EndpointKind::Encoder: return "encoder";
 case EndpointKind::JointState: return "joint_state";
 case EndpointKind::Actuator: return "actuator";
 case EndpointKind::Power: return "power";
 case EndpointKind::Map: return "map";
 }
 return "unknown";
}

void ProviderConformanceMonitor::expect(const EndpointDescriptor& descriptor) {
 auto& state = sources_[descriptor.kind];
 state.expected = descriptor;
 state.has_expected = true;
}

void ProviderConformanceMonitor::fail(TimePoint now, const std::string& source_id,
 const std::string& rule, const std::string& detail) {
 // One entry per source and rule keeps a fifty-hertz fault from producing
 // thousands of identical lines while still naming every broken contract.
 const std::string key = source_id + "|" + rule;
 if (!reported_.insert(key).second)
 return;
 failures_.push_back({now.seconds(), source_id, rule, detail});
}

void ProviderConformanceMonitor::observe(const ObservationFrame& frame, TimePoint now) {
 if (!started_) {
 started_ = true;
 start_time_ = now;
 }
 ++frames_;

 std::string structure_reason;
 if (!valid_observation_frame(frame, &structure_reason))
 fail(now, "frame", "structure", structure_reason);

 if (frame.absolute_pose && !config_.allow_absolute_pose) {
 fail(now, frame.absolute_pose->header.source_id, "absolute_pose",
 "provider published a map pose without declaring that capability");
 }

 std::vector<Measurement> measurements;
 if (frame.lidar)
 measurements.push_back({EndpointKind::Lidar, &frame.lidar->header});
 if (frame.lidar_point_cloud)
 measurements.push_back({EndpointKind::Lidar, &frame.lidar_point_cloud->header});
 if (frame.encoders)
 measurements.push_back({EndpointKind::Encoder, &frame.encoders->header});
 if (frame.actuator_feedback)
 measurements.push_back({EndpointKind::JointState, &frame.actuator_feedback->header});
 if (frame.power)
 measurements.push_back({EndpointKind::Power, &frame.power->header});
 if (frame.imu)
 measurements.push_back({EndpointKind::Imu, &frame.imu->header});

 for (const auto& measurement : measurements) {
 const auto& header = *measurement.header;
 auto& state = sources_[measurement.kind];
 state.silence_reported = false;
 ++measurements_;
 const std::string source = state.has_expected ? state.expected.source_id
 : header.source_id;
 if (state.has_expected &&
 (header.frame_id != state.expected.frame_id ||
 header.source_id != state.expected.source_id)) {
 fail(now, source, "identity",
 "measured frame '" + header.frame_id + "' from '" + header.source_id +
 "' does not match the registered endpoint '" +
 state.expected.frame_id + "' from '" + state.expected.source_id + "'");
 }
 if (!std::isfinite(header.stamp_s) || header.stamp_s < 0.0 ||
 header.stamp_s > kMaximumRepresentableStampS) {
 fail(now, source, "timestamp", "sample stamp is not a representable time");
 continue;
 }
 const TimePoint sample_time = TimePoint::from_seconds(header.stamp_s);
 if (!state.seen) {
 state.seen = true;
 state.first_receive_time = now;
 } else {
 if (sample_time < state.last_sample_time) {
 fail(now, source, "ordering",
 "sample time moved backwards by " +
 std::to_string(state.last_sample_time.seconds() - sample_time.seconds()) + " s");
 }
 if (header.sequence <= state.last_sequence) {
 fail(now, source, "sequence",
 "sequence " + std::to_string(header.sequence) +
 " did not increase beyond " + std::to_string(state.last_sequence));
 }
 }
 const double age_s = now.seconds() - sample_time.seconds();
 if (age_s > config_.maximum_sample_age_s) {
 fail(now, source, "freshness",
 "sample is " + std::to_string(age_s) + " s old");
 } else if (-age_s > config_.maximum_future_skew_s) {
 fail(now, source, "freshness",
 "sample is " + std::to_string(-age_s) + " s in the future");
 }
 state.last_sample_time = sample_time;
 state.last_sequence = header.sequence;
 state.last_receive_time = now;
 ++state.samples;
 if (state.has_expected && state.expected.nominal_rate_hz > 0.0 &&
 state.samples >= std::max<std::size_t>(2, config_.minimum_rate_samples) &&
 !state.rate_reported) {
 const double span_s = state.last_receive_time.seconds() - state.first_receive_time.seconds();
 if (span_s > 0.0) {
 const double observed = static_cast<double>(state.samples - 1) / span_s;
 const double nominal = state.expected.nominal_rate_hz;
 const double tolerance = nominal * config_.rate_tolerance_ratio;
 if (std::abs(observed - nominal) > tolerance) {
 fail(now, source, "rate",
 "observed " + std::to_string(observed) + " Hz against declared " +
 std::to_string(nominal) + " Hz");
 }
 state.rate_reported = true;
 }
 }
 }
}

void ProviderConformanceMonitor::check_liveness(TimePoint now) {
 if (!started_) {
 started_ = true;
 start_time_ = now;
 }
 for (auto& [kind, state] : sources_) {
 (void)kind;
 if (!state.has_expected || state.expected.nominal_rate_hz <= 0.0)
 continue;
 if (state.silence_reported)
 continue;
 const double limit_s = std::max(config_.silence_timeout_s,
 3.0 / state.expected.nominal_rate_hz);
 const TimePoint reference = state.seen ? state.last_receive_time : start_time_;
 const double silent_s = now.seconds() - reference.seconds();
 if (silent_s > limit_s) {
 fail(now, state.expected.source_id, "silence",
 "no sample for " + std::to_string(silent_s) + " s; limit " +
 std::to_string(limit_s) + " s");
 state.silence_reported = true;
 }
 }
}

ProviderConformanceReport ProviderConformanceMonitor::report(TimePoint now) const {
 ProviderConformanceReport report;
 report.frames = frames_;
 report.measurements = measurements_;
 report.duration_s = started_ ? now.seconds() - start_time_.seconds() : 0.0;
 report.failures = failures_;
 for (const auto& [kind, state] : sources_) {
 if (!state.seen || state.samples < 2)
 continue;
 const double span_s = state.last_receive_time.seconds() - state.first_receive_time.seconds();
 if (span_s <= 0.0)
 continue;
 const std::string key = state.has_expected ? state.expected.source_id
 : endpoint_kind_name(kind);
 report.observed_rate_hz[key] =
 static_cast<double>(state.samples - 1) / span_s;
 }
 report.passed = failures_.empty() && frames_ > 0 && measurements_ > 0;
 return report;
}

} // namespace sim::runtime
