#include "robot/iface/conformance.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace lost_signal::robot::iface {
namespace {

constexpr double kNsPerS = 1e9;

// Native valid_observation_frame() returns bool without a reason string, so
// the monitor re-states which structural property broke for the report.
std::string structure_detail(const ObservationFrame& frame) {
  if (frame.source.empty()) return "empty source id";
  if (frame.stamp_ns == core::kInvalidTime || frame.stamp_ns < 0)
    return "invalid sample timestamp";
  if (frame.sequence == 0) return "sequence is zero (must start at 1)";
  if (frame.empty()) return "frame carries no data";
  return "structural check failed";
}

}  // namespace

const char* endpoint_role_name(EndpointRole role) {
  switch (role) {
    case EndpointRole::kLidar:
      return "lidar";
    case EndpointRole::kImu:
      return "imu";
    case EndpointRole::kEncoder:
      return "encoder";
    case EndpointRole::kActuator:
      return "actuator";
    case EndpointRole::kPower:
      return "power";
  }
  return "unknown";
}

const char* frame_name(core::Frame frame) {
  switch (frame) {
    case core::Frame::kMap:
      return "map";
    case core::Frame::kOdom:
      return "odom";
    case core::Frame::kBaseLink:
      return "base_link";
    case core::Frame::kLidarLink:
      return "lidar_link";
    case core::Frame::kImuLink:
      return "imu_link";
    case core::Frame::kUnknown:
      return "unknown";
  }
  return "unknown";
}

ProviderConformanceMonitor::ProviderConformanceMonitor(
    ProviderConformanceConfig config)
    : config_(config) {}

void ProviderConformanceMonitor::expect(const EndpointDescriptor& descriptor,
                                        double nominal_rate_hz) {
  SourceState& state = sources_[descriptor.endpoint_id];
  state.expected = descriptor;
  state.has_expected = true;
  state.nominal_rate_hz = nominal_rate_hz;
}

void ProviderConformanceMonitor::fail(core::MonotonicNs now_ns,
                                     const std::string& source,
                                     const std::string& rule,
                                     const std::string& detail) {
  // One entry per source and rule keeps a fifty-hertz fault from producing
  // thousands of identical lines while still naming every broken contract.
  const std::string key = source + "|" + rule;
  if (!reported_.insert(key).second) return;
  failures_.push_back(
      ProviderConformanceFailure{now_ns, source, rule, detail});
}

void ProviderConformanceMonitor::observe(const ObservationFrame& frame,
                                         core::MonotonicNs now_ns) {
  if (!started_) {
    started_ = true;
    start_ns_ = now_ns;
  }
  ++frames_;

  if (!valid_observation_frame(frame))
    fail(now_ns, frame.source.empty() ? "frame" : frame.source, "structure",
         structure_detail(frame));

  // Native frames have no absolute-pose slot; a map-frame stamp is the
  // equivalent localization-contract crossing.
  if (frame.frame == core::Frame::kMap && !config_.allow_absolute_pose) {
    fail(now_ns, frame.source, "absolute_pose",
         "provider published a map-frame pose without declaring that "
         "capability");
  }

  // Native frames carry exactly one measurement each, keyed by source id.
  // An unknown source creates an unregistered entry: it is tracked for
  // ordering/freshness, but rate/silence rules need an expect() nominal rate.
  SourceState& state = sources_[frame.source];
  state.silence_reported = false;
  if (!state.seen) state.observed_source = frame.source;
  ++measurements_;

  const std::string& source =
      state.has_expected ? state.expected.endpoint_id : state.observed_source;

  // Identity on the native boundary is the frame convention: the lookup is
  // already keyed by source id, so the checkable half is the frame half.
  // frame_name() maps the Frame enum to its REP-105 string for the compare.
  if (state.has_expected && !state.expected.frame_id.empty() &&
      std::string(frame_name(frame.frame)) != state.expected.frame_id) {
    fail(now_ns, source, "identity",
         "measured frame '" + std::string(frame_name(frame.frame)) +
             "' does not match the registered endpoint '" +
             state.expected.frame_id + "'");
  }

  if (frame.stamp_ns == core::kInvalidTime || frame.stamp_ns < 0) {
    fail(now_ns, source, "timestamp",
         "sample stamp is not a representable time");
    return;
  }
  if (!state.seen) {
    state.seen = true;
    state.first_receive_ns = now_ns;
  } else {
    if (frame.stamp_ns < state.last_sample_time_ns) {
      fail(now_ns, source, "ordering",
           "sample time moved backwards by " +
               std::to_string((state.last_sample_time_ns - frame.stamp_ns) /
                              kNsPerS) +
               " s");
    }
    if (frame.sequence <= state.last_sequence) {
      fail(now_ns, source, "sequence",
           "sequence " + std::to_string(frame.sequence) +
               " did not increase beyond " +
               std::to_string(state.last_sequence));
    }
  }
  const core::MonotonicNs age_ns = now_ns - frame.stamp_ns;
  if (age_ns > config_.maximum_sample_age_ns) {
    fail(now_ns, source, "freshness",
         "sample is " + std::to_string(age_ns / kNsPerS) + " s old");
  } else if (-age_ns > config_.maximum_future_skew_ns) {
    fail(now_ns, source, "freshness",
         "sample is " + std::to_string(-age_ns / kNsPerS) +
             " s in the future");
  }
  state.last_sample_time_ns = frame.stamp_ns;
  state.last_sequence = frame.sequence;
  state.last_receive_ns = now_ns;
  ++state.samples;
  if (state.has_expected && state.nominal_rate_hz > 0.0 &&
      state.samples >=
          std::max<std::size_t>(2, config_.minimum_rate_samples) &&
      !state.rate_reported) {
    const core::MonotonicNs span_ns =
        state.last_receive_ns - state.first_receive_ns;
    if (span_ns > 0) {
      const double observed =
          static_cast<double>(state.samples - 1) * kNsPerS / span_ns;
      const double nominal = state.nominal_rate_hz;
      const double tolerance = nominal * config_.rate_tolerance_ratio;
      if (std::abs(observed - nominal) > tolerance) {
        fail(now_ns, source, "rate",
             "observed " + std::to_string(observed) +
                 " Hz against declared " + std::to_string(nominal) + " Hz");
      }
      state.rate_reported = true;
    }
  }
}

void ProviderConformanceMonitor::check_liveness(core::MonotonicNs now_ns) {
  if (!started_) {
    started_ = true;
    start_ns_ = now_ns;
  }
  for (auto& entry : sources_) {
    SourceState& state = entry.second;
    if (!state.has_expected || state.nominal_rate_hz <= 0.0) continue;
    if (state.silence_reported) continue;
    const auto period_ns = static_cast<core::MonotonicNs>(
        3.0 * kNsPerS / state.nominal_rate_hz);
    const core::MonotonicNs limit_ns =
        std::max(config_.silence_timeout_ns, period_ns);
    const core::MonotonicNs reference =
        state.seen ? state.last_receive_ns : start_ns_;
    const core::MonotonicNs silent_ns = now_ns - reference;
    if (silent_ns > limit_ns) {
      fail(now_ns, state.expected.endpoint_id, "silence",
           "no sample for " + std::to_string(silent_ns / kNsPerS) +
               " s; limit " + std::to_string(limit_ns / kNsPerS) + " s");
      state.silence_reported = true;
    }
  }
}

ProviderConformanceReport ProviderConformanceMonitor::report(
    core::MonotonicNs now_ns) const {
  ProviderConformanceReport out;
  out.frames = frames_;
  out.measurements = measurements_;
  out.duration_ns = started_ ? now_ns - start_ns_ : 0;
  out.failures = failures_;
  for (const auto& entry : sources_) {
    const SourceState& state = entry.second;
    if (!state.seen || state.samples < 2) continue;
    const core::MonotonicNs span_ns =
        state.last_receive_ns - state.first_receive_ns;
    if (span_ns <= 0) continue;
    const std::string key = state.has_expected ? state.expected.endpoint_id
                                               : state.observed_source;
    out.observed_rate_hz[key] =
        static_cast<double>(state.samples - 1) * kNsPerS / span_ns;
  }
  out.passed = failures_.empty() && frames_ > 0 && measurements_ > 0;
  return out;
}

}  // namespace lost_signal::robot::iface
