#pragma once

#include <cstddef>
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>

#include "robot/core/time.hpp"
#include "robot/iface/endpoint.hpp"
#include "robot/iface/observation.hpp"

namespace lost_signal::robot::iface {

// Stream contract for a provider adapter. Structural payload checks live in
// valid_observation_frame(); this monitor adds the properties that only a
// stream can exhibit: identity, ordering, freshness, rate and silence. Sim,
// replay and native adapters run the same monitor before their frames may
// reach ROBOT, so a vendor driver cannot pass bring-up with data that only
// looks valid one frame at a time.
//
// Time base: core::MonotonicNs (int64 nanoseconds on the ROBOT monotonic
// clock). All durations in the config are nanoseconds, matching the codebase
// style (e.g. SafetyConfig::stale_timeout_ns). Rates stay in Hz (double).
struct ProviderConformanceConfig {
  // A sample may be at most this old, or this far into the future, relative
  // to the caller's monotonic clock.
  core::MonotonicNs maximum_sample_age_ns = 300'000'000;
  core::MonotonicNs maximum_future_skew_ns = 20'000'000;
  // Observed rate is compared with the expected nominal rate once enough
  // samples exist. A provider may not silently halve or double its rate.
  double rate_tolerance_ratio = 0.35;
  std::size_t minimum_rate_samples = 8;
  // A source that stays silent longer than this (or three nominal periods)
  // is a failure, not a pause.
  core::MonotonicNs silence_timeout_ns = 500'000'000;
  // Map-frame poses are opt-in. A LiDAR/IMU odometry provider that starts
  // publishing in core::Frame::kMap has crossed the localization contract,
  // and a native adapter must not forward a device pose into ROBOT by
  // accident.
  bool allow_absolute_pose = false;
};

struct ProviderConformanceFailure {
  core::MonotonicNs time_ns = core::kInvalidTime;
  std::string source;
  std::string rule;
  std::string detail;
};

struct ProviderConformanceReport {
  bool passed = false;
  std::size_t frames = 0;
  std::size_t measurements = 0;  // one measurement per native frame
  core::MonotonicNs duration_ns = 0;
  // Observed rate per source, measured between receive times.
  std::map<std::string, double> observed_rate_hz;
  std::vector<ProviderConformanceFailure> failures;
};

class ProviderConformanceMonitor {
 public:
  explicit ProviderConformanceMonitor(ProviderConformanceConfig config = {});

  // Declares what an endpoint is expected to publish, keyed by
  // descriptor.endpoint_id. Registering the same endpoint again replaces the
  // previous expectation (stream state is kept).
  //
  // NOTE: the native EndpointDescriptor carries no nominal rate, so it is
  // passed explicitly. A non-positive rate disables the rate and silence
  // rules for that source; identity/ordering/freshness still apply.
  void expect(const EndpointDescriptor& descriptor,
              double nominal_rate_hz = 0.0);

  // Feeds one provider frame. `now_ns` is the ROBOT monotonic clock, which is
  // independent of the device clock and of the sample timestamps.
  void observe(const ObservationFrame& frame, core::MonotonicNs now_ns);

  // Reports sources that stopped publishing. Call at the deployment rate.
  void check_liveness(core::MonotonicNs now_ns);

  // Records a boundary rejection that cannot be represented by an
  // ObservationFrame (for example a typed MCU capability mismatch). Keeping
  // it in the same report makes native and provider faults observable through
  // one fail-closed conformance result.
  void reject(core::MonotonicNs now_ns, const std::string& source,
              const std::string& rule, const std::string& detail);

  ProviderConformanceReport report(core::MonotonicNs now_ns) const;

 private:
  struct SourceState {
    EndpointDescriptor expected;
    bool has_expected = false;
    double nominal_rate_hz = 0.0;
    std::string observed_source;  // frame.source for unregistered sources
    bool seen = false;
    std::uint64_t last_sequence = 0;
    core::MonotonicNs last_sample_time_ns = core::kInvalidTime;
    core::MonotonicNs first_receive_ns = core::kInvalidTime;
    core::MonotonicNs last_receive_ns = core::kInvalidTime;
    std::size_t samples = 0;
    bool rate_reported = false;
    bool silence_reported = false;
  };

  void fail(core::MonotonicNs now_ns, const std::string& source,
            const std::string& rule, const std::string& detail);
 void observe_measurement(const MeasurementHeader& header,
 core::MonotonicNs now_ns);

  ProviderConformanceConfig config_;
  std::map<std::string, SourceState> sources_;  // keyed by endpoint id
  std::set<std::string> reported_;              // source|rule dedup
  std::vector<ProviderConformanceFailure> failures_;
  std::size_t frames_ = 0;
  std::size_t measurements_ = 0;
  bool started_ = false;
  core::MonotonicNs start_ns_ = core::kInvalidTime;
};

const char* endpoint_role_name(EndpointRole role);
const char* frame_name(core::Frame frame);

}  // namespace lost_signal::robot::iface
