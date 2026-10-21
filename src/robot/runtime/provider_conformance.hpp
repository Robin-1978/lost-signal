#pragma once

#include "robot/runtime/endpoint_registry.hpp"
#include "robot/runtime/observation.hpp"
#include "robot/runtime/runtime_channel.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace sim::runtime {

// Stream contract for a provider adapter. Structural payload checks live in
// valid_observation_frame(); this monitor adds the properties that only a
// stream can exhibit: identity, ordering, freshness, rate and silence. Native,
// replay and simulated adapters run the same monitor before their frames may
// reach ROBOT, so a vendor driver cannot pass bring-up with data that only
// looks valid one frame at a time.
struct ProviderConformanceConfig {
 // A sample may be at most this old, or this far into the future, relative to
 // the caller's monotonic clock.
 double maximum_sample_age_s{0.30};
 double maximum_future_skew_s{0.02};
 // Observed rate is compared with the endpoint's declared nominal rate once
 // enough samples exist. A provider may not silently halve or double its rate.
 double rate_tolerance_ratio{0.35};
 std::size_t minimum_rate_samples{8};
 // A source that stays silent longer than this (or three nominal periods) is a
 // failure, not a pause.
 double silence_timeout_s{0.50};
 // Absolute poses are opt-in. A LiDAR/IMU odometry provider that starts
 // publishing a map pose has crossed the localization contract, and a native
 // adapter must not forward a device pose into ROBOT by accident.
 bool allow_absolute_pose{false};
};

struct ProviderConformanceFailure {
 double time_s{0.0};
 std::string source_id;
 std::string rule;
 std::string detail;
};

struct ProviderConformanceReport {
 bool passed{false};
 std::size_t frames{0};
 std::size_t measurements{0};
 double duration_s{0.0};
 // Observed rate per source, measured between receive times.
 std::map<std::string, double> observed_rate_hz;
 std::vector<ProviderConformanceFailure> failures;
};

class ProviderConformanceMonitor {
 public:
 explicit ProviderConformanceMonitor(ProviderConformanceConfig config = {})
 : config_(config) {}

 // Declares what an endpoint is expected to publish. Registering the same kind
 // again replaces the previous expectation.
 void expect(const EndpointDescriptor& descriptor);

 // Feeds one provider frame. `now` is the ROBOT monotonic clock, which is
 // independent of the device clock and of the sample timestamps.
 void observe(const ObservationFrame& frame, TimePoint now);

 // Reports sources that stopped publishing. Call at the deployment rate.
 void check_liveness(TimePoint now);

 ProviderConformanceReport report(TimePoint now) const;

 private:
 struct SourceState {
 EndpointDescriptor expected;
 bool has_expected{false};
 bool seen{false};
 std::uint64_t last_sequence{0};
 TimePoint last_sample_time;
 TimePoint first_receive_time;
 TimePoint last_receive_time;
 std::size_t samples{0};
 bool rate_reported{false};
 bool silence_reported{false};
 };

 void fail(TimePoint now, const std::string& source_id, const std::string& rule,
 const std::string& detail);

 ProviderConformanceConfig config_;
 std::map<EndpointKind, SourceState> sources_;
 std::set<std::string> reported_;
 std::vector<ProviderConformanceFailure> failures_;
 std::size_t frames_{0};
 std::size_t measurements_{0};
 bool started_{false};
 TimePoint start_time_;
};

const char* endpoint_kind_name(EndpointKind kind);

} // namespace sim::runtime
