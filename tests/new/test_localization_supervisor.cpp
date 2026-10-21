#include "robot/autonomy/localization/localization_supervisor.hpp"

#include <cmath>
#include <iostream>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

namespace {

sim::PoseObservation measurement(double time_s) {
 sim::PoseObservation value;
 value.header = {time_s, 1, "map", 0.0, sim::kMeasurementSchemaVersion,
 "test.localizer"};
 return value;
}

} // namespace

int main() {
 sim::autonomy::LocalizationSupervisorConfig config;
 config.correction_interval_s = 0.2;
 config.minimum_translation_m = 0.05;
 config.maximum_idle_interval_s = 5.0;
 config.global_retry_interval_s = 2.0;
 config.local_failure_recovery_threshold = 2;
 config.lost_position_sigma_m = 0.6;
 config.lost_heading_sigma_rad = 0.2;
 sim::autonomy::LocalizationSupervisor supervisor(config);
 supervisor.reset(true);

 std::string reason;
const sim::autonomy::LocalizationHealth healthy{true, 0.05, 0.01, 1.0, 0.0};
CHECK(supervisor.next(0.0, {0.0, 0.0}, 0.0, healthy, &reason) ==
 sim::autonomy::LocalizationAction::LocalCorrection);
 supervisor.record_local(true, 0.0, measurement(0.0));
CHECK(supervisor.next(0.05, {0.01, 0.0}, 0.0, healthy, &reason) ==
 sim::autonomy::LocalizationAction::Defer);

 // One rejected scan is evidence only. It must not launch a whole-map search.
CHECK(supervisor.next(0.25, {0.10, 0.0}, 0.0, healthy, &reason) ==
 sim::autonomy::LocalizationAction::LocalCorrection);
 supervisor.record_local(false, 0.25, measurement(0.25));
CHECK(supervisor.next(0.50, {0.20, 0.0}, 0.0, healthy, &reason) ==
 sim::autonomy::LocalizationAction::LocalCorrection);
supervisor.record_local(false, 0.50, measurement(0.50));
CHECK(supervisor.next(0.75, {0.30, 0.0}, 0.0, healthy, &reason) ==
 sim::autonomy::LocalizationAction::GlobalRelocalization);
supervisor.record_global(true, 0.75, measurement(0.75));

 // Covariance, rather than failure count, is the recovery trigger.
const sim::autonomy::LocalizationHealth uncertain{true, 0.61, 0.01, 1.0, 0.0};
CHECK(supervisor.next(1.00, {0.30, 0.0}, 0.0, uncertain, &reason) ==
 sim::autonomy::LocalizationAction::Defer);
CHECK(supervisor.next(2.75, {0.30, 0.0}, 0.0, uncertain, &reason) ==
 sim::autonomy::LocalizationAction::GlobalRelocalization);
supervisor.record_global(false, 2.75, measurement(2.75));
 CHECK(supervisor.localized());

 // A low-covariance stationary robot is not matched at LiDAR cadence.
CHECK(supervisor.next(2.80, {0.30, 0.0}, 0.0, healthy, &reason) ==
 sim::autonomy::LocalizationAction::Defer);
CHECK(supervisor.next(7.80, {0.30, 0.0}, 0.0, healthy, &reason) ==
 sim::autonomy::LocalizationAction::LocalCorrection);
supervisor.record_local(true, 7.80, measurement(7.80));

// Sustained quality loss and stale evidence recover globally even while the
// covariance remains small. A single low-quality sample is not enough by
// itself because the estimator's confidence carries observation age.
const sim::autonomy::LocalizationHealth low_quality{true, 0.05, 0.01, 0.20, 0.0};
CHECK(supervisor.next(8.1, {0.30, 0.0}, 0.0, low_quality, &reason) ==
 sim::autonomy::LocalizationAction::GlobalRelocalization);
supervisor.record_global(false, 8.1, measurement(8.1));
const sim::autonomy::LocalizationHealth stale{true, 0.05, 0.01, 1.0, 2.5};
CHECK(supervisor.next(8.4, {0.30, 0.0}, 0.0, stale, &reason) ==
 sim::autonomy::LocalizationAction::Defer);
CHECK(supervisor.next(10.2, {0.30, 0.0}, 0.0, stale, &reason) ==
 sim::autonomy::LocalizationAction::GlobalRelocalization);
 CHECK(std::isfinite(supervisor.local_window({1.0, 2.0}).min_m.x));
 return 0;
}
