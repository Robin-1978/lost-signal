#pragma once

#include "robot/runtime/map_provider.hpp"

#include <cstddef>
#include <string>

namespace sim::autonomy {

// Policy that decides when a localization provider may run, and whether it may
// fall back to a whole-map search. This is ROBOT policy: an expensive global
// search runs only when the pose estimate is no longer trustworthy, never
// merely because one scan happened to be hard to match.
//
// The deployable stack follows the same shape as AMCL/multi-hypothesis
// relocalization: continuous correction stays local, and global recovery is a
// bounded retry that must also respect the map provider query window.
struct LocalizationSupervisorConfig {
 // Correction cadence. A 10-40 Hz LiDAR does not imply a 10-40 Hz matcher.
 double correction_interval_s{0.20};
 // Minimum spacing between whole-map searches once recovery started.
 double global_retry_interval_s{2.0};
 // Motion that makes a new local correction worth running.
 double minimum_translation_m{0.05};
 double minimum_heading_rad{0.02};
 // A stationary robot is still re-checked this often.
 double maximum_idle_interval_s{5.0};
 // Map region handed to the local matcher around the odometric prior.
 // The view must contain the entire sensor footprint plus the local search
 // neighbourhood. Thirty metres keeps this bounded on large maps while
 // covering the small authored and warehouse maps used by the simulator.
 double local_window_half_extent_m{30.0};
 double lost_position_sigma_m{0.45};
 double lost_heading_sigma_rad{0.25};
 // A matcher rejection is not itself a loss of pose. Repeated independent
 // rejections are evidence that the local hypothesis is no longer usable and
 // may request the rate-limited global recovery path.
 std::size_t local_failure_recovery_threshold{3};
};

enum class LocalizationAction {
 Defer,               // not due yet; keep the current estimate
 LocalCorrection,     // bounded scan-to-map correction around the prior
 GlobalRelocalization // whole-map hypothesis search
};

// Health supplied by the estimator to the correction scheduler. Sigma drives
// pose recovery; confidence and age prevent a stationary, stale or
// low-information estimate from appearing healthy merely because odometry
// noise happens to remain small.
struct LocalizationHealth {
 bool valid{true};
 double position_sigma_m{0.05};
 double heading_sigma_rad{0.01};
 double confidence{1.0};
 double age_s{0.0};
};

const char* localization_action_name(LocalizationAction action);

class LocalizationSupervisor {
 public:
 explicit LocalizationSupervisor(LocalizationSupervisorConfig config = {})
 : config_(config) {}

 // Deployment tuning arrives from robot_data; the schedule itself stays in
 // ROBOT so a hardware bring-up and a simulator run share one policy.
 void set_config(const LocalizationSupervisorConfig& config) { config_ = config; }

 void reset(bool localized_at_start);

 LocalizationAction next(double now_s, Vec2 odom_position, double odom_heading_rad,
 const LocalizationHealth& health,
 std::string* reason = nullptr);

 // Records the outcome of the action returned by next(). A rejected local
 // correction is kept as evidence; a rejected global search keeps the retry
 // backoff, so a hard scan cannot spin the whole map search.
 void record_local(bool accepted, double now_s, const PoseObservation& measurement);
 void record_global(bool accepted, double now_s, const PoseObservation& measurement);

 bool localized() const { return localized_; }
 std::size_t consecutive_failures() const { return consecutive_local_failures_; }
 runtime::MapBounds2D local_window(Vec2 odom_position) const;
 const LocalizationSupervisorConfig& config() const { return config_; }

 private:
 bool movement_due(double now_s, Vec2 position, double heading) const;
 void accept(const PoseObservation& measurement, double now_s);

 LocalizationSupervisorConfig config_;
 bool localized_{false};
 bool have_run_{false};
 double last_run_s_{0.0};
 double last_global_attempt_s_{-1.0e9};
 Vec2 last_position_;
 double last_heading_rad_{0.0};
 std::size_t consecutive_local_failures_{0};
};

} // namespace sim::autonomy
