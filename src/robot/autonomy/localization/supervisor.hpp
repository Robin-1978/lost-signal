#pragma once

#include <cstddef>
#include <string>

#include "robot/autonomy/localization/scan_matcher.hpp"
#include "robot/autonomy/localization/startup_selector.hpp"

namespace lost_signal::robot::autonomy {

// Scheduling policy for scan-match corrections: do they run now, and has
// the estimate degraded to the point of needing whole-map recovery?
// A global search runs only when the pose is no longer trustworthy, never
// because one scan was hard to match.
enum class LocalizationAction {
  kDefer = 0,               // not due; keep the dead-reckoned estimate
  kLocalCorrection = 1,     // bounded scan-to-map correction around the prior
  kGlobalRelocalization = 2,  // whole-map hypothesis search (rate-limited)
};

// Startup mode selects which recovery path may run while unlocalized
// (see startup_selector.hpp for the vocabulary): kGlobalSearch is the
// standard cadence; kSlamOnly disables the global branch for missions
// with no surveyed map to search (SLAM loop closures still establish the
// frame through the normal record path).

inline const char* localization_action_name(LocalizationAction action) {
  switch (action) {
    case LocalizationAction::kDefer:
      return "defer";
    case LocalizationAction::kLocalCorrection:
      return "local_correction";
    case LocalizationAction::kGlobalRelocalization:
      return "global_relocalization";
  }
  return "unknown";
}

struct LocalizationSupervisorConfig {
  double correction_interval_s = 0.20;  // a 10-40 Hz lidar need not imply a
                                       // 10-40 Hz matcher
  double global_retry_interval_s = 2.0;
  double minimum_translation_m = 0.05;
  double minimum_heading_rad = 0.02;
  double maximum_idle_interval_s = 5.0;  // stationary robots still re-check
  double local_window_half_extent_m = 30.0;
  double lost_position_sigma_m = 0.45;
  double lost_heading_sigma_rad = 0.25;
 // Innovation gates for corrections from a trusted odometry prior. A scan
 // match that exceeds these uncertainty-scaled bounds is evidence to count
 // as a failed local correction, not a pose to blend in blindly.
 double local_innovation_sigma_factor = 4.0;
 double local_innovation_floor_m = 0.35;
 double local_innovation_floor_rad = 0.20;
 // A localized trajectory may accept a wider global jump only after its
 // covariance has crossed the loss threshold. Below that threshold this is
 // the maximum trusted jump allowed from a whole-map candidate.
 double trusted_global_jump_m = 0.75;
 double trusted_global_jump_sigma_factor = 3.0;
 // Diagnostic threshold retained for telemetry compatibility. Rejections
 // never trigger global recovery by themselves; fused covariance must cross
 // the loss threshold first.
  std::size_t local_failure_recovery_threshold = 3;
};

struct LocalizationHealth {
  bool valid = true;
  double position_sigma_m = 0.05;
  double heading_sigma_rad = 0.01;
  double confidence = 1.0;
  double age_s = 0.0;
};

struct MapWindow2D {
  double min_x_m = 0.0, min_y_m = 0.0;
  double max_x_m = 0.0, max_y_m = 0.0;
};

class LocalizationSupervisor {
 public:
  explicit LocalizationSupervisor(LocalizationSupervisorConfig config = {});

  void set_config(const LocalizationSupervisorConfig& config) {
    config_ = config;
  }
  void reset(bool localized_at_start);
  void set_startup_mode(StartupMode mode) { mode_ = mode; }
  StartupMode startup_mode() const { return mode_; }

  LocalizationAction next(double now_s, double odom_x_m, double odom_y_m,
                          double odom_heading_rad,
                          const LocalizationHealth& health,
                          std::string* reason = nullptr);
  void record_local(bool accepted, const ScanMatchResult& measurement);
  void record_global(bool accepted, const ScanMatchResult& measurement);

  bool localized() const { return localized_; }
  std::size_t consecutive_failures() const {
    return consecutive_local_failures_;
  }
  MapWindow2D local_window(double odom_x_m, double odom_y_m) const;
  const LocalizationSupervisorConfig& config() const { return config_; }

 private:
  bool movement_due(double now_s, double x_m, double y_m,
                    double heading_rad) const;
  void accept();

  LocalizationSupervisorConfig config_;
  StartupMode mode_ = StartupMode::kGlobalSearch;
  bool localized_ = false;
  bool have_run_ = false;
  double last_run_s_ = 0.0;
  double last_global_attempt_s_ = -1.0e9;
  double last_x_m_ = 0.0;
  double last_y_m_ = 0.0;
  double last_heading_rad_ = 0.0;
  std::size_t consecutive_local_failures_ = 0;
};

}  // namespace lost_signal::robot::autonomy
