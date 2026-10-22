#pragma once

#include <string>

namespace lost_signal::robot::autonomy {

// Startup/recovery selection: which localization frontend owns the pose.
// Pure decision — the coordinator acts on it. Never invents a map pose
// after a rejected match; the SLAM-only path trusts the odometry origin
// and lets the session bound the drift it can observe.
//
//   - raster + operator/dock prior -> kSeedAndTrack (seed_pose, then local)
//   - raster, no prior            -> kGlobalSearch (whole-map recovery)
//   - no raster                   -> kSlamOnly (seed odometry origin once,
//                                    SLAM frontend + local map only)
enum class StartupMode {
  kSeedAndTrack,
  kGlobalSearch,
  kSlamOnly,
};

struct StartupDecision {
  StartupMode mode = StartupMode::kGlobalSearch;
  std::string reason = "init";
};

inline StartupDecision select_startup_mode(bool has_raster,
                                           bool has_prior_pose) {
  if (!has_raster) {
    return {StartupMode::kSlamOnly, "no static raster: trust odometry origin"};
  }
  if (has_prior_pose) {
    return {StartupMode::kSeedAndTrack, "known start pose on a surveyed map"};
  }
  return {StartupMode::kGlobalSearch, "unknown start on a surveyed map"};
}

inline const char* startup_mode_name(StartupMode mode) {
  switch (mode) {
    case StartupMode::kSeedAndTrack:
      return "seed-and-track";
    case StartupMode::kGlobalSearch:
      return "global-search";
    case StartupMode::kSlamOnly:
      return "slam-only";
  }
  return "unknown";
}

}  // namespace lost_signal::robot::autonomy
