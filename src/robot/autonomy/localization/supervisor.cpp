#include "robot/autonomy/localization/supervisor.hpp"

#include <algorithm>
#include <cmath>

#include "robot/core/geometry.hpp"

namespace lost_signal::robot::autonomy {
namespace {

// Radius of the ellipsoid containing `probability` for three independent
// normalized Gaussian pose errors (x, y, yaw). For k=3 the chi-square CDF
// has a closed form, so a bounded bisection avoids a table or tuned N-sigma
// constant.
double pose_quantile_3d(double probability) {
 if (!(probability > 0.0 && probability < 1.0)) return 0.0;
 const auto cdf = [](double radius) {
 const double x = radius * radius;
 return std::erf(std::sqrt(0.5 * x)) -
 std::sqrt(2.0 * x / core::kPi) * std::exp(-0.5 * x);
 };
 double low = 0.0;
 double high = 16.0;
 for (int iteration = 0; iteration < 80; ++iteration) {
 const double middle = 0.5 * (low + high);
 if (cdf(middle) < probability) low = middle;
 else high = middle;
 }
 return high;
}

} // namespace

LocalizationSupervisor::LocalizationSupervisor(LocalizationSupervisorConfig config)
    : config_(config) {}

void LocalizationSupervisor::reset(bool localized_at_start) {
  localized_ = localized_at_start;
  have_run_ = false;
  last_run_s_ = 0.0;
  last_global_attempt_s_ = -1.0e9;
  last_x_m_ = 0.0;
  last_y_m_ = 0.0;
  last_heading_rad_ = 0.0;
  consecutive_local_failures_ = 0;
}

bool LocalizationSupervisor::movement_due(double now_s, double x_m,
                                          double y_m,
                                          double heading_rad) const {
  if (!have_run_) return true;
  if (now_s - last_run_s_ >= config_.maximum_idle_interval_s) return true;
  return std::hypot(x_m - last_x_m_, y_m - last_y_m_) >=
             config_.minimum_translation_m ||
         std::abs(core::wrap_angle(heading_rad - last_heading_rad_)) >=
             config_.minimum_heading_rad;
}

LocalizationAction LocalizationSupervisor::next(
    double now_s, double odom_x_m, double odom_y_m, double odom_heading_rad,
    const LocalizationHealth& health, std::string* reason) {
  const bool retry_due =
      now_s - last_global_attempt_s_ >= config_.global_retry_interval_s;
  if (!localized_) {
    if (mode_ == StartupMode::kSlamOnly) {
      if (reason) *reason = "slam-only startup: no surveyed map to search";
      return LocalizationAction::kDefer;
    }
    if (retry_due) {
      last_global_attempt_s_ = now_s;
      last_run_s_ = now_s;
      last_x_m_ = odom_x_m;
      last_y_m_ = odom_y_m;
      last_heading_rad_ = odom_heading_rad;
      have_run_ = true;
      if (reason) *reason = "initial pose is unknown";
      return LocalizationAction::kGlobalRelocalization;
    }
    if (reason) *reason = "global relocalization backoff";
    return LocalizationAction::kDefer;
  }
  const bool finite_uncertainty =
      std::isfinite(health.position_sigma_m) &&
      std::isfinite(health.heading_sigma_rad);
  const bool pose_uncertain =
      !finite_uncertainty ||
      health.position_sigma_m >= config_.lost_position_sigma_m ||
      health.heading_sigma_rad >= config_.lost_heading_sigma_rad;
 // A rejected scan match is weak negative evidence: wheel/IMU odometry can
 // remain well constrained in feature-poor space. Global recovery begins
 // only when the fused state is invalid or its covariance says the local
 // frame is actually lost. Confidence/age and the rejection counter remain
 // diagnostics; neither may independently cause a whole-map search.
 if (pose_uncertain || !health.valid) {
    if (retry_due) {
      last_global_attempt_s_ = now_s;
      last_run_s_ = now_s;
      last_x_m_ = odom_x_m;
      last_y_m_ = odom_y_m;
      last_heading_rad_ = odom_heading_rad;
      have_run_ = true;
      if (reason) {
        *reason = !finite_uncertainty
                      ? "pose covariance is invalid"
                      : pose_uncertain
                            ? "pose covariance exceeded recovery threshold"
 : !health.valid
                                        ? "localization invalid"
 : "pose covariance exceeded recovery threshold";
      }
      return LocalizationAction::kGlobalRelocalization;
    }
    if (reason) *reason = "pose covariance high; global retry backoff";
    return LocalizationAction::kDefer;
  }
  if (!have_run_ || (now_s - last_run_s_ >= config_.correction_interval_s &&
                     movement_due(now_s, odom_x_m, odom_y_m, odom_heading_rad))) {
    last_run_s_ = now_s;
    last_x_m_ = odom_x_m;
    last_y_m_ = odom_y_m;
    last_heading_rad_ = odom_heading_rad;
    have_run_ = true;
    if (reason) *reason = "bounded local correction due";
    return LocalizationAction::kLocalCorrection;
  }
  if (reason) *reason = "local correction cadence or motion threshold not due";
  return LocalizationAction::kDefer;
}

void LocalizationSupervisor::accept() {
  localized_ = true;
  consecutive_local_failures_ = 0;
}

void LocalizationSupervisor::record_local(bool accepted,
                                          const ScanMatchResult& /*m*/) {
  if (accepted) {
    accept();
    return;
  }
  ++consecutive_local_failures_;
}

void LocalizationSupervisor::record_global(bool accepted,
                                           const ScanMatchResult& /*m*/) {
  if (accepted) accept();
}

MapWindow2D LocalizationSupervisor::local_window(double odom_x_m,
                                                 double odom_y_m) const {
  const double half = std::max(0.5, config_.local_window_half_extent_m);
  return {odom_x_m - half, odom_y_m - half, odom_x_m + half, odom_y_m + half};
}

double LocalizationSupervisor::global_recovery_radius_m(
    double position_sigma_m) const {
  const double floor_m =
      std::isfinite(config_.minimum_global_recovery_radius_m)
          ? std::max(0.0, config_.minimum_global_recovery_radius_m)
          : 0.0;
  const double probability = config_.global_recovery_probability;
  if (!(position_sigma_m >= 0.0) || !std::isfinite(position_sigma_m) ||
      !(probability > 0.0 && probability < 1.0)) {
    return floor_m;
  }
 return std::max(floor_m,
 pose_quantile_3d(probability) * position_sigma_m);
}

double LocalizationSupervisor::global_recovery_heading_radius_rad(
 double heading_sigma_rad) const {
 const double floor_rad =
 std::isfinite(config_.minimum_global_recovery_heading_rad)
 ? std::max(0.0, config_.minimum_global_recovery_heading_rad)
 : 0.0;
 const double probability = config_.global_recovery_probability;
 if (!(heading_sigma_rad >= 0.0) || !std::isfinite(heading_sigma_rad) ||
 !(probability > 0.0 && probability < 1.0)) {
 return floor_rad;
 }
 return std::min(core::kPi, std::max(
 floor_rad, pose_quantile_3d(probability) * heading_sigma_rad));
}

double LocalizationSupervisor::global_recovery_quantile() const {
  return pose_quantile_3d(config_.global_recovery_probability);
}

bool LocalizationSupervisor::accept_global_innovation(
 double position_innovation_m, double heading_innovation_rad,
 double position_sigma_m, double heading_sigma_rad) const {
 if (!std::isfinite(position_innovation_m) ||
 !std::isfinite(heading_innovation_rad) ||
 position_innovation_m < 0.0 || heading_innovation_rad < 0.0) {
 return false;
 }
 const double quantile = pose_quantile_3d(
 config_.global_recovery_probability);
 if (!(quantile > 0.0)) return false;
 const double position_scale = std::max(
 std::isfinite(position_sigma_m) ? std::max(0.0, position_sigma_m) : 0.0,
 std::max(0.0, config_.minimum_global_recovery_radius_m) / quantile);
 const double heading_scale = std::max(
 std::isfinite(heading_sigma_rad) ? std::max(0.0, heading_sigma_rad) : 0.0,
 std::max(0.0, config_.minimum_global_recovery_heading_rad) / quantile);
 if (!(position_scale > 0.0) || !(heading_scale > 0.0)) return false;
 const double normalized_position = position_innovation_m / position_scale;
 const double normalized_heading = heading_innovation_rad / heading_scale;
 return normalized_position * normalized_position +
 normalized_heading * normalized_heading <= quantile * quantile;
}

}  // namespace lost_signal::robot::autonomy
