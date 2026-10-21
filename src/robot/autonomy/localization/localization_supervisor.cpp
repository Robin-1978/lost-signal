#include "robot/autonomy/localization/localization_supervisor.hpp"

#include <cmath>

namespace sim::autonomy {

const char* localization_action_name(LocalizationAction action) {
 switch (action) {
 case LocalizationAction::Defer: return "defer";
 case LocalizationAction::LocalCorrection: return "local_correction";
 case LocalizationAction::GlobalRelocalization: return "global_relocalization";
 }
 return "unknown";
}

void LocalizationSupervisor::reset(bool localized_at_start) {
 localized_ = localized_at_start;
 have_run_ = false;
 last_run_s_ = 0.0;
 last_global_attempt_s_ = -1.0e9;
 last_position_ = {};
 last_heading_rad_ = 0.0;
 consecutive_local_failures_ = 0;
}

bool LocalizationSupervisor::movement_due(double now_s, Vec2 position,
 double heading) const {
 if (!have_run_)
 return true;
 if (now_s - last_run_s_ >= config_.maximum_idle_interval_s)
 return true;
 return std::hypot(position.x - last_position_.x, position.y - last_position_.y) >=
 config_.minimum_translation_m ||
 std::abs(wrap_angle(heading - last_heading_rad_)) >= config_.minimum_heading_rad;
}

LocalizationAction LocalizationSupervisor::next(
 double now_s, Vec2 odom_position, double odom_heading_rad,
 const LocalizationHealth& health, std::string* reason) {
 const bool retry_due = now_s - last_global_attempt_s_ >= config_.global_retry_interval_s;
 if (!localized_) {
 if (retry_due) {
 last_global_attempt_s_ = now_s;
 last_run_s_ = now_s;
 last_position_ = odom_position;
 last_heading_rad_ = odom_heading_rad;
 have_run_ = true;
 if (reason) *reason = "initial pose is unknown";
 return LocalizationAction::GlobalRelocalization;
 }
 if (reason) *reason = "global relocalization backoff";
 return LocalizationAction::Defer;
 }
 const bool finite_uncertainty = std::isfinite(health.position_sigma_m) &&
 std::isfinite(health.heading_sigma_rad);
 const bool pose_uncertain = !finite_uncertainty ||
 health.position_sigma_m >= config_.lost_position_sigma_m ||
 health.heading_sigma_rad >= config_.lost_heading_sigma_rad;
 const bool repeated_local_failure = config_.local_failure_recovery_threshold > 0 &&
 consecutive_local_failures_ >= config_.local_failure_recovery_threshold;
 const bool evidence_untrusted = !health.valid ||
 !std::isfinite(health.confidence) || !std::isfinite(health.age_s) ||
 health.confidence < 0.35 || health.age_s > 2.0;
 if (pose_uncertain || evidence_untrusted || repeated_local_failure) {
 if (retry_due) {
 last_global_attempt_s_ = now_s;
 last_run_s_ = now_s;
 last_position_ = odom_position;
 last_heading_rad_ = odom_heading_rad;
 have_run_ = true;
 if (reason) *reason = !finite_uncertainty
 ? "pose covariance is invalid"
 : pose_uncertain ? "pose covariance exceeded recovery threshold"
 : repeated_local_failure ? "local correction rejected repeatedly"
 : !health.valid ? "localization invalid"
 : health.age_s > 2.0 ? "localization evidence is stale"
 : "localization confidence below recovery threshold";
 return LocalizationAction::GlobalRelocalization;
 }
 if (reason) *reason = "pose covariance high; global retry backoff";
 return LocalizationAction::Defer;
 }
 if (!have_run_ || (now_s - last_run_s_ >= config_.correction_interval_s &&
 movement_due(now_s, odom_position, odom_heading_rad))) {
 last_run_s_ = now_s;
 last_position_ = odom_position;
 last_heading_rad_ = odom_heading_rad;
 have_run_ = true;
 if (reason) *reason = "bounded local correction due";
 return LocalizationAction::LocalCorrection;
 }
 if (reason) *reason = "local correction cadence or motion threshold not due";
 return LocalizationAction::Defer;
}

void LocalizationSupervisor::accept(const PoseObservation&, double) {
 localized_ = true;
 consecutive_local_failures_ = 0;
}

void LocalizationSupervisor::record_local(bool accepted, double now_s,
 const PoseObservation& measurement) {
 if (accepted) {
 accept(measurement, now_s);
 return;
 }
 ++consecutive_local_failures_;
}

void LocalizationSupervisor::record_global(bool accepted, double now_s,
 const PoseObservation& measurement) {
 if (accepted)
 accept(measurement, now_s);
}

runtime::MapBounds2D LocalizationSupervisor::local_window(Vec2 odom_position) const {
 const double half = std::max(0.5, config_.local_window_half_extent_m);
 return {{odom_position.x - half, odom_position.y - half},
 {odom_position.x + half, odom_position.y + half}};
}

} // namespace sim::autonomy
