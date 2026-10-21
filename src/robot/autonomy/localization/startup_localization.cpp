#include "robot/autonomy/localization/startup_localization.hpp"

#include <cmath>
#include <utility>

namespace sim::autonomy {
namespace {

LocalizationResult relocalizing_result(double timestamp_s, std::string reason) {
 LocalizationResult result;
 result.state = LocalizationState::Relocalizing;
 result.rejection_reason = std::move(reason);
 result.pose.header = {timestamp_s, 0, "map", 0.0,
 kMeasurementSchemaVersion, "robot.startup-localization"};
 return result;
}

} // namespace

StartupLocalization::StartupLocalization(StartupLocalizationConfig config)
 : config_(std::move(config)),
 global_localizer_(config_.known_map),
 mapless_slam_(config_.mapless_slam) {}

void StartupLocalization::reset() {
 initialized_ = false;
 mapless_slam_.reset();
}

StartupLocalizationUpdate StartupLocalization::update(
 const ObservationFrame& observations,
 const EstimatedState& odometry_prior,
 const runtime::MapSnapshot* known_map,
 runtime::TimePoint sample_time) {
 const double stamp_s = sample_time.seconds();
 StartupLocalizationUpdate output;
 if (!std::isfinite(stamp_s) || stamp_s < 0.0) {
 output.localization.state = LocalizationState::Degraded;
 output.localization.rejection_reason = "invalid startup sample time";
 output.diagnostic = output.localization.rejection_reason;
 return output;
 }

 if (config_.mode == StartupLocalizationMode::KnownMap) {
 if (!known_map) {
 output.localization = relocalizing_result(stamp_s, "known map unavailable");
 output.diagnostic = output.localization.rejection_reason;
 return output;
 }
 if (!observations.lidar) {
 output.localization = relocalizing_result(stamp_s, "waiting for initial lidar scan");
 output.diagnostic = output.localization.rejection_reason;
 return output;
 }
 // The global provider intentionally receives an origin prior. It performs a
 // full-map search and therefore does not depend on a simulator/fused pose or
 // a caller-provided initial guess.
 output.localization = global_localizer_.localize_unknown_start(
 *observations.lidar, *known_map, stamp_s);
 if (!output.localization.accepted) {
 output.localization.state = LocalizationState::Relocalizing;
 output.diagnostic = output.localization.rejection_reason;
 return output;
 }
 initialized_ = true;
 output.diagnostic = "known-map startup localized";
 return output;
 }

 output.mapping = mapless_slam_.update(observations, odometry_prior, sample_time);
 const auto& mapping = *output.mapping;
 if (mapping.state == MappingState::Mapping && mapping.published_map) {
 output.localization.accepted = true;
 output.localization.state = LocalizationState::Localized;
 output.localization.pose = mapping.pose;
 output.localization.score = 1.0;
 output.localization.overlap = 1.0;
 output.localization.pose.header.source_id = "robot.startup-mapless-slam";
 initialized_ = true;
 output.diagnostic = "mapless SLAM startup initialized";
 } else {
 output.localization.state = initialized_ ? LocalizationState::Degraded
 : LocalizationState::Relocalizing;
 output.localization.rejection_reason = mapping.diagnostic;
 output.diagnostic = mapping.diagnostic;
 }
 return output;
}

} // namespace sim::autonomy
