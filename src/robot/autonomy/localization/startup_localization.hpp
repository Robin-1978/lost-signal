#pragma once

#include "robot/autonomy/localization/global_map_localizer.hpp"
#include "robot/autonomy/mapping/online_slam.hpp"

#include <memory>
#include <optional>
#include <string>

namespace sim::autonomy {

enum class StartupLocalizationMode {
 KnownMap,
 MaplessSlam,
};

struct StartupLocalizationConfig {
 StartupLocalizationMode mode{StartupLocalizationMode::KnownMap};
 GlobalMapLocalizationConfig known_map{};
 OnlineSlamConfig mapless_slam{};
};

// Owns only startup/recovery selection. It does not estimate from simulator
// truth and it never invents a map pose after a rejected match. A known-map
// startup searches the complete bounded map, so the odometry prior may be an
// arbitrary finite origin. Mapless startup delegates to the local SLAM
// provider and publishes the same pose/state contract.
struct StartupLocalizationUpdate {
 LocalizationResult localization;
 std::optional<MapBuilderUpdate> mapping;
 std::string diagnostic;
};

class StartupLocalization final {
 public:
 explicit StartupLocalization(StartupLocalizationConfig config = {});

 void reset();

 StartupLocalizationUpdate update(
 const ObservationFrame& observations,
 const EstimatedState& odometry_prior,
 const runtime::MapSnapshot* known_map,
 runtime::TimePoint sample_time);

 private:
 StartupLocalizationConfig config_;
 GlobalMapLocalizer global_localizer_;
 OnlineSlamMapBuilder mapless_slam_;
 bool initialized_{false};
};

} // namespace sim::autonomy
