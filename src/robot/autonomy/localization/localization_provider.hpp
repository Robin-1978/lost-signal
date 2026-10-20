#pragma once

#include "robot/runtime/map_provider.hpp"
#include "robot/runtime/observation.hpp"

#include <cstddef>
#include <string>

namespace sim::autonomy {

enum class LocalizationState {
 Unlocalized,
 Localized,
 Degraded,
 Relocalizing,
};

// Provider-neutral result shared by known-map localization, particle
// relocalization and future online SLAM frontends. A rejected result is still
// evidence: consumers must not treat it as an implicit identity correction.
struct LocalizationResult {
 bool accepted{false};
 LocalizationState state{LocalizationState::Unlocalized};
 PoseObservation pose;
 double score{0.0};
 double overlap{0.0};
 std::size_t usable_returns{0};
 std::string rejection_reason;
};

class ILocalizationProvider {
 public:
 virtual ~ILocalizationProvider() = default;

 virtual LocalizationResult localize(
 const Scan& scan, const Vec2& odom_position, double odom_heading_rad,
 const runtime::MapSnapshot& map, double timestamp_s) const = 0;
};

} // namespace sim::autonomy
