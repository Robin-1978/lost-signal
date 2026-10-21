#include "robot/autonomy/localization/startup_localization.hpp"

#include <algorithm>
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

sim::runtime::MapSnapshot make_map() {
 sim::runtime::MapSnapshot map;
 map.map_id = "startup-map";
 map.frame_id = "map";
 map.revision = 1;
 map.provenance = {"test", "startup", "1"};
 auto& grid = map.grid.emplace();
 grid.width = 70;
 grid.height = 70;
 grid.resolution_m = 0.1;
 grid.origin_m = {0.0, 0.0};
 grid.occupancy.assign(70 * 70, 0);
 const auto occupy = [&](int x, int y) {
 grid.occupancy[static_cast<std::size_t>(y) * grid.width + x] = 100;
 };
 for (int i = 0; i < 70; ++i) {
 occupy(i, 0); occupy(i, 69); occupy(0, i); occupy(69, i);
 }
 for (int y = 10; y < 58; ++y) occupy(48, y);
 for (int x = 12; x < 48; ++x) occupy(x, 20);
 for (int x = 25; x < 62; ++x) occupy(x, 52);
 for (int y = 35; y < 61; ++y) occupy(30, y);
 return map;
}

sim::Scan scan_at(const sim::runtime::MapSnapshot& map, sim::Vec2 pose) {
 sim::Scan scan;
 scan.n_rays = 180;
 scan.max_range = 6.0;
 scan.angle_min_rad = -sim::kPi;
 scan.angle_increment_rad = 2.0 * sim::kPi / scan.n_rays;
 scan.header = {5.0, 1, "laser", 0.0,
 sim::kMeasurementSchemaVersion, "test.startup-lidar"};
 for (int i = 0; i < scan.n_rays; ++i) {
 const double angle = scan.angle_min_rad + scan.angle_increment_rad * i;
 double range = scan.max_range;
 for (double d = 0.05; d < scan.max_range; d += 0.025) {
 const sim::Vec2 point{pose.x + d * std::cos(angle), pose.y + d * std::sin(angle)};
 const auto cell = sim::runtime::world_to_grid(*map.grid, point);
 if (!cell) break;
 const auto index = static_cast<std::size_t>(cell->y) * map.grid->width + cell->x;
 if (map.grid->occupancy[index] >= 50) { range = d; break; }
 }
 scan.ranges.push_back(range);
 scan.hit.push_back(range < scan.max_range ? 1 : 0);
 scan.intensities.push_back(1.0);
 }
 return scan;
}

} // namespace

int main() {
 const auto map = make_map();
 const auto scan = scan_at(map, {2.8, 3.7});

 sim::autonomy::StartupLocalizationConfig config;
 config.known_map.coarse_field_resolution_m = 0.1;
 config.known_map.coarse_position_step_m = 0.2;
 config.known_map.minimum_overlap = 0.15;
config.known_map.minimum_score_margin = 0.03;
 config.known_map.minimum_returns = 8;
 config.known_map.maximum_returns = 96;
 config.known_map.maximum_evaluations = 500000;
 sim::autonomy::StartupLocalization startup(config);
 sim::EstimatedState unknown_prior;
 unknown_prior.position = {0.0, 0.0};
 unknown_prior.heading_rad = 0.0;
 unknown_prior.confidence = 0.0;

 sim::ObservationFrame empty;
 empty.time_s = 5.0;
 auto waiting = startup.update(empty, unknown_prior, &map,
 sim::runtime::TimePoint::from_seconds(5.0));
 CHECK(!waiting.localization.accepted);
 CHECK(waiting.localization.state == sim::autonomy::LocalizationState::Relocalizing);

 sim::ObservationFrame observation;
 observation.time_s = 5.0;
 observation.lidar = scan;
 const auto localized = startup.update(observation, unknown_prior, &map,
 sim::runtime::TimePoint::from_seconds(5.0));
 CHECK(localized.localization.accepted);
 CHECK(localized.localization.state == sim::autonomy::LocalizationState::Localized);
 CHECK(std::hypot(localized.localization.pose.position.x - 2.8,
 localized.localization.pose.position.y - 3.7) < 0.35);

 sim::autonomy::StartupLocalizationConfig mapless_config;
 mapless_config.mode = sim::autonomy::StartupLocalizationMode::MaplessSlam;
 mapless_config.mapless_slam.width = 120;
 mapless_config.mapless_slam.height = 120;
 mapless_config.mapless_slam.resolution_m = 0.1;
 mapless_config.mapless_slam.maximum_range_m = 6.0;
 sim::autonomy::StartupLocalization mapless(mapless_config);
 const auto built = mapless.update(observation, unknown_prior, nullptr,
 sim::runtime::TimePoint::from_seconds(5.0));
 CHECK(built.mapping);
 CHECK(built.mapping->state == sim::autonomy::MappingState::Mapping);
 CHECK(built.localization.accepted);
 CHECK(built.localization.state == sim::autonomy::LocalizationState::Localized);
 CHECK(built.localization.pose.header.source_id == "robot.startup-mapless-slam");

 // A symmetric boundary cannot establish a unique global pose. The startup
 // contract must remain in recovery instead of turning a weak match into a
 // map-frame correction.
 auto symmetric_map = map;
 std::fill(symmetric_map.grid->occupancy.begin(),
 symmetric_map.grid->occupancy.end(), 0);
 for (int i = 0; i < 70; ++i) {
 symmetric_map.grid->occupancy[static_cast<std::size_t>(i)] = 100;
 symmetric_map.grid->occupancy[static_cast<std::size_t>(69) * 70 + i] = 100;
 symmetric_map.grid->occupancy[static_cast<std::size_t>(i) * 70] = 100;
 symmetric_map.grid->occupancy[static_cast<std::size_t>(i) * 70 + 69] = 100;
 }
 sim::autonomy::StartupLocalization ambiguous_startup(config);
 auto symmetric_observation = observation;
 symmetric_observation.lidar = scan_at(symmetric_map, {3.5, 3.5});
 const auto ambiguous = ambiguous_startup.update(
 symmetric_observation, unknown_prior, &symmetric_map,
 sim::runtime::TimePoint::from_seconds(5.0));
 CHECK(!ambiguous.localization.accepted);
 CHECK(ambiguous.localization.state == sim::autonomy::LocalizationState::Relocalizing);
 return 0;
}
