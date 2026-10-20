#include "robot/autonomy/localization/known_map_localizer.hpp"

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
 map.map_id = "known-map-test";
 map.frame_id = "map";
 map.revision = 3;
 map.provenance = {"authored", "test", "1"};
 auto& grid = map.grid.emplace();
 grid.width = 80;
 grid.height = 80;
 grid.resolution_m = 0.1;
 grid.occupancy.assign(80 * 80, 0);
 auto occupy = [&](int x, int y) { grid.occupancy[static_cast<std::size_t>(y) * 80 + x] = 100; };
 for (int i = 0; i < 80; ++i) {
 occupy(i, 0); occupy(i, 79); occupy(0, i); occupy(79, i);
 }
 for (int y = 12; y < 66; ++y) occupy(53, y);
 for (int x = 18; x < 54; ++x) occupy(x, 24);
 for (int x = 30; x < 68; ++x) occupy(x, 56);
 return map;
}

sim::Scan scan_at(const sim::runtime::MapSnapshot& map, sim::Vec2 pose, double heading) {
 sim::Scan scan;
 scan.n_rays = 360;
 scan.max_range = 7.0;
 scan.angle_increment_rad = 2.0 * sim::kPi / scan.n_rays;
 scan.header = {12.0, 1, "laser", 0.0, sim::kMeasurementSchemaVersion, "test"};
 for (int i = 0; i < scan.n_rays; ++i) {
 const double angle = 2.0 * sim::kPi * i / scan.n_rays;
 double range = scan.max_range;
 for (double d = 0.05; d < scan.max_range; d += 0.025) {
 const sim::Vec2 point{pose.x + d * std::cos(heading + angle),
 pose.y + d * std::sin(heading + angle)};
 const auto cell = sim::runtime::world_to_grid(*map.grid, point);
 if (!cell)
 break;
 const auto index = static_cast<std::size_t>(cell->y) * map.grid->width + cell->x;
 if (map.grid->occupancy[index] >= 50) {
 range = d;
 break;
 }
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
 const sim::Vec2 truth{3.2, 4.1};
 const double truth_heading = 0.18;
 const auto scan = scan_at(map, truth, truth_heading);
 sim::KnownMapLocalizationConfig config;
 config.search_translation_m = 0.5;
 config.search_heading_rad = 0.2;
 config.translation_step_m = 0.05;
 config.heading_step_rad = 0.035;
 config.minimum_overlap = 0.20;
 config.minimum_returns = 12;
 sim::KnownMapLocalizer localizer(config);
 const auto result = localizer.localize(scan, {3.05, 4.22}, 0.11, map, 12.0);
 CHECK(result.accepted);
 CHECK(result.state == sim::autonomy::LocalizationState::Localized);
 CHECK(std::hypot(result.pose.position.x - truth.x,
 result.pose.position.y - truth.y) <= 0.12);
 CHECK(std::abs(sim::wrap_angle(result.pose.heading_rad - truth_heading)) <= 0.08);
 CHECK(result.pose.header.source_id == "robot.known-map-lidar");

 auto empty = scan;
 empty.hit.assign(empty.hit.size(), 0);
 const auto rejected = localizer.localize(empty, truth, truth_heading, map, 12.0);
 CHECK(!rejected.accepted);
 CHECK(rejected.state == sim::autonomy::LocalizationState::Degraded);
 CHECK(rejected.rejection_reason == "insufficient geometric returns");

 auto stale = scan;
 stale.header.stamp_s = 2.0;
 const auto stale_result = localizer.localize(stale, truth, truth_heading, map, 12.0);
 CHECK(!stale_result.accepted);
 CHECK(stale_result.state == sim::autonomy::LocalizationState::Degraded);
 CHECK(stale_result.rejection_reason == "invalid or stale scan");
 return 0;
}
