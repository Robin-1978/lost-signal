#include "robot/autonomy/mapping/online_slam.hpp"

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

sim::runtime::MapGrid make_environment() {
 sim::runtime::MapGrid map;
 map.width = 240;
 map.height = 240;
 map.resolution_m = 0.1;
 map.origin_m = {-12.0, -12.0};
 map.occupancy.assign(static_cast<std::size_t>(map.width) * map.height, 0);
 const auto occupy = [&](int x, int y) {
 map.occupancy[static_cast<std::size_t>(y) * map.width + x] = 100;
 };
 for (int i = 0; i < 240; ++i) {
 occupy(i, 0); occupy(i, 239); occupy(0, i); occupy(239, i);
 }
 // Asymmetric landmarks avoid the false confidence of a feature-free corridor.
 for (int y = 25; y < 210; ++y) occupy(150, y);
 for (int x = 30; x < 150; ++x) occupy(x, 80);
 for (int x = 80; x < 210; ++x) occupy(x, 170);
 for (int y = 110; y < 200; ++y) occupy(75, y);
 return map;
}

sim::Scan scan_at(const sim::runtime::MapGrid& map, sim::Vec2 pose,
 double stamp_s, std::uint64_t sequence) {
 sim::Scan scan;
 scan.n_rays = 180;
 scan.max_range = 8.0;
 scan.angle_min_rad = -sim::kPi;
 scan.angle_increment_rad = 2.0 * sim::kPi / scan.n_rays;
 scan.scan_period_s = 0.1;
 scan.header = {stamp_s, sequence, "lidar_link", 0.0,
 sim::kMeasurementSchemaVersion, "test.replay-lidar"};
 for (int i = 0; i < scan.n_rays; ++i) {
 const double angle = scan.angle_min_rad + scan.angle_increment_rad * i;
 double range = scan.max_range;
 for (double distance = 0.05; distance < scan.max_range; distance += 0.025) {
 const sim::Vec2 point{pose.x + distance * std::cos(angle),
 pose.y + distance * std::sin(angle)};
 const auto cell = sim::runtime::world_to_grid(map, point);
 if (!cell) break;
 const auto index = static_cast<std::size_t>(cell->y) * map.width + cell->x;
 if (map.occupancy[index] >= 50) {
 range = distance;
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
 sim::autonomy::OnlineSlamConfig config;
 config.width = 240;
 config.height = 240;
 config.resolution_m = 0.1;
 config.maximum_range_m = 8.0;
 config.translation_search_m = 0.30;
 config.translation_step_m = 0.05;
 config.heading_search_rad = 0.20;
 config.heading_step_rad = 0.05;
 config.minimum_match_score = 0.15;
 config.minimum_match_margin = 0.005;
 sim::autonomy::OnlineSlamMapBuilder builder(config);
 const auto environment = make_environment();

 sim::EstimatedState prior;
 prior.position = {100.0, 50.0};
 prior.confidence = 1.0;
 double maximum_error = 0.0;
 std::uint64_t previous_revision = 0;
 std::size_t previous_known_cells = 0;
 for (std::uint64_t sequence = 0; sequence < 30; ++sequence) {
 const double stamp_s = 1.0 + 0.1 * static_cast<double>(sequence);
 const double truth_x = 0.1 * static_cast<double>(sequence);
 sim::ObservationFrame observations;
 observations.time_s = stamp_s;
 observations.lidar = scan_at(environment, {truth_x, 0.0}, stamp_s, sequence);
 // The prior contains bounded wheel/IMU drift. SLAM must correct it when the
 // map has enough geometric evidence and retain it only when the match is weak.
 prior.position = {100.0 + truth_x + 0.06 * std::sin(0.4 * sequence), 50.0};
 prior.linear_velocity_mps = 1.0;
 const auto update = builder.update(
 observations, prior, sim::runtime::TimePoint::from_seconds(stamp_s));
 CHECK(update.state == sim::autonomy::MappingState::Mapping);
 CHECK(update.published_map);
 CHECK(update.map_revision == sequence + 1);
 CHECK(update.map_revision > previous_revision);
 const auto error = std::hypot(update.pose.position.x - truth_x,
 update.pose.position.y);
 maximum_error = std::max(maximum_error, error);
 const auto known_cells = static_cast<std::size_t>(std::count_if(
 update.published_map->grid->occupancy.begin(),
 update.published_map->grid->occupancy.end(),
 [](std::int8_t value) { return value >= 0; }));
 CHECK(known_cells >= previous_known_cells);
 previous_known_cells = known_cells;
 previous_revision = update.map_revision;
 }
 // This is a held-out bounded replay gate, not a claim of unlimited drift
 // performance or loop closure.
 CHECK(maximum_error < 0.12);
 CHECK(previous_known_cells > 100);
 return 0;
}
